# Copyright 2025 PickNik Inc.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the PickNik Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""Integration tests for the lab_sim objective library."""

import time
import weakref
from typing import Any, Optional

import pytest
import rclpy
from rclpy.action import ActionClient
from rclpy.client import Client
from rclpy.node import Node
from control_msgs.action import GripperCommand
from controller_manager_msgs.srv import ListControllers, SwitchController
from moveit_pro_test_utils.objective_test_fixture import (
    EndStateSpec,
    ExecuteObjectiveResource,
    JointTarget,
    MUJOCO_RESET_HOOK,
    ObjectPoseTarget,
    SIM_RESETTER,
    execute_objective_resource as execute_objective_resource,
    get_objective_pytest_params,
    reset_simulation_before_test as reset_simulation_before_test,
    run_objective,
)

CONTROLLER_MANAGER_SERVICE_TIMEOUT_S = 10.0
# The very first reset can run before ros2_control has come up at all, so the
# initial wait for list_controllers gets a cold-start budget. Once the service
# has appeared on the graph, subsequent waits return immediately.
CONTROLLER_MANAGER_COLD_START_TIMEOUT_S = 120.0

# Cached controller_manager service clients, keyed weakly by owning node. The
# reset hook runs before every test; caching avoids re-running DDS discovery
# for the same two services on each reset (mirrors the reset_keyframe_client()
# caching in ExecuteObjectiveResource). The weak keying drops the entry when
# the node is garbage-collected, so a recreated node can never receive clients
# that were created on a dead node.
_controller_manager_clients: (
    "weakref.WeakKeyDictionary[Node, tuple[Client, Client]]"
) = weakref.WeakKeyDictionary()


def _get_controller_manager_clients(node: Node) -> tuple[Client, Client]:
    """Return cached (list_controllers, switch_controller) clients for ``node``."""
    if node not in _controller_manager_clients:
        _controller_manager_clients[node] = (
            node.create_client(ListControllers, "/controller_manager/list_controllers"),
            node.create_client(
                SwitchController, "/controller_manager/switch_controller"
            ),
        )
    return _controller_manager_clients[node]


def _call_service(
    node: Node, client: Client, request: Any, timeout_sec: float
) -> Optional[Any]:
    """Call a service and spin the node until the response arrives or times out.

    Returns the response, or None when the service is unavailable or the call
    does not complete within ``timeout_sec``.
    """
    if not client.wait_for_service(timeout_sec=timeout_sec):
        return None
    future = client.call_async(request)
    deadline = time.monotonic() + timeout_sec
    while not future.done() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    if not future.done():
        future.cancel()
        return None
    return future.result()


def _wait_until_controllers_active(
    node: Node,
    list_client: Client,
    controller_names: list[str],
    timeout_sec: float,
) -> bool:
    """Poll list_controllers until every named controller reports "active".

    Returns False when the controllers have not all reached "active" within
    ``timeout_sec``.
    """
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        response = _call_service(
            node, list_client, ListControllers.Request(), timeout_sec
        )
        if response is not None:
            active = {
                controller.name
                for controller in response.controller
                if controller.state == "active"
            }
            if set(controller_names) <= active:
                return True
        rclpy.spin_once(node, timeout_sec=0.1)
    return False


def _controller_safe_mujoco_reset(
    context: ExecuteObjectiveResource,
    keyframe_name: str,
    timeout_sec: float,
    settle_sec: float,
) -> Optional[bool]:
    """Reset the MuJoCo keyframe with active controllers cycled around the reset.

    The plain keyframe reset writes the keyframe qpos/ctrl into the sim, but an
    active position-holding controller (joint_trajectory_controller after an
    interrupted or cancelled motion) re-commands its stale hold setpoint on the
    very next update cycle, driving the arm straight back to the pre-reset pose.
    The next test then fails PlanToJointGoal start-state validation with
    "initial joint positions are not valid: self-collision" — the repeating CI
    flake on mpc_pose_tracking_static_point_cloud_avoidance_with_sphere_down_sample
    and grasp_pose_using_yaml.

    Cycling fixes that: deactivate every active controller that claims command
    interfaces (broadcasters claim none and are left alone), reset the keyframe,
    then reactivate the same set. Re-activation re-initializes each controller's
    hold state at the post-reset (keyframe) position.
    """
    node = context.node
    list_client, switch_client = _get_controller_manager_clients(node)
    # Fail closed when the controllers cannot be enumerated: without the
    # active-controller list the cycling guarantee is gone and a stale hold
    # setpoint could survive the reset — exactly the flake this hook removes.
    # Still perform the plain keyframe reset first to restore the best state
    # we can before the fixture aborts the test.
    if not list_client.wait_for_service(
        timeout_sec=CONTROLLER_MANAGER_COLD_START_TIMEOUT_S
    ):
        print(
            "[lab_sim] list_controllers not advertised within "
            f"{CONTROLLER_MANAGER_COLD_START_TIMEOUT_S:.1f}s; cannot verify "
            "controller state around the keyframe reset.",
            flush=True,
        )
        MUJOCO_RESET_HOOK(context, keyframe_name, timeout_sec, settle_sec)
        return False
    list_response = _call_service(
        node,
        list_client,
        ListControllers.Request(),
        CONTROLLER_MANAGER_SERVICE_TIMEOUT_S,
    )
    if list_response is None:
        print(
            "[lab_sim] list_controllers call timed out; cannot verify "
            "controller state around the keyframe reset.",
            flush=True,
        )
        MUJOCO_RESET_HOOK(context, keyframe_name, timeout_sec, settle_sec)
        return False
    holding_controllers = [
        controller.name
        for controller in list_response.controller
        if controller.state == "active" and controller.claimed_interfaces
    ]
    deactivate_ok = True
    if holding_controllers:
        # STRICT: the set was computed from the live "active" list one call
        # ago and nothing switches controllers between tests, so a failure
        # here is a real problem, not an already-inactive controller.
        deactivate_request = SwitchController.Request()
        deactivate_request.deactivate_controllers = holding_controllers
        deactivate_request.strictness = SwitchController.Request.STRICT
        deactivate_response = _call_service(
            node,
            switch_client,
            deactivate_request,
            CONTROLLER_MANAGER_SERVICE_TIMEOUT_S,
        )
        deactivate_ok = deactivate_response is not None and deactivate_response.ok
        if not deactivate_ok:
            # The reset below still runs to restore the best state we can,
            # but the stale-hold-pose problem this hook exists to fix may
            # recur — fail closed via the return value so the fixture aborts
            # the test loudly instead of running on polluted state.
            print(
                "[lab_sim] Failed to deactivate controllers "
                f"{holding_controllers} before keyframe reset.",
                flush=True,
            )
    # settle_sec=0 here: settling is deferred until after reactivation below.
    reset_ok = MUJOCO_RESET_HOOK(context, keyframe_name, timeout_sec, 0.0)
    reactivate_ok = True
    if holding_controllers:
        # BEST_EFFORT on reactivation: a STRICT failure on one controller
        # would leave the rest deactivated too. The verification below is the
        # real gate.
        activate_request = SwitchController.Request()
        activate_request.activate_controllers = holding_controllers
        activate_request.strictness = SwitchController.Request.BEST_EFFORT
        activate_response = _call_service(
            node,
            switch_client,
            activate_request,
            CONTROLLER_MANAGER_SERVICE_TIMEOUT_S,
        )
        reactivate_ok = activate_response is not None and activate_response.ok
        if reactivate_ok:
            # SwitchController returns ok when the switch is accepted, not
            # when on_activate has run — confirm via list_controllers that
            # every cycled controller reports "active" again before settling.
            reactivate_ok = _wait_until_controllers_active(
                node,
                list_client,
                holding_controllers,
                CONTROLLER_MANAGER_SERVICE_TIMEOUT_S,
            )
        if not reactivate_ok:
            print(
                "[lab_sim] Failed to reactivate controllers "
                f"{holding_controllers} after keyframe reset.",
                flush=True,
            )
    if settle_sec > 0:
        # Best-effort settle for the reactivated controllers to converge on
        # the keyframe pose before the next test ticks behaviors.
        time.sleep(settle_sec)
    return bool(reset_ok) and deactivate_ok and reactivate_ok


# Override the fixture's default lab_sim registration ("last call wins") with
# the controller-cycling variant above.
SIM_RESETTER.register("lab_sim", _controller_safe_mujoco_reset)

# Looping objectives to cancel partway through
cancel_objectives = {
    "3 Waypoints Pick and Place",
    "Classical Pick and Place",
    "Cycle Between Waypoints",
    "Get AprilTag Pose from Image",
    "Grasp Pose Tuning With April Tag",
    "Grasp Pose Using Yaml",
    "Pick April Tag Labeled Object With Approval",
    "Plan and Save Trajectory",
    "Record and Replay Scanning Motion",
    "Stationary Admittance",
}

# Objectives to skip entirely from integration testing
skip_objectives = {
    "AddBottlesToPlanningScene",
    "Grasp Planning",
    "Joint Diagnostic",
    "ML Auto Grasp Object from Clicked Point",  # Skipped because there is no primary ui to switch to in ci
    "ML Find Bottles on Table from Image Exemplar",  # Skipped because it looks for a file on a home path
    "ML Segment Image",
    "ML Segment Image Loop",
    "ML Segment Point Cloud",  # Requires GPU for ONNX inference; falls back to CPU and times out waiting for /wrist_camera/points on CI runners without a camera warmup delay.
    "ML Segment Point Cloud from Clicked Point",
    "MPC Pose Tracking",
    "MPC Pose Tracking With Point Cloud Avoidance",
    "Octomap Example",  # Requires user input to clear the octomap.
    "Pick 1 Pill Bottle with ML",
    "Pick All Bottles with AprilTags",
    "Pick All Pill Bottles",
    "Pick up Object",
    "Place Object",
    # Flaky on CI after the MuJoCo 3.2.7 -> 3.6.0 upgrade: the JTAC compliance loop
    # is sensitive to the simulator falling under realtime, and we see ~50% flake
    # rate even with the 0.003 s scene timestep and CPU pinning. See PR #615.
    "Push Button With a Trajectory",
    "Record Square Trajectory",
    "Scan Scene - Multiple Point Clouds",
    "Stack Objects with ICP",  # Skipped because there is no primary ui to switch to in ci
    "Stitch Multiple Point Clouds Together",
    "Marker Visualization Example",  # Server not available for GetTextFromUser
    "Register CAD Part",  # TODO: remove this for 9.2.0 and fix it correctly
    # DoTeleoperateAction immediately rejects the goal when no UI tab is subscribed to
    # /moveit_pro_ui/do_teleoperate/goal. The objective never reaches RUNNING status, so
    # the cancel flow times out. This is a headless-CI limitation, not a bug in the objective.
    "Teleoperate",
}


# This is a workaround to avoid the test running before ros control is ready
def wait_for_gripper_action_server(timeout: int = 180):
    node = rclpy.create_node("test_node")
    action_client = ActionClient(
        node, GripperCommand, "/robotiq_gripper_controller/gripper_cmd"
    )
    if not action_client.wait_for_server(timeout_sec=timeout):
        raise TimeoutError(
            "Timeout waiting for robotiq_gripper_controller gripper action server to start"
        )
    return True


# --- End-state correctness checks ---
#
# A small, deterministic set of objectives gets an end-state assertion beyond
# SUCCESS/FAILURE. Broad population is a follow-up tied to #17769.
#
# "Look at Table" drives the manipulator to the saved "Look at Table" waypoint;
# the expected target is resolved at runtime from /get_saved_waypoints (same
# source of truth the objective uses). Unlike hangar_sim, the saved waypoint's
# full joint set (arm + rail + gripper) is commanded or held at its stored
# value, so no per-joint filtering is needed (validated live: worst error
# 2.6e-3 rad across all 8 joints).
JOINT_CHECK_OBJECTIVE = "Look at Table"
JOINT_CHECK_WAYPOINT = "Look at Table"

# "Move Flasks to Burners" places flask_1 and flask_2 onto the desk burners.
# Expected positions are MuJoCo ground truth read from TF after a validated
# run (picknik_mujoco_ros broadcasts every non-robot free-joint body as
# mj_world -> <body>); run-to-run repeatability measured at < 3 mm, and the
# two burners sit ~0.11 m apart, so the default 0.05 m tolerance both absorbs
# physics noise and catches a flask landing on the wrong burner. Position-only:
# the flask is radially symmetric, so its final yaw is not deterministic.
OBJECT_CHECK_OBJECTIVE = "Move Flasks to Burners"
FLASK_END_POSES = {
    "flask_1": [0.6583, 0.5767, 0.5465],
    "flask_2": [0.5446, 0.5775, 0.5465],
}

EXPECTED_END_STATE_BY_ID = {
    OBJECT_CHECK_OBJECTIVE: EndStateSpec(
        objects=[
            ObjectPoseTarget(
                object_frame=frame,
                reference_frame="world",
                position=position,
            )
            for frame, position in FLASK_END_POSES.items()
        ]
    ),
}


def _expected_end_state_by_id(
    objective_id: str,
    resource: ExecuteObjectiveResource,
) -> Optional[dict[str, EndStateSpec]]:
    """Build the end-state spec for objectives we assert on; None otherwise.

    The joint-check waypoint is resolved lazily, only for the objective under
    test, so a missing /get_saved_waypoints surfaces solely on the objective
    that needs it rather than penalizing the whole suite.
    """
    if objective_id == JOINT_CHECK_OBJECTIVE:
        target = resource.get_waypoint_target(JOINT_CHECK_WAYPOINT)
        return {objective_id: EndStateSpec(joints=JointTarget(positions=target))}
    if objective_id == OBJECT_CHECK_OBJECTIVE:
        return EXPECTED_END_STATE_BY_ID
    return None


@pytest.mark.parametrize(
    "objective_id, should_cancel",
    get_objective_pytest_params("lab_sim", cancel_objectives, skip_objectives),
)
def test_all_objectives(
    objective_id: str,
    should_cancel: bool,
    execute_objective_resource: ExecuteObjectiveResource,
):
    wait_for_gripper_action_server()
    expected_end_state_by_id = _expected_end_state_by_id(
        objective_id, execute_objective_resource
    )
    try:
        run_objective(
            objective_id,
            should_cancel,
            execute_objective_resource,
            expected_end_state_by_id=expected_end_state_by_id,
        )
    except AssertionError as e:
        mode = "cancel" if should_cancel else "execute"
        pytest.fail(f"Objective '{objective_id}' failed to {mode}: {e}")
    except Exception as e:
        mode = "cancel" if should_cancel else "execute"
        pytest.fail(
            f"Objective '{objective_id}' hit an unexpected error during {mode}: {type(e).__name__}: {e}"
        )
