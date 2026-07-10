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

"""Integration tests for the hangar_sim objective library."""

import time
from pathlib import Path

import pytest
import rclpy
import tf2_ros
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry
import yaml
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from control_msgs.action import GripperCommand
from controller_manager_msgs.srv import ListControllers
from moveit_pro_test_utils.objective_test_fixture import (
    EndStateSpec,
    ExecuteObjectiveResource,
    JointTarget,
    MUJOCO_RESET_HOOK,
    MUJOCO_RESET_SERVICE,
    SIM_RESETTER,
    execute_objective_resource as execute_objective_resource,
    get_objective_pytest_params,
    reset_simulation_before_test as reset_simulation_before_test,
    run_objective,
    wait_for_action_servers,
    wait_for_services,
)

# hangar_sim is MuJoCo-backed (declares a mujoco_model), so it advertises
# /mujoco_system/reset_keyframe. Register it with the fixture's reset hook here
# -- the fixture documents this module as the per-sim fan-out point -- so the
# autouse reset_simulation_before_test fixture resets the keyframe between
# objectives. Registering here keeps this an example_ws-only change instead of
# editing the fixture's built-in defaults in moveit_pro.
SIM_RESETTER.register("hangar_sim", MUJOCO_RESET_HOOK)

# Loop-style objectives cancelled mid-execution rather than waited to
# completion. "Plan Path Along Surface - Loop" wraps the surface-following
# sequence in KeepRunningUntilFailure, so it never terminates on its own.
#
# NOTE (jazzy watch-item): this objective ticks SendPointCloudToUI. If the
# first dispatched CI run reproduces the jazzy `pcl::fromPCLPointCloud2: No
# data to copy` crash, move this entry to skip_objectives and cite the
# PR #19531 subscriber-gated render-skip follow-up.
cancel_objectives = {
    "Plan Path Along Surface - Loop",
}

# Objectives skipped entirely, each with the reason it cannot run headless in
# CI. The non-runnable objectives (runnable="false" in their MetadataFields)
# are filtered out by the fixture and need not be listed here.
skip_objectives = {
    # ML segmentation / grasp pipelines: ONNX inference runs on CPU in CI
    # (the GPU runner backs MuJoCo's EGL render only), so these time out --
    # same class lab_sim skips for the same reason.
    "ML Move Boxes to Loading Zone",
    "Move Boxes Looping",  # KeepRunningUntilFailure loop over the ML pick pipeline above.
    # SAM3 diagnostic: needs the moveit_pro_sam3 model package, which the CI
    # image does not ship (GetMasks2DFromExemplar fails to resolve the encoder
    # model path). Same ML-inference class as the skips above.
    "Calibrate SAM3 Mask Areas",
    "Segment Image from Point",
    "Segment Image from Text Prompt",
    "Segment Point Cloud from Clicked Point",
    # User input required: no primary UI is attached in headless CI, so these
    # block on a pose-from-user prompt or an MTC-solution/path approval that
    # never arrives.
    "Navigate to Clicked Point",  # GetPoseFromUser + WaitForUserPathApproval.
    "Navigate to Clicked Point with Replanning",  # GetPoseFromUser.
    "Find and Spray Plane",  # Ungated WaitForMTCSolutionApproval.
    "Solution - Find and Spray Plane",  # Ungated WaitForMTCSolutionApproval.
    "Solution - Spray Plane",  # Ungated WaitForMTCSolutionApproval.
    # ValidateTrajectory rejects the intentionally-colliding demo path, so the
    # tree falls back to WaitForMTCSolutionApproval, which blocks headless.
    "Cartesian Path with Collision Checking",
    # IsUserAvailable returns FAILURE headless, so its top-level Sequence
    # fails before reaching WaitForMTCSolutionApproval.
    "Solution - Draw Square",
    "Cartesian Plan Simple Square",
    # Core-library objectives (moveit_pro_objectives) aggregated into every
    # config. Both need a primary UI and are skipped by lab_sim for the same
    # reasons.
    "Teleoperate",  # DoTeleoperateAction rejects the goal with no UI subscribed.
    "Marker Visualization Example",  # GetTextFromUser server unavailable headless.
    # Jazzy CI-runner flakes (pass on the slower humble runner, intermittent on
    # jazzy). Skipped so the suite is deterministic on both distros.
    #
    # "Solution - Draw Picknik" is the suite's longest objective and sits on
    # the 90 s execute-timeout boundary on the jazzy runner. When it overruns,
    # the mid-execution halt leaves the joint_trajectory_controller commanding
    # the interrupted draw pose; the 5 s inter-test reset cannot recover it, so
    # the next objectives plan from a self-colliding start. Skipping removes the
    # timeout flake and its downstream sim-state pollution.
    "Solution - Draw Picknik",
    # "Point-to-Point Trajectory" interpolates to the "Arm Forward" waypoint on
    # joint_trajectory_controller, which also owns the mecanum linear_x_joint
    # (config/control/picknik_ur.ros2_control.yaml). The interpolated terminal
    # velocity carries a floating-point residual (~3.8e-7) that the controller's
    # nonzero-terminal-velocity validation intermittently rejects ("Velocity of
    # last trajectory point of joint linear_x_joint is not zero"). Real
    # controller/interpolation bug, not a test issue -- skip pending that fix.
    "Point-to-Point Trajectory",
}

# Action servers the hangar_sim tree types need before any objective runs.
# hangar_sim drives a vacuum gripper through ros2_control's
# GripperActionController (config/control/picknik_ur.ros2_control.yaml), which
# advertises this GripperCommand action once control comes up.
required_action_servers: list[tuple[type, str]] = [
    (GripperCommand, "/vacuum_gripper/gripper_cmd"),
]


@pytest.fixture(scope="module", autouse=True)
def wait_for_mujoco_reset_service(
    execute_objective_resource: ExecuteObjectiveResource,
) -> None:
    """Block until the MuJoCo reset service is advertised, once per module.

    hangar_sim's startup is heavy (ridgeback base + nav2 + the full controller
    set + a large scene), so ``/mujoco_system/reset_keyframe`` comes up well
    after the agent heartbeat that gates ``execute_objective_resource``. The
    autouse ``reset_simulation_before_test`` hook only allows a 5 s budget per
    test, which the first reset blows in CI. This module-scoped fixture runs
    before any per-test reset and warms the service up with a generous timeout,
    so each per-test reset then finds it immediately.
    """
    wait_for_services(
        execute_objective_resource.node, [MUJOCO_RESET_SERVICE], timeout_sec=120.0
    )


# Controllers the config spawns at startup (config.yaml ros2_control
# controllers_active_at_startup + controllers_inactive_at_startup), read from
# the config itself so the list cannot drift from it. The spawners run with
# --controller-manager-timeout 180 and can lag several minutes behind the
# first tests on a loaded CI runner, so the first controller-using objective
# (cartesian_draw_geometry_from_file) raced them and failed SwitchController
# with "Controller joint_trajectory_controller is not an existing controller"
# or "command interface platform_velocity_controller/linear_x_joint/velocity
# is not available".
_CONFIG_YAML_PATH = Path(__file__).parent.parent / "config" / "config.yaml"


def _spawned_controllers_from_config() -> frozenset[str]:
    """Read the spawned-controller names from hangar_sim's config.yaml."""
    with _CONFIG_YAML_PATH.open("r", encoding="utf-8") as config_file:
        ros2_control = yaml.safe_load(config_file)["ros2_control"]
    return frozenset(
        ros2_control["controllers_active_at_startup"]
        + ros2_control["controllers_inactive_at_startup"]
    )


REQUIRED_CONTROLLERS = _spawned_controllers_from_config()
# A controller in either of these states is loaded and configured; "inactive"
# is enough for a chained controller to export its command interfaces.
CONFIGURED_CONTROLLER_STATES = ("inactive", "active")
CONTROLLER_LOAD_TIMEOUT_S = 300.0
LIST_CONTROLLERS_CALL_TIMEOUT_S = 10.0


@pytest.fixture(scope="module", autouse=True)
def wait_for_controllers_loaded(
    execute_objective_resource: ExecuteObjectiveResource,
) -> None:
    """Block until every spawned controller is loaded and configured.

    Waits for each controller in ``REQUIRED_CONTROLLERS`` to report
    ``inactive`` or ``active`` from ``/controller_manager/list_controllers``,
    once per module. ``inactive`` is enough: a configured chained controller
    (platform_velocity_controller) already exports the command interfaces the
    whole-body joint_trajectory_controller claims on activation.

    Defined after ``wait_for_mujoco_reset_service`` on purpose: pytest runs
    module-scoped autouse fixtures in definition order, and the MuJoCo reset
    service must be warm before the first per-test reset regardless of how
    long the controller wait takes. Keep this fixture below that one.
    """
    node = execute_objective_resource.node
    client = node.create_client(ListControllers, "/controller_manager/list_controllers")
    missing = set(REQUIRED_CONTROLLERS)
    deadline = time.monotonic() + CONTROLLER_LOAD_TIMEOUT_S
    try:
        while time.monotonic() < deadline:
            if not client.wait_for_service(timeout_sec=1.0):
                continue
            future = client.call_async(ListControllers.Request())
            call_deadline = time.monotonic() + LIST_CONTROLLERS_CALL_TIMEOUT_S
            while not future.done() and time.monotonic() < call_deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
            if not future.done():
                # Abandoning an in-flight future and destroying the client
                # later trips rcl "use after free" errors — cancel it first.
                future.cancel()
                # Pace the retry instead of re-issuing the call back-to-back.
                rclpy.spin_once(node, timeout_sec=0.5)
                continue
            response = future.result()
            configured = {
                controller.name
                for controller in response.controller
                if controller.state in CONFIGURED_CONTROLLER_STATES
            }
            missing = set(REQUIRED_CONTROLLERS) - configured
            if not missing:
                return
            rclpy.spin_once(node, timeout_sec=0.5)
        pytest.fail(
            f"Controllers not loaded and configured within "
            f"{CONTROLLER_LOAD_TIMEOUT_S:.1f}s: {sorted(missing)}. "
            "Objectives that switch controllers cannot run."
        )
    finally:
        node.destroy_client(client)


# Fixed and end-effector frames the Cartesian objectives visualize and plan
# against (VisualizePose / PlanCartesianPath look up TF_FIXED_FRAME ->
# TF_TIP_FRAME).
TF_FIXED_FRAME = "world"
TF_TIP_FRAME = "grasp_link"
TF_WARMUP_TIMEOUT_S = 120.0


@pytest.fixture(scope="module", autouse=True)
def wait_for_robot_tf(
    execute_objective_resource: ExecuteObjectiveResource,
) -> None:
    """Block until ``world -> grasp_link`` is on ``/tf``, once per module.

    ``grasp_link``'s transform only appears after robot_state_publisher and the
    arm controllers start streaming joint states, which lags the agent
    heartbeat on a cold backend. The Cartesian objectives look it up
    immediately, so on the slower jazzy runner the first one
    (``cartesian_draw_geometry_from_file``) raced TF and failed with "Cannot
    find transform between frames ``world`` and ``grasp_link``". Spinning a
    short-lived listener until the transform resolves removes that startup race
    without masking the objective.
    """
    node = execute_objective_resource.node
    buffer = tf2_ros.Buffer()
    # spin_thread=False: we pump the node manually below, matching the rest of
    # the fixture's single-threaded spin model (no background TF thread).
    tf2_ros.TransformListener(buffer, node, spin_thread=False)
    deadline = time.monotonic() + TF_WARMUP_TIMEOUT_S
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if buffer.can_transform(TF_FIXED_FRAME, TF_TIP_FRAME, Time()):
            return
    pytest.fail(
        f"{TF_FIXED_FRAME} -> {TF_TIP_FRAME} transform not available within "
        f"{TF_WARMUP_TIMEOUT_S:.1f}s; cannot run Cartesian objectives."
    )


# --- End-state correctness checks ---
#
# A small, deterministic set of objectives gets an end-state assertion beyond
# SUCCESS/FAILURE. Broad population is out of scope (follow-up tied to #17769).
#
# "Move to Arm Upright" drives the manipulator group to the "Arm upright"
# waypoint via the joint_trajectory_controller. We resolve the expected target
# at runtime from /get_saved_waypoints (same source of truth the objective
# uses), then restrict the comparison to the manipulator arm joints: the saved
# waypoint's joint_state also carries the mecanum base joints (linear_x/y,
# rotational_yaw), which this objective does not command, so asserting them
# would compare against an uncommanded pose.
JOINT_CHECK_OBJECTIVE = "Move to Arm Upright"
JOINT_CHECK_WAYPOINT = "Arm upright"
MANIPULATOR_ARM_JOINTS = (
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
)


def _expected_end_state_by_id(
    objective_id: str,
    resource: ExecuteObjectiveResource,
) -> dict[str, EndStateSpec] | None:
    """Build the end-state spec for objectives we assert on; None otherwise.

    Resolves the waypoint target lazily, only for the objective under test, so
    a missing /get_saved_waypoints surfaces solely on the objective that needs
    it rather than penalizing the whole suite.
    """
    if objective_id != JOINT_CHECK_OBJECTIVE:
        return None
    waypoint_joints = resource.get_waypoint_target(JOINT_CHECK_WAYPOINT)
    arm_target = {joint: waypoint_joints[joint] for joint in MANIPULATOR_ARM_JOINTS}
    return {objective_id: EndStateSpec(joints=JointTarget(positions=arm_target))}


BASE_LINK_FRAME = "ridgeback_base_link"
# Phase 1: how long to wait for the FIRST transform into the base link (and the
# first /odom message) before declaring the stack broken. Generous on purpose:
# it only bounds startup latency on a loaded CI runner, not the assertion.
TF_FIRST_SIGHTING_TIMEOUT_S = 60.0
# Phase 2: once the frame has been seen, how long to keep sampling to catch
# additional (conflicting) parents. Competing publishers broadcast at 10-120 Hz,
# so a second parent would appear hundreds of times within this window.
TF_PARENT_SAMPLE_DURATION_S = 4.0


def test_base_link_has_single_tf_parent(
    execute_objective_resource: ExecuteObjectiveResource,
) -> None:
    """The live transform into ridgeback_base_link must have exactly one publisher.

    Regression guard for the three-parent TF conflict (MuJoCo lidar fill-in
    chain, MuJoCo odom TF, and robot_state_publisher all claiming the frame),
    which made the robot oscillate below base_link in the web UI.
    robot_state_publisher owns the edge via the virtual-rail chain; a second
    parent appearing here means a MuJoCo TF publisher was re-enabled (check
    base_link_name / odom_publish_tf in the ros2_control xacro) or fuse's
    publish_tf was turned back on.
    """
    node = execute_objective_resource.node
    parents: set[str] = set()
    odom_z_samples: list[float] = []

    # Watching /tf only is correct while rotational_yaw_joint (the edge into
    # ridgeback_base_link) is a moving joint; if it ever becomes fixed the edge
    # moves to /tf_static and this test must follow it there.
    def collect(msg: TFMessage) -> None:
        for transform in msg.transforms:
            if transform.child_frame_id == BASE_LINK_FRAME:
                parents.add(transform.header.frame_id)

    def collect_odom(msg: Odometry) -> None:
        odom_z_samples.append(msg.pose.pose.position.z)

    subscription = node.create_subscription(TFMessage, "/tf", collect, 100)
    # Piggyback an odom_planar check: odom_zero_z was a silently-ignored
    # parameter for months, so pin the real one's effect (z zeroed in /odom)
    # rather than trusting the spelling.
    odom_subscription = node.create_subscription(
        Odometry, "/odom", collect_odom, qos_profile_sensor_data
    )
    # Phase 1: wait for the first sighting so slow stack bring-up on a loaded
    # CI runner cannot masquerade as "wrong publishers" (or eat into the
    # sampling window below).
    first_sighting_deadline = time.monotonic() + TF_FIRST_SIGHTING_TIMEOUT_S
    while time.monotonic() < first_sighting_deadline and not (
        parents and odom_z_samples
    ):
        rclpy.spin_once(node, timeout_sec=0.1)

    # Distinguish "frame never seen" (a startup failure or renamed frame) from
    # "wrong publishers" so failures read correctly.
    assert parents, (
        f"No transform into {BASE_LINK_FRAME} observed on /tf within "
        f"{TF_FIRST_SIGHTING_TIMEOUT_S:.1f}s — robot_state_publisher may not "
        f"be up, or the frame was renamed."
    )

    # Phase 2: the frame is live; sample long enough that any competing
    # publisher (10-120 Hz) would land in `parents`.
    sample_deadline = time.monotonic() + TF_PARENT_SAMPLE_DURATION_S
    while time.monotonic() < sample_deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_subscription(subscription)
    node.destroy_subscription(odom_subscription)
    assert parents == {"virtual_rail_link_2"}, (
        f"Expected robot_state_publisher's virtual_rail_link_2 as the sole TF "
        f"parent of {BASE_LINK_FRAME}, saw parents: {sorted(parents)}"
    )
    assert (
        odom_z_samples
    ), "No /odom messages observed; MuJoCo odom publisher may be off."
    assert all(abs(z) < 1e-6 for z in odom_z_samples), (
        f"odom_planar should zero /odom pose z for this planar base; "
        f"saw max |z| = {max(abs(z) for z in odom_z_samples):.4f}"
    )


@pytest.mark.parametrize(
    "objective_id, should_cancel",
    get_objective_pytest_params("hangar_sim", cancel_objectives, skip_objectives),
)
def test_all_objectives(
    objective_id: str,
    should_cancel: bool,
    execute_objective_resource: ExecuteObjectiveResource,
):
    """Run (or cancel) each hangar_sim objective and assert it completes without error."""
    wait_for_action_servers(
        execute_objective_resource.node,
        [name for _, name in required_action_servers],
    )
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
            f"Objective '{objective_id}' hit an unexpected error during {mode}: "
            f"{type(e).__name__}: {e}"
        )
