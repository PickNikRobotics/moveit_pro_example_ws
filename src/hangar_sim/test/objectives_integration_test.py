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

import pytest
import rclpy
import tf2_ros
from rclpy.time import Time
from control_msgs.action import GripperCommand
from moveit_pro_test_utils.objective_test_fixture import (
    ExecuteObjectiveResource,
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
    try:
        run_objective(objective_id, should_cancel, execute_objective_resource)
    except AssertionError as e:
        mode = "cancel" if should_cancel else "execute"
        pytest.fail(f"Objective '{objective_id}' failed to {mode}: {e}")
    except Exception as e:
        mode = "cancel" if should_cancel else "execute"
        pytest.fail(
            f"Objective '{objective_id}' hit an unexpected error during {mode}: "
            f"{type(e).__name__}: {e}"
        )
