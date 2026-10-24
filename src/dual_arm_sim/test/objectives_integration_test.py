# Copyright 2026 PickNik Inc.
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

"""Integration tests for the dual_arm_sim objective library."""

import math

import pytest

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

# dual_arm_sim is a MuJoCo-backed sim: reset the keyframe between tests with the
# default reset hook.
SIM_RESETTER.register("dual_arm_sim", MUJOCO_RESET_HOOK)

# Looping objectives to cancel partway through rather than run to completion.
cancel_objectives: set[str] = set()

# Objectives to skip entirely. Teleoperate waits on UI input, and the block
# find/sort objectives drive ML text-prompt segmentation that the headless CI
# backend cannot satisfy. Expand this set from the first weekly CI run rather
# than guessing more aggressively up front.
skip_objectives: set[str] = {
    "Teleoperate",  # Waits on UI teleoperation input.
    "Navigate to Clicked Point",  # GetPoseFromUser waits on a click in the UI.
    "Marker Visualization Example",  # GetTextFromUser server unavailable headless.
    # Hand-eye calibration Objectives from moveit_pro_objectives: they generate or
    # move through calibration waypoints and detect a ChArUco board, which only
    # hand_eye_calibration_sim provides.
    "Calibrate Camera Intrinsics",
    "Calibrate Eye In Hand Camera",
    "Calibrate Eye To Hand Camera",
    "Calibrate Multiple Cameras",
    "Generate Calibration Waypoints",
    "Writing Demo",  # Long-running drawing objective times out on the CI backend.
    "Find Green Block",  # ML text-prompt segmentation.
    "Find Red Block",  # ML text-prompt segmentation.
    "Sort Blocks",  # ML perception + multi-step grasp pipeline.
}

# End-state correctness checks beyond SUCCESS/FAILURE. Each target is resolved at
# runtime from /get_saved_waypoints (the same source of truth the objective moves
# to), restricted to the joint group the objective commands. "Move Left Home" and
# "Move Right Home" drive one arm, without the spine, to the shared "Home"
# waypoint. "Whole Body Move to Waypoint" is checked by _whole_body_end_state.
expected_end_state_by_id = {
    "Move Left Home": EndStateSpec(
        joints=JointTarget(
            waypoint="Home", joint_group_name="left_manipulator_without_spine"
        )
    ),
    "Move Right Home": EndStateSpec(
        joints=JointTarget(
            waypoint="Home", joint_group_name="right_manipulator_without_spine"
        )
    ),
    "Stow Arms for Navigation": EndStateSpec(
        joints=JointTarget(waypoint="Stow", joint_group_name="manipulator")
    ),
}

WHOLE_BODY_OBJECTIVE = "Whole Body Move to Waypoint"
WHOLE_BODY_WAYPOINT = "Whole Body Demo"
PLANAR_JOINTS = ("planar_x", "planar_y", "planar_theta")
# Looser than body_jtc's planar goal tolerance, tighter than the fixture default.
BASE_POSITION_TOLERANCE_M = 0.02
BASE_YAW_TOLERANCE_RAD = 0.02


def _whole_body_end_state(resource: ExecuteObjectiveResource) -> EndStateSpec:
    """Spine and arm joints from the waypoint, and the base as the pose of base_link.

    The fixture reads one /joint_states message, and the odometry bridge publishes
    the planar joints in a separate one, so the base is checked through TF.
    """
    joints = resource.get_waypoint_target(
        WHOLE_BODY_WAYPOINT, joint_group_name="whole_body"
    )
    x, y, yaw = (joints.pop(name) for name in PLANAR_JOINTS)
    return EndStateSpec(
        joints=JointTarget(positions=joints),
        objects=[
            ObjectPoseTarget(
                object_frame="base_link",
                reference_frame="world",
                position=[x, y, 0.0],
                orientation_xyzw=[0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)],
                position_tolerance=BASE_POSITION_TOLERANCE_M,
                orientation_tolerance=BASE_YAW_TOLERANCE_RAD,
            )
        ],
    )


@pytest.mark.parametrize(
    "objective_id, should_cancel",
    get_objective_pytest_params("dual_arm_sim", cancel_objectives, skip_objectives),
)
def test_all_objectives(
    objective_id: str,
    should_cancel: bool,
    execute_objective_resource: ExecuteObjectiveResource,
) -> None:
    """Run (or cancel) each dual_arm_sim objective and assert it completes without error."""
    end_states = dict(expected_end_state_by_id)
    if objective_id == WHOLE_BODY_OBJECTIVE and not should_cancel:
        end_states[objective_id] = _whole_body_end_state(execute_objective_resource)
    try:
        run_objective(
            objective_id,
            should_cancel,
            execute_objective_resource,
            expected_end_state_by_id=end_states,
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
