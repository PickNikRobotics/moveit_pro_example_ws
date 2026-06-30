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

import pytest

from moveit_pro_test_utils.objective_test_fixture import (
    EndStateSpec,
    ExecuteObjectiveResource,
    JointTarget,
    MUJOCO_RESET_HOOK,
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
    "Marker Visualization Example",  # GetTextFromUser server unavailable headless.
    "Writing Demo",  # Long-running drawing objective times out on the CI backend.
    "Find Green Block",  # ML text-prompt segmentation.
    "Find Red Block",  # ML text-prompt segmentation.
    "Sort Blocks",  # ML perception + multi-step grasp pipeline.
}

# End-state correctness checks beyond SUCCESS/FAILURE. "Move Left Home" and
# "Move Right Home" each drive one arm to the shared "Home" waypoint. The
# target is resolved at runtime from /get_saved_waypoints (the same source of
# truth the objective moves to), restricted to that arm's joint group so the
# uncommanded arm's joints are not compared.
expected_end_state_by_id = {
    "Move Left Home": EndStateSpec(
        joints=JointTarget(waypoint="Home", joint_group_name="left_manipulator")
    ),
    "Move Right Home": EndStateSpec(
        joints=JointTarget(waypoint="Home", joint_group_name="right_manipulator")
    ),
}


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
