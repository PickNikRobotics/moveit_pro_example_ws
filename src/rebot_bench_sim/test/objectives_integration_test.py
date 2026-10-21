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
"""Integration tests for the rebot_bench_sim objective library."""

import pytest

from moveit_pro_test_utils.objective_test_fixture import (
    ExecuteObjectiveResource,
    MUJOCO_RESET_HOOK,
    SIM_RESETTER,
    execute_objective_resource as execute_objective_resource,
    get_objective_pytest_params,
    reset_simulation_before_test as reset_simulation_before_test,
    run_objective,
)

# Reset the MuJoCo keyframe between tests.
SIM_RESETTER.register("rebot_bench_sim", MUJOCO_RESET_HOOK)

# Looping objectives to cancel partway through rather than run to completion.
cancel_objectives: set[str] = set()

# Objectives to skip entirely: they need a UI, an operator, or a ChArUco board
# the bench scene does not have.
skip_objectives: set[str] = {
    "Teleoperate",  # DoTeleoperateAction rejects the goal with no UI subscribed.
    "Marker Visualization Example",  # GetTextFromUser server unavailable headless.
    "Clear Snapshot",  # No octomap: the reBot configs define no 3D sensors.
    # Each test starts from the Zeroed Rest keyframe. With joint2's and
    # joint3's former 0 rad lower limits the plan's time-optimal profile dipped
    # below them and was rejected; not rechecked against the current limits.
    # Both run from Raised (README).
    "Plan reBot Joint Move",
    "Execute reBot Joint Trajectory",  # Executes what Plan reBot Joint Move saved.
    "Calibrate Camera Intrinsics",
    "Calibrate Eye In Hand Camera",
    "Calibrate Eye To Hand Camera",
    "Calibrate Multiple Cameras",
    "Generate Calibration Waypoints",
    # The pick-and-stack entry points segment the cubes with SAM3, which needs
    # the moveit_pro_sam3 model package the CI image does not ship; Clear reBot
    # Stack Scene removes the geometry only they add. Covered by regressions;
    # docs/stack_validation holds Stack Blue on Red twin runs, and the README
    # describes the unstack and cycle twin runs.
    "Locate reBot Cubes",
    "Prepare reBot Stack",
    "Pick Blue Cube",
    "Stack Blue on Red",
    "Unstack Blue from Red",  # Needs blue stacked on red.
    "Stack and Unstack Cycle",
    "Recorded Stack Leg",  # SAM3 perception plus a Trainer recording session.
    "Recorded Unstack Leg",
    "Recover Cube from Wall",  # SAM3 perception and a wall-side placement.
    "Clear reBot Stack Scene",
}


@pytest.mark.parametrize(
    "objective_id, should_cancel",
    get_objective_pytest_params("rebot_bench_sim", cancel_objectives, skip_objectives),
)
def test_all_objectives(
    objective_id: str,
    should_cancel: bool,
    execute_objective_resource: ExecuteObjectiveResource,
) -> None:
    """Run (or cancel) each rebot_bench_sim objective and assert it completes without error."""
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
