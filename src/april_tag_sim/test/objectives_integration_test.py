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

"""Integration tests for the april_tag_sim objective library."""

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

# april_tag_sim is a MuJoCo-backed sim: reset the keyframe between tests with
# the default reset hook (no controller cycling needed here — that variant is
# specific to lab_sim's interrupted-motion flake).
SIM_RESETTER.register("april_tag_sim", MUJOCO_RESET_HOOK)

# Looping objectives to cancel partway through rather than run to completion.
cancel_objectives: set[str] = set()

# Objectives to skip entirely. These need a camera operator, a clicked point,
# or an AprilTag-detection-driven grasp that the headless CI backend cannot
# satisfy. SUCCESS-or-skip keeps the suite deterministic; expand this set from
# the first weekly CI run rather than guessing more aggressively up front.
skip_objectives: set[str] = {
    "Collect AprilTag Detection Data",  # Data-collection loop driven by a camera feed.
    "Collect Angled AprilTag Detection Data",  # Data-collection loop driven by a camera feed.
    "Collect Parallel AprilTag Detection Data",  # Data-collection loop driven by a camera feed.
    "Pick April Tag Labeled Object",  # Requires AprilTag detection + a live grasp.
    # Core-library objectives aggregated into every config; both need a
    # primary UI and cannot run in headless CI.
    "Teleoperate",  # DoTeleoperateAction rejects the goal with no UI subscribed.
    "Marker Visualization Example",  # GetTextFromUser server unavailable headless.
}


@pytest.mark.parametrize(
    "objective_id, should_cancel",
    get_objective_pytest_params("april_tag_sim", cancel_objectives, skip_objectives),
)
def test_all_objectives(
    objective_id: str,
    should_cancel: bool,
    execute_objective_resource: ExecuteObjectiveResource,
) -> None:
    """Run (or cancel) each april_tag_sim objective and assert it completes without error."""
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
