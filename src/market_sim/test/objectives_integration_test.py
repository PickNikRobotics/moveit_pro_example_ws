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

"""Integration tests for the market_sim objective library."""

import time

import pytest

from moveit_pro_test_utils.objective_test_fixture import (
    DEFAULT_OBJECTIVE_WAIT_S,
    ExecuteObjectiveResource,
    MUJOCO_RESET_HOOK,
    SIM_RESETTER,
    execute_objective_resource as execute_objective_resource,
    get_objective_pytest_params,
    reset_simulation_before_test as reset_simulation_before_test,
    run_objective,
)

# Longer than reset_relocalizer.py's last AMCL seed after a reset, with a margin for AMCL to
# take it; a drive started sooner plans from where the previous test left the base.
RELOCALIZE_WAIT_S = 7.0


def reset_and_relocalize(context, keyframe_name, timeout_sec, settle_sec):
    """Reset the MuJoCo keyframe, then wait until AMCL is re-seeded at the true pose."""
    restored = MUJOCO_RESET_HOOK(context, keyframe_name, timeout_sec, settle_sec)
    if restored:
        time.sleep(RELOCALIZE_WAIT_S)
    return restored


SIM_RESETTER.register("market_sim", reset_and_relocalize)

# Every Navigate to Aisle drive starts at the start pose; the far aisles take longer than the
# default budget at Nav2's speed limit.
EXECUTE_TIMEOUT_OVERRIDES_S = {
    f"Navigate to Aisle {aisle}": 360.0 for aisle in ("B2", "E5", "I2")
}

# Looping objectives to cancel partway through rather than run to completion.
cancel_objectives: set[str] = set()

# Objectives to skip entirely: they wait for input from the UI, which the headless CI
# backend cannot give. Expand this set from the first CI run rather than guessing.
skip_objectives: set[str] = {
    "Navigate with NavFn",  # Waits for a clicked goal.
    "Navigate with SMAC",  # Waits for a clicked goal.
    "Navigate to Clicked Point",  # Waits for a clicked goal.
    "Localize Robot",  # Waits for an interactive marker.
    "Request Teleoperation",  # Waits on UI teleoperation input.
    # Core-library objectives aggregated into every config; both need a
    # primary UI and cannot run in headless CI.
    "Teleoperate",  # DoTeleoperateAction rejects the goal with no UI subscribed.
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
    # One drive per kind of row runs (B2, E5, I2); these drive the same SubTree to other goals,
    # which test_market_scene.py and test_nav2_footprint.py check without a simulator.
    "Navigate to Aisle A1",
    "Navigate to Aisle C3",
    "Navigate to Aisle D4",
    "Navigate to Aisle E1",
}


@pytest.mark.parametrize(
    "objective_id, should_cancel",
    get_objective_pytest_params("market_sim", cancel_objectives, skip_objectives),
)
def test_all_objectives(
    objective_id: str,
    should_cancel: bool,
    execute_objective_resource: ExecuteObjectiveResource,
) -> None:
    """Run (or cancel) each market_sim objective and assert it completes without error."""
    try:
        run_objective(
            objective_id,
            should_cancel,
            execute_objective_resource,
            objective_wait_time=EXECUTE_TIMEOUT_OVERRIDES_S.get(
                objective_id, DEFAULT_OBJECTIVE_WAIT_S
            ),
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
