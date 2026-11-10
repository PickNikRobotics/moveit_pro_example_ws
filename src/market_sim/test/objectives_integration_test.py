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

import os
import time

import pytest
from controller_manager_msgs.srv import ListControllers

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

# config.yaml's controllers_active_at_startup. The agent heartbeat the fixture waits on does
# not cover the spawners, and this config declares enough controllers to outlast it.
STARTUP_CONTROLLERS = (
    "joint_state_broadcaster",
    "imu_sensor_broadcaster",
    "left_gripper_controller",
    "right_gripper_controller",
    "base_jgvc",
    "joint_trajectory_admittance_controller",
)
# The spawners load serially; measured well over a minute for this controller set.
CONTROLLERS_READY_TIMEOUT_S = 240.0


@pytest.fixture(scope="module", autouse=True)
def controllers_active(execute_objective_resource):
    """Block until every startup controller is active, so the first drive is not a race.

    Without it the first SwitchController fails with "is not an existing controller",
    because the spawners are still loading when the heartbeat reports the agent ready.
    """
    context = execute_objective_resource
    client = context.list_controllers_client()
    assert client.wait_for_service(
        timeout_sec=CONTROLLERS_READY_TIMEOUT_S
    ), "controller_manager/list_controllers never appeared"
    deadline = time.monotonic() + CONTROLLERS_READY_TIMEOUT_S
    missing = set(STARTUP_CONTROLLERS)
    while missing and time.monotonic() < deadline:
        listed = context.spin_until_future_complete(
            client.call_async(ListControllers.Request()), timeout_sec=10.0
        )
        if listed is not None:
            missing = set(STARTUP_CONTROLLERS) - {
                c.name for c in listed.controller if c.state == "active"
            }
        if missing:
            time.sleep(2.0)
    assert not missing, f"controllers never became active: {sorted(missing)}"


def reset_and_relocalize(context, keyframe_name, timeout_sec, settle_sec):
    """Reset the MuJoCo keyframe, then wait until AMCL is re-seeded at the true pose."""
    restored = MUJOCO_RESET_HOOK(context, keyframe_name, timeout_sec, settle_sec)
    if restored:
        time.sleep(RELOCALIZE_WAIT_S)
    return restored


SIM_RESETTER.register("market_sim", reset_and_relocalize)

# Budget for a Navigate to Aisle drive. The near and far rows are kept as separate names
# because the E rows sit at the far end of the store, but both are set to the same value.
NEAR_DRIVE_BUDGET_S = 300.0
FAR_DRIVE_BUDGET_S = 300.0
FAR_DRIVES = ("E1", "E5")
EXECUTE_TIMEOUT_OVERRIDES_S = {
    f"Navigate to Aisle {aisle}": NEAR_DRIVE_BUDGET_S for aisle in ("B2", "I2")
}
EXECUTE_TIMEOUT_OVERRIDES_S.update(
    {f"Navigate to Aisle {aisle}": FAR_DRIVE_BUDGET_S for aisle in FAR_DRIVES}
)
# About 25 m from the start pose, over 70 s at the 0.35 m/s speed limit.
EXECUTE_TIMEOUT_OVERRIDES_S["Navigate to Demo Dock D4"] = 180.0

# Looping objectives to cancel partway through rather than run to completion.
cancel_objectives: set[str] = set()

# Objectives to skip entirely, because the headless CI backend cannot run them; each entry
# gives its reason. Expand this set from the first CI run rather than guessing.
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
    # Manual-only: survey loops of roughly 400 m and 210 m that need a Runtime started with
    # MARKET_SIM_SLAM=true and MARKET_SIM_GROUND_TRUTH_NAV=true, which this launch never sets.
    "Build Market Map",
    "Finish Market Map Survey",
    # Manual-only: the demo backs blindly out of the D4 dock, so it must start docked, not at
    # the keyframe; its continuation starts partway along B2. Each drives over 90 m.
    "Demo D4 B2 South and Return",
    "Continue B2 South and Return D4",
}

# One drive per kind of row runs by default (B2, E5, I2), to stay inside the CTest timeout;
# the rest reach the same SubTree with other goals. Set MARKET_SIM_ALL_DRIVES to run all
# seven, which needs a longer timeout than CI gives this target.
OTHER_AISLE_DRIVES = {
    "Navigate to Aisle A1",
    "Navigate to Aisle C3",
    "Navigate to Aisle D4",
    "Navigate to Aisle E1",
}
if not os.environ.get("MARKET_SIM_ALL_DRIVES"):
    skip_objectives |= OTHER_AISLE_DRIVES
else:
    EXECUTE_TIMEOUT_OVERRIDES_S.update(
        {
            name: (
                FAR_DRIVE_BUDGET_S
                if name.rsplit(" ", 1)[-1] in FAR_DRIVES
                else NEAR_DRIVE_BUDGET_S
            )
            for name in OTHER_AISLE_DRIVES
        }
    )


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
