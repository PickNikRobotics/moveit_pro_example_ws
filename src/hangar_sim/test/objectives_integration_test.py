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

import pytest
from moveit_pro_test_utils.objective_test_fixture import (
    ExecuteObjectiveResource,
    MUJOCO_RESET_HOOK,
    SIM_RESETTER,
    execute_objective_resource as execute_objective_resource,
    get_objective_pytest_params,
    reset_simulation_before_test as reset_simulation_before_test,
    run_objective,
    wait_for_action_servers,
)

# hangar_sim is MuJoCo-backed but is not in the harness's default reset
# registry (only lab_sim and core_integration_config are). Register the
# standard MuJoCo keyframe-reset hook at import time so the autouse
# reset_simulation_before_test fixture resets the "default" keyframe between
# tests instead of silently no-op'ing and letting state leak across tests.
SIM_RESETTER.register("hangar_sim", MUJOCO_RESET_HOOK)

# Looping objectives to cancel partway through.
cancel_objectives = {
    "Move Boxes Looping",
    "Plan Path Along Surface - Loop",
}

# Objectives to skip entirely from integration testing — either they need a
# live operator interaction with no headless equivalent, or they can't run
# at-or-under realtime on the CI runner within the per-objective timeout.
skip_objectives = {
    # --- Need live operator interaction (no headless equivalent in CI) ---
    # GetPoseFromUser ("click a goal in the 3D Visualization pane") + nav2.
    "Navigate to Clicked Point",
    "Navigate to Clicked Point with Replanning",
    # Reads pixel_coords from an unset blackboard key populated by a UI click.
    "Segment Image from Point",
    "Segment Point Cloud from Clicked Point",
    # DoTeleoperateAction rejects the goal when no UI tab is subscribed to
    # /moveit_pro_ui/do_teleoperate/goal (mirrors lab_sim skipping "Teleoperate").
    "Teleoperate",
    # GetTextFromUser server is not available headless (mirrors lab_sim skipping
    # "Marker Visualization Example").
    "Marker Visualization Example",
    # --- Need ML models not present in CI ---
    # Needs models/clip.onnx + models/clipseg.onnx from the moveit_pro_clipseg
    # submodule, which is not checked out in CI (mirrors lab_sim skipping its
    # "ML Segment *" objectives).
    "Segment Image from Text Prompt",
    # Needs ML perception models not available in CI; hits the 90 s per-objective
    # timeout waiting on inference.
    "ML Move Boxes to Loading Zone",
    # --- Exceed the 90 s per-objective timeout on the CI runner ---
    # Long whole-body cartesian/spray-coverage motions over the airplane that
    # don't return within 90 s on the non-realtime CI runner (each consumed the
    # full per-objective timeout in CI, driving the suite past its CTest budget).
    "Cartesian Path with Collision Checking",
    "Find and Spray Plane",
    "Solution - Find and Spray Plane",
}


# ros2_control's gripper action must be live before any gripper objective is
# runnable. Query it through the fixture's long-lived node instead of spinning
# up a throwaway node + ActionClient per test, which would flood DDS discovery
# on hangar_sim's already-busy graph (nav2 / slam_toolbox / fuse).
VACUUM_GRIPPER_ACTION = "/vacuum_gripper/gripper_cmd"


@pytest.mark.parametrize(
    "objective_id, should_cancel",
    get_objective_pytest_params("hangar_sim", cancel_objectives, skip_objectives),
)
def test_all_objectives(
    objective_id: str,
    should_cancel: bool,
    execute_objective_resource: ExecuteObjectiveResource,
):
    wait_for_action_servers(
        execute_objective_resource.node, [VACUUM_GRIPPER_ACTION], timeout_sec=180
    )
    try:
        run_objective(objective_id, should_cancel, execute_objective_resource)
    except AssertionError as e:
        mode = "cancel" if should_cancel else "execute"
        pytest.fail(f"Objective '{objective_id}' failed to {mode}: {e}")
    except Exception as e:
        mode = "cancel" if should_cancel else "execute"
        pytest.fail(
            f"Objective '{objective_id}' hit an unexpected error during "
            f"{mode}: {type(e).__name__}: {e}"
        )
