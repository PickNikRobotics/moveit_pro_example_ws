import time

import pytest
from moveit_pro_test_utils.objective_test_fixture import (
    ExecuteObjectiveResource,
    execute_objective_resource as execute_objective_resource,
    get_objective_pytest_params,
    run_objective,
)

cancel_objectives = {
    "3 Waypoints Pick and Place",
    "Cycle Between Waypoints",
    "Grasp Object from Text Prompt",
    "Grasp Planning",
    "Grasp Pose Tuning With April Tag",
    "Grasp Pose Using Yaml",
    "Joint Diagnostic",
    "Loop Detect AprilTag",
    "Pick And Place Example",
    "Pick April Tag Labeled Object With Approval",
    "Place Object",
    "Plan and Save Trajectory",
    "Record and Replay Scanning Motion",
}
skip_objectives = {
    "Grasp Planning",
    "Joint Diagnostic",
    "ML Segment Image from Text Prompt",
    "ML Segment Point Cloud from Clicked Point",
    "Pick up Cube",
    "Place Object",
    "Teleoperate",
}


@pytest.mark.parametrize(
    "objective_id, should_cancel",
    get_objective_pytest_params("lab_sim", cancel_objectives, skip_objectives),
)
def test_all_objectives(
    objective_id: str,
    should_cancel: bool,
    execute_objective_resource: ExecuteObjectiveResource,
):
    run_objective(objective_id, should_cancel, execute_objective_resource)
