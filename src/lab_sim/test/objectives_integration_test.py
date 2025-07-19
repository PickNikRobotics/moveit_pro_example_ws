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

import pytest
import rclpy
from rclpy.action import ActionClient
from control_msgs.action import GripperCommand
from moveit_pro_test_utils.objective_test_fixture import (
    ExecuteObjectiveResource,
    execute_objective_resource as execute_objective_resource,
    get_objective_pytest_params,
    run_objective,
)

cancel_objectives = {
    "3 Waypoints Pick and Place",
    "Cycle Between Waypoints",
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
    "Classical Pick and Place",
    "ML Auto Grasp Object from Text Prompt",
}

skip_objectives = {
    "Grasp Object from Text Prompt",  # https://github.com/PickNikRobotics/moveit_pro/issues/13236
    "Grasp Planning",
    "Joint Diagnostic",
    "Fuse Multiple Views",
    "MPC Pose Tracking",
    "MPC Pose Tracking With Point Cloud Avoidance",
    "ML Segment Image from Text Prompt",
    "ML Segment Point Cloud from Clicked Point",
    "ML Auto Grasp Object from Clicked Point",  # Skipped because there is no primary ui to switch to in ci
    "Pick up Cube",
    "Place Object",
    "Record Square Trajectory",
    "Teleoperate",
}

# This is a workaround to avoid the test running before ros control is ready
def wait_for_gripper_action_server(timeout: int = 180):
    node = rclpy.create_node("test_node")
    action_client = ActionClient(
        node, GripperCommand, "/robotiq_gripper_controller/gripper_cmd"
    )
    if not action_client.wait_for_server(timeout_sec=timeout):
        raise TimeoutError(
            "Timeout waiting for robotiq_gripper_controller gripper action server to start"
        )
    return True


@pytest.mark.parametrize(
    "objective_id, should_cancel",
    get_objective_pytest_params("lab_sim", cancel_objectives, skip_objectives),
)
def test_all_objectives(
    objective_id: str,
    should_cancel: bool,
    execute_objective_resource: ExecuteObjectiveResource,
):
    wait_for_gripper_action_server()
    run_objective(objective_id, should_cancel, execute_objective_resource)
