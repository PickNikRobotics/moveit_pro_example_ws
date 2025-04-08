import multiprocessing
import os
import signal
import time
from os import environ
from pathlib import Path
from typing import Optional

import pytest
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from launch import LaunchDescription
import launch
import launch_ros

from moveit_msgs.msg import MoveItErrorCodes
from moveit_studio_agent.objective_test_common import (
    generate_agent_launch_description,
    generate_robot_drivers_launch_description,
)
from moveit_studio_bridge.launch.launch_descriptions import (
    generate_studio_bridge_launch_description,
)
from moveit_studio_rest_api.routers.objective_api.objective_common import (
    parse_objective,
    parse_objective_xml,
)
from moveit_studio_sdk_msgs.action import DoObjectiveSequence
from moveit_studio_utils_py.system_config import (
    SystemConfigParser,
    get_behavior_folder,
    get_config_folder,
    get_config_package,
)


def get_runnable_config_objectives(package_name):
    environ["STUDIO_CONFIG_PACKAGE"] = package_name
    system_config_parser = SystemConfigParser()
    objective_xml_paths = get_all_objective_paths(system_config_parser)
    return [
        str(path) for path in objective_xml_paths if parse_objective(path).is_runnable
    ]


def get_all_objective_paths(system_config_parser: SystemConfigParser) -> list[Path]:
    objective_xml_paths = []
    objectives_dirs = system_config_parser.get_objective_library_paths()
    for objectives_dir in objectives_dirs:
        objective_xml_paths.extend(
            [objectives_dir / f.name for f in objectives_dir.glob("*.xml")]
        )

    return objective_xml_paths


def get_runnable_objective_xml(objective_xml_path: Path) -> Optional[str]:
    with open(objective_xml_path) as f:
        objective_xml = f.read()
    objective = parse_objective_xml(objective_xml)
    if objective.is_runnable:
        return objective_xml
    return None


def generate_rest_api_launch_description():
    return LaunchDescription(
        [
            launch_ros.actions.Node(
                package="moveit_studio_rest_api",
                executable="web_server_node",
                name="web_server_node",
                output="both",
            )
        ]
    )


class ActionData:
    _instance = None

    def __new__(cls, *args, **kwargs):
        if cls._instance is None:
            cls._instance = super().__new__(cls)
        return cls._instance

    def __init__(self):
        self.result = None
        self.goal_handle = None
        self.send_goal_future = None
        self.get_result_future = None
        self.cancel_future = None
        self.cancel_response = None

    def reset(self):
        self.result = None
        self.goal_handle = None
        self.send_goal_future = None
        self.get_result_future = None
        self.cancel_future = None


class DoObjectiveSequenceResource:
    def __init__(self, action_data: ActionData):
        self.node = Node("do_objective_node")
        self.do_objective_sequence_client = ActionClient(
            self.node, DoObjectiveSequence, "/do_objective"
        )
        self.action_data = action_data

    def wait_for_server(self):
        self.do_objective_sequence_client.wait_for_server()

    def send_goal(self, goal):
        self.action_data.reset()
        self.action_data.send_goal_future = (
            self.do_objective_sequence_client.send_goal_async(goal)
        )

    def cancel_goal(self):
        if self.action_data.goal_handle is not None:
            self.action_data.cancel_future = (
                self.action_data.goal_handle.cancel_goal_async()
            )

    def wait_for_cancel_response(self, timeout_sec):
        self.callback_on_future_complete(
            self.action_data.cancel_future,
            self._cancel_done_callback,
            timeout_sec=timeout_sec,
        )

    def wait_for_goal_response(self, timeout_sec):
        self.callback_on_future_complete(
            self.action_data.send_goal_future,
            self._goal_response_callback,
            timeout_sec=timeout_sec,
        )

    def wait_for_result(self, timeout_sec):
        self.callback_on_future_complete(
            self.action_data.get_result_future,
            self._get_result_callback,
            timeout_sec=timeout_sec,
        )

    def callback_on_future_complete(self, future, callback, timeout_sec=-1):
        current_time = self.node.get_clock().now()
        while not future.done():
            if (
                timeout_sec - (self.node.get_clock().now() - current_time).to_msg().sec
                == 0
            ):
                return
            rclpy.spin_once(self.node)

        callback(future)

    def _cancel_done_callback(self, future):
        self.action_data.cancel_response = future.result()

    def _get_result_callback(self, future):
        self.action_data.result = future.result().result

    def _goal_response_callback(self, future):
        self.action_data.goal_handle = future.result()
        if not self.action_data.goal_handle.accepted:
            return

        self.action_data.get_result_future = (
            self.action_data.goal_handle.get_result_async()
        )


@pytest.fixture(scope="module")
def do_objective_sequence_resource():
    all_launch_services_descriptions = [
        generate_robot_drivers_launch_description(),
        generate_agent_launch_description(),
        generate_rest_api_launch_description(),
        generate_studio_bridge_launch_description(),
    ]

    moveit_pro_launch_service = launch.LaunchService()
    moveit_pro_launch_description = LaunchDescription(all_launch_services_descriptions)
    moveit_pro_launch_service.include_launch_description(moveit_pro_launch_description)

    def run_launch_file():
        _ = moveit_pro_launch_service.run()

    p = multiprocessing.Process(target=run_launch_file)
    p.start()

    if not rclpy.ok():
        rclpy.init()

    action_data = ActionData()
    do_objective_sequence_resource = DoObjectiveSequenceResource(action_data)
    do_objective_sequence_resource.wait_for_server()

    accepted = False
    attempts = 0
    while not accepted and attempts < 30:
        goal = DoObjectiveSequence.Goal()
        goal.objective_name = "Clear Snapshot"
        do_objective_sequence_resource.send_goal(goal)
        do_objective_sequence_resource.wait_for_goal_response(5)
        accepted = action_data.goal_handle.accepted
        do_objective_sequence_resource.wait_for_result(5)
        time.sleep(5)
        attempts += 1

    assert action_data.result.error_code.val == MoveItErrorCodes.SUCCESS

    yield do_objective_sequence_resource
    os.kill(p.pid, signal.SIGINT)
    p.join()


@pytest.mark.parametrize(
    "objective_xml_path", get_runnable_config_objectives("lab_sim")
)
def test_objective_server(
    objective_xml_path, do_objective_sequence_resource: DoObjectiveSequenceResource
):
    cancel_objectives = {
        "Teleoperate",
        "Joint Diagnostic",
        "Segment Point Cloud from Clicked Point",
        "Place Object",
        "Loop Detect AprilTag",
        "Grasp Pose Tuning With April Tag",
        "Grasp Object from Text Prompt",
        "Grasp Planning",
        "3 Waypoints Pick and Place",
        "Pick And Place Example",
        "Record and Replay Scanning Motion",
        "Plan and Save Trajectory",
        "Cycle Between Waypoints",
        "Pick April Tag Labeled Object With Approval",
    }
    # We need to skip any objectives that use /switch_primary_view
    skip_objectives = {
        "Joint Diagnostic",
        "Pick up Cube",
        "Segment Image from Text Prompt",
        "Segment Point Cloud from Clicked Point",
        "Place Object",
        "Segment Point Cloud from Clicked Point",
        "Grasp Planning",
    }

    objective = parse_objective(objective_xml_path)
    if objective.id in skip_objectives:
        return

    # access the action client goal data
    action_data = do_objective_sequence_resource.action_data

    goal = DoObjectiveSequence.Goal()
    goal.objective_name = objective.id
    # send the action goal to run the objective
    do_objective_sequence_resource.send_goal(goal)

    # Wait a few seconds for the response. The response indicates whether the request was accepted or rejected.
    do_objective_sequence_resource.wait_for_goal_response(5)
    assert action_data.goal_handle.accepted

    if not objective.id in cancel_objectives:
        # Wait for the objective to finish
        do_objective_sequence_resource.wait_for_result(90)
        assert action_data.result.error_code.val == MoveItErrorCodes.SUCCESS
    else:
        time.sleep(10)
        do_objective_sequence_resource.cancel_goal()
        do_objective_sequence_resource.wait_for_cancel_response(5)
        ERROR_NONE = 0
        ERROR_UNKNOWN_GOAL_ID = 3

        if action_data.cancel_response.return_code == ERROR_NONE:
            do_objective_sequence_resource.wait_for_result(5)
            assert action_data.result.error_code.val == MoveItErrorCodes.PREEMPTED
        else:
            assert action_data.cancel_response.return_code == ERROR_UNKNOWN_GOAL_ID
