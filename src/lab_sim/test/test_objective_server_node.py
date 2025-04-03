import time

import pytest
import os
import signal
import multiprocessing
from os import environ
from pathlib import Path

import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_studio_utils_py.system_config import (
    SystemConfigParser,
    get_behavior_folder,
    get_config_package,
    get_config_folder
)
from moveit_studio_rest_api.routers.objective_api.objective_common import (
    parse_objective,
    parse_objective_xml,
)

from moveit_studio_agent.objective_test_common import (
    generate_robot_drivers_launch_description, generate_agent_launch_description
)
from moveit_studio_bridge.launch.launch_descriptions import generate_studio_bridge_launch_description

from moveit_studio_sdk_msgs.action import DoObjectiveSequence
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from moveit_msgs.msg import MoveItErrorCodes


def get_runnable_config_objectives(package_name):
    environ["STUDIO_CONFIG_PACKAGE"] = package_name
    system_config_parser = SystemConfigParser()
    objective_xml_paths = get_all_objective_paths(system_config_parser)
    runnable_objectives_xml = get_runnable_objectives_xml(objective_xml_paths)
    return runnable_objectives_xml


def get_all_objective_paths(system_config_parser: SystemConfigParser) -> list[Path]:
    objective_xml_paths = []
    objectives_dirs = system_config_parser.get_objective_library_paths()
    for objectives_dir in objectives_dirs:
        objective_xml_paths.extend([objectives_dir / f.name for f in objectives_dir.glob("*.xml")])

    return objective_xml_paths


def get_runnable_objectives_xml(objective_xml_paths: list[Path]) -> list[str]:
    runnable_objectives_xml = []
    for objective_xml_path in objective_xml_paths:
        with open(objective_xml_path) as f:
            objective_xml = f.read()
        objective = parse_objective_xml(objective_xml)
        if objective.is_runnable:
            runnable_objectives_xml.append(objective_xml)

    return runnable_objectives_xml


def generate_rest_api_launch_description():
    return LaunchDescription(
        [
            Node(
                package="moveit_studio_rest_api",
                executable="web_server_node",
                name="web_server_node",
                output="both",
            )
        ]
    )


@pytest.fixture(scope="module")
def do_objective_sequence_fixture():
    print("Setup")
    all_launch_services_descriptions = [generate_robot_drivers_launch_description(),
                                        generate_agent_launch_description(), generate_rest_api_launch_description(),
                                        generate_studio_bridge_launch_description()
                                        ]

    moveit_pro_launch_service = launch.LaunchService()
    moveit_pro_launch_description = LaunchDescription(all_launch_services_descriptions)
    moveit_pro_launch_service.include_launch_description(moveit_pro_launch_description)

    def worker():
        _ = moveit_pro_launch_service.run()

    p = multiprocessing.Process(target=worker)
    p.start()

    if not rclpy.ok():
        rclpy.init()

    node = rclpy.node.Node("do_objective_node")
    do_objective_sequence_client = ActionClient(node, DoObjectiveSequence, "/do_objective")
    do_objective_sequence_client.wait_for_server()
    print("server ready")

    yield node, do_objective_sequence_client
    os.kill(p.pid, signal.SIGINT)
    p.join()
    print("Teardown")


@pytest.mark.parametrize("objective_xml", get_runnable_config_objectives("lab_sim"))
def test_objective_server(objective_xml, do_objective_sequence_fixture):
    """Tests the sum of the example data."""
    node, do_objective_sequence_client = do_objective_sequence_fixture
    objective = parse_objective_xml(objective_xml)

    goal = DoObjectiveSequence.Goal()
    goal.objective_name = objective.id

    goal_handle = do_objective_sequence_client.send_goal_async(goal)
    result_list = []

    def get_result_callback(future):
        result = future.result().result
        result_list.append(result.error_code == MoveItErrorCodes.SUCCESS)


    def goal_response_callback(future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            node.get_logger().info('Goal rejected :(')
            return

        node.get_logger().info('Goal accepted :)')

        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(get_result_callback)


    goal_handle.add_done_callback(goal_response_callback)

    time.sleep(5)
    rclpy.spin_until_future_complete(node, goal_handle, timeout_sec=10)
    del goal_handle

    assert len(objective_xml) > 0
