#  Copyright (c) 2025 Franka Robotics GmbH
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.

############################################################################
# Parameters:
# controller_name: Name of the controller to spawn (required, no default)
# robot_config_file: Path to the robot configuration file to load
#                   (default: franka.config.yaml in franka_bringup/config)
#
# The example.launch.py launch file provides a flexible and unified interface
# for launching Franka Robotics example controllers via the 'controller_name'
# parameter, such as 'elbow_example_controller'.
# Example:
# ros2 launch franka_bringup example.launch.py controller_name:=elbow_example_controller
#
# This script "includes" franka.launch.py to declare core component nodes,
# including: robot_state_publisher, ros2_control_node, joint_state_publisher,
# joint_state_broadcaster, franka_robot_state_broadcaster, and optionally
# franka_gripper and rviz, with support for namespaced and non-namespaced
# environments as defined in franka.config.yaml. RViz is launched if
# 'use_rviz' is set to true in the configuration file.
#
# The default robot_config_file is franka.config.yaml in the
# franka_bringup/config directory. See that file for its own documentation.
#
# This approach improves upon the earlier individual launch scripts, which
# varied in structure and lacked namespace support, offering a more consistent
# and maintainable solution. While some may favor the older scripts for their
# specific use cases, example.launch.py enhances scalability and ease of use
# for a wide range of Franka Robotics applications.
#
# Ensure the specified  controller_name matches a controller defined in
#  controllers.yaml to avoid runtime errors.
############################################################################


import os
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# Add the path to the `utils` folder
package_share = get_package_share_directory("franka_bringup")
utils_path = os.path.join(package_share, "..", "..", "lib", "franka_bringup", "utils")
sys.path.append(os.path.abspath(utils_path))

# Iterates over the uncommented lines in file specified by the robot_config_file parameter.
# "Includes" franka.launch.py for each active (uncommented) Robot.
# That file is well documented.
# The function also checks if the 'use_rviz' parameter is set to true in the YAML file.
# If so, it includes a node for RViz to visualize the robot's state.
# The function returns a list of nodes to be launched.


def generate_robot_nodes(context):
    nodes = []
    # Left arm
    nodes.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [
                        FindPackageShare("franka_dual_arm_config_hw"),
                        "launch",
                        "left_franka.launch.py",
                    ]
                )
            ),
            launch_arguments={
                "arm_id": "",
                "arm_prefix": "left",
                "namespace": "left",
                "urdf_file": "fr3/fr3.urdf.xacro",
                "robot_ip": "172.16.0.4",
                "load_gripper": "true",
                "use_fake_hardware": "false",
                "fake_sensor_commands": "false",
                "joint_sources": "joint_state_broadcaster, left_velocity_force_controller",
                "joint_state_rate": "30",
            }.items(),
        )
    )
    # Right arm
    nodes.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [
                        FindPackageShare("franka_dual_arm_config_hw"),
                        "launch",
                        "right_franka.launch.py",
                    ]
                )
            ),
            launch_arguments={
                "arm_id": "",
                "arm_prefix": "right",
                "namespace": "right",
                "urdf_file": "fr3/fr3.urdf.xacro",
                "robot_ip": "172.16.0.5",
                "load_gripper": "true",
                "use_fake_hardware": "false",
                "fake_sensor_commands": "false",
                "joint_sources": "joint_states, franka_gripper/joint_states",
                "joint_state_rate": "30",
            }.items(),
        )
    )
    nodes.append(
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            name="joint_state_publisher",
            parameters=[
                {
                    "source_list": [
                        "right/joint_states",
                        "right/franka_gripper/joint_states",
                        "left/joint_states",
                        "left/franka_gripper/joint_states",
                    ],
                    "rate": 50,
                    "use_robot_description": False,
                }
            ],
            output="screen",
        ),
    )
    return nodes


# The generate_launch_description function is the entry point (like "main")
# It is called by the ROS 2 launch system when the launch file is executed.
# via: ros2 launch franka_bringup example.launch.py ARGS...
# This function must return a LaunchDescription object containing nodes to be launched.
# it calls the generate_robot_nodes function to get the list of nodes to be launched.


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "robot_config_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("franka_dual_arm_config_hw"),
                        "config/control",
                        "franka.config.yaml",
                    ]
                ),
                description="Path to the robot configuration file to load",
            ),
            OpaqueFunction(function=generate_robot_nodes),
        ]
    )
