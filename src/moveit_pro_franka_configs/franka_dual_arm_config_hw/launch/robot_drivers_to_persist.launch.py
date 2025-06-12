# Copyright 2022 PickNik Inc.
# All rights reserved.
#
# Unauthorized copying of this code base via any medium is strictly prohibited.
# Proprietary and confidential.


from launch import LaunchDescription
from launch_ros.actions import Node

from moveit_studio_utils_py.launch_common import empty_gen
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from moveit_studio_utils_py.system_config import (
    SystemConfigParser,
)


def generate_launch_description():
    system_config_parser = SystemConfigParser()
    hw_config = system_config_parser.get_hardware_config()

    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        PathJoinSubstitution(
                            [
                                FindPackageShare("franka_gripper"),
                                "launch",
                                "gripper.launch.py",
                            ]
                        )
                    ]
                ),
                launch_arguments={
                    "robot_ip": "192.168.19.21",
                    "joint_names": '["right_fr3_finger_joint1","right_fr3_finger_joint2"]',
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        PathJoinSubstitution(
                            [
                                FindPackageShare("franka_gripper"),
                                "launch",
                                "gripper.launch.py",
                            ]
                        )
                    ]
                ),
                launch_arguments={
                    "robot_ip": "192.168.19.22",
                    "joint_names": '["left_fr3_finger_joint1","left_fr3_finger_joint2"]',
                }.items(),
            ),
        ]
    )
