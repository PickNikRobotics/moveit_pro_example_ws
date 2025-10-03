# Copyright 2022 PickNik Inc.
# All rights reserved.
#
# Unauthorized copying of this code base via any medium is strictly prohibited.
# Proprietary and confidential.


from launch import LaunchDescription

from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
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
                    "joint_names": '["fr3_finger_joint1","fr3_finger_joint2"]',
                }.items(),
            )
        ]
    )
