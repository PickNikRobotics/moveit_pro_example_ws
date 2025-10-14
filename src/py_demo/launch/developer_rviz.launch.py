# Copyright 2021 PickNik Inc.
# All rights reserved.
#
# Unauthorized copying of this code base via any medium is strictly prohibited.
# Proprietary and confidential.

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription

from moveit_studio_utils_py.launch_common import get_launch_file


def generate_launch_description():

    nodes_to_launch = [
        IncludeLaunchDescription(
            get_launch_file(
                "moveit_studio_agent",
                "launch/developer_rviz.launch.py",
            ),
        ),
        IncludeLaunchDescription(
            get_launch_file(
                "moveit_studio_agent",
                "launch/robot_drivers.launch.py",
            ),
        ),
        IncludeLaunchDescription(
            get_launch_file(
                "moveit_studio_agent",
                "launch/studio_agent.launch.py",
            ),
        ),
    ]

    
    return LaunchDescription(nodes_to_launch)
