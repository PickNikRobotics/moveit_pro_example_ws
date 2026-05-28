# Copyright 2024 PickNik Inc.
# All rights reserved.
#
# Unauthorized copying of this code base via any medium is strictly prohibited.
# Proprietary and confidential.

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node

from moveit_studio_utils_py.launch_common import empty_gen
from moveit_studio_utils_py.system_config import (
    SystemConfigParser,
)


def generate_launch_description():
    system_config_parser = SystemConfigParser()
    controller_config = system_config_parser.get_ros2_control_config()

    protective_stop_manager_node = Node(
        package="moveit_studio_kinova_pstop_manager",
        executable="protective_stop_manager_node",
        name="protective_stop_manager_node",
        output="both",
        parameters=[
            {
                "controllers_default_active": controller_config.get(
                    "controllers_active_at_startup", empty_gen()
                ),
                "controllers_default_not_active": controller_config.get(
                    "controllers_inactive_at_startup", empty_gen()
                ),
            }
        ],
    )

    site_share = get_package_share_directory("kinova_gen3_site_config")

    wrist_camera_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(f"{site_share}/launch/cameras.launch.xml")
    )

    zed_camera_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(f"{site_share}/launch/zed_camera.launch.xml")
    )

    return LaunchDescription(
        [
            protective_stop_manager_node,
            wrist_camera_launch,
            zed_camera_launch,
        ]
    )
