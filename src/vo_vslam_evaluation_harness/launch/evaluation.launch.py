# Copyright 2026 PickNik Inc.
# All rights reserved.
#
# Unauthorized copying of this code base via any medium is strictly prohibited.
# Proprietary and confidential.

"""Launch the candidate-agnostic VO evaluation node."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from moveit_studio_utils_py.launch_common import NodeWithAnsiLogging


def generate_launch_description() -> LaunchDescription:
    """Create the evaluation launch description."""
    config = LaunchConfiguration("config")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config",
                default_value=get_package_share_directory("vo_vslam_evaluation_harness")
                + "/config/harness.yaml",
            ),
            NodeWithAnsiLogging(
                package="vo_vslam_evaluation_harness",
                executable="vo_vslam_evaluation_node",
                name="vo_vslam_evaluation_harness",
                parameters=[config],
            ),
        ]
    )
