# Copyright 2026 PickNik Inc.
# All rights reserved.
#
# Unauthorized copying of this code base via any medium is strictly prohibited.
# Proprietary and confidential.

"""Launch timestamped TartanAir stereo benchmark playback."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from moveit_studio_utils_py.launch_common import NodeWithAnsiLogging


def generate_launch_description() -> LaunchDescription:
    """Create the benchmark playback launch description."""
    return LaunchDescription(
        [
            DeclareLaunchArgument("dataset_root"),
            NodeWithAnsiLogging(
                package="vo_vslam_evaluation_harness",
                executable="tartanair_stereo_playback",
                name="tartanair_stereo_playback",
                parameters=[{"dataset_root": LaunchConfiguration("dataset_root")}],
            ),
        ]
    )
