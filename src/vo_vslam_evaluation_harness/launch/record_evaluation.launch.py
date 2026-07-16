# Copyright 2026 PickNik Inc.
# All rights reserved.
#
# Unauthorized copying of this code base via any medium is strictly prohibited.
# Proprietary and confidential.

"""Record the fixed VO/VSLAM evaluation topic contract."""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def _recording_topics() -> list[str]:
    """Load the single installed/source topic manifest."""
    manifest = Path(__file__).resolve().parents[1] / "config" / "rosbag_topics.txt"
    topics = [
        line.strip()
        for line in manifest.read_text(encoding="utf-8").splitlines()
        if line.strip() and not line.lstrip().startswith("#")
    ]
    if not topics:
        raise ValueError(f"rosbag topic manifest is empty: {manifest}")
    return topics


def generate_launch_description() -> LaunchDescription:
    """Create a rosbag recording process for the stable topic manifest."""
    output = LaunchConfiguration("output")
    return LaunchDescription(
        [
            DeclareLaunchArgument("output", default_value="vo_evaluation_bag"),
            ExecuteProcess(
                cmd=["ros2", "bag", "record", "-o", output, *_recording_topics()],
                output="both",
            ),
        ]
    )
