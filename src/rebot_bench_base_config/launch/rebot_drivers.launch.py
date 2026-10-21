# Copyright 2026 PickNik Inc.
# SPDX-License-Identifier: BSD-3-Clause
"""Persistent backend owns optional cameras, USB leader and thermal recovery."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    launch = Path(get_package_share_directory("rebot_bench_base_config")) / "launch"
    return LaunchDescription(
        [
            IncludeLaunchDescription(PythonLaunchDescriptionSource(str(launch / name)))
            for name in ("cameras.launch.py", "rebot_leader.launch.py")
        ]
    )
