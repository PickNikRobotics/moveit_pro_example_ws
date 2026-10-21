#!/usr/bin/env python3
# Copyright 2026 PickNik Inc.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the PickNik Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
"""Launch the twin's compressed camera topics and the explicitly marked simulation grasp proxy, never camera drivers."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import yaml


def generate_launch_description():
    share = Path(get_package_share_directory("rebot_bench_sim"))
    layout = yaml.safe_load((share / "description/bench_layout.yaml").read_text())
    # The episode recorder's training config records the compressed topics.
    cameras = Node(
        package="rebot_bench_sim",
        executable="sim_compressed_cameras.py",
        output="screen",
    )
    # Inherit exactly the base YAML leader selection, without real camera drivers.
    leader = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(
                Path(get_package_share_directory("rebot_bench_base_config"))
                / "launch/rebot_leader.launch.py"
            )
        )
    )
    if not layout.get("sim_grasp_proxy", False):
        return LaunchDescription([cameras, leader])
    return LaunchDescription(
        [
            cameras,
            leader,
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "sim_grasp_proxy_controller",
                    "--param-file",
                    str(share / "config/sim_grasp_proxy.yaml"),
                ],
                output="screen",
            ),
            Node(
                package="rebot_bench_sim",
                executable="sim_grasp_proxy.py",
                output="screen",
            ),
        ]
    )
