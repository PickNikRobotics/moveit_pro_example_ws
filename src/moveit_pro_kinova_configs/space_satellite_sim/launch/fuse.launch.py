#! /usr/bin/env python3

# Copyright 2024 PickNik Robotics
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from moveit_studio_utils_py.launch_common import NodeWithAnsiLogging


TAG_OFFSETS = (("0", "-0.2"), ("0.2", "0.2"), ("-0.2", "0.2"))
TAG_SUFFIXES = ("", "_wrist")


def generate_launch_description():
    pkg_dir = FindPackageShare("space_satellite_sim")

    return LaunchDescription(
        [
            # Create a fixed transform between odom and world.
            NodeWithAnsiLogging(
                package="tf2_ros",
                executable="static_transform_publisher",
                output="log",
                arguments=[
                    "--x",
                    "0",
                    "--y",
                    "0",
                    "--z",
                    "0",
                    "--roll",
                    "0",
                    "--pitch",
                    "0",
                    "--yaw",
                    "0",
                    "--frame-id",
                    "world",
                    "--child-frame-id",
                    "odom",
                ],
            ),
            # Set the rigid transforms between the AR tags and the target frame to be tracked. Each
            # camera's detector names its tags differently, so each gets its own copy of the offsets.
            *(
                NodeWithAnsiLogging(
                    package="tf2_ros",
                    executable="static_transform_publisher",
                    output="log",
                    arguments=[
                        "--x",
                        x,
                        "--y",
                        y,
                        "--z",
                        "0",
                        "--roll",
                        "0",
                        "--pitch",
                        "0",
                        "--yaw",
                        "0",
                        "--frame-id",
                        f"satellite_tag_{tag}{suffix}",
                        "--child-frame-id",
                        f"target_frame_satellite_tag_{tag}{suffix}",
                    ],
                )
                for suffix in TAG_SUFFIXES
                for tag, (x, y) in enumerate(TAG_OFFSETS)
            ),
            # run our estimator
            NodeWithAnsiLogging(
                package="fuse_optimizers",
                executable="fixed_lag_smoother_node",
                name="state_estimator",
                parameters=[
                    PathJoinSubstitution([pkg_dir, "config", "fuse", "fuse.yaml"])
                ],
            ),
            # run visualization
            # Node(
            #     package="rviz2",
            #     executable="rviz2",
            #     name="rviz",
            #     arguments=[
            #         "-d",
            #         [PathJoinSubstitution([pkg_dir, "config", "fuse", "fuse.rviz"])],
            #         "--ros-args",
            #         "--log-level",
            #         "warn",
            #     ],
            # ),
        ]
    )
