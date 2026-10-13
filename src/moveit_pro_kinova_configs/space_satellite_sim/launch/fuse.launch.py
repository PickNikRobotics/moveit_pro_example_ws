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
from launch.actions import ExecuteProcess
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_dir = FindPackageShare("space_satellite_sim")

    return LaunchDescription(
        [
            # create a fixed transform between odom and world
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=["0", "0", "0", "0", "0", "0", "world", "odom"],
            ),
            # Set the rigid transforms between the AR tags and the target frame to be tracked. Each
            # camera's detector names its tags differently, so each gets its own copy of the offsets.
            *(
                Node(
                    package="tf2_ros",
                    executable="static_transform_publisher",
                    arguments=[
                        x,
                        y,
                        "0",
                        "0",
                        "0",
                        "0",
                        f"satellite_tag_{tag}{suffix}",
                        f"target_frame_satellite_tag_{tag}{suffix}",
                    ],
                )
                for suffix in ("", "_wrist")
                for tag, (x, y) in enumerate(
                    (("0", "-0.2"), ("0.2", "0.2"), ("-0.2", "0.2"))
                )
            ),
            # run our estimator
            Node(
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
