# Copyright 2026 PickNik Inc.
# SPDX-License-Identifier: BSD-3-Clause
"""Explicitly connect a FashionStar leader using its existing calibrated plugin."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def bridge_node(context):
    arguments = [
        "--leader-python",
        LaunchConfiguration("leader_python").perform(context),
    ]
    if LaunchConfiguration("allow_real").perform(context) == "true":
        arguments.append("--allow-real")
    # The bridge ignores further stop signals while it returns HOME (up to 120 s);
    # launch's default SIGKILL after 10 s would cut that return short.
    return [
        Node(
            package="rebot_bench_base_config",
            executable="rebot_leader_bridge.py",
            name="rebot_leader_bridge",
            output="screen",
            arguments=arguments,
            sigterm_timeout="130",
            parameters=[
                {
                    "publish_rate": float(
                        LaunchConfiguration("publish_rate").perform(context)
                    )
                }
            ],
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "leader_python",
                description="Interpreter with the Seeed leader plugin installed",
            ),
            DeclareLaunchArgument(
                "publish_rate",
                default_value="100.0",
                description="Startup command/collision tick rate in Hz, 20 through 100",
            ),
            DeclareLaunchArgument(
                "allow_real",
                default_value="false",
                choices=["true", "false"],
                description="Requires separately approved physical commissioning",
            ),
            OpaqueFunction(function=bridge_node),
        ]
    )
