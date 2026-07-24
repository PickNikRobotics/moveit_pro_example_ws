#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            # /get_action_chunk bridge to the host inference server (docker/serve_policy.py);
            # ExecutePolicy calls this service, so it must be up wherever the Agent runs.
            Node(
                package="kinova_vla_test_sim",
                executable="get_action_chunk_adapter.py",
                name="get_action_chunk_adapter",
                output="log",
            ),
        ]
    )
