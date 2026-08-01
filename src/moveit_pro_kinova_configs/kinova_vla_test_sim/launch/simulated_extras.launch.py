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
            # Republishes the trajectory controller's commanded setpoint (plus the
            # latched gripper command) as /joint_commands, which the Trainer records
            # and Forge maps to the training `action`. Without it a Behavior-Tree
            # oracle records no command stream and conversion falls back to
            # next-state action labels.
            Node(
                package="kinova_vla_test_sim",
                executable="joint_command_bridge.py",
                name="joint_command_bridge",
                output="log",
            ),
        ]
    )
