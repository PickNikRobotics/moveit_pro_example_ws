# Copyright 2026 PickNik Inc.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the PickNik Inc. nor the names of its contributors
#      may be used to endorse or promote products derived from this software
#      without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DAMAGES ARISING IN ANY WAY
# OUT OF THE USE OF THIS SOFTWARE.

"""Launch the licensed, vendor-neutral collision_monitor node for lab_sim.

This brings up ONLY the monitor node (and its component container), configured
by ``config/lab_sim_collision_monitor.yaml``. It is meant to run *in parallel*
with the lab_sim MoveIt Pro backend (robot drivers + agent + bridge), which is
what supplies the topics this node consumes:

  * /robot_description           (URDF, transient_local)
  * /robot_description_semantic  (SRDF for the ACM, transient_local)
  * /joint_states                (live joint positions/velocities)

The ``collision_monitor_component`` plugin is provided by the pre-bundled,
license-gated monitor overlay/image — it is NOT built from this workspace. If
that overlay is absent the container will fail to load the component and the
launch fails loudly. With the overlay present, the monitor's engine constructor
still requires a valid MOVEIT_LICENSE_KEY; without one it throws and the node
dies (the intended IP/licensing gate).

Outputs:
  * <node>/trip   (std_msgs/String) one message per trip with reason + contacts
  * <node>/status (std_msgs/Bool)   latched engine state every tick
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    params_file = LaunchConfiguration("params_file")
    monitor_namespace = LaunchConfiguration("monitor_namespace")

    declared_args = [
        DeclareLaunchArgument(
            "params_file",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("lab_sim_collision_monitor"),
                    "config",
                    "lab_sim_collision_monitor.yaml",
                ]
            ),
            description="Parameters for the collision_monitor node.",
        ),
        DeclareLaunchArgument(
            "monitor_namespace",
            default_value="collision_monitor",
            description="Namespace for the monitor node. Trip/status topics are "
            "published under <namespace>/collision_monitor/{trip,status}.",
        ),
    ]

    monitor_container = ComposableNodeContainer(
        name="collision_monitor_container",
        namespace=monitor_namespace,
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                # Provided by the licensed, pre-bundled monitor overlay/image.
                package="collision_monitor_ros",
                plugin="collision_monitor_ros::CollisionMonitorComponent",
                name="collision_monitor",
                namespace=monitor_namespace,
                parameters=[params_file],
                # Robot description/semantic are published on absolute topics by
                # the MoveIt Pro backend; the component subscribes to absolute
                # /robot_description(_semantic) and /joint_states already.
            )
        ],
        output="screen",
    )

    return LaunchDescription([*declared_args, monitor_container])
