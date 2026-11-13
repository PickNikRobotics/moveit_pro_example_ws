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

"""Frames, the odometry bridge and Nav2 for dual_arm_sim.

world -> map and map -> odom are fixed identities: the odometry is exact, so there is
nothing for AMCL to correct. The planar joints (odom -> base_link) come from the bridge.
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def static_identity(parent, child):
    return Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name=f"static_tf_{parent}_to_{child}",
        arguments=["--frame-id", parent, "--child-frame-id", child],
        output="log",
    )


def generate_launch_description():
    share = FindPackageShare("dual_arm_sim")
    params_file = PathJoinSubstitution([share, "params", "nav2_params.yaml"])

    # The only publisher of the planar joints and of /odom, from the simulator's true pose.
    odometry = Node(
        package="dual_arm_sim",
        executable="odometry_joint_state_publisher.py",
        name="odometry_joint_state_publisher",
        parameters=[{"noise_enabled": False}],
        output="log",
    )
    # Nav2's /cmd_vel (base frame) becomes world-axis planar joint velocities for base_jgvc.
    base_twist_to_planar = Node(
        package="dual_arm_sim",
        executable="base_twist_to_planar.py",
        name="base_twist_to_planar",
        parameters=[{"ground_truth_topic": "/ground_truth/odom"}],
        output="log",
    )
    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        parameters=[
            params_file,
            {"yaml_filename": PathJoinSubstitution([share, "maps", "scene.yaml"])},
        ],
        output="log",
    )
    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        parameters=[{"autostart": True, "node_names": ["map_server"]}],
        output="log",
    )
    # Nav2's planner, controller, behavior trees and the rest, ending at /cmd_vel.
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("nav2_bringup"), "launch", "navigation_launch.py"]
            )
        ),
        launch_arguments={
            "params_file": params_file,
            # The MoveIt Pro hardware plugins stamp wall time.
            "use_sim_time": "false",
            "autostart": "true",
        }.items(),
    )
    return LaunchDescription(
        [
            static_identity("world", "map"),
            static_identity("map", "odom"),
            # Anchors the simulator's own frames, such as the scene camera.
            static_identity("world", "mj_world"),
            odometry,
            base_twist_to_planar,
            map_server,
            lifecycle_manager,
            navigation,
        ]
    )
