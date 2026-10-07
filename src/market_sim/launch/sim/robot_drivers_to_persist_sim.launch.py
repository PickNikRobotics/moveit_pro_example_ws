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

"""Processes that stay up for the whole MoveIt Pro session in the market simulation.

This is the config's ``additional_driver_launch_file``: frames, the lidar scans, the
odometry bridge, Beluga AMCL and Nav2. The scan, odometry and velocity nodes are
mobile_fr3_duo_sim's; this file replaces only its map, localization and Nav2 setup.

Frames, one publisher per edge:

    world -> map                 static identity
    world -> mj_world            static identity; the simulator hangs scene frames off mj_world
    map -> odom                  Beluga AMCL, the localization correction
    odom -> ... -> base_link     robot_state_publisher, through planar_x, planar_y, planar_theta
    base_link -> lidar_*_ROS     static; the frames the flattened scans are stamped in

The description is rooted at ``odom``. The odometry bridge is the only publisher of the
three planar joints: the simulator's true base pose plus drift, so AMCL has a real
correction to make. The simulator publishes no base TF and no planar joint states
(joint_state_broadcaster's ``joints`` list leaves them out).
"""

import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

LOCALIZATION_NODES = ["map_server", "amcl"]
NAVIGATION_NODES = [
    "controller_server",
    "smoother_server",
    "planner_server",
    "behavior_server",
    "bt_navigator",
    "waypoint_follower",
    "velocity_smoother",
]


def static_transform(parent, child, x=0.0, y=0.0, z=0.0, yaw=0.0):
    return Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name=f"static_tf_{parent}_to_{child}",
        output="log",
        arguments=[
            "--x",
            str(x),
            "--y",
            str(y),
            "--z",
            str(z),
            "--yaw",
            str(yaw),
            "--frame-id",
            parent,
            "--child-frame-id",
            child,
        ],
    )


def lifecycle_manager(name, nodes):
    return Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name=name,
        output="screen",
        parameters=[{"use_sim_time": False, "autostart": True, "node_names": nodes}],
    )


def generate_launch_description():
    share = get_package_share_directory("market_sim")
    params_file = LaunchConfiguration("params_file")
    map_yaml = LaunchConfiguration("map")
    log_level = LaunchConfiguration("log_level")
    nav2_args = ["--ros-args", "--log-level", log_level]

    declared = [
        DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(share, "params", "nav2_params.yaml"),
            description="Parameters for Beluga AMCL and every Nav2 node.",
        ),
        DeclareLaunchArgument(
            "map",
            default_value=os.path.join(share, "maps", "market.yaml"),
            description="Map that map_server serves; generated with the scene.",
        ),
        DeclareLaunchArgument(
            "log_level", default_value="info", description="Nav2 and AMCL log level."
        ),
    ]

    frames = [
        static_transform("world", "map"),
        static_transform("world", "mj_world"),
        # Scan frames: z up, X at beam 0 of each sweep, at the lidar mounting points.
        static_transform(
            "base_link",
            "lidar_front_ROS",
            x=0.3275,
            y=0.2175,
            z=0.19065,
            yaw=math.radians(-92.5),
        ),
        static_transform(
            "base_link",
            "lidar_rear_ROS",
            x=-0.3275,
            y=-0.2175,
            z=0.19065,
            yaw=math.radians(87.5),
        ),
    ]

    # SICK nanoScan3 values, measured on the real scanner.
    lidar_flattener = Node(
        package="mobile_fr3_duo_sim",
        executable="lidar_flattener.py",
        name="lidar_flattener",
        output="log",
        parameters=[
            {
                "angle_min": 0.0,
                "angle_max": math.radians(275.0),
                "trim_low_deg": 6.0,
                "trim_high_deg": 6.0,
                "range_min": 0.10,
                "range_max": 40.0,
                "range_noise_stddev": 0.003,
                "range_quantum": 0.001,
            }
        ],
    )
    # AMCL reads one scan topic, so both scans are merged into one 360 deg /scan in base_link.
    # The lidar clouds arrive one render period after their stamp, so a merge cycle's newest
    # scans are up to two periods old; a tighter limit drops cycles and starves AMCL.
    scan_merger = Node(
        package="mobile_fr3_duo_sim",
        executable="scan_merger.py",
        name="scan_merger",
        output="log",
        parameters=[{"max_scan_age_sec": 0.35}],
    )

    odometry = Node(
        package="mobile_fr3_duo_sim",
        executable="odometry_joint_state_publisher.py",
        name="odometry_joint_state_publisher",
        output="log",
    )
    # A keyframe reset teleports the base; AMCL cannot follow, so re-seed it at the true pose.
    relocalizer = Node(
        package="market_sim",
        executable="reset_relocalizer.py",
        name="reset_relocalizer",
        output="log",
    )
    base_velocity = Node(
        package="mobile_fr3_duo_sim",
        executable="base_twist_to_planar.py",
        name="base_twist_to_planar",
        output="log",
    )

    localization = [
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            output="screen",
            parameters=[params_file, {"yaml_filename": map_yaml}],
            arguments=nav2_args,
        ),
        Node(
            package="beluga_amcl",
            executable="amcl_node",
            name="amcl",
            output="screen",
            parameters=[params_file],
            arguments=nav2_args,
        ),
        lifecycle_manager("lifecycle_manager_localization", LOCALIZATION_NODES),
    ]

    def nav2_node(package, executable, remappings=()):
        return Node(
            package=package,
            executable=executable,
            name=executable,
            output="screen",
            parameters=[params_file],
            arguments=nav2_args,
            remappings=list(remappings),
        )

    # The controller feeds the smoother, whose output is /cmd_vel; recoveries publish /cmd_vel directly.
    to_smoother = [("cmd_vel", "cmd_vel_nav")]
    navigation = [
        nav2_node("nav2_controller", "controller_server", to_smoother),
        nav2_node("nav2_smoother", "smoother_server"),
        nav2_node("nav2_planner", "planner_server"),
        nav2_node("nav2_behaviors", "behavior_server"),
        nav2_node("nav2_bt_navigator", "bt_navigator"),
        nav2_node("nav2_waypoint_follower", "waypoint_follower"),
        nav2_node(
            "nav2_velocity_smoother",
            "velocity_smoother",
            to_smoother + [("cmd_vel_smoothed", "cmd_vel")],
        ),
        lifecycle_manager("lifecycle_manager_navigation", NAVIGATION_NODES),
    ]

    return LaunchDescription(
        declared
        + frames
        + [lidar_flattener, scan_merger, odometry, base_velocity, relocalizer]
        + localization
        + navigation
    )
