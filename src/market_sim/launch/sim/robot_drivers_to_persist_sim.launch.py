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

This is the config's ``additional_driver_launch_file``: frames, the self-filtered and
synchronized lidar scans, planar encoder/IMU fusion, the base velocity bridge, reset
synchronization, Beluga AMCL or slam_toolbox, and Nav2.

Frames, one publisher per edge:

    world -> map                 static identity
    world -> mj_world            static identity; the simulator hangs scene frames off mj_world
    map -> odom                  Beluga AMCL, or slam_toolbox in mapping mode
    odom -> ... -> base_link     robot_state_publisher, through planar_x, planar_y, planar_theta
    base_link -> lidar_*_ROS     static; the frames the flattened scans are stamped in

The odometry bridge publishes the planar joints from Fuse's /odom_filtered.
"""

import math
import os
import shutil

import yaml

from ament_index_python.packages import get_package_share_directory
from nav2_common.launch import RewrittenYaml
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.substitutions import (
    LaunchConfiguration,
    EnvironmentVariable,
    PythonExpression,
)
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from moveit_studio_utils_py.launch_common import NodeWithAnsiLogging as Node

# Nav2's own navigate-to-pose tree, and the copy of it this launch file writes with the Smac
# planner selected instead. BehaviorTree.CPP will not accept a relative tree path, so both are
# absolute and bt_navigator needs no particular working directory.
STOCK_NAV_TO_POSE_TREE = (
    "/opt/ros/jazzy/share/nav2_bt_navigator/behavior_trees/"
    "navigate_to_pose_w_replanning_and_recovery.xml"
)
SMAC_NAV_TO_POSE_TREE = "/tmp/market_sim/behavior_trees/navigate_to_pose_smac.xml"
NAV_THROUGH_POSES_TREE = "/tmp/market_sim/behavior_trees/navigate_through_poses.xml"
NAVFN_SELECTOR = 'default_planner="GridBased"'
SMAC_SELECTOR = 'default_planner="SmacLattice"'


def write_smac_nav_tree():
    """Write Nav2's stock navigate-to-pose tree with the Smac planner selected.

    Generated at launch rather than installed, so it cannot drift from the stock tree the
    installed Nav2 ships. The selector check fails loudly if that tree stops defaulting to NavFn.
    """
    with open(STOCK_NAV_TO_POSE_TREE) as stock:
        tree = stock.read()
    if tree.count(NAVFN_SELECTOR) != 1:
        raise RuntimeError(
            f"{STOCK_NAV_TO_POSE_TREE} no longer selects NavFn by default"
        )
    os.makedirs(os.path.dirname(SMAC_NAV_TO_POSE_TREE), exist_ok=True)
    with open(SMAC_NAV_TO_POSE_TREE, "w") as smac:
        smac.write(tree.replace(NAVFN_SELECTOR, SMAC_SELECTOR))


LOCALIZATION_NODES = ["map_server", "localization_map_server", "amcl"]
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


def configure_navigation_parameters(params_file, *, ground_truth=False):
    """Expand the shared footprint before ROS parses YAML; optionally isolate survey navigation."""
    rewrites = {
        "global_frame": "world",
        "local_frame": "world",
        "robot_base_frame": "base_link_ground_truth",
        "odom_topic": "/ground_truth/odom",
        "global_costmap.global_costmap.ros__parameters.static_layer.map_topic": "/navigation_map",
    }
    for costmap in ("local_costmap", "global_costmap"):
        for side in ("front", "rear"):
            rewrites[
                f"{costmap}.{costmap}.ros__parameters.obstacle_layer.scan_{side}.topic"
            ] = f"/navigation_scan_{side}"
    return RewrittenYaml(
        source_file=params_file,
        root_key="",
        param_rewrites=rewrites if ground_truth else {},
        convert_types=True,
    )


def require_mapping_for_truth_navigation(context):
    """Keep the optional truth-based survey mode out of ordinary AMCL navigation."""
    if not IfCondition(LaunchConfiguration("slam")).evaluate(context):
        raise RuntimeError(
            "MARKET_SIM_GROUND_TRUTH_NAV requires slam:=true for a mapping survey; set the flag false for lidar-localized navigation."
        )
    return []


def generate_launch_description():
    share = get_package_share_directory("market_sim")
    write_smac_nav_tree()
    shutil.copyfile(
        os.path.join(share, "params", "navigate_through_poses.xml"),
        NAV_THROUGH_POSES_TREE,
    )
    slam = LaunchConfiguration("slam")
    params_file = configure_navigation_parameters(LaunchConfiguration("params_file"))
    map_yaml = LaunchConfiguration("map")
    localization_map_yaml = LaunchConfiguration("localization_map")
    log_level = LaunchConfiguration("log_level")
    nav2_args = ["--ros-args", "--log-level", log_level]

    declared = [
        DeclareLaunchArgument(
            "slam",
            default_value=EnvironmentVariable("MARKET_SIM_SLAM", default_value="false"),
            description="Build a map with slam_toolbox instead of loading maps and AMCL.",
        ),
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
            "localization_map",
            default_value=os.path.join(share, "maps", "market_localization.yaml"),
            description="Map that AMCL localizes on: what the lidars see at their height.",
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

    # SICK nanoScan3 geometry from the Duo MJCF and the base configuration.
    lidar_flattener = Node(
        package="market_sim",
        executable="lidar_flattener.py",
        name="lidar_flattener",
        output="log",
        remappings=[
            ("/scan_front", "/scan_front_raw"),
            ("/scan_rear", "/scan_rear_raw"),
        ],
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
    # Exclude returns inside the stowed Duo envelope; navigation stows the arms before driving.
    with open(os.path.join(share, "params", "nav2_params.yaml")) as stream:
        navigation_params = yaml.safe_load(stream)
    footprint = yaml.safe_load(
        navigation_params["local_costmap"]["local_costmap"]["ros__parameters"][
            "footprint"
        ]
    )
    self_filter = {
        "filter1": {
            "name": "duo_self_returns",
            "type": "laser_filters/LaserScanBoxFilter",
            "params": {
                "box_frame": "base_link",
                "min_x": min(point[0] for point in footprint),
                "max_x": max(point[0] for point in footprint),
                "min_y": min(point[1] for point in footprint),
                "max_y": max(point[1] for point in footprint),
                "min_z": -1.0,
                "max_z": 2.0,
                "invert": False,
            },
        }
    }
    scan_filters = [
        Node(
            package="laser_filters",
            executable="scan_to_scan_filter_chain",
            name=f"{side}_self_filter",
            parameters=[self_filter],
            remappings=[
                ("scan", f"/scan_{side}_raw"),
                ("scan_filtered", f"/scan_{side}"),
            ],
        )
        for side in ("front", "rear")
    ]

    # AMCL reads one scan topic, so both scans are merged into one 360 deg /scan in base_link.
    # The lidar clouds arrive one render period after their stamp, so a merge cycle's newest
    # scans are up to two periods old; a tighter limit drops cycles and starves AMCL.
    scan_merger = Node(
        package="market_sim",
        executable="scan_merger.py",
        name="scan_merger",
        output="log",
        parameters=[{"max_scan_age_sec": 0.2, "merge_rate_hz": 20.0}],
    )

    # Reset synchronization holds Nav2 commands until Fuse, planar TF and (outside SLAM) AMCL agree.
    relocalizer = Node(
        package="market_sim",
        executable="reset_relocalizer.py",
        name="reset_relocalizer",
        parameters=[
            {
                "use_amcl": ParameterValue(
                    PythonExpression(["'", slam, "'.lower() not in ('true', '1')"]),
                    value_type=bool,
                )
            }
        ],
    )
    with open(
        os.path.join(share, "config", "control", "mobile_fr3_duo.ros2_control.yaml")
    ) as stream:
        base_limits = yaml.safe_load(stream)["base_jvc"]["ros__parameters"][
            "max_joint_velocity"
        ]
    # Convert body-frame commands to the simulation's planar actuators; fused pose alone reaches MoveIt.
    base_velocity = Node(
        package="market_sim",
        executable="base_twist_to_planar.py",
        name="base_twist_to_planar",
        parameters=[
            {
                "max_linear_velocity": min(base_limits[:2]),
                "max_angular_velocity": base_limits[2],
            }
        ],
    )
    odometry = Node(
        package="market_sim",
        executable="odometry_joint_state_publisher.py",
        name="odometry_joint_state_publisher",
    )

    state_estimator = Node(
        package="fuse_optimizers",
        executable="fixed_lag_smoother_node",
        name="state_estimator",
        parameters=[
            os.path.join(share, "config", "fuse", "fuse.yaml"),
            {"use_sim_time": False},
        ],
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
        # The costmaps keep the full /map; AMCL reads this one (amcl.map_topic in the params).
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="localization_map_server",
            output="screen",
            parameters=[
                {
                    "use_sim_time": False,
                    "yaml_filename": localization_map_yaml,
                    "topic_name": "localization_map",
                    "frame_id": "map",
                }
            ],
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

    # slam_toolbox publishes map -> odom and /map; map_saver serves the save service.
    mapping = GroupAction(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("slam_toolbox"),
                        "launch",
                        "online_sync_launch.py",
                    )
                ),
                launch_arguments={
                    "use_sim_time": "false",
                    "slam_params_file": params_file,
                    "autostart": "true",
                }.items(),
            ),
            Node(
                package="nav2_map_server",
                executable="map_saver_server",
                name="map_saver",
                output="screen",
                parameters=[params_file],
                arguments=nav2_args,
            ),
            lifecycle_manager("lifecycle_manager_slam", ["map_saver"]),
        ],
        condition=IfCondition(slam),
    )

    truth_navigation = os.environ.get(
        "MARKET_SIM_GROUND_TRUTH_NAV", "false"
    ).lower() in ("true", "1")
    navigation_parameters = (
        configure_navigation_parameters(params_file, ground_truth=True)
        if truth_navigation
        else params_file
    )
    truth_nodes = []
    if truth_navigation:
        declared.append(OpaqueFunction(function=require_mapping_for_truth_navigation))
        truth_nodes = [
            Node(
                package="market_sim",
                executable="navigation_scan_relay.py",
                name="navigation_scan_relay",
            ),
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="navigation_map_server",
                parameters=[
                    {
                        "use_sim_time": False,
                        "yaml_filename": map_yaml,
                        "topic_name": "navigation_map",
                        "frame_id": "world",
                    }
                ],
            ),
            lifecycle_manager(
                "lifecycle_manager_navigation_map", ["navigation_map_server"]
            ),
        ]

    def nav2_node(package, executable, remappings=()):
        return Node(
            package=package,
            executable=executable,
            name=executable,
            output="screen",
            parameters=[navigation_parameters],
            arguments=nav2_args,
            remappings=list(remappings),
        )

    # The controller feeds the smoother; both its output and recoveries publish body-frame /nav2_cmd_vel.
    to_smoother = [("cmd_vel", "cmd_vel_nav")]
    navigation = [
        nav2_node("nav2_controller", "controller_server", to_smoother),
        nav2_node("nav2_smoother", "smoother_server"),
        nav2_node("nav2_planner", "planner_server"),
        nav2_node("nav2_behaviors", "behavior_server", [("cmd_vel", "nav2_cmd_vel")]),
        nav2_node("nav2_bt_navigator", "bt_navigator"),
        nav2_node("nav2_waypoint_follower", "waypoint_follower"),
        nav2_node(
            "nav2_velocity_smoother",
            "velocity_smoother",
            to_smoother + [("cmd_vel_smoothed", "nav2_cmd_vel")],
        ),
        lifecycle_manager("lifecycle_manager_navigation", NAVIGATION_NODES),
    ]

    return LaunchDescription(
        declared
        + frames
        + [
            lidar_flattener,
            scan_merger,
            base_velocity,
            odometry,
            relocalizer,
            state_estimator,
        ]
        + scan_filters
        + [GroupAction(localization, condition=UnlessCondition(slam)), mapping]
        + truth_nodes
        + navigation
    )
