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

"""Market AMCL/Nav2 with lunar_sim's A300 wheel odometry, lidars, and camera."""

import math
import os
import shutil

import yaml

from ament_index_python.packages import get_package_share_directory
from nav2_common.launch import RewrittenYaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from moveit_studio_utils_py.launch_common import NodeWithAnsiLogging as Node

# Navigate with SMAC's tree: Nav2's stock tree with the Smac planner as the selector's default.
# Nav2 opens a behavior_tree_path in this container, so it is written here at a fixed path.
STOCK_NAV_TO_POSE_TREE = (
    "/opt/ros/jazzy/share/nav2_bt_navigator/behavior_trees/"
    "navigate_to_pose_w_replanning_and_recovery.xml"
)
SMAC_NAV_TO_POSE_TREE = "/tmp/market_sim/behavior_trees/navigate_to_pose_smac.xml"
NAV_THROUGH_POSES_TREE = "/tmp/market_sim/behavior_trees/navigate_through_poses.xml"
NAVFN_SELECTOR = 'default_planner="GridBased"'
SMAC_SELECTOR = 'default_planner="SmacLattice"'

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


def ground_truth_navigation_parameters(params_file):
    rewrites = {
        "global_frame": "world",
        "local_frame": "world",
        "robot_base_frame": "chassis_link_ground_truth",
        "odom_topic": "/ground_truth/odom",
        "global_costmap.global_costmap.ros__parameters.static_layer.map_topic": "/navigation_map",
    }
    for costmap in ("local_costmap", "global_costmap"):
        for side in ("front", "rear"):
            rewrites[
                f"{costmap}.{costmap}.ros__parameters.obstacle_layer.scan_{side}.topic"
            ] = f"/navigation_scan_{side}"
    navigation_parameters = RewrittenYaml(
        source_file=params_file,
        root_key="",
        param_rewrites=rewrites,
        convert_types=True,
    )
    return navigation_parameters


def generate_launch_description():
    share = get_package_share_directory("market_sim")
    with open(STOCK_NAV_TO_POSE_TREE) as stock:
        tree = stock.read()
    if tree.count(NAVFN_SELECTOR) != 1:
        raise RuntimeError(
            f"{STOCK_NAV_TO_POSE_TREE} no longer selects NavFn by default"
        )
    os.makedirs(os.path.dirname(SMAC_NAV_TO_POSE_TREE), exist_ok=True)
    with open(SMAC_NAV_TO_POSE_TREE, "w") as smac:
        smac.write(tree.replace(NAVFN_SELECTOR, SMAC_SELECTOR))
    shutil.copyfile(
        os.path.join(share, "params", "navigate_through_poses.xml"),
        NAV_THROUGH_POSES_TREE,
    )
    slam = LaunchConfiguration("slam")
    params_file = LaunchConfiguration("params_file")
    map_yaml = LaunchConfiguration("map")
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
            x=0.45,
            z=0.3208593,
            yaw=math.radians(-135.0),
        ),
        static_transform(
            "base_link",
            "lidar_rear_ROS",
            x=-0.2728,
            z=0.6500193,
            yaw=math.radians(45.0),
        ),
    ]

    # The A300 lidar MJCF declares 270 degrees and 25 m range.
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
                "angle_max": math.radians(270.0),
                "trim_low_deg": 0.0,
                "trim_high_deg": 0.0,
                "range_min": 0.10,
                "range_max": 25.0,
                "range_noise_stddev": 0.003,
                "range_quantum": 0.001,
            }
        ],
    )
    # Reuse the navigation footprint as the self-return exclusion box. The A300
    # lidars see their own enclosure and arch; these returns are not map obstacles.
    with open(os.path.join(share, "params", "nav2_params.yaml")) as stream:
        navigation_params = yaml.safe_load(stream)
    footprint = yaml.safe_load(
        navigation_params["local_costmap"]["local_costmap"]["ros__parameters"][
            "footprint"
        ]
    )
    self_filter = {
        "filter1": {
            "name": "a300_self_returns",
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

    # A keyframe reset teleports the base; AMCL cannot follow, so re-seed it at the true pose.
    relocalizer = Node(
        package="market_sim",
        executable="reset_relocalizer.py",
        name="reset_relocalizer",
        output="log",
        condition=UnlessCondition(slam),
    )
    # Nav2 publishes Twist; the A300 diff-drive controller consumes TwistStamped.
    base_velocity = Node(
        package="market_sim",
        executable="stamp_nav_velocity.py",
        name="stamp_nav_velocity",
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
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="localization_map_server",
            output="screen",
            parameters=[
                {
                    "use_sim_time": False,
                    "yaml_filename": os.path.join(share, "maps", "localization.yaml"),
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
            remappings=[("map", "localization_map")],
            parameters=[params_file],
            arguments=nav2_args,
        ),
        lifecycle_manager("lifecycle_manager_localization", LOCALIZATION_NODES),
    ]

    mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("nav2_bringup"), "launch", "slam_launch.py"
            )
        ),
        condition=IfCondition(slam),
        launch_arguments={
            "use_sim_time": "false",
            "params_file": params_file,
            "autostart": "true",
        }.items(),
    )

    navigation_parameters = params_file
    truth_nodes = []
    if os.environ.get("MARKET_SIM_GROUND_TRUTH_NAV", "false").lower() == "true":
        navigation_parameters = ground_truth_navigation_parameters(params_file)
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

    # The controller feeds the smoother, whose output is /cmd_vel; recoveries publish /cmd_vel directly.
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
        + [lidar_flattener, scan_merger, base_velocity, relocalizer, state_estimator]
        + scan_filters
        + [GroupAction(localization, condition=UnlessCondition(slam)), mapping]
        + truth_nodes
        + navigation
    )
