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

# Lidar scan filtering, fuse state estimation, and Nav2 for lunar_sim, launched with the persistent
# drivers (config.yaml additional_driver_launch_file).
#
# TF: mj_world -> map (static, at the spawn pose) -> odom (slam_toolbox with slam: true, beluga_amcl with
# localization: true, otherwise a static identity) -> footprint (fuse) -> base_link ... (robot_state_publisher).

import math
import os
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetLaunchConfiguration
)
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from moveit_studio_utils_py.launch_common import NodeWithAnsiLogging
from launch_ros.descriptions import ParameterFile
from moveit_studio_utils_py.system_config import SystemConfigParser
from nav2_common.launch import RewrittenYaml


def _urdf_param(name):
    """Return a config.yaml urdf_params value, so the map anchor follows the scene MuJoCo loads."""
    urdf_params = (
        SystemConfigParser().get_hardware_config().robot_description.urdf_params
    )
    for param in urdf_params:
        if name in param:
            return param[name]
    raise RuntimeError(f"config.yaml urdf_params has no '{name}'.")


def _launch_bool(value):
    """Return a ROS launch boolean string for a config.yaml value."""
    if not isinstance(value, bool):
        raise TypeError(f"Expected a boolean config.yaml value, got {value!r}.")
    return "true" if value else "false"


def _select_map_mode(context):
    """Validate both switches with ROS launch semantics before selecting one TF owner."""
    slam = IfCondition(LaunchConfiguration("slam")).evaluate(context)
    localization = IfCondition(LaunchConfiguration("localization")).evaluate(context)
    mode = "slam" if slam else "localization" if localization else "static"
    return [SetLaunchConfiguration("lunar_map_mode", mode)]


def _spawn_pose(scene_path, keyframe):
    """Return the chassis (x, y, yaw) in mj_world from the scene's keyframe qpos (freejoint first)."""
    key = ET.parse(scene_path).getroot().find(f"keyframe/key[@name='{keyframe}']")
    if key is None:
        raise RuntimeError(f"Keyframe '{keyframe}' not found in {scene_path}.")
    x, y, _, qw, qx, qy, qz = (float(v) for v in key.get("qpos").split()[:7])
    yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
    return x, y, yaw


def generate_launch_description():
    config_dir = get_package_share_directory("lunar_sim")
    launch_dir = os.path.join(config_dir, "launch", "sim")

    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    autostart = LaunchConfiguration("autostart")
    use_respawn = LaunchConfiguration("use_respawn")
    log_level = LaunchConfiguration("log_level")
    run_amcl = LaunchConfigurationEquals("lunar_map_mode", "localization")

    declare_args = [
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(config_dir, "params", "nav2_params.yaml"),
            description="Full path to the ROS 2 parameters file for every Nav2 node",
        ),
        DeclareLaunchArgument("autostart", default_value="true"),
        DeclareLaunchArgument("use_respawn", default_value="False"),
        DeclareLaunchArgument("log_level", default_value="info"),
        # Defaults come from config.yaml urdf_params, so the operator switches them there.
        DeclareLaunchArgument(
            "slam",
            default_value=_launch_bool(_urdf_param("slam")),
            description="Build a map with slam_toolbox. Takes precedence over localization.",
        ),
        DeclareLaunchArgument(
            "localization",
            default_value=_launch_bool(_urdf_param("localization")),
            description="Run beluga_amcl against the map. With neither this nor slam, map -> odom is a static identity and nothing corrects odometry drift.",
        ),
        DeclareLaunchArgument(
            "map",
            default_value=os.path.join(config_dir, "maps", "lunar_map.yaml"),
            description="Full path to the map yaml file to load",
        ),
    ]

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            param_rewrites={"use_sim_time": use_sim_time},
            convert_types=True,
        ),
        allow_substs=True,
    )

    # The containers take the whole params file: nodes a component creates internally, such as the
    # costmaps inside controller_server and planner_server, read their parameters from it.
    def container(name, condition=None):
        return NodeWithAnsiLogging(
            name=name,
            condition=condition,
            package="rclcpp_components",
            executable="component_container_isolated",
            parameters=[configured_params, {"autostart": autostart}],
            arguments=["--ros-args", "--log-level", log_level],
            remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        )

    # Localization runs in its own container so a crash in a navigation node cannot take map -> odom down with it.
    nav2_container = container("nav2_container")
    localization_container = container("localization_container", run_amcl)

    slam_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("nav2_bringup"), "launch", "slam_launch.py"
            )
        ),
        condition=LaunchConfigurationEquals("lunar_map_mode", "slam"),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "autostart": autostart,
            "use_respawn": use_respawn,
            "params_file": params_file,
        }.items(),
    )

    localization_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, "localization_launch.py")
        ),
        condition=run_amcl,
        launch_arguments={
            "map": LaunchConfiguration("map"),
            "use_sim_time": use_sim_time,
            "autostart": autostart,
            "params_file": params_file,
            "container_name": "localization_container",
        }.items(),
    )

    navigation_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(config_dir, "launch", "sim", "navigation_launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "autostart": autostart,
            "params_file": params_file,
            "container_name": "nav2_container",
        }.items(),
    )

    # Puts the map origin at the spawn pose, where odometry starts, so map poses match MuJoCo ground truth.
    spawn_x, spawn_y, spawn_yaw = _spawn_pose(
        os.path.join(config_dir, _urdf_param("mujoco_model")),
        _urdf_param("mujoco_keyframe"),
    )
    static_tf_world_to_map = NodeWithAnsiLogging(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_world_to_map",
        output="log",
        arguments=[
            "--x",
            str(spawn_x),
            "--y",
            str(spawn_y),
            "--yaw",
            str(spawn_yaw),
            "--frame-id",
            "mj_world",
            "--child-frame-id",
            "map",
        ],
    )

    # With neither SLAM nor AMCL nothing corrects odometry drift, so map -> odom is a static identity.
    static_tf_map_to_odom = NodeWithAnsiLogging(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_map_to_odom",
        output="log",
        arguments=["--frame-id", "map", "--child-frame-id", "odom"],
        condition=LaunchConfigurationEquals("lunar_map_mode", "static"),
    )

    laser_filter_params = os.path.join(config_dir, "params", "laser_filter_params.yaml")

    def laser_filter(side):
        return NodeWithAnsiLogging(
            package="laser_filters",
            executable="scan_to_scan_filter_chain",
            name=f"laser_filter_{side}",
            remappings=[
                ("scan", f"/scan_{side}"),
                ("scan_filtered", f"/scan_{side}_filtered"),
            ],
            parameters=[
                laser_filter_params,
                {"use_sim_time": use_sim_time},
                {"qos_overrides./scan.subscription.reliability": "best_effort"},
            ],
            output="log",
        )

    # Both filtered scans on one topic, a scan at a time, for beluga_amcl's single scan input.
    def scan_relay(side):
        return NodeWithAnsiLogging(
            condition=run_amcl,
            package="topic_tools",
            executable="relay",
            name=f"scan_{side}_relay",
            parameters=[
                {
                    "use_sim_time": use_sim_time,
                    "input_topic": f"/scan_{side}_filtered",
                    "output_topic": "/scan_interleaved",
                }
            ],
            output="log",
        )

    fuse_state_estimator = NodeWithAnsiLogging(
        package="fuse_optimizers",
        executable="fixed_lag_smoother_node",
        name="state_estimator",
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[
            os.path.join(config_dir, "config", "fuse", "fuse.yaml"),
            {"use_sim_time": use_sim_time},
        ],
    )

    return LaunchDescription(
        declare_args
        + [
            OpaqueFunction(function=_select_map_mode),
            nav2_container,
            localization_container,
            slam_cmd,
            localization_cmd,
            navigation_cmd,
            static_tf_world_to_map,
            static_tf_map_to_odom,
            laser_filter("front"),
            laser_filter("rear"),
            scan_relay("front"),
            scan_relay("rear"),
            fuse_state_estimator,
        ]
    )
