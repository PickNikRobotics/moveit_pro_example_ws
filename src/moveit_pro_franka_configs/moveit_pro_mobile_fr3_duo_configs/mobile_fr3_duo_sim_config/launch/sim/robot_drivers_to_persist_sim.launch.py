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

"""Sim-only driver persist launch for mobile_fr3_duo_sim_config's mobile_fr3_duo.

Spawns the nodes that must run alongside the MoveIt Pro agent in simulation:
    - sensor_qos_relay:     bridges /cmd_vel onto the mecanum controller's reference
                            topic and republishes /odom + /imu at RELIABLE QoS.
    - static TFs:           mj_world -> world -> map -> odom. The MuJoCo plugin
                            publishes mj_world -> base from the MJCF body hierarchy,
                            anchoring the URDF links to world. AMCL is not used.
    - Nav2 bringup:         composable container running map_server + the navigation
                            stack with parameters from params/nav2_params.yaml.

The odometry_joint_state_publisher script is intentionally not launched: the
picknik_mujoco_ros hardware plugin already exposes linear_x_joint /
linear_y_joint / rotational_yaw_joint state through joint_state_broadcaster.

Lidar-driven obstacle layers and fuse-based localization are intentionally
omitted until rangefinder sensors are added to the MuJoCo scene (see
hangar_sim for that wiring).
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml, ReplaceString


def generate_launch_description():
    config_dir = get_package_share_directory("mobile_fr3_duo_sim_config")
    config_launch_dir = os.path.join(config_dir, "launch/sim")

    namespace = LaunchConfiguration("namespace")
    use_namespace = LaunchConfiguration("use_namespace")
    map_yaml_file = LaunchConfiguration("map")
    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    autostart = LaunchConfiguration("autostart")
    use_composition = LaunchConfiguration("use_composition")
    use_respawn = LaunchConfiguration("use_respawn")
    log_level = LaunchConfiguration("log_level")

    # Nav2 publishes /cmd_vel from the velocity_smoother. The container-level remap
    # forwards it onto the body-frame mecanum controller (which has
    # body_frame_control: true) so the chassis interprets the twist in base_link.
    remappings = [
        ("/tf", "tf"),
        ("/tf_static", "tf_static"),
        ("/cmd_vel", "/platform_velocity_controller_nav2/cmd_vel_unstamped"),
    ]

    param_substitutions = {"use_sim_time": use_sim_time, "yaml_filename": map_yaml_file}

    params_file = ReplaceString(
        source_file=params_file,
        replacements={"<robot_namespace>": ("/", namespace)},
        condition=IfCondition(use_namespace),
    )

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True,
        ),
        allow_substs=True,
    )

    stdout_linebuf_envvar = SetEnvironmentVariable(
        "RCUTILS_LOGGING_BUFFERED_STREAM", "1"
    )

    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace", default_value="", description="Top-level namespace"
    )

    declare_use_namespace_cmd = DeclareLaunchArgument(
        "use_namespace",
        default_value="false",
        description="Whether to apply a namespace to the navigation stack",
    )

    declare_map_yaml_cmd = DeclareLaunchArgument(
        "map",
        default_value=os.path.join(config_dir, "maps", "mobile_fr3_duo_sim_config_map.yaml"),
        description="Full path to map yaml file to load",
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation clock if true",
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(config_dir, "params", "nav2_params.yaml"),
        description="Full path to the ROS2 parameters file to use for all launched nodes",
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        "autostart",
        default_value="true",
        description="Automatically startup the nav2 stack",
    )

    declare_use_composition_cmd = DeclareLaunchArgument(
        "use_composition",
        default_value="True",
        description="Whether to use composed bringup",
    )

    declare_use_respawn_cmd = DeclareLaunchArgument(
        "use_respawn",
        default_value="False",
        description="Whether to respawn if a node crashes. Applied when composition is disabled.",
    )

    declare_log_level_cmd = DeclareLaunchArgument(
        "log_level", default_value="info", description="log level"
    )

    bringup_cmd_group = GroupAction(
        [
            PushRosNamespace(condition=IfCondition(use_namespace), namespace=namespace),
            Node(
                condition=IfCondition(use_composition),
                name="nav2_container",
                package="rclcpp_components",
                executable="component_container_isolated",
                parameters=[configured_params, {"autostart": autostart}],
                arguments=["--ros-args", "--log-level", log_level],
                remappings=remappings,
                output="screen",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(config_launch_dir, "localization_launch.py")
                ),
                launch_arguments={
                    "namespace": namespace,
                    "map": map_yaml_file,
                    "use_sim_time": use_sim_time,
                    "autostart": autostart,
                    "params_file": params_file,
                    "use_composition": use_composition,
                    "use_respawn": use_respawn,
                    "container_name": "nav2_container",
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(config_launch_dir, "navigation_launch.py")
                ),
                launch_arguments={
                    "namespace": namespace,
                    "use_sim_time": use_sim_time,
                    "autostart": autostart,
                    "params_file": params_file,
                    "use_composition": use_composition,
                    "use_respawn": use_respawn,
                    "container_name": "nav2_container",
                }.items(),
            ),
        ]
    )

    sensor_qos_relay = Node(
        package="mobile_fr3_duo_sim_config",
        executable="odom_qos_relay.py",
        name="sensor_qos_relay",
        output="log",
    )

    # The MuJoCo plugin publishes mj_world as its TF root (representing the MJCF
    # worldbody) and also broadcasts mj_world -> base from the MJCF body hierarchy.
    # Anchoring mj_world -> world makes the full chain:
    #   mj_world -> world -> map -> odom  (static frames, identity)
    #   mj_world -> base                  (from MuJoCo plugin, encodes planar position)
    #   base -> base_link                 (from robot_state_publisher via URDF)
    static_tf_mj_world_to_world = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_mj_world_to_world",
        output="log",
        arguments=["0", "0", "0", "0", "0", "0", "mj_world", "world"],
    )

    static_tf_world_to_map = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_world_to_map",
        output="log",
        arguments=["0", "0", "0", "0", "0", "0", "world", "map"],
    )

    static_tf_map_to_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_map_to_odom",
        output="log",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
    )

    ld = LaunchDescription()

    ld.add_action(stdout_linebuf_envvar)

    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)

    ld.add_action(bringup_cmd_group)

    ld.add_action(sensor_qos_relay)
    ld.add_action(static_tf_mj_world_to_world)
    ld.add_action(static_tf_world_to_map)
    ld.add_action(static_tf_map_to_odom)

    return ld
