# Copyright 2023 PickNik Inc.
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

import os

import yaml
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    SetEnvironmentVariable,
    Shutdown,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
from launch_ros.descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml, ReplaceString


def _check_fuse_publish_odom(context, *args, **kwargs):
    """Fail fast if use_fuse:=true but MuJoCo is still configured to publish odom TF.

    When fuse is enabled it is the sole authority on odom → ridgeback_base_link.
    MuJoCo must not also publish that edge or consumers will see jittery, interleaved updates.
    Set 'publish_odom: "false"' in config.yaml under hardware.robot_description.urdf_params.
    """
    if LaunchConfiguration("use_fuse").perform(context).lower() != "true":
        return []

    config_path = os.path.join(
        get_package_share_directory("hangar_sim"), "config", "config.yaml"
    )
    with open(config_path) as f:
        robot_config = yaml.safe_load(f)

    urdf_params = (
        robot_config.get("hardware", {})
        .get("robot_description", {})
        .get("urdf_params", [])
    )
    publish_odom = True  # MuJoCo default
    for param in urdf_params:
        if isinstance(param, dict) and "publish_odom" in param:
            publish_odom = str(param["publish_odom"]).lower() not in (
                "false",
                "0",
                "no",
            )
            break

    if publish_odom:
        return [
            LogInfo(
                msg=(
                    "use_fuse:=true but MuJoCo is still configured to publish"
                    " odom -> ridgeback_base_link. When fuse is enabled, fuse must be the sole"
                    " publisher of that TF edge. Set 'publish_odom: false' in config.yaml under"
                    " hardware.robot_description.urdf_params, then rebuild hangar_sim."
                )
            ),
            Shutdown(reason="publish_odom must be false when use_fuse:=true"),
        ]
    return []


def generate_launch_description():
    # Get the relevant directories
    # The config directory (Contains the map and the parameters)
    config_dir = get_package_share_directory("hangar_sim")
    config_launch_dir = os.path.join(config_dir, "launch/sim")

    # The following is adapted from: https://github.com/ros-navigation/navigation2/blob/humble/nav2_bringup/launch/bringup_launch.py
    # Contains the launch files that we want to utilize for bringing up Nav2
    bringup_dir = get_package_share_directory("nav2_bringup")
    launch_dir = os.path.join(bringup_dir, "launch")

    # Create the launch configuration variables
    namespace = LaunchConfiguration("namespace")
    use_namespace = LaunchConfiguration("use_namespace")
    slam = LaunchConfiguration("slam")
    map_yaml_file = LaunchConfiguration("map")
    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    autostart = LaunchConfiguration("autostart")
    use_composition = LaunchConfiguration("use_composition")
    use_respawn = LaunchConfiguration("use_respawn")
    log_level = LaunchConfiguration("log_level")

    rviz_config_file = LaunchConfiguration("rviz_config_file")
    use_rviz = LaunchConfiguration("use_rviz")

    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO(orduno) Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56
    remappings = [
        ("/tf", "tf"),
        ("/tf_static", "tf_static"),
        ("/cmd_vel", "/platform_velocity_controller_nav2/cmd_vel_unstamped"),
    ]

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {"use_sim_time": use_sim_time, "yaml_filename": map_yaml_file}

    # Only it applies when `use_namespace` is True.
    # '<robot_namespace>' keyword shall be replaced by 'namespace' launch argument
    # in config file 'nav2_multirobot_params.yaml' as a default & example.
    # User defined config file should contain '<robot_namespace>' keyword for the replacements.
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

    declare_slam_cmd = DeclareLaunchArgument(
        "slam", default_value="False", description="Whether run a SLAM"
    )

    declare_map_yaml_cmd = DeclareLaunchArgument(
        "map",
        default_value=os.path.join(config_dir, "maps", "hangar_map.yaml"),
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

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        "rviz_config_file",
        default_value=os.path.join(bringup_dir, "rviz", "nav2_default_view.rviz"),
        description="Full path to the RVIZ config file to use",
    )

    declare_use_rviz_cmd = DeclareLaunchArgument(
        "use_rviz", default_value="True", description="Whether to start RVIZ"
    )

    declare_use_fuse_cmd = DeclareLaunchArgument(
        "use_fuse",
        default_value="false",
        description="Whether to launch the fuse state estimator",
    )

    # Specify the actions
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
                    os.path.join(launch_dir, "slam_launch.py")
                ),
                condition=IfCondition(slam),
                launch_arguments={
                    "namespace": namespace,
                    "use_sim_time": use_sim_time,
                    "autostart": autostart,
                    "use_respawn": use_respawn,
                    "params_file": params_file,
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(config_launch_dir, "localization_launch.py")
                ),
                condition=IfCondition(PythonExpression(["not ", slam])),
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

    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "rviz_launch.py")),
        condition=IfCondition(use_rviz),
        launch_arguments={
            "namespace": namespace,
            "use_namespace": use_namespace,
            "rviz_config": rviz_config_file,
        }.items(),
    )

    # Static TF between mj_world and map frame — anchors the nav2 map frame to the MuJoCo
    # simulation world.
    static_tf_world_to_map = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_world_to_map",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "mj_world", "map"],
    )

    # Static map→odom bootstrap: anchors 'odom' in the TF tree before MuJoCo finishes loading so
    # bt_navigator can initialize without an "unconnected trees" TF error. Not used in slam mode
    # because slam_toolbox owns and publishes map→odom dynamically.
    static_tf_map_to_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_map_to_odom",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "map", "odom"],
        condition=IfCondition(PythonExpression(["not ", slam])),
    )

    # Static TF connecting MoveIt's planning root ('world') to the simulation root ('mj_world').
    # The UI (pose-utils.ts) hardcodes 'world' as the reference frame for all user-clicked poses,
    # so this link is required for nav2 goals to be transformable to 'map'.
    static_tf_mj_world_to_world = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_mj_world_to_world",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "mj_world", "world"],
    )

    # QoS relay to bridge BEST_EFFORT odom and IMU to RELIABLE for fuse
    sensor_qos_relay = Node(
        package="hangar_sim",
        executable="odom_qos_relay.py",
        name="sensor_qos_relay",
        output="log",
    )

    hangar_sim_pkg = FindPackageShare("hangar_sim")

    # Angular bounds filter: clips chassis self-hitting beams (±93° to ±135°).
    # One filter instance per lidar; each publishes its filtered scan to
    # /scan_{front,rear}_filtered for Nav2 to consume as an independent
    # obstacle observation source.
    laser_filter_params = PathJoinSubstitution(
        [hangar_sim_pkg, "params", "laser_filter_params.yaml"]
    )

    laser_filter_front_node = Node(
        package="laser_filters",
        executable="scan_to_scan_filter_chain",
        name="laser_angular_filter_front",
        remappings=[
            ("scan", "/scan_front"),
            ("scan_filtered", "/scan_front_filtered"),
        ],
        parameters=[
            laser_filter_params,
            {"use_sim_time": use_sim_time},
            {"qos_overrides./scan.subscription.reliability": "best_effort"},
        ],
        output="log",
    )

    laser_filter_rear_node = Node(
        package="laser_filters",
        executable="scan_to_scan_filter_chain",
        name="laser_angular_filter_rear",
        remappings=[
            ("scan", "/scan_rear"),
            ("scan_filtered", "/scan_rear_filtered"),
        ],
        parameters=[
            laser_filter_params,
            {"use_sim_time": use_sim_time},
            {"qos_overrides./scan.subscription.reliability": "best_effort"},
        ],
        output="log",
    )

    # Fuse state estimator for mobile base localization
    fuse_state_estimator = Node(
        package="fuse_optimizers",
        executable="fixed_lag_smoother_node",
        name="state_estimator",
        parameters=[
            PathJoinSubstitution([hangar_sim_pkg, "config", "fuse", "fuse.yaml"])
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_fuse")),
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)

    # Declare the launch options — all args must be declared before the OpaqueFunction runs.
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(declare_use_fuse_cmd)
    ld.add_action(OpaqueFunction(function=_check_fuse_publish_odom))

    # Add the actions to launch all of the navigation nodes
    ld.add_action(bringup_cmd_group)

    # uncomment to bring up Nav2 RViz
    # ld.add_action(declare_rviz_config_file_cmd)
    # ld.add_action(declare_use_rviz_cmd)
    # ld.add_action(rviz_cmd)

    ld.add_action(static_tf_world_to_map)
    ld.add_action(static_tf_mj_world_to_world)
    ld.add_action(static_tf_map_to_odom)
    ld.add_action(sensor_qos_relay)
    ld.add_action(laser_filter_front_node)
    ld.add_action(laser_filter_rear_node)
    ld.add_action(fuse_state_estimator)

    return ld
