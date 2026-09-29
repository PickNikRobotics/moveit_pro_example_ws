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

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
import launch.logging
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition, UnlessCondition
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
from moveit_studio_utils_py.system_config import SystemConfigParser
from nav2_common.launch import RewrittenYaml, ReplaceString


def _urdf_param(name, default):
    """Read a urdf_params value from config.yaml.

    The forward stereo cameras are gated by a xacro arg, which only config.yaml can set, so
    this launch file reads the same key rather than declaring a second switch of its own. Two
    switches would let the cameras render with no publisher -- the exact state this removes --
    and nothing would report the mismatch.
    """
    urdf_params = (
        SystemConfigParser().get_hardware_config().robot_description.urdf_params
    )
    for param in urdf_params:
        if name in param:
            return param[name]
    return default


def _warn_unsupported_localization(context, *args, **kwargs):
    """Warn on the one unsupported point in the slam x use_fuse x localization cube.

    Ownership of map -> odom, and of odom -> world, across the three flags:

      slam  use_fuse  localization | map -> odom      odom -> world       verdict
      ------------------------------------------------------------------------------
      true  *         *            | slam_toolbox     static / drift      supported
      false false     true         | beluga AMCL      static identity     supported (no drift
                                   |                                      to correct: odom == world)
      false false     false        | static identity  static identity     supported (pure truth)
      false true      true         | beluga AMCL      odom_world_drift    supported -- the target
      false true      false        | static identity  odom_world_drift    UNSUPPORTED

    The last row is the hole: odom_world_drift injects fuse's estimate error into odom -> base and
    nothing is left to correct it, so the robot's pose in the map free-runs away from the truth
    while every node reports healthy. launch has no LogWarn action, so evaluate the combination
    here and log at WARNING via an OpaqueFunction -- LogInfo would blend into normal launch output
    and defeat the point.

    (PR #790 warned on a different row, use_fuse:=false + localization:=true, because its
    amcl_odom_gate was the sole map -> odom publisher and only launched with fuse. There is no gate
    here and amcl.tf_broadcast stays true, so beluga always publishes its own correction and that
    row is fine.)
    """
    if (
        LaunchConfiguration("slam").perform(context).lower() == "false"
        and LaunchConfiguration("use_fuse").perform(context).lower() == "true"
        and LaunchConfiguration("localization").perform(context).lower() == "false"
    ):
        launch.logging.get_logger("hangar_sim").warning(
            "UNSUPPORTED: use_fuse:=true with localization:=false and slam:=false -- "
            "odom_world_drift makes odom -> ridgeback_base_link resolve to fuse's estimate, but "
            "map -> odom is a static identity, so nothing corrects the estimate's drift and the "
            "robot's pose in the map free-runs. Use localization:=true (default) to let "
            "beluga_amcl correct it, slam:=true to let slam_toolbox correct it, or use_fuse:=false "
            "for the intentional ground-truth fallback."
        )
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
    localization = LaunchConfiguration("localization")
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

    # Create our own temporary YAML files that include substitutions.
    # odom_topic is rewritten in navigation_launch.py instead: configured_params here reaches only
    # the component containers, which do not consume it.
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

    declare_localization_cmd = DeclareLaunchArgument(
        "localization",
        default_value="True",
        description="Run beluga_amcl for map-based localization. Set False to use a static map->odom TF instead.",
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
        default_value="true",
        description=(
            "Launch the fuse state estimator and the odom_world_drift publisher, so navigation "
            "runs on the estimated base pose. Set false to fall back to the simulator's ground "
            "truth (odom -> world becomes a static identity)."
        ),
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
            # Localization (map_server, AMCL) runs in its own container so a
            # crash in any navigation node cannot take localization down with it. The
            # map -> odom transform AMCL publishes is what connects the MuJoCo scene
            # frames (cameras under mj_world -> map) to MoveIt's planning frame
            # (world under odom); if this container dies, every point-cloud-to-world
            # transform in the product breaks, not just navigation.
            Node(
                condition=IfCondition(use_composition),
                name="localization_container",
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
                    "container_name": "localization_container",
                    "localization": localization,
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
                    "use_fuse": LaunchConfiguration("use_fuse"),
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

    # Static map->odom TF fallback: only used when neither SLAM nor AMCL is publishing it.
    # Unaffected by use_fuse: amcl.tf_broadcast stays true, so beluga always publishes its correction.
    static_tf_map_to_odom = Node(
        condition=IfCondition(
            PythonExpression(
                [
                    "'",
                    slam,
                    "'.lower() == 'false' and '",
                    localization,
                    "'.lower() == 'false'",
                ]
            )
        ),
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_map_to_odom",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "map", "odom"],
    )

    # Static TF anchoring MoveIt's planning root ('world') under the odometry frame.
    # robot_state_publisher owns the only live chain into ridgeback_base_link
    # (world -> virtual_rail_... -> ridgeback_base_link), so 'odom' must sit above
    # 'world' for REP-105 semantics: AMCL's live map->odom correction then shifts the
    # whole robot subtree, and its own odom->base lookups resolve through this link.
    # (Previously this was mj_world->world, and the 'odom' frame only existed because
    # MuJoCo broadcast a competing odom->ridgeback_base_link TF — removed in this
    # change.) The UI (pose-utils.ts) hardcodes 'world' for user-clicked poses, so
    # this link also keeps nav2 goals transformable to 'map'.
    #
    # Identity while fuse is off; odom_world_drift owns this edge when fuse is on.
    static_tf_odom_to_world = Node(
        condition=UnlessCondition(LaunchConfiguration("use_fuse")),
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_odom_to_world",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "odom", "world"],
    )

    # Publishes odom -> world as the difference between fuse's estimate and MuJoCo truth.
    odom_world_drift = Node(
        condition=IfCondition(LaunchConfiguration("use_fuse")),
        package="hangar_sim",
        executable="odom_world_drift",
        name="odom_world_drift",
        # "both": the stale-estimate warning is operator-facing; log-only hides it in the container.
        output="both",
        respawn=LaunchConfiguration("use_respawn"),
        respawn_delay=2.0,
        parameters=[{"use_sim_time": use_sim_time}],
    )

    hangar_sim_pkg = FindPackageShare("hangar_sim")

    # Pairs and rectifies the two MuJoCo renders. Started only when the cameras exist:
    # enable_vo gates both, so the publisher never waits on topics nobody publishes.
    # TODO(#22640): when the stereo VO stack lands (#21469), include its launch file here
    # under this same flag so one switch brings up the cameras, this publisher and VO.
    enable_vo = _urdf_param("enable_vo", False)

    forward_stereo_publisher = Node(
        package="hangar_sim",
        executable="forward_stereo_publisher.py",
        name="forward_stereo_publisher",
        parameters=[
            PathJoinSubstitution([hangar_sim_pkg, "params", "forward_stereo.yaml"]),
            {"use_sim_time": use_sim_time},
        ],
        output="log",
    )

    # The two TIM571s are DEPTH_TYPE=THREE_D_LIDAR cameras in the MJCF, so
    # picknik_mujoco_ros publishes them as organized PointCloud2 on
    # /lidar_{front,rear}/points and never as a LaserScan. This node reads the
    # elevation-0 row of each cloud and republishes it as the /scan_{front,rear}
    # LaserScan the filter chains below, AMCL, slam_toolbox and both costmap
    # obstacle layers already consume. Topic names, message types, frames and the
    # 0 to 270 deg angular window are all unchanged from the <rangefinder> path,
    # so nothing downstream needed touching.
    lidar_flattener = Node(
        package="hangar_sim",
        executable="lidar_flattener.py",
        name="lidar_flattener",
        parameters=[{"use_sim_time": use_sim_time}],
        output="log",
    )

    # The frame each flattened scan is published in: z-up, X at the scan's angle_min
    # (beam 0), which is what params/laser_filter_params.yaml computes its self-hit
    # arcs against. MuJoCo used to broadcast these itself, as a by-product of the
    # <rangefinder> path; a camera gets no such frame, so they are published here.
    #
    # Static, and parented straight to ridgeback_base_link rather than to the
    # lidar_*_mount bodies, for two reasons. The mounts are fixed in the MJCF, so
    # the offsets below are exact and cannot drift. And the plugin only puts those
    # mount bodies on /tf at tf_publish_rate, arriving before every link carries its
    # real pose: a scan frame resolved through that chain can latch a wrong mount for
    # the rest of the session and publish beams that keep their ranges but land at
    # wrong bearings, which reads as a map that will not close rather than as a
    # sensor fault. Values are read out of description/ur5e_ridgeback.xml:
    # lidar_front_mount at (+0.45, 0, 0.15) with the fan centered on +X, so beam 0
    # sits at -135 deg; lidar_rear_mount at (-0.45, 0, 0.15) with the fan centered on
    # -X, so beam 0 sits at +45 deg.
    static_tf_lidar_front_ros = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_lidar_front_ros",
        output="log",
        arguments=[
            "0.45",
            "0.0",
            "0.15",
            "-2.3561945",
            "0.0",
            "0.0",
            "ridgeback_base_link",
            "lidar_front_ROS",
        ],
    )

    static_tf_lidar_rear_ros = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_lidar_rear_ros",
        output="log",
        arguments=[
            "-0.45",
            "0.0",
            "0.15",
            "0.7853982",
            "0.0",
            "0.0",
            "ridgeback_base_link",
            "lidar_rear_ROS",
        ],
    )

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

    # Both filtered scans on one topic, a scan at a time: relayed rather than
    # merged, so no message can carry two instants. Launched here rather than with
    # AMCL so /scan_interleaved also exists when slam:=true.
    scan_front_relay = Node(
        package="topic_tools",
        executable="relay",
        name="scan_front_relay",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "input_topic": "/scan_front_filtered",
                "output_topic": "/scan_interleaved",
            }
        ],
        output="log",
    )

    scan_rear_relay = Node(
        package="topic_tools",
        executable="relay",
        name="scan_rear_relay",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "input_topic": "/scan_rear_filtered",
                "output_topic": "/scan_interleaved",
            }
        ],
        output="log",
    )

    # Fuse state estimator for mobile base localization
    fuse_state_estimator = Node(
        package="fuse_optimizers",
        executable="fixed_lag_smoother_node",
        name="state_estimator",
        respawn=LaunchConfiguration("use_respawn"),
        respawn_delay=2.0,
        parameters=[
            PathJoinSubstitution([hangar_sim_pkg, "config", "fuse", "fuse.yaml"]),
            {"use_sim_time": use_sim_time},
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_fuse")),
    )

    # Guards the one unsupported flag combination; the docstring carries the matrix.
    warn_unsupported_localization = OpaqueFunction(
        function=_warn_unsupported_localization
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)

    # Declare the launch options.
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_localization_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(declare_use_fuse_cmd)

    # Add the actions to launch all of the navigation nodes
    ld.add_action(bringup_cmd_group)

    # uncomment to bring up Nav2 RViz
    # ld.add_action(declare_rviz_config_file_cmd)
    # ld.add_action(declare_use_rviz_cmd)
    # ld.add_action(rviz_cmd)

    ld.add_action(static_tf_world_to_map)
    ld.add_action(static_tf_odom_to_world)
    ld.add_action(odom_world_drift)
    ld.add_action(static_tf_map_to_odom)
    ld.add_action(warn_unsupported_localization)
    if enable_vo:
        ld.add_action(forward_stereo_publisher)
    ld.add_action(lidar_flattener)
    ld.add_action(static_tf_lidar_front_ros)
    ld.add_action(static_tf_lidar_rear_ros)
    ld.add_action(laser_filter_front_node)
    ld.add_action(laser_filter_rear_node)
    ld.add_action(scan_front_relay)
    ld.add_action(scan_rear_relay)
    ld.add_action(fuse_state_estimator)

    return ld
