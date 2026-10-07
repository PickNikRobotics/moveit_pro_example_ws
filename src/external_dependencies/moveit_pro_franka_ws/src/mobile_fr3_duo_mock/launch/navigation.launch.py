import os
import tempfile

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def merge_params(base, overlay):
    """Merge a Nav2 parameter overlay into a base: mappings merge, every other value replaces."""
    merged = dict(base)
    for key, value in overlay.items():
        if isinstance(value, dict) and isinstance(merged.get(key), dict):
            merged[key] = merge_params(merged[key], value)
        else:
            merged[key] = value
    return merged


def static_identity(parent, child):
    return Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name=f"static_tf_{parent}_to_{child}",
        arguments=["--frame-id", parent, "--child-frame-id", child],
        output="log",
    )


def nav2(context):
    share = FindPackageShare("mobile_fr3_duo_mock").perform(context)
    with open(os.path.join(share, "params", "nav2_params.yaml")) as f:
        params = yaml.safe_load(f)
    overlay_path = LaunchConfiguration("params_overlay").perform(context)
    if overlay_path:
        with open(overlay_path) as f:
            params = merge_params(params, yaml.safe_load(f))
    params_file = tempfile.NamedTemporaryFile(
        mode="w", prefix="mobile_fr3_duo_nav2_", suffix=".yaml", delete=False
    )
    with params_file:
        yaml.safe_dump(params, params_file)

    use_amcl = LaunchConfiguration("use_amcl").perform(context).lower() == "true"
    localization_nodes = ["map_server", "amcl"] if use_amcl else ["map_server"]
    actions = [
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            parameters=[
                params_file.name,
                {"yaml_filename": os.path.join(share, "maps", "room.yaml")},
            ],
            output="log",
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_localization",
            parameters=[{"autostart": True, "node_names": localization_nodes}],
            output="log",
        ),
        # Nav2's planner, controller, behavior trees and the rest, ending at /cmd_vel.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [FindPackageShare("nav2_bringup"), "launch", "navigation_launch.py"]
                )
            ),
            launch_arguments={
                "params_file": params_file.name,
                # The MoveIt Pro hardware plugins stamp wall time.
                "use_sim_time": "false",
                "autostart": "true",
            }.items(),
        ),
    ]
    if use_amcl:
        actions.append(
            Node(
                package="nav2_amcl",
                executable="amcl",
                name="amcl",
                parameters=[params_file.name],
                output="log",
            )
        )
    return actions


def generate_launch_description():
    # Frames: world -> map (fixed) -> odom -> planar joints -> base_link (robot_state_publisher).
    # Without AMCL, map -> odom is fixed, because mock hardware has no drift.
    use_amcl = LaunchConfiguration("use_amcl")
    mock_odometry = LaunchConfiguration("mock_odometry")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_amcl",
                default_value="false",
                description="Run AMCL for map -> odom instead of a fixed transform.",
            ),
            DeclareLaunchArgument(
                "mock_odometry",
                default_value="true",
                description="Publish /odom from the mock planar joint states.",
            ),
            DeclareLaunchArgument(
                "params_overlay",
                default_value="",
                description="Nav2 parameter file layered on this package's params/nav2_params.yaml.",
            ),
            DeclareLaunchArgument(
                "heading_topic",
                default_value="/odom",
                description="Odometry whose heading turns /cmd_vel into world-axis planar velocities.",
            ),
            static_identity("world", "map"),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="static_tf_map_to_odom",
                arguments=["--frame-id", "map", "--child-frame-id", "odom"],
                condition=UnlessCondition(use_amcl),
                output="log",
            ),
            Node(
                package="mobile_fr3_duo_mock",
                executable="joint_state_odometry.py",
                name="joint_state_odometry",
                condition=IfCondition(mock_odometry),
                output="log",
            ),
            # Nav2 drives the base through this bridge: /cmd_vel (base frame) becomes
            # world-axis velocities on the planar joints, for base_jgvc.
            Node(
                package="mobile_fr3_duo_mock",
                executable="base_twist_to_planar.py",
                name="base_twist_to_planar",
                parameters=[{"ground_truth_topic": LaunchConfiguration("heading_topic")}],
                output="log",
            ),
            OpaqueFunction(function=nav2),
        ]
    )
