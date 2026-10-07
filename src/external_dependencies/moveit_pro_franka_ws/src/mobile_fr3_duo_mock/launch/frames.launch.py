from launch import LaunchDescription
from launch_ros.actions import Node


def static_identity(parent, child):
    return Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name=f"static_tf_{parent}_to_{child}",
        arguments=["--frame-id", parent, "--child-frame-id", child],
        output="log",
    )


def generate_launch_description():
    # Mock hardware has no localization, so map -> odom is fixed; robot_state_publisher owns odom -> base_link.
    # mobile_fr3_duo_sim replaces this file with one where AMCL publishes map -> odom.
    return LaunchDescription(
        [
            static_identity("world", "map"),
            static_identity("map", "odom"),
        ]
    )
