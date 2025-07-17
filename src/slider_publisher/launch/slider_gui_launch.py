from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="slider_publisher",
                executable="slider_gui_node",
                name="slider_gui_node",
                output="screen",
            ),
        ]
    )
