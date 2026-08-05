# Copyright 2022 PickNik Inc.
# All rights reserved.
#
# Unauthorized copying of this code base via any medium is strictly prohibited.
# Proprietary and confidential.


from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Mock the UR Dashboard Client
    mock_dashboard_client = Node(
        package="moveit_studio_kinova_pstop_manager",
        executable="mock_kinova_client_node",
        name="fault_controller",
        output="both",
    )
    # TODO(livanov93): run kinova's protective_stop_manager_node

    return LaunchDescription([mock_dashboard_client])
