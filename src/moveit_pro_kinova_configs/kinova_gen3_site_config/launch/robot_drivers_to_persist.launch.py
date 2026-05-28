# Copyright 2024 PickNik Inc.
# All rights reserved.
#
# Unauthorized copying of this code base via any medium is strictly prohibited.
# Proprietary and confidential.

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node

from moveit_studio_utils_py.launch_common import empty_gen
from moveit_studio_utils_py.system_config import (
    SystemConfigParser,
)


def generate_launch_description():
    system_config_parser = SystemConfigParser()
    controller_config = system_config_parser.get_ros2_control_config()

    protective_stop_manager_node = Node(
        package="moveit_studio_kinova_pstop_manager",
        executable="protective_stop_manager_node",
        name="protective_stop_manager_node",
        output="both",
        parameters=[
            {
                "controllers_default_active": controller_config.get(
                    "controllers_active_at_startup", empty_gen()
                ),
                "controllers_default_not_active": controller_config.get(
                    "controllers_inactive_at_startup", empty_gen()
                ),
            }
        ],
    )

    site_share = get_package_share_directory("kinova_gen3_site_config")

    wrist_camera_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(f"{site_share}/launch/cameras.launch.xml")
    )

    # Both ZED 2i cameras are addressed by their stable /dev/v4l/by-path
    # symlink rather than /dev/videoN. The kernel reshuffles the video* numbers
    # on replug/reboot (and when the second camera was added), which silently
    # broke /dev/video0. by-path is keyed on the physical USB port, so each
    # camera stays pinned as long as it's plugged into the same port. (by-id is
    # NOT usable here: both cameras report the identical ZED 2i model string,
    # so their by-id symlinks collide and only one camera gets one.) The
    # "-video-index0" node is the capture node; index1 is the metadata node.
    #
    # If you move a camera to a different USB port, update the path below to
    # match its new `ls /dev/v4l/by-path/` entry.
    zed_camera_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(f"{site_share}/launch/zed_camera.launch.xml"),
        launch_arguments={
            "namespace": "scene_camera",
            "frame_id": "scene_camera_link",
            "video_device": "/dev/v4l/by-path/pci-0000:0f:00.3-usb-0:1:1.0-video-index0",
        }.items(),
    )

    zed_camera_2_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(f"{site_share}/launch/zed_camera.launch.xml"),
        launch_arguments={
            "namespace": "scene_camera_2",
            "frame_id": "scene_camera_2_link",
            "video_device": "/dev/v4l/by-path/pci-0000:0f:00.4-usb-0:2:1.0-video-index0",
        }.items(),
    )

    return LaunchDescription(
        [
            protective_stop_manager_node,
            wrist_camera_launch,
            zed_camera_launch,
            zed_camera_2_launch,
        ]
    )
