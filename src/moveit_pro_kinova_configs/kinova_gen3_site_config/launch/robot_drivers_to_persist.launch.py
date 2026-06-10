# Copyright 2024 PickNik Inc.
# All rights reserved.
#
# Unauthorized copying of this code base via any medium is strictly prohibited.
# Proprietary and confidential.

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
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
            # On a SEPARATE USB controller (pci-0000:71:00.4, dark-blue 3.2 Gen2
            # port, SuperSpeed 5 Gbps) so the two ZED 2i don't share an xHCI
            # controller. scene_camera_2 stays on 0b:00.0 (light-blue port).
            "video_device": "/dev/v4l/by-path/pci-0000:71:00.4-usb-0:1:1.0-video-index0",
        }.items(),
    )

    zed_camera_2_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(f"{site_share}/launch/zed_camera.launch.xml"),
        launch_arguments={
            "namespace": "scene_camera_2",
            "frame_id": "scene_camera_2_link",
            "video_device": "/dev/v4l/by-path/pci-0000:0b:00.0-usb-0:5:1.0-video-index0",
        }.items(),
    )

    # Start scene_camera_2 well after the other drivers. Brought up alongside
    # them, scene_camera_2 advertises its topics but publishes ZERO frames: its
    # v4l2_camera_node hangs at "Starting camera" (VIDIOC_STREAMON) with no
    # "Streaming: YES" and never retries. The device is fine (a direct
    # `v4l2-ctl --stream-mmap` on it captures 30fps) and relaunching the node
    # alone, once everything else is settled, always streams -- so it's a
    # startup-contention race, not bandwidth (already mitigated by VGA) or
    # hardware. The long pole is the wrist (kinova_vision) camera, which retries
    # its own stream-start for ~30s before succeeding; scene_camera_2 must
    # STREAMON after that window has cleared. 40s gives margin past the observed
    # ~34s wrist settle. (A 10s delay was not enough -- it landed mid-storm.)
    delayed_zed_camera_2_launch = TimerAction(
        period=40.0, actions=[zed_camera_2_launch]
    )

    return LaunchDescription(
        [
            protective_stop_manager_node,
            wrist_camera_launch,
            zed_camera_launch,
            delayed_zed_camera_2_launch,
        ]
    )
