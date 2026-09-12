#!/usr/bin/env python3

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

"""Driver-container launch file: everything that opens a host device by name.

Wired in via ``config/config.yaml``'s ``hardware.additional_driver_launch_file``
(``moveit_studio_utils_py.system_config.HardwareConfig`` requires this to be a
Python launch file - an XML one is rejected). Only the drivers container
bind-mounts the host's ``/dev``; the Agent/runtime container (``runtime.launch.xml``)
does not, so a udev-named device - the leader's Feetech bus
(``/dev/so101_leader``) or either camera's ``/dev/v4l/by-id`` path - is only
reachable from here. Putting these nodes in ``runtime.launch.xml`` instead
fails at startup with "No such file or directory" for the leader port and
"not a valid V4L2 device" for both cameras, even though the same paths exist
under `docker exec` into the *drivers* container.

Included unconditionally, same as ``runtime.launch.xml`` - on mock hardware
the bridge instance below still resolves to its fake source (see
``so101_arm_bridge.launch.py``) and the two cameras still need whatever real
USB devices are plugged in, mock or not.

Every launch argument's default comes from ``config/so101_drivers.yaml`` (see
*Tuning the mirror* in the README) - edit that file and restart the instance
to change one, no rebuild needed (``config/`` is installed via colcon's
symlink-install). A launch argument passed at invocation
(``ros2 launch ... <arg>:=<value>``) still overrides whatever the YAML says.
"""

import os
from pathlib import Path

import yaml
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def load_driver_defaults():
    """Parse config/so101_drivers.yaml, this file's launch-argument defaults."""
    path = (
        Path(get_package_share_directory("so101_base_config"))
        / "config"
        / "so101_drivers.yaml"
    )
    return yaml.safe_load(path.read_text())


def camera_node(context, device_arg, brightness_arg, namespace):
    """usb_cam 0.8.1 cannot follow a relative by-id symlink.

    It resolves a symlinked ``video_device`` by reading the link target and
    prepending ``/dev/`` (upstream ``usb_cam_node.cpp``); the by-id links here
    are relative (``../../video4``), so that yields ``/dev/../../video4`` -
    not a real path. Resolving with ``os.path.realpath`` here and handing
    usb_cam the real ``/dev/videoN`` sidesteps it; the by-id path stays the
    configured, stable value everywhere else (arguments, defaults, README).
    ``realpath`` on a by-id path that does not exist at all returns it
    unchanged, so a bench without that camera plugged in still gets usb_cam's
    own clear "not available" error naming the by-id path.

    ``brightness`` is set explicitly rather than left at usb_cam's own
    unset-by-default (-1, "leave the driver's power-on value alone"): the two
    cameras' power-on brightness both read 50, but on different v4l2 control
    scales - see *Cameras* in the README for the per-model scale and the
    chosen defaults.
    """
    device = os.path.realpath(LaunchConfiguration(device_arg).perform(context))
    brightness = int(LaunchConfiguration(brightness_arg).perform(context))
    return [
        Node(
            package="usb_cam",
            executable="usb_cam_node_exe",
            name="usb_cam_node",
            namespace=namespace,
            output="both",
            parameters=[
                {
                    "video_device": device,
                    "image_width": 640,
                    "image_height": 480,
                    "framerate": 30.0,
                    "frame_id": namespace,
                    "camera_name": namespace,
                    "brightness": brightness,
                    "autoexposure": True,
                }
            ],
        )
    ]


def generate_launch_description():
    bridge_launch_file = (
        get_package_share_directory("so101_base_config")
        + "/launch/so101_arm_bridge.launch.py"
    )
    defaults = load_driver_defaults()
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "wrist_camera_device",
                default_value=str(defaults["wrist_camera_device"]),
            ),
            DeclareLaunchArgument(
                "scene_camera_device",
                default_value=str(defaults["scene_camera_device"]),
            ),
            # See *Cameras* in the README: the wrist camera's brightness
            # control is signed (-64..64), the scene camera's is unsigned
            # (0..255), and their power-on defaults (both numerically 50)
            # land at opposite ends of usable on the two scales.
            DeclareLaunchArgument(
                "wrist_camera_brightness",
                default_value=str(defaults["wrist_camera_brightness"]),
            ),
            DeclareLaunchArgument(
                "scene_camera_brightness",
                default_value=str(defaults["scene_camera_brightness"]),
            ),
            # Mirror-speed tuning, forwarded to so101_arm_bridge.launch.py
            # below - see *Tuning the mirror* in the README.
            DeclareLaunchArgument(
                "real_slew_rate_rad_s",
                default_value=str(defaults["real_slew_rate_rad_s"]),
            ),
            DeclareLaunchArgument(
                "leader_wrist_roll_offset_rad",
                default_value=str(defaults["leader_wrist_roll_offset_rad"]),
            ),
            DeclareLaunchArgument(
                "leader_timeout_s",
                default_value=str(defaults["leader_timeout_s"]),
            ),
            DeclareLaunchArgument(
                "leader_port",
                default_value=str(defaults["leader_port"]),
            ),
            # The Mirror SO101 Follower source: node name "so101_arm_bridge",
            # source "auto" (its own launch file's defaults) - see
            # "Leader-driven mirroring on real hardware" in the README.
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(bridge_launch_file),
                launch_arguments={
                    "real_slew_rate_rad_s": LaunchConfiguration("real_slew_rate_rad_s"),
                    "leader_wrist_roll_offset_rad": LaunchConfiguration(
                        "leader_wrist_roll_offset_rad"
                    ),
                    "leader_timeout_s": LaunchConfiguration("leader_timeout_s"),
                    "leader_port": LaunchConfiguration("leader_port"),
                }.items(),
            ),
            OpaqueFunction(
                function=camera_node,
                args=["wrist_camera_device", "wrist_camera_brightness", "wrist_camera"],
            ),
            OpaqueFunction(
                function=camera_node,
                args=["scene_camera_device", "scene_camera_brightness", "scene_camera"],
            ),
        ]
    )
