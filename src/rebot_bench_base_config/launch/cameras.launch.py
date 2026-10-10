# Copyright 2026 PickNik Inc.
# SPDX-License-Identifier: BSD-3-Clause
"""Optional real RGB cameras, with all robot TF supplied by the mock URDF."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import yaml


def launch_cameras(context):
    """Resolve settings only after the cameras-on condition has passed."""
    width = int(LaunchConfiguration("image_width").perform(context))
    wrist_height = int(LaunchConfiguration("wrist_image_height").perform(context))
    scene_height = int(LaunchConfiguration("scene_image_height").perform(context))
    fps = int(LaunchConfiguration("framerate").perform(context))
    quality = int(LaunchConfiguration("scene_mjpeg_quality").perform(context))
    if min(width, wrist_height, scene_height, fps) <= 0 or not 1 <= quality <= 100:
        raise ValueError(
            "Camera dimensions/fps must be positive; MJPEG quality must be 1–100"
        )
    # The scene camera publishes ISP output at a fixed 1/2 scale, which ignores
    # i_width/i_height, so any other size would be silently replaced by 640x400.
    if width != 640 or scene_height != 400:
        raise ValueError(
            "DepthAI ISP output requires image_width=640 and scene_image_height=400"
        )

    return [
        Node(
            package="realsense2_camera",
            executable="realsense2_camera_node",
            name="wrist_mounted_camera",
            namespace="",
            output="screen",
            parameters=[
                {
                    # camera_name sets frame IDs independently of the ROS node name.
                    "camera_name": "d435",
                    "device_type": "D435",
                    "serial_no": ParameterValue(
                        LaunchConfiguration("wrist_serial_no"), value_type=str
                    ),
                    "enable_color": True,
                    "rgb_camera.color_profile": f"{width},{wrist_height},{fps}",
                    "rgb_camera.color_format": "RGB8",
                    "enable_depth": False,
                    "enable_infra1": False,
                    "enable_infra2": False,
                    "enable_gyro": False,
                    "enable_accel": False,
                    "pointcloud.enable": False,
                    "align_depth.enable": False,
                    "publish_tf": False,
                    "color_qos": "SENSOR_DATA",
                    "color_info_qos": "SENSOR_DATA",
                }
            ],
        ),
        Node(
            package="depthai_ros_driver",
            executable="camera_node",
            name="scene_camera",
            namespace="",
            output="screen",
            parameters=[
                {
                    "camera.i_mx_id": ParameterValue(
                        LaunchConfiguration("scene_serial_no"), value_type=str
                    ),
                    "camera.i_usb_speed": "HIGH",
                    "camera.i_pipeline_type": "RGB",
                    "camera.i_nn_type": "none",
                    "pipeline_gen.i_enable_imu": False,
                    "pipeline_gen.i_enable_sync": False,
                    "camera.i_enable_ir": False,
                    "camera.i_publish_tf_from_calibration": False,
                    "camera.i_rs_compat": True,
                    "color.i_resolution": "800P",
                    # Take the ISP output at an explicit 1/2 scale rather than
                    # the video path's crop. The video path at 640x480 keeps only
                    # the central 640x480 of the 1280x800 sensor - 59.1 of its
                    # 97.1 degrees. The ratio must be stated: left to its own
                    # default the ISP picks 2/3 and then centre-crops to the
                    # requested size, which covers only the central 960x600
                    # (80.7 deg). At 1/2 the whole sensor maps to 640x400 at
                    # offset (0,0), measured on the bench, so the full field of
                    # view survives and the driver's width-proportional K and P
                    # come out as exactly the native intrinsics halved - correct,
                    # with no CameraInfo correction needed anywhere.
                    "color.i_set_isp_scale": True,
                    "color.i_isp_num": 1,
                    "color.i_isp_den": 2,
                    "color.i_output_isp": True,
                    "color.i_width": width,
                    "color.i_height": scene_height,
                    "color.i_fps": float(fps),
                    # DepthAI v2's MJPEG decoder uses OpenCV's BGR output.
                    # RGB would label those bytes rgb8 without swapping channels.
                    "color.i_color_order": "BGR",
                    "color.r_set_luma_denoise": True,
                    "color.r_luma_denoise": 0,
                    "color.r_set_chroma_denoise": True,
                    "color.r_chroma_denoise": 0,
                    "color.r_set_sharpness": True,
                    "color.r_sharpness": 0,
                    "color.i_low_bandwidth": True,
                    "color.i_low_bandwidth_profile": 4,  # DepthAI MJPEG
                    "color.i_low_bandwidth_quality": quality,
                    "color.i_publish_compressed": False,
                    "color.i_enable_preview": False,
                    "color.i_enable_lazy_publisher": False,
                }
            ],
        ),
    ]


def generate_launch_description():
    path = (
        Path(get_package_share_directory("rebot_bench_base_config"))
        / "config/cameras.yaml"
    )
    defaults = yaml.safe_load(path.read_text())
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name,
                default_value=(
                    str(value).lower() if isinstance(value, bool) else str(value)
                ),
            )
            for name, value in defaults.items()
        ]
        + [
            OpaqueFunction(
                function=launch_cameras,
                condition=IfCondition(LaunchConfiguration("enable_cameras")),
            )
        ]
    )
