# Copyright 2026 PickNik Inc.
# All rights reserved.
#
# Unauthorized copying of this code base via any medium is strictly prohibited.
# Proprietary and confidential.

"""ROS 2 publisher for timestamped TartanAir-style stereo sequences."""

from __future__ import annotations

from math import isfinite
from pathlib import Path
from typing import Any

import cv2
import numpy as np

from .calibration import StereoCalibration
from .playback import discover_tartanair_sequence, load_tartanair_poses


def validate_playback_parameters(
    rate: float,
    fx: float,
    fy: float,
    cx: float,
    cy: float,
    baseline_m: float,
) -> StereoCalibration:
    """Validate timer and CameraInfo values before creating ROS entities."""
    if not isfinite(rate) or rate <= 0.0:
        raise ValueError("publish_rate_hz must be finite and positive")
    timer_period = 1.0 / rate
    if not isfinite(timer_period):
        raise ValueError("derived timer period must be finite")
    if timer_period < 1e-9:
        raise ValueError("derived timer period must represent at least one nanosecond")
    calibration = StereoCalibration(fx, fy, cx, cy, baseline_m)
    if not isfinite(calibration.fx * calibration.baseline_m):
        raise ValueError("derived CameraInfo projection values must be finite")
    return calibration


def run_playback_node() -> None:
    """Publish stereo images, calibration, and ground truth with source stamps."""
    try:
        import rclpy
        from cv_bridge import CvBridge
        from geometry_msgs.msg import PoseStamped
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from rclpy.time import Time
        from sensor_msgs.msg import CameraInfo, Image
    except ImportError as error:
        raise RuntimeError(
            "playback requires rclpy, cv_bridge, geometry_msgs, and sensor_msgs"
        ) from error

    class TartanAirPlayback(Node):
        """Publish one validated benchmark frame per timer tick."""

        def __init__(self) -> None:
            super().__init__("tartanair_stereo_playback")
            defaults: dict[str, Any] = {
                "dataset_root": "",
                "publish_rate_hz": 10.0,
                "left_image_topic": "/stereo/left/image_rect",
                "right_image_topic": "/stereo/right/image_rect",
                "left_camera_info_topic": "/stereo/left/camera_info",
                "right_camera_info_topic": "/stereo/right/camera_info",
                "ground_truth_topic": "/ground_truth/pose",
                "left_frame": "camera_left_optical",
                "right_frame": "camera_right_optical",
                "ground_truth_frame": "world",
                "fx": 320.0,
                "fy": 320.0,
                "cx": 320.0,
                "cy": 240.0,
                "baseline_m": 0.25,
            }
            for name, value in defaults.items():
                self.declare_parameter(name, value)
            root_value = self.get_parameter("dataset_root").value
            if not root_value:
                raise ValueError("dataset_root parameter is required")
            self._frames = discover_tartanair_sequence(Path(root_value))
            self._poses = load_tartanair_poses(Path(root_value), len(self._frames))
            rate = float(self.get_parameter("publish_rate_hz").value)
            self._calibration = validate_playback_parameters(
                rate,
                float(self.get_parameter("fx").value),
                float(self.get_parameter("fy").value),
                float(self.get_parameter("cx").value),
                float(self.get_parameter("cy").value),
                float(self.get_parameter("baseline_m").value),
            )
            self._bridge = CvBridge()
            self._index = 0
            self._left_image = self.create_publisher(
                Image,
                self.get_parameter("left_image_topic").value,
                qos_profile_sensor_data,
            )
            self._right_image = self.create_publisher(
                Image,
                self.get_parameter("right_image_topic").value,
                qos_profile_sensor_data,
            )
            self._left_info = self.create_publisher(
                CameraInfo,
                self.get_parameter("left_camera_info_topic").value,
                qos_profile_sensor_data,
            )
            self._right_info = self.create_publisher(
                CameraInfo,
                self.get_parameter("right_camera_info_topic").value,
                qos_profile_sensor_data,
            )
            self._ground_truth = self.create_publisher(
                PoseStamped, self.get_parameter("ground_truth_topic").value, 10
            )
            self._time_type = Time
            self._timer = self.create_timer(1.0 / rate, self._publish_next)

        def _camera_info(self, width: int, height: int, right: bool) -> Any:
            fx = self._calibration.fx
            fy = self._calibration.fy
            cx = self._calibration.cx
            cy = self._calibration.cy
            baseline = self._calibration.baseline_m
            message = CameraInfo()
            message.width = width
            message.height = height
            message.distortion_model = "plumb_bob"
            message.d = [0.0] * 5
            message.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
            tx = -fx * baseline if right else 0.0
            message.p = [fx, 0.0, cx, tx, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
            return message

        def _publish_next(self) -> None:
            if self._index >= len(self._frames):
                self.get_logger().info("benchmark playback complete")
                self._timer.cancel()
                return
            frame = self._frames[self._index]
            left = cv2.imread(str(frame.left), cv2.IMREAD_GRAYSCALE)
            right = cv2.imread(str(frame.right), cv2.IMREAD_GRAYSCALE)
            if left is None or right is None:
                raise RuntimeError(f"failed to read stereo frame {frame.left.name}")
            if left.shape != right.shape:
                raise ValueError("left and right image dimensions differ")
            stamp = self._time_type(seconds=frame.timestamp).to_msg()
            left_msg = self._bridge.cv2_to_imgmsg(left, encoding="mono8")
            right_msg = self._bridge.cv2_to_imgmsg(right, encoding="mono8")
            left_msg.header.stamp = stamp
            right_msg.header.stamp = stamp
            left_msg.header.frame_id = self.get_parameter("left_frame").value
            right_msg.header.frame_id = self.get_parameter("right_frame").value
            left_info = self._camera_info(left.shape[1], left.shape[0], False)
            right_info = self._camera_info(right.shape[1], right.shape[0], True)
            left_info.header = left_msg.header
            right_info.header = right_msg.header
            pose = PoseStamped()
            pose.header.stamp = stamp
            pose.header.frame_id = self.get_parameter("ground_truth_frame").value
            row = np.asarray(self._poses[self._index], dtype=np.float64)
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = row[:3]
            quaternion = row[3:7]
            norm = float(np.linalg.norm(quaternion))
            if norm <= 0.0:
                raise ValueError("ground-truth quaternion must be nonzero")
            quaternion = quaternion / norm
            pose.pose.orientation.x, pose.pose.orientation.y = quaternion[:2]
            pose.pose.orientation.z, pose.pose.orientation.w = quaternion[2:]
            self._left_image.publish(left_msg)
            self._right_image.publish(right_msg)
            self._left_info.publish(left_info)
            self._right_info.publish(right_info)
            self._ground_truth.publish(pose)
            self._index += 1

    rclpy.init()
    node = TartanAirPlayback()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
