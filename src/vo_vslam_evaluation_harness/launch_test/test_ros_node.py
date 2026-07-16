#!/usr/bin/env python3

# Copyright 2026 PickNik Inc.
# All rights reserved.
#
# Unauthorized copying of this code base via any medium is strictly prohibited.
# Proprietary and confidential.

"""Launch integration test for synchronized ROS input and failure output."""

from __future__ import annotations

from time import monotonic

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
from nav_msgs.msg import Odometry
import pytest
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


@pytest.mark.launch_test
def generate_test_description() -> launch.LaunchDescription:
    """Launch the real harness with a deliberately missing lazy engine."""
    harness = launch_ros.actions.Node(
        package="vo_vslam_evaluation_harness",
        executable="vo_vslam_evaluation_node",
        parameters=[
            {
                "runtime_adapter": "tensorrt_fp32_tf32",
                "engine_path": "/deliberately/missing/aliked.engine",
                "sync_slop_seconds": 0.01,
            }
        ],
    )
    return launch.LaunchDescription([harness, launch_testing.actions.ReadyToTest()])


class TestSynchronizedFailureContract:
    """Verify real ROS messages exercise synchronization and safe failure output."""

    @classmethod
    def setup_class(cls) -> None:
        """Create publishers and result subscribers."""
        rclpy.init()
        cls.node = rclpy.create_node("vo_vslam_harness_integration_test")
        cls.odometry: list[Odometry] = []
        cls.diagnostics: list[DiagnosticArray] = []
        cls.left_image = cls.node.create_publisher(
            Image, "/stereo/left/image_rect", qos_profile_sensor_data
        )
        cls.right_image = cls.node.create_publisher(
            Image, "/stereo/right/image_rect", qos_profile_sensor_data
        )
        cls.left_info = cls.node.create_publisher(
            CameraInfo, "/stereo/left/camera_info", qos_profile_sensor_data
        )
        cls.right_info = cls.node.create_publisher(
            CameraInfo, "/stereo/right/camera_info", qos_profile_sensor_data
        )
        cls.odometry_subscription = cls.node.create_subscription(
            Odometry, "/vo/odometry", cls.odometry.append, 10
        )
        cls.diagnostics_subscription = cls.node.create_subscription(
            DiagnosticArray, "/vo/diagnostics", cls.diagnostics.append, 10
        )

    @classmethod
    def teardown_class(cls) -> None:
        """Destroy the test node and ROS context."""
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_synchronized_input_preserves_stamp_and_reports_failure(self) -> None:
        """Missing inference fails explicitly without fake output or covariance."""

        def endpoint_count() -> int:
            return min(
                self.left_image.get_subscription_count(),
                self.right_image.get_subscription_count(),
                self.left_info.get_subscription_count(),
                self.right_info.get_subscription_count(),
                self.odometry_subscription.get_publisher_count(),
                self.diagnostics_subscription.get_publisher_count(),
            )

        deadline = monotonic() + 5.0
        while monotonic() < deadline and endpoint_count() == 0:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        assert endpoint_count() > 0, "ROS endpoints did not complete discovery"

        stamp = rclpy.time.Time(seconds=42.25).to_msg()
        left_image = self._image("camera_left_optical", stamp)
        right_image = self._image("camera_right_optical", stamp)
        left_info = self._camera_info("camera_left_optical", stamp, right=False)
        right_info = self._camera_info("camera_right_optical", stamp, right=True)
        self.left_image.publish(left_image)
        self.right_image.publish(right_image)
        self.left_info.publish(left_info)
        self.right_info.publish(right_info)

        deadline = monotonic() + 10.0
        while monotonic() < deadline and not self.diagnostics:
            rclpy.spin_once(self.node, timeout_sec=0.1)

        assert self.diagnostics, "diagnostics were not published"
        assert not self.odometry, "invalid input must not publish odometry"
        status = self.diagnostics[-1].status[0]
        assert self.diagnostics[-1].header.stamp == stamp
        assert status.level == DiagnosticStatus.ERROR
        assert "TensorRT engine does not exist" in status.message
        values = {value.key: value.value for value in status.values}
        assert values["tracking_state"] == "lost"
        assert values["covariance_status"] == "unknown"
        assert values["unsynchronized_messages"] == "0"

        diagnostic_count = len(self.diagnostics)
        self.left_image.publish(left_image)
        self.right_image.publish(right_image)
        self.left_info.publish(left_info)
        self.right_info.publish(right_info)
        deadline = monotonic() + 1.0
        while monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        assert len(self.diagnostics) == diagnostic_count
        assert not self.odometry

    @staticmethod
    def _image(frame_id: str, stamp: object) -> Image:
        message = Image()
        message.header.frame_id = frame_id
        message.header.stamp = stamp
        message.height = 6
        message.width = 8
        message.encoding = "mono8"
        message.step = 8
        message.data = bytes(48)
        return message

    @staticmethod
    def _camera_info(frame_id: str, stamp: object, right: bool) -> CameraInfo:
        message = CameraInfo()
        message.header.frame_id = frame_id
        message.header.stamp = stamp
        message.height = 6
        message.width = 8
        fx = 4.0
        tx = -fx * 0.25 if right else 0.0
        message.p = [fx, 0.0, 4.0, tx, 0.0, fx, 3.0, 0.0, 0.0, 0.0, 1.0, 0.0]
        return message
