# Copyright 2026 PickNik Inc.
# All rights reserved.
#
# Unauthorized copying of this code base via any medium is strictly prohibited.
# Proprietary and confidential.

"""Synchronized stereo ROS 2 visual-odometry node."""

from __future__ import annotations

from typing import Any

import numpy as np

from .adapters import (
    EagerAlikedLightGlueAdapter,
    TensorRTAlikedLightGlueAdapter,
    TensorRTConfig,
)
from .calibration import StereoCalibration
from .core import (
    StereoVisualOdometry,
    TrackingResult,
    TrackingState,
    VOConfig,
    validate_rotation_matrix,
)
from .ros_contract import diagnostic_values


def _quaternion_from_rotation(
    rotation: np.ndarray,
) -> tuple[float, float, float, float]:
    """Convert a proper rotation matrix to an xyzw unit quaternion."""
    matrix = validate_rotation_matrix(rotation)
    trace = float(np.trace(matrix))
    if trace > 0.0:
        scale = np.sqrt(trace + 1.0) * 2.0
        quaternion = (
            (matrix[2, 1] - matrix[1, 2]) / scale,
            (matrix[0, 2] - matrix[2, 0]) / scale,
            (matrix[1, 0] - matrix[0, 1]) / scale,
            0.25 * scale,
        )
    else:
        index = int(np.argmax(np.diag(matrix)))
        if index == 0:
            scale = np.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2.0
            quaternion = (
                0.25 * scale,
                (matrix[0, 1] + matrix[1, 0]) / scale,
                (matrix[0, 2] + matrix[2, 0]) / scale,
                (matrix[2, 1] - matrix[1, 2]) / scale,
            )
        elif index == 1:
            scale = np.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2.0
            quaternion = (
                (matrix[0, 1] + matrix[1, 0]) / scale,
                0.25 * scale,
                (matrix[1, 2] + matrix[2, 1]) / scale,
                (matrix[0, 2] - matrix[2, 0]) / scale,
            )
        else:
            scale = np.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2.0
            quaternion = (
                (matrix[0, 2] + matrix[2, 0]) / scale,
                (matrix[1, 2] + matrix[2, 1]) / scale,
                0.25 * scale,
                (matrix[1, 0] - matrix[0, 1]) / scale,
            )
    values = np.asarray(quaternion, dtype=np.float64)
    if not np.all(np.isfinite(values)):
        raise ValueError("rotation produced a non-finite quaternion")
    maximum = float(np.max(np.abs(values)))
    if maximum == 0.0:
        raise ValueError("rotation produced a zero quaternion")
    scaled = values / maximum
    norm = float(np.linalg.norm(scaled))
    if not np.isfinite(norm) or norm == 0.0:
        raise ValueError("rotation produced an invalid quaternion norm")
    normalized = scaled / norm
    return (
        float(normalized[0]),
        float(normalized[1]),
        float(normalized[2]),
        float(normalized[3]),
    )


def input_failure_result(
    timestamp: float, last_world_from_camera: np.ndarray, error: Exception
) -> TrackingResult:
    """Represent rejected input without inventing a replacement odometry pose."""
    return TrackingResult(
        timestamp=float(timestamp),
        state=TrackingState.LOST,
        world_from_camera=np.asarray(last_world_from_camera, dtype=np.float64).copy(),
        failure_reason=f"{type(error).__name__}: {error}",
    )


def unknown_covariance() -> list[float]:
    """Represent covariance as unavailable without publishing negative variance."""
    return [float("nan")] * 36


def validate_frame_contract(
    left_image_frame: str,
    left_info_frame: str,
    right_image_frame: str,
    right_info_frame: str,
    odometry_child_frame: str,
) -> None:
    """Require source optical frames to match metadata and published pose semantics."""
    if not all(
        (
            left_image_frame,
            left_info_frame,
            right_image_frame,
            right_info_frame,
            odometry_child_frame,
        )
    ):
        raise ValueError(
            "image, CameraInfo, and odometry child frame IDs must be nonempty"
        )
    if left_image_frame != left_info_frame:
        raise ValueError("left Image and CameraInfo frame IDs differ")
    if right_image_frame != right_info_frame:
        raise ValueError("right Image and CameraInfo frame IDs differ")
    if left_image_frame != odometry_child_frame:
        raise ValueError("left optical frame must equal the odometry child frame")


def validate_frame_continuity(
    active_frames: tuple[str, str] | None, current_frames: tuple[str, str]
) -> tuple[str, str]:
    """Lock stereo coordinate-frame identities for one node lifetime."""
    if active_frames is not None and current_frames != active_frames:
        raise ValueError("stereo frame IDs changed; restart is required")
    return current_frames


def validate_timestamp_continuity(
    last_timestamp: float | None, current_timestamp: float
) -> float:
    """Require monotonic finite source time across tracker replacement."""
    current_timestamp = float(current_timestamp)
    if not np.isfinite(current_timestamp) or current_timestamp < 0.0:
        raise ValueError("timestamp must be finite and nonnegative")
    if last_timestamp is not None and current_timestamp <= last_timestamp:
        raise ValueError("timestamps must be strictly increasing")
    return current_timestamp


def run_node() -> None:
    """Initialize and spin the synchronized stereo ROS node."""
    try:
        import message_filters
        import rclpy
        from cv_bridge import CvBridge
        from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
        from nav_msgs.msg import Odometry
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import CameraInfo, Image
    except ImportError as error:
        raise RuntimeError(
            "ROS node requires rclpy, message_filters, cv_bridge, and ROS messages"
        ) from error

    class StereoVONode(Node):
        """Synchronize stereo/calibration and publish odometry plus diagnostics."""

        def __init__(self) -> None:
            super().__init__("vo_vslam_evaluation_harness")
            defaults: dict[str, Any] = {
                "left_image_topic": "/stereo/left/image_rect",
                "right_image_topic": "/stereo/right/image_rect",
                "left_camera_info_topic": "/stereo/left/camera_info",
                "right_camera_info_topic": "/stereo/right/camera_info",
                "odometry_topic": "/vo/odometry",
                "diagnostics_topic": "/vo/diagnostics",
                "odom_frame": "odom",
                "base_frame": "camera_left_optical",
                "runtime_adapter": "tensorrt_fp32_tf32",
                "engine_path": "",
                "engine_metadata_path": "",
                "device": "cuda",
                "max_keypoints": 768,
                "aliked_model_name": "aliked-n16",
                "lightglue_layers": 9,
                "lightglue_depth_confidence": 0.95,
                "lightglue_width_confidence": 0.99,
                "lightglue_filter_threshold": 0.1,
                "engine_preprocess_resize": 1024,
                "engine_score_threshold": 0.2,
                "sync_queue_size": 10,
                "sync_slop_seconds": 0.01,
                "min_disparity_px": 0.5,
                "max_depth_m": 100.0,
                "min_pnp_points": 6,
                "pnp_reprojection_error_px": 0.75,
                "pnp_confidence": 0.999,
                "pnp_iterations": 100,
                "reset_after_failures": 3,
            }
            for name, value in defaults.items():
                self.declare_parameter(name, value)
            self._odom_frame = str(self.get_parameter("odom_frame").value)
            self._base_frame = str(self.get_parameter("base_frame").value)
            if not self._odom_frame or not self._base_frame:
                raise ValueError("odom_frame and base_frame must be nonempty")
            self._adapter = self._make_adapter()
            self._config = VOConfig(
                min_disparity_px=self.get_parameter("min_disparity_px").value,
                max_depth_m=self.get_parameter("max_depth_m").value,
                min_pnp_points=self.get_parameter("min_pnp_points").value,
                pnp_reprojection_error_px=self.get_parameter(
                    "pnp_reprojection_error_px"
                ).value,
                pnp_confidence=self.get_parameter("pnp_confidence").value,
                pnp_iterations=self.get_parameter("pnp_iterations").value,
                reset_after_failures=self.get_parameter("reset_after_failures").value,
            )
            self._bridge = CvBridge()
            self._tracker: StereoVisualOdometry | None = None
            self._calibration: StereoCalibration | None = None
            self._source_frames: tuple[str, str] | None = None
            self._last_source_timestamp: float | None = None
            self._last_world_from_camera = np.eye(4, dtype=np.float64)
            self._received = [0, 0, 0, 0]
            self._synchronized = 0
            self._odometry_publisher = self.create_publisher(
                Odometry, self.get_parameter("odometry_topic").value, 10
            )
            self._diagnostic_publisher = self.create_publisher(
                DiagnosticArray, self.get_parameter("diagnostics_topic").value, 10
            )
            topics = [
                (Image, "left_image_topic"),
                (Image, "right_image_topic"),
                (CameraInfo, "left_camera_info_topic"),
                (CameraInfo, "right_camera_info_topic"),
            ]
            self._subscribers = [
                message_filters.Subscriber(
                    self,
                    message_type,
                    self.get_parameter(parameter).value,
                    qos_profile=qos_profile_sensor_data,
                )
                for message_type, parameter in topics
            ]
            for index, subscriber in enumerate(self._subscribers):
                subscriber.registerCallback(
                    lambda _message, stream=index: self._count_received(stream)
                )
            self._synchronizer = message_filters.ApproximateTimeSynchronizer(
                self._subscribers,
                queue_size=self.get_parameter("sync_queue_size").value,
                slop=self.get_parameter("sync_slop_seconds").value,
            )
            self._synchronizer.registerCallback(self._stereo_callback)

        def _make_adapter(self) -> Any:
            runtime = self.get_parameter("runtime_adapter").value
            if runtime == "eager":
                return EagerAlikedLightGlueAdapter(
                    max_keypoints=self.get_parameter("max_keypoints").value,
                    model_name=self.get_parameter("aliked_model_name").value,
                    lightglue_layers=self.get_parameter("lightglue_layers").value,
                    depth_confidence=self.get_parameter(
                        "lightglue_depth_confidence"
                    ).value,
                    width_confidence=self.get_parameter(
                        "lightglue_width_confidence"
                    ).value,
                    filter_threshold=self.get_parameter(
                        "lightglue_filter_threshold"
                    ).value,
                    device=self.get_parameter("device").value,
                )
            precisions = {
                "tensorrt_fp32_tf32": "fp32_tf32",
                "tensorrt_fp32_strict": "fp32_strict",
                "tensorrt_fp16": "fp16",
            }
            if runtime not in precisions:
                raise ValueError(
                    "runtime_adapter must be eager, tensorrt_fp32_tf32, "
                    "tensorrt_fp32_strict, or tensorrt_fp16"
                )
            resize = self.get_parameter("engine_preprocess_resize").value
            return TensorRTAlikedLightGlueAdapter(
                TensorRTConfig(
                    engine_path=self.get_parameter("engine_path").value,
                    engine_metadata_path=self.get_parameter(
                        "engine_metadata_path"
                    ).value,
                    precision=precisions[runtime],
                    preprocess_resize=resize or None,
                    score_threshold=self.get_parameter("engine_score_threshold").value,
                )
            )

        def _count_received(self, stream: int) -> None:
            self._received[stream] += 1

        def _stereo_callback(
            self, left_image: Any, right_image: Any, left_info: Any, right_info: Any
        ) -> None:
            self._synchronized += 1
            timestamp: float | None = None
            try:
                timestamp = validate_timestamp_continuity(
                    self._last_source_timestamp,
                    left_image.header.stamp.sec
                    + left_image.header.stamp.nanosec * 1e-9,
                )
                self._last_source_timestamp = timestamp
                validate_frame_contract(
                    left_image.header.frame_id,
                    left_info.header.frame_id,
                    right_image.header.frame_id,
                    right_info.header.frame_id,
                    self._base_frame,
                )
                source_frames = validate_frame_continuity(
                    self._source_frames,
                    (left_image.header.frame_id, right_image.header.frame_id),
                )
                if (
                    left_image.width != left_info.width
                    or left_image.height != left_info.height
                ):
                    raise ValueError("left Image and CameraInfo dimensions differ")
                if (
                    right_image.width != right_info.width
                    or right_image.height != right_info.height
                ):
                    raise ValueError("right Image and CameraInfo dimensions differ")
                calibration = StereoCalibration.from_projection_matrices(
                    left_info.p, right_info.p
                )
                left = self._bridge.imgmsg_to_cv2(left_image, desired_encoding="mono8")
                right = self._bridge.imgmsg_to_cv2(
                    right_image, desired_encoding="mono8"
                )
                if calibration != self._calibration:
                    self._calibration = calibration
                    self._tracker = StereoVisualOdometry(
                        self._adapter,
                        calibration,
                        self._config,
                        initial_world_from_camera=self._last_world_from_camera,
                    )
                self._source_frames = source_frames
                if self._tracker is None:
                    raise RuntimeError("tracker calibration was not initialized")
                result = self._tracker.process(left, right, timestamp)
                self._last_world_from_camera = result.world_from_camera.copy()
            except Exception as error:
                self.get_logger().error(f"{type(error).__name__}: {error}")
                if timestamp is None:
                    return
                result = input_failure_result(
                    timestamp, self._last_world_from_camera, error
                )
                self._publish(result, left_image.header.stamp, publish_odometry=False)
                return
            self._publish(result, left_image.header.stamp, publish_odometry=True)

        def _publish(
            self, result: TrackingResult, stamp: Any, publish_odometry: bool
        ) -> None:
            if publish_odometry:
                odometry = Odometry()
                odometry.header.stamp = stamp
                odometry.header.frame_id = self._odom_frame
                odometry.child_frame_id = self._base_frame
                translation = result.world_from_camera[:3, 3]
                odometry.pose.pose.position.x = float(translation[0])
                odometry.pose.pose.position.y = float(translation[1])
                odometry.pose.pose.position.z = float(translation[2])
                quaternion = _quaternion_from_rotation(result.world_from_camera[:3, :3])
                odometry.pose.pose.orientation.x = quaternion[0]
                odometry.pose.pose.orientation.y = quaternion[1]
                odometry.pose.pose.orientation.z = quaternion[2]
                odometry.pose.pose.orientation.w = quaternion[3]
                odometry.pose.covariance = unknown_covariance()
                odometry.twist.covariance = unknown_covariance()
                self._odometry_publisher.publish(odometry)

            diagnostics = DiagnosticArray()
            diagnostics.header.stamp = stamp
            status = DiagnosticStatus()
            status.name = "vo_vslam_evaluation_harness/tracking"
            status.hardware_id = self._adapter.name
            status.message = result.failure_reason or result.state.value
            status.level = {
                TrackingState.TRACKING: DiagnosticStatus.OK,
                TrackingState.INITIALIZING: DiagnosticStatus.WARN,
                TrackingState.DEGRADED: DiagnosticStatus.WARN,
                TrackingState.LOST: DiagnosticStatus.ERROR,
            }[result.state]
            unsynchronized_messages = max(
                0, sum(self._received) - 4 * self._synchronized
            )
            status.values = [
                KeyValue(key=key, value=value)
                for key, value in diagnostic_values(
                    result, unsynchronized_messages, self._adapter.name
                ).items()
            ]
            diagnostics.status = [status]
            self._diagnostic_publisher.publish(diagnostics)

    rclpy.init()
    node = StereoVONode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main() -> None:
    """Console entry point."""
    run_node()
