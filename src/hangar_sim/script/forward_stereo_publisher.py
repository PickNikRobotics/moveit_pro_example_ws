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

"""Publish a synchronized, calibrated stereo contract for ideal MuJoCo cameras."""

import math
import threading

import numpy as np
import rclpy
from rclpy.clock import JumpThreshold
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


class ForwardStereoPublisher(Node):
    """Pair ideal rendered images by timestamp and publish stereo calibration."""

    def __init__(self, parameter_overrides=None):
        super().__init__(
            "forward_stereo_publisher", parameter_overrides=parameter_overrides
        )

        self.width = self.declare_parameter("width", 1280).value
        self.height = self.declare_parameter("height", 720).value
        vertical_fov_degrees = self.declare_parameter(
            "vertical_fov_degrees", 50.53401584672457
        ).value
        self.baseline_m = self.declare_parameter("baseline_m", 0.075).value
        self.left_frame_id = self.declare_parameter(
            "left_frame_id", "forward_stereo_left_optical_frame"
        ).value
        self.right_frame_id = self.declare_parameter(
            "right_frame_id", "forward_stereo_right_optical_frame"
        ).value
        left_source_topic = self.declare_parameter(
            "left_source_topic", "/forward_stereo_left/color"
        ).value
        right_source_topic = self.declare_parameter(
            "right_source_topic", "/forward_stereo_right/color"
        ).value
        left_image_topic = self.declare_parameter(
            "left_image_topic", "/forward_stereo/left/image_rect"
        ).value
        right_image_topic = self.declare_parameter(
            "right_image_topic", "/forward_stereo/right/image_rect"
        ).value
        left_info_topic = self.declare_parameter(
            "left_camera_info_topic", "/forward_stereo/left/camera_info"
        ).value
        right_info_topic = self.declare_parameter(
            "right_camera_info_topic", "/forward_stereo/right/camera_info"
        ).value

        self.focal_length_px = self.height / (
            2.0 * math.tan(math.radians(vertical_fov_degrees) / 2.0)
        )
        self.principal_x_px = self.width / 2.0
        self.principal_y_px = self.height / 2.0
        self.queue_depth = 5
        self.left_images = {}
        self.right_images = {}
        self.pairing_lock = threading.Lock()
        self.reported_bad_dimensions = False
        self.reported_bad_encoding = False

        self.clock_jump_callback = self.get_clock().create_jump_callback(
            JumpThreshold(
                min_forward=None,
                min_backward=Duration(nanoseconds=-1),
                on_clock_change=True,
            ),
            post_callback=self.reset_pairing_buffers,
        )

        self.left_image_publisher = self.create_publisher(
            Image, left_image_topic, qos_profile_sensor_data
        )
        self.right_image_publisher = self.create_publisher(
            Image, right_image_topic, qos_profile_sensor_data
        )
        self.left_info_publisher = self.create_publisher(
            CameraInfo, left_info_topic, qos_profile_sensor_data
        )
        self.right_info_publisher = self.create_publisher(
            CameraInfo, right_info_topic, qos_profile_sensor_data
        )
        self.create_subscription(
            Image, left_source_topic, self.receive_left, qos_profile_sensor_data
        )
        self.create_subscription(
            Image, right_source_topic, self.receive_right, qos_profile_sensor_data
        )

    @staticmethod
    def stamp_nanoseconds(image):
        return image.header.stamp.sec * 1_000_000_000 + image.header.stamp.nanosec

    def receive_left(self, image):
        self.receive_image(image, self.left_images, self.right_images)

    def receive_right(self, image):
        self.receive_image(image, self.right_images, self.left_images)

    def receive_image(self, image, own_queue, other_queue):
        if image.width != self.width or image.height != self.height:
            if not self.reported_bad_dimensions:
                self.get_logger().error(
                    "Forward stereo source dimensions do not match calibration: "
                    f"received {image.width}x{image.height}, expected "
                    f"{self.width}x{self.height}"
                )
                self.reported_bad_dimensions = True
            return

        stamp = self.stamp_nanoseconds(image)
        pair = None
        with self.pairing_lock:
            own_queue[stamp] = image
            if stamp in other_queue:
                pair = (
                    self.left_images.pop(stamp),
                    self.right_images.pop(stamp),
                )

            while len(own_queue) > self.queue_depth:
                del own_queue[min(own_queue)]

        if pair is not None:
            self.publish_pair(*pair)

    def reset_pairing_buffers(self, _time_jump):
        with self.pairing_lock:
            self.left_images.clear()
            self.right_images.clear()

    def make_camera_info(self, stamp, frame_id, projection_tx):
        info = CameraInfo()
        info.header.stamp = stamp
        info.header.frame_id = frame_id
        info.height = self.height
        info.width = self.width
        info.distortion_model = "plumb_bob"
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        info.k = [
            self.focal_length_px,
            0.0,
            self.principal_x_px,
            0.0,
            self.focal_length_px,
            self.principal_y_px,
            0.0,
            0.0,
            1.0,
        ]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [
            self.focal_length_px,
            0.0,
            self.principal_x_px,
            projection_tx,
            0.0,
            self.focal_length_px,
            self.principal_y_px,
            0.0,
            0.0,
            0.0,
            1.0,
            0.0,
        ]
        return info

    def convert_to_mono8(self, source_image, frame_id):
        if source_image.encoding != "rgb8":
            if not self.reported_bad_encoding:
                self.get_logger().error(
                    "Forward stereo source encoding does not match MuJoCo's "
                    f"RGB contract: received {source_image.encoding}, expected rgb8"
                )
                self.reported_bad_encoding = True
            return None

        expected_row_bytes = self.width * 3
        expected_buffer_bytes = source_image.step * self.height
        if (
            source_image.step < expected_row_bytes
            or len(source_image.data) != expected_buffer_bytes
        ):
            if not self.reported_bad_encoding:
                self.get_logger().error(
                    "Forward stereo source image buffer size does not match step * height"
                )
                self.reported_bad_encoding = True
            return None

        rgb_rows = np.frombuffer(source_image.data, dtype=np.uint8).reshape(
            self.height, source_image.step
        )
        rgb = rgb_rows[:, :expected_row_bytes].reshape(self.height, self.width, 3)
        # Integer BT.601 luma conversion. The OV9282 pair is monochrome, so
        # publish mono8 rather than MuJoCo's native rgb8 render representation.
        rgb_16 = rgb.astype(np.uint16)
        mono = (
            77 * rgb_16[:, :, 0] + 150 * rgb_16[:, :, 1] + 29 * rgb_16[:, :, 2]
        ) >> 8

        image = Image()
        image.header = source_image.header
        image.header.frame_id = frame_id
        image.height = self.height
        image.width = self.width
        image.encoding = "mono8"
        image.is_bigendian = False
        image.step = self.width
        image.data = mono.astype(np.uint8).tobytes()
        return image

    def publish_pair(self, left_image, right_image):
        left_mono = self.convert_to_mono8(left_image, self.left_frame_id)
        right_mono = self.convert_to_mono8(right_image, self.right_frame_id)
        if left_mono is None or right_mono is None:
            return

        left_info = self.make_camera_info(
            left_mono.header.stamp, self.left_frame_id, 0.0
        )
        right_info = self.make_camera_info(
            right_mono.header.stamp,
            self.right_frame_id,
            -self.focal_length_px * self.baseline_m,
        )

        self.left_image_publisher.publish(left_mono)
        self.left_info_publisher.publish(left_info)
        self.right_image_publisher.publish(right_mono)
        self.right_info_publisher.publish(right_info)


def main(args=None):
    rclpy.init(args=args)
    node = ForwardStereoPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
