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

import math
from pathlib import Path
import sys
import time

import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


sys.path.insert(0, str(Path(__file__).parents[1] / "script"))
from forward_stereo_publisher import ForwardStereoPublisher  # noqa: E402


def spin_until(executor, predicate, timeout_seconds=5.0):
    deadline = time.monotonic() + timeout_seconds
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.05)
        if predicate():
            return True
    return False


def make_rgb_image(width, height, stamp, rgb):
    image = Image()
    image.header.stamp = stamp
    image.height = height
    image.width = width
    image.encoding = "rgb8"
    image.step = width * 3
    image.data = bytes(rgb) * (width * height)
    return image


@pytest.fixture
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


def test_synchronized_pair_publishes_oak_compatible_contract(ros_context):
    width = 4
    height = 2
    vertical_fov_degrees = 55.0
    baseline_m = 0.075
    publisher = ForwardStereoPublisher(
        parameter_overrides=[
            Parameter("width", value=width),
            Parameter("height", value=height),
            Parameter("vertical_fov_degrees", value=vertical_fov_degrees),
            Parameter("baseline_m", value=baseline_m),
        ]
    )
    probe = Node("forward_stereo_test_probe")
    executor = SingleThreadedExecutor()
    executor.add_node(publisher)
    executor.add_node(probe)

    received = {}
    subscriptions = [
        probe.create_subscription(
            Image,
            "/forward_stereo/left/image_rect",
            lambda message: received.setdefault("left_image", message),
            qos_profile_sensor_data,
        ),
        probe.create_subscription(
            Image,
            "/forward_stereo/right/image_rect",
            lambda message: received.setdefault("right_image", message),
            qos_profile_sensor_data,
        ),
        probe.create_subscription(
            CameraInfo,
            "/forward_stereo/left/camera_info",
            lambda message: received.setdefault("left_info", message),
            qos_profile_sensor_data,
        ),
        probe.create_subscription(
            CameraInfo,
            "/forward_stereo/right/camera_info",
            lambda message: received.setdefault("right_info", message),
            qos_profile_sensor_data,
        ),
    ]
    left_source = probe.create_publisher(
        Image, "/forward_stereo_left/color", qos_profile_sensor_data
    )
    right_source = probe.create_publisher(
        Image, "/forward_stereo_right/color", qos_profile_sensor_data
    )

    def discovery_complete():
        return (
            left_source.get_subscription_count() == 1
            and right_source.get_subscription_count() == 1
            and all(
                subscription.get_publisher_count() == 1
                for subscription in subscriptions
            )
        )

    try:
        assert spin_until(
            executor, discovery_complete
        ), "DDS discovery did not complete"

        stamp = probe.get_clock().now().to_msg()
        left_source.publish(make_rgb_image(width, height, stamp, (255, 0, 0)))
        right_source.publish(make_rgb_image(width, height, stamp, (0, 255, 0)))

        assert spin_until(executor, lambda: len(received) == 4)
        left_image = received["left_image"]
        right_image = received["right_image"]
        left_info = received["left_info"]
        right_info = received["right_info"]

        assert left_image.header.stamp == stamp
        assert right_image.header.stamp == stamp
        assert left_info.header.stamp == stamp
        assert right_info.header.stamp == stamp
        assert left_image.header.frame_id == "forward_stereo_left_optical_frame"
        assert right_image.header.frame_id == "forward_stereo_right_optical_frame"
        assert left_image.encoding == "mono8"
        assert right_image.encoding == "mono8"
        assert left_image.step == width
        assert right_image.step == width
        # BT.601 luma of pure red / pure green. Values come from OpenCV's
        # rounding; a +/-1 LSB shift would just mean a different rounding
        # rule, so pin the channel weighting rather than the exact integer.
        assert bytes(left_image.data) == bytes([76]) * (width * height)
        assert bytes(right_image.data) == bytes([150]) * (width * height)

        focal_length_px = height / (
            2.0 * math.tan(math.radians(vertical_fov_degrees) / 2.0)
        )
        assert left_info.width == width
        assert left_info.height == height
        assert left_info.distortion_model == "plumb_bob"
        assert list(left_info.d) == [0.0] * 5
        assert list(left_info.r) == [
            1.0,
            0.0,
            0.0,
            0.0,
            1.0,
            0.0,
            0.0,
            0.0,
            1.0,
        ]
        assert math.isclose(left_info.k[0], focal_length_px)
        assert math.isclose(left_info.k[4], focal_length_px)
        assert left_info.p[3] == 0.0
        assert math.isclose(right_info.p[3], -focal_length_px * baseline_m)

        oversized_image = make_rgb_image(width, height, stamp, (0, 0, 0))
        oversized_image.data = bytes(oversized_image.data) + b"\x00"
        assert publisher.convert_to_mono8(oversized_image, "test_frame") is None
        truncated_image = make_rgb_image(width, height, stamp, (0, 0, 0))
        truncated_image.data = bytes(truncated_image.data)[:-1]
        assert publisher.convert_to_mono8(truncated_image, "test_frame") is None

        received.clear()
        left_source.publish(make_rgb_image(width, height, stamp, (255, 0, 0)))
        stamp_ns = publisher.stamp_nanoseconds(
            make_rgb_image(width, height, stamp, (0, 0, 0))
        )
        assert spin_until(executor, lambda: stamp_ns in publisher.left_images)
        publisher.reset_pairing_buffers(None)
        assert not publisher.left_images
        assert not publisher.right_images

        right_source.publish(make_rgb_image(width, height, stamp, (0, 255, 0)))
        assert spin_until(executor, lambda: stamp_ns in publisher.right_images)
        assert not received

        left_source.publish(make_rgb_image(width, height, stamp, (255, 0, 0)))
        assert spin_until(executor, lambda: len(received) == 4)
    finally:
        executor.remove_node(probe)
        executor.remove_node(publisher)
        probe.destroy_node()
        publisher.destroy_node()


def make_stamp(sec, nanosec=0):
    """Build a builtin_interfaces Time via a throwaway Image header."""
    image = Image()
    image.header.stamp.sec = sec
    image.header.stamp.nanosec = nanosec
    return image.header.stamp


def test_backward_source_stamp_clears_pairing_buffers(ros_context):
    """A rewound source timeline must not fuse frames from two simulator epochs.

    Pairing keys on source header stamps, so a deterministic sim reset that
    replays earlier stamps could otherwise match a buffered pre-reset frame
    against a post-reset frame carrying the same stamp.
    """
    width = 4
    height = 2
    publisher = ForwardStereoPublisher(
        parameter_overrides=[
            Parameter("width", value=width),
            Parameter("height", value=height),
        ]
    )
    try:
        # GIVEN a buffered left frame from the pre-reset epoch
        late_stamp = make_stamp(1000)
        publisher.receive_left(make_rgb_image(width, height, late_stamp, (255, 0, 0)))
        assert publisher.left_images, "expected the pre-reset frame to be buffered"

        # WHEN a frame arrives with an earlier source stamp (sim reset replay)
        early_stamp = make_stamp(500)
        publisher.receive_right(make_rgb_image(width, height, early_stamp, (0, 255, 0)))

        # THEN the stale pre-reset frame is dropped rather than kept for pairing
        assert 1000 * 1_000_000_000 not in publisher.left_images
        # AND only the post-reset frame remains buffered
        assert list(publisher.right_images) == [500 * 1_000_000_000]
    finally:
        publisher.destroy_node()


def test_forward_source_stamps_preserve_pairing_buffers(ros_context):
    """A normally advancing timeline must not trigger the rewind guard."""
    width = 4
    height = 2
    publisher = ForwardStereoPublisher(
        parameter_overrides=[
            Parameter("width", value=width),
            Parameter("height", value=height),
        ]
    )
    try:
        # GIVEN a buffered left frame
        publisher.receive_left(
            make_rgb_image(width, height, make_stamp(500), (255, 0, 0))
        )

        # WHEN a later, unmatched right frame arrives
        publisher.receive_right(
            make_rgb_image(width, height, make_stamp(501), (0, 255, 0))
        )

        # THEN the earlier frame stays buffered awaiting its partner
        assert 500 * 1_000_000_000 in publisher.left_images
    finally:
        publisher.destroy_node()


@pytest.mark.parametrize(
    "kwargs, expected_name",
    [
        ({"width": 0}, "width"),
        ({"height": -1}, "height"),
        ({"vertical_fov_degrees": 0.0}, "vertical_fov_degrees"),
        ({"vertical_fov_degrees": 180.0}, "vertical_fov_degrees"),
        ({"vertical_fov_degrees": float("nan")}, "vertical_fov_degrees"),
        ({"baseline_m": 0.0}, "baseline_m"),
        ({"baseline_m": -0.075}, "baseline_m"),
    ],
)
def test_invalid_calibration_is_rejected(kwargs, expected_name):
    """Degenerate intrinsics must fail loudly, naming the offending parameter."""
    valid = {
        "width": 1280,
        "height": 720,
        "vertical_fov_degrees": 50.53401584672457,
        "baseline_m": 0.075,
    }
    valid.update(kwargs)

    with pytest.raises(ValueError, match=expected_name):
        ForwardStereoPublisher.validate_calibration(**valid)


def test_valid_calibration_is_accepted():
    """The shipped OAK-D Pro profile passes validation."""
    ForwardStereoPublisher.validate_calibration(1280, 720, 50.53401584672457, 0.075)


def test_pairing_resumes_immediately_after_a_rewind(ros_context):
    """After a rewind the replayed epoch must pair, not stall.

    Regression: retaining the pre-rewind high-water mark re-triggered the guard
    on every replayed frame, so each partner was cleared as it arrived and no
    pair was ever published until stamps climbed past the old peak.
    """
    width = 4
    height = 2
    publisher = ForwardStereoPublisher(
        parameter_overrides=[
            Parameter("width", value=width),
            Parameter("height", value=height),
        ]
    )
    published = []
    publisher.publish_pair = lambda left, right: published.append((left, right))
    try:
        # GIVEN a buffered frame from the pre-rewind epoch
        publisher.receive_left(
            make_rgb_image(width, height, make_stamp(1000), (255, 0, 0))
        )

        # WHEN the timeline rewinds and BOTH halves of a replayed frame arrive
        publisher.receive_right(
            make_rgb_image(width, height, make_stamp(500), (0, 255, 0))
        )
        publisher.receive_left(
            make_rgb_image(width, height, make_stamp(500), (255, 0, 0))
        )

        # THEN the replayed frame pairs instead of being discarded
        assert len(published) == 1, "replayed epoch must pair after a rewind"
        left, right = published[0]
        assert publisher.stamp_nanoseconds(left) == 500 * 1_000_000_000
        assert publisher.stamp_nanoseconds(right) == 500 * 1_000_000_000
    finally:
        publisher.destroy_node()


def test_pairing_skips_conversion_when_nothing_is_subscribed(ros_context):
    """With no subscribers the pair is dropped rather than converted.

    A normal hangar_sim run (navigation, manipulation, CI) subscribes to none of
    the four contract topics, and the mono8 conversion is the node's dominant
    cost -- so it must not run for frames no one reads.
    """
    width = 4
    height = 2
    publisher = ForwardStereoPublisher(
        parameter_overrides=[
            Parameter("width", value=width),
            Parameter("height", value=height),
        ]
    )
    converted = []
    original_convert = publisher.convert_to_mono8
    publisher.convert_to_mono8 = lambda *a, **k: (
        converted.append(a) or original_convert(*a, **k)
    )
    try:
        # GIVEN no subscriber on any contract topic
        assert not publisher.has_output_subscribers()

        # WHEN a complete pair arrives
        stamp = make_stamp(500)
        publisher.receive_left(make_rgb_image(width, height, stamp, (255, 0, 0)))
        publisher.receive_right(make_rgb_image(width, height, stamp, (0, 255, 0)))

        # THEN the expensive conversion never runs
        assert converted == []
    finally:
        publisher.destroy_node()


def test_pairing_publishes_once_a_subscriber_attaches(ros_context):
    """A subscriber on any contract topic re-enables the stream."""
    width = 4
    height = 2
    publisher = ForwardStereoPublisher(
        parameter_overrides=[
            Parameter("width", value=width),
            Parameter("height", value=height),
        ]
    )
    probe = Node("forward_stereo_lazy_probe")
    executor = SingleThreadedExecutor()
    executor.add_node(publisher)
    executor.add_node(probe)
    received = []
    probe.create_subscription(
        Image,
        "/forward_stereo/left/image_rect",
        received.append,
        qos_profile_sensor_data,
    )
    try:
        # GIVEN a subscriber that has been discovered
        assert spin_until(
            executor, publisher.has_output_subscribers
        ), "subscriber was never discovered"

        # WHEN a complete pair arrives
        stamp = probe.get_clock().now().to_msg()
        publisher.receive_left(make_rgb_image(width, height, stamp, (255, 0, 0)))
        publisher.receive_right(make_rgb_image(width, height, stamp, (0, 255, 0)))

        # THEN the pair is converted and published
        assert spin_until(executor, lambda: len(received) == 1)
        assert received[0].encoding == "mono8"
    finally:
        executor.remove_node(probe)
        executor.remove_node(publisher)
        probe.destroy_node()
        publisher.destroy_node()
