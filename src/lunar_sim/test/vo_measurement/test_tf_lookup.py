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

"""Regression for a camera dynamic edge with an intermediate TF parent."""

import os
import time

from geometry_msgs.msg import TransformStamped
import rclpy
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from scipy.spatial.transform import Rotation
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener
from tf_lookup import (
    dynamic_tf_qos_profile,
    wait_for_transform,
)


def _transform(
    parent: str,
    child: str,
    stamp_s: int,
    translation: tuple[float, float, float],
    yaw: float,
) -> TransformStamped:
    transform = TransformStamped()
    transform.header.frame_id = parent
    transform.child_frame_id = child
    transform.header.stamp.sec = stamp_s
    transform.transform.translation.x = translation[0]
    transform.transform.translation.y = translation[1]
    transform.transform.translation.z = translation[2]
    quaternion = Rotation.from_euler("z", yaw).as_quat()
    transform.transform.rotation.x = quaternion[0]
    transform.transform.rotation.y = quaternion[1]
    transform.transform.rotation.z = quaternion[2]
    transform.transform.rotation.w = quaternion[3]
    return transform


def test_wait_for_transform_spins_for_late_dds_data_and_times_out_bounded() -> None:
    """A late TF producer must be serviced while waiting, while a missing edge stays bounded."""
    assert os.environ.get(
        "ROS_DOMAIN_ID"
    ), "CTest did not allocate an isolated ROS domain"
    context = rclpy.context.Context()
    rclpy.init(context=context)
    node = Node("vo_tf_late_arrival_regression", context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    tf_buffer = Buffer(cache_time=Duration(seconds=30.0), node=node)
    tf_listener = TransformListener(
        tf_buffer,
        node,
        spin_thread=False,
        qos=dynamic_tf_qos_profile(),
    )
    publisher = node.create_publisher(TFMessage, "/tf", dynamic_tf_qos_profile())
    published = False

    def publish_late_chain() -> None:
        nonlocal published
        if published:
            return
        publisher.publish(
            TFMessage(
                transforms=[
                    _transform(
                        "chassis_link",
                        "resolved_camera_mount",
                        10,
                        (0.2, 0.1, 0.3),
                        0.1,
                    ),
                    _transform(
                        "resolved_camera_mount",
                        "oakd_left_optical_frame",
                        10,
                        (0.3, 0.05, 0.32),
                        0.2,
                    ),
                ]
            )
        )
        published = True

    timer = None
    try:
        discovery_deadline = time.monotonic() + 5.0
        while publisher.get_subscription_count() < 1:
            assert (
                time.monotonic() < discovery_deadline
            ), "TF listener was not discovered"
            executor.spin_once(timeout_sec=0.05)

        timer = node.create_timer(0.1, publish_late_chain)
        stamp = Time(nanoseconds=10_000_000_000)
        transform = wait_for_transform(
            node,
            tf_buffer,
            "chassis_link",
            "oakd_left_optical_frame",
            stamp,
            timeout_sec=2.0,
            executor=executor,
        )
        assert published
        assert transform.header.frame_id == "chassis_link"
        assert transform.child_frame_id == "oakd_left_optical_frame"

        started = time.monotonic()
        try:
            wait_for_transform(
                node,
                tf_buffer,
                "missing_parent",
                "missing_child",
                stamp,
                timeout_sec=0.2,
                executor=executor,
            )
        except TimeoutError as error:
            assert "missing_parent <- missing_child" in str(error)
        else:
            raise AssertionError("Missing TF unexpectedly became available")
        elapsed = time.monotonic() - started
        assert 0.15 <= elapsed < 1.0
    finally:
        if timer is not None:
            node.destroy_timer(timer)
        node.destroy_publisher(publisher)
        tf_listener.unregister()
        executor.remove_node(node)
        node.destroy_node()
        executor.shutdown()
        rclpy.shutdown(context=context)
