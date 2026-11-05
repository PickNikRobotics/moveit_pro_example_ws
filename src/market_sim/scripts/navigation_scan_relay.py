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

"""Anchor navigation scans to simulator truth while leaving SLAM's scans unchanged."""

import copy

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_ros import (
    Buffer,
    StaticTransformBroadcaster,
    TransformListener,
    TransformException,
)


class NavigationScanRelay(Node):
    """Reuse rigid sensor extrinsics beneath the measured simulator base frame."""

    def __init__(self):
        super().__init__("navigation_scan_relay")
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.broadcaster = StaticTransformBroadcaster(self)
        self.frames = {}
        self.publishers_by_side = {}
        for side in ("front", "rear"):
            self.publishers_by_side[side] = self.create_publisher(
                LaserScan, f"/navigation_scan_{side}", qos_profile_sensor_data
            )
            self.create_subscription(
                LaserScan,
                f"/scan_{side}",
                lambda msg, side=side: self.relay(msg, side),
                qos_profile_sensor_data,
            )

    def relay(self, msg, side):
        if side not in self.frames:
            try:
                transform = self.buffer.lookup_transform(
                    "base_link", msg.header.frame_id, rclpy.time.Time()
                )
            except TransformException:
                self.get_logger().warn(
                    f"Navigation scan {side} is waiting for the rigid base_link to {msg.header.frame_id} transform. Check robot_state_publisher if scans remain unavailable.",
                    throttle_duration_sec=5.0,
                )
                return
            transform.header.frame_id = "base_link_ground_truth"
            transform.header.stamp = self.get_clock().now().to_msg()
            transform.child_frame_id = f"navigation_lidar_{side}"
            self.broadcaster.sendTransform(transform)
            self.frames[side] = transform.child_frame_id
        scan = copy.deepcopy(msg)
        scan.header.frame_id = self.frames[side]
        self.publishers_by_side[side].publish(scan)


def main():
    rclpy.init()
    node = NavigationScanRelay()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
