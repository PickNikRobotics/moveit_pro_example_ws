#!/usr/bin/env python3
"""Anchor navigation scans to simulator truth while leaving SLAM's scans unchanged."""

import copy

import rclpy
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
    """Reuse rigid sensor extrinsics beneath the measured simulator chassis frame."""

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
                    "chassis_link", msg.header.frame_id, rclpy.time.Time()
                )
            except TransformException:
                self.get_logger().warn(
                    f"Navigation scan {side} is waiting for the rigid chassis_link to {msg.header.frame_id} transform. Check robot_state_publisher if scans remain unavailable.",
                    throttle_duration_sec=5.0,
                )
                return
            transform.header.frame_id = "chassis_link_ground_truth"
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
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
