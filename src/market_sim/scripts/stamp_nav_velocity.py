#!/usr/bin/env python3
# Copyright 2026 PickNik Inc.
# SPDX-License-Identifier: BSD-3-Clause

"""Stamp Nav2's base-frame velocity for the A300 differential-drive controller."""

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node


class StampNavVelocity(Node):
    """Preserve each Nav2 command and stamp it in the A300 ground frame."""

    def __init__(self):
        super().__init__("stamp_nav_velocity")
        self.publisher = self.create_publisher(TwistStamped, "/cmd_vel", 10)
        self.subscription = self.create_subscription(
            Twist, "/nav2_cmd_vel", self.on_velocity, 10
        )

    def on_velocity(self, velocity):
        command = TwistStamped()
        command.header.stamp = self.get_clock().now().to_msg()
        command.header.frame_id = "footprint"
        command.twist = velocity
        self.publisher.publish(command)


def main():
    rclpy.init()
    node = StampNavVelocity()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
