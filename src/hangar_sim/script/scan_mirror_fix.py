#!/usr/bin/env python3

# Copyright 2026 PickNik Inc.
# All rights reserved.
#
# Unauthorized copying of this code base via any medium is strictly prohibited.
# Proprietary and confidential.

"""Relay node that reverses the LaserScan ranges array to correct the Y-axis flip.

The MuJoCo lidar plugin publishes lidar_front_ROS / lidar_rear_ROS frames with a
180-degree roll relative to the robot body. This flips the Y-axis, causing obstacles
to appear mirrored left<->right in the costmap. Reversing the ranges array
(range[i] <-> range[N-1-i]) mathematically cancels this flip: the physical beam
pointing at body angle phi gets mapped to scan slot at angle -phi, which the TF
then places back at phi.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
)


BEST_EFFORT_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
)


class ScanMirrorFix(Node):
    def __init__(self):
        super().__init__("scan_mirror_fix")
        self.sub = self.create_subscription(
            LaserScan, "/scan", self.callback, BEST_EFFORT_QOS
        )
        self.pub = self.create_publisher(LaserScan, "/scan_mirrored", BEST_EFFORT_QOS)

    def callback(self, msg):
        msg.ranges = list(reversed(msg.ranges))
        if msg.intensities:
            msg.intensities = list(reversed(msg.intensities))
        # angle_min/max stay the same; the reversed array now maps correctly
        # through the 180-deg roll TF to the correct body-frame angles.
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ScanMirrorFix()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
