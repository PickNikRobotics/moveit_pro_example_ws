#!/usr/bin/env python3
# Copyright 2026 PickNik Inc.
# SPDX-License-Identifier: BSD-3-Clause
"""Republish a depth image with every depth multiplied by a calibration scale.

A stereo camera whose depth reads long by a constant fraction places every
point too far along its camera ray; scaling the depth before the point cloud is
built moves each point back along that ray.
"""

import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

DTYPES = {"16UC1": np.uint16, "mono16": np.uint16, "32FC1": np.float32}


def scale_depth(message, scale):
    """Return a copy of a 16UC1/mono16 (mm) or 32FC1 (m) depth image scaled by `scale`; 0 stays invalid."""
    dtype = np.dtype(DTYPES[message.encoding]).newbyteorder(
        ">" if message.is_bigendian else "<"
    )
    depth = np.frombuffer(bytes(message.data), dtype=dtype)
    scaled = depth.astype(np.float64) * scale
    if dtype.kind == "u":
        scaled = np.clip(np.rint(scaled), 0, np.iinfo(dtype).max)
    out = Image(
        header=message.header,
        height=message.height,
        width=message.width,
        encoding=message.encoding,
        is_bigendian=message.is_bigendian,
        step=message.step,
    )
    out.data = scaled.astype(dtype).tobytes()
    return out


def main():
    rclpy.init()
    node = rclpy.create_node("scale_depth")
    scale = node.declare_parameter("scale", 1.0).value
    publisher = node.create_publisher(Image, "depth_scaled", qos_profile_sensor_data)
    node.create_subscription(
        Image,
        "depth",
        lambda message: publisher.publish(scale_depth(message, scale)),
        qos_profile_sensor_data,
    )
    rclpy.spin(node)


if __name__ == "__main__":
    main()
