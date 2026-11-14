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

"""Project the simulated Mid-360 using the survey map's exact h170 scan contract.

The fixed inverted mount puts native Z down. Select base height 1.70 +/-0.02 m
and exclude sensor-XY ranges below 0.50 m, without hiding any other returns in
/livox/lidar. The scan origin is the sensor's XY projected onto base Z=0, as in
the study, with X forward and Y left. No odometry or ground truth is read here.

pointcloud_to_laserscan uses edge-floor bins and only a planar range gate. This
adapter preserves the study's nearest-centre circular bins, separate 3D source
range validation and 0.50 m self gate while advertising range_min=0.1 m.
"""

import math

from livox_mid360_cloud import stamp_ns
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2, PointField

SOURCE_FRAME = "livox_frame"
SCAN_FRAME = "livox_slice"
SENSOR_BASE_XYZ = np.array([0.210, 0.0, 1.700])
SLICE_HEIGHT_M = 1.70
SLICE_HALF_WIDTH_M = 0.02
SELF_XY_GATE_M = 0.50
BEAMS = 333
ANGLE_INCREMENT = 2 * math.pi / BEAMS
ANGLE_MIN = -math.pi + math.pi / BEAMS


def slice_livox_cloud(msg):
    """Preserve acquisition time, nearest planar hits and infinity for unobserved bins."""
    fields = {(f.name, f.offset, f.datatype, f.count) for f in msg.fields}
    expected = {(n, i * 4, PointField.FLOAT32, 1) for i, n in enumerate("xyz")}
    if (
        msg.header.frame_id != SOURCE_FRAME
        or (msg.width, msg.height) != (BEAMS, 60)
        or fields != expected
        or len(msg.fields) != 3
        or msg.point_step != 12
        or msg.row_step != BEAMS * 12
        or len(msg.data) != BEAMS * 60 * 12
    ):
        raise ValueError(
            "Slice source must be a 333x60 packed XYZ cloud in livox_frame"
        )
    native = np.frombuffer(
        msg.data, dtype=">f4" if msg.is_bigendian else "<f4"
    ).reshape(-1, 3)
    body = native.astype(np.float64) * [1, -1, -1] + SENSOR_BASE_XYZ
    xy = body[:, :2] - SENSOR_BASE_XYZ[:2]
    ranges = np.linalg.norm(xy, axis=1)
    source_range = np.linalg.norm(native, axis=1)
    valid = (
        np.isfinite(body).all(axis=1) & (source_range >= 0.1) & (source_range <= 40.0)
    )
    keep = (
        valid
        & (np.abs(body[:, 2] - SLICE_HEIGHT_M) <= SLICE_HALF_WIDTH_M)
        & (ranges >= SELF_XY_GATE_M)
    )
    bins = np.zeros(len(native), dtype=np.int32)
    angles = np.arctan2(xy[:, 1], xy[:, 0])
    bins[valid] = (
        np.round((angles[valid] - ANGLE_MIN) / ANGLE_INCREMENT).astype(np.int32) % BEAMS
    )
    scan_ranges = np.full(BEAMS, np.inf, dtype=np.float32)
    np.minimum.at(scan_ranges, bins[keep], ranges[keep].astype(np.float32))
    scan = LaserScan()
    scan.header.stamp = msg.header.stamp
    scan.header.frame_id = SCAN_FRAME
    scan.angle_min = ANGLE_MIN
    scan.angle_max = ANGLE_MIN + (BEAMS - 1) * ANGLE_INCREMENT
    scan.angle_increment = ANGLE_INCREMENT
    scan.time_increment = 0.0
    scan.scan_time = 0.1
    scan.range_min = 0.1
    scan.range_max = 40.0
    scan.ranges = scan_ranges.tolist()
    return scan


class LivoxLocalizationScan(Node):
    """One scan per fresh source acquisition; never replay a stopped source."""

    def __init__(self):
        super().__init__("livox_localization_scan")
        self._last_stamp = None
        self._publisher = self.create_publisher(
            LaserScan, "/livox/scan", qos_profile_sensor_data
        )
        self.create_subscription(
            PointCloud2, "/livox/lidar", self._receive, qos_profile_sensor_data
        )

    def _receive(self, msg):
        stamp = stamp_ns(msg)
        if self._last_stamp is not None and stamp <= self._last_stamp:
            return
        try:
            scan = slice_livox_cloud(msg)
        except ValueError as error:
            self.get_logger().warning(str(error), throttle_duration_sec=5.0)
            return
        self._publisher.publish(scan)
        self._last_stamp = stamp


def main(args=None):
    rclpy.init(args=args)
    node = LivoxLocalizationScan()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
