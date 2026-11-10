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

"""Shape the native MuJoCo lidar into a Mid-360 cloud without changing SICK navigation.

The native image-based scanner has a symmetric vertical FOV. Its level camera
renders -70..+70 degrees at 1-degree pitch; retaining rows 63..122 gives the
Mid-360's -7..+52 degrees at every azimuth. Overscan keeps those rays inside the
render tiles even on MoveIt Pro 10.2. Pitching the camera instead would rotate
the whole 360-degree cone, not create the sensor's asymmetric FOV.

The source adds range noise and dropouts in picknik_mujoco_ros, before this node.
A 10 Hz timer consumes only the latest fresh sweep of that 20 Hz source. Original
acquisition stamps are retained; old or repeated sweeps are never replayed.
The output is XYZ only, not a livox_ros_driver2 packet/intensity/IMU emulator.
"""

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField

SOURCE_WIDTH = 333
SOURCE_HEIGHT = 141
FIRST_ROW = 63
LAST_ROW = 123
SOURCE_FRAME = "livox_mid360_optical_frame"
OUTPUT_FRAME = "livox_frame"
PUBLISH_PERIOD_SEC = 0.1
MAX_SOURCE_AGE_SEC = 0.25


def stamp_ns(msg):
    """Integer acquisition time, so equality and ordering do not depend on float precision."""
    return msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec


def shape_livox_cloud(msg):
    """Crop the declared native rays and express their coordinates in the co-located lidar frame.

    Reject a different producer contract instead of silently publishing mislabeled
    or mis-shaped measurements. The installed native producer uses tightly packed
    XYZ float32; both byte orders are handled without mutating the input buffer.
    """
    fields = {(f.name, f.offset, f.datatype, f.count) for f in msg.fields}
    expected = {(n, i * 4, PointField.FLOAT32, 1) for i, n in enumerate("xyz")}
    if (
        msg.header.frame_id != SOURCE_FRAME
        or (msg.width, msg.height) != (SOURCE_WIDTH, SOURCE_HEIGHT)
        or fields != expected
        or len(msg.fields) != 3
        or msg.point_step != 12
        or msg.row_step != SOURCE_WIDTH * 12
        or len(msg.data) != SOURCE_WIDTH * SOURCE_HEIGHT * 12
    ):
        raise ValueError(
            "Mid-360 source must be a 333x141 packed XYZ cloud in its optical frame"
        )
    optical = np.frombuffer(
        msg.data, dtype=">f4" if msg.is_bigendian else "<f4"
    ).reshape(SOURCE_HEIGHT, SOURCE_WIDTH, 3)[FIRST_ROW:LAST_ROW]
    # Optical X right, Y down, Z forward -> lidar X forward, Y left, Z up.
    xyz = np.stack((optical[..., 2], -optical[..., 0], -optical[..., 1]), axis=-1)
    xyz = xyz.astype("<f4")
    # An invalid beam stays wholly NaN. Do not manufacture a zero-length return.
    xyz[~np.isfinite(xyz).all(axis=-1)] = np.nan
    cloud = PointCloud2()
    cloud.header.stamp = msg.header.stamp
    cloud.header.frame_id = OUTPUT_FRAME
    cloud.width = SOURCE_WIDTH
    cloud.height = LAST_ROW - FIRST_ROW
    cloud.fields = [
        PointField(name=n, offset=i * 4, datatype=PointField.FLOAT32, count=1)
        for i, n in enumerate("xyz")
    ]
    cloud.point_step = 12
    cloud.row_step = cloud.width * cloud.point_step
    cloud.is_bigendian = False
    cloud.is_dense = False
    cloud.data = xyz.tobytes()
    return cloud


class LivoxMid360Cloud(Node):
    """One cloud publisher, bounded to 10 Hz and never a source of stale/duplicate scans."""

    def __init__(self):
        super().__init__("livox_mid360_cloud")
        self._latest = None
        self._published_stamp = None
        self._publisher = self.create_publisher(
            PointCloud2, "/livox/lidar", qos_profile_sensor_data
        )
        self.create_subscription(
            PointCloud2, "/livox/mujoco/points", self._receive, qos_profile_sensor_data
        )
        self.create_timer(PUBLISH_PERIOD_SEC, self._publish_latest)

    def _receive(self, msg):
        # A late DDS sample cannot replace a newer acquisition.
        if self._latest is None or stamp_ns(msg) > stamp_ns(self._latest):
            self._latest = msg

    def _publish_latest(self):
        msg = self._latest
        if msg is None:
            return
        stamp = stamp_ns(msg)
        if self._published_stamp is not None and stamp <= self._published_stamp:
            return
        age = (self.get_clock().now().nanoseconds - stamp) * 1e-9
        if age < 0.0 or age > MAX_SOURCE_AGE_SEC:
            return
        try:
            cloud = shape_livox_cloud(msg)
        except ValueError as error:
            self.get_logger().warning(str(error), throttle_duration_sec=5.0)
            return
        self._publisher.publish(cloud)
        self._published_stamp = stamp


def main(args=None):
    rclpy.init(args=args)
    node = LivoxMid360Cloud()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
