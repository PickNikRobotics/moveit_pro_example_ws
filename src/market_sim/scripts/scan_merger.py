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

# Copied from the PickNik sim of this robot, reformatted by this repository's formatter;
# its defaults (1440 beams, 10 Hz, base_link, 0.20 m minimum) already match this package's scans.

"""Merges /scan_front + /scan_rear (each a 275 deg sweep trimmed by 6 deg at
each end, blind notch in opposite corners) into a single 360 deg /scan in
base_link, so AMCL has one scan topic to localize against. AMCL has no
multi-source observation model like the costmap's obstacle_layer does, so
this merge has to happen upstream.
slam_toolbox takes one scan topic too, so mapping mode reads the same output.

The merged scan is what both localization and mapping register against, so a
fault here shows up as a map that will not build and a pose that jumps rather
than as an obvious sensor error. See _lookup_transform for the startup race
that used to do exactly that.

The sources are produced by this package's lidar_flattener from the MuJoCo 3D
lidars' point clouds, 1651 beams each, the figure the real nanoScan3 reports.
That is eighteen times the beam count the per-beam rangefinders carried, so the
projection below is vectorized: one matrix multiply per source instead of a
do_transform_point call per beam.
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener


class ScanMerger(Node):
    """Projects each source scan's beams into base_link and bins them into one output scan."""

    def __init__(self):
        super().__init__("scan_merger")

        self.declare_parameter("front_topic", "/scan_front")
        self.declare_parameter("rear_topic", "/scan_rear")
        self.declare_parameter("output_topic", "/scan")
        self.declare_parameter("target_frame", "base_link")
        # Bin count is set by angular ACCURACY, not by how many bins fill. Do NOT lower it to
        # "fill" the scan; it MOVES TOGETHER with nav2's max_beams.
        self.declare_parameter("num_output_beams", 1440)
        self.declare_parameter("merge_rate_hz", 10.0)
        self.declare_parameter("max_scan_age_sec", 0.2)
        # A backstop, not the mechanism: the housing returns are trimmed in lidar_flattener now. It
        # stays because nothing real can sit this close without the base already in collision.
        self.declare_parameter("min_source_range", 0.20)

        self.target_frame = self.get_parameter("target_frame").value
        self.num_output_beams = self.get_parameter("num_output_beams").value
        self.max_scan_age = self.get_parameter("max_scan_age_sec").value
        self.min_source_range = self.get_parameter("min_source_range").value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        qos_sub = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
        )
        qos_pub = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self._latest_front = None
        self._latest_rear = None
        self._mount_offset = 0.0

        self.create_subscription(
            LaserScan,
            self.get_parameter("front_topic").value,
            self._front_callback,
            qos_sub,
        )
        self.create_subscription(
            LaserScan,
            self.get_parameter("rear_topic").value,
            self._rear_callback,
            qos_sub,
        )
        self.scan_pub = self.create_publisher(
            LaserScan, self.get_parameter("output_topic").value, qos_pub
        )

        merge_period = 1.0 / self.get_parameter("merge_rate_hz").value
        self.create_timer(merge_period, self._merge_and_publish)

    def _front_callback(self, msg):
        self._latest_front = msg

    def _rear_callback(self, msg):
        self._latest_rear = msg

    def _lookup_transform(self, source_frame, stamp):
        """Resolve the mount every cycle rather than caching: fresh costs nothing against a static
        transform and stays correct if the frame goes dynamic again.
        """
        try:
            return self.tf_buffer.lookup_transform(
                self.target_frame, source_frame, stamp
            )
        except Exception:
            # Fall back to the newest available: the mount is rigid, so any fully
            # populated sample is as good, and this rides out startup and jitter.
            try:
                return self.tf_buffer.lookup_transform(
                    self.target_frame, source_frame, rclpy.time.Time()
                )
            except Exception as ex:
                self.get_logger().warn(
                    f"Waiting for transform {source_frame} -> {self.target_frame}: {ex}",
                    throttle_duration_sec=2.0,
                )
                return None

    def _project_scan_into_bins(
        self, msg, ranges_out, angle_min_out, angle_increment_out
    ):
        transform = self._lookup_transform(
            msg.header.frame_id, rclpy.time.Time.from_msg(msg.header.stamp)
        )
        if transform is None:
            return

        ranges = np.asarray(msg.ranges, dtype=np.float64)
        # A beam that hit nothing reaches here as inf. The floor also drops the returns
        # from the robot's own body, which are real measurements of the wrong thing.
        keep = (
            np.isfinite(ranges)
            & (ranges <= msg.range_max)
            & (ranges >= msg.range_min)
            & (ranges >= self.min_source_range)
        )
        if not keep.any():
            return

        idx = np.nonzero(keep)[0]
        angles = msg.angle_min + idx * msg.angle_increment
        pts = np.stack(
            [
                ranges[idx] * np.cos(angles),
                ranges[idx] * np.sin(angles),
                np.zeros(idx.size),
            ],
            axis=1,
        )

        q = transform.transform.rotation
        t = transform.transform.translation
        # Moving the origin from the lidar to base_link lengthens the far beams by the mount
        # offset, so the published range_max has to cover ranges past the sources' own.
        self._mount_offset = max(self._mount_offset, math.hypot(t.x, t.y))
        # Rotation matrix from the quaternion, applied to every beam at once. The per-point
        # tf2_geometry_msgs call this replaces cost more than the rest of the merge together.
        xx, yy, zz = q.x * q.x, q.y * q.y, q.z * q.z
        xy, xz, yz = q.x * q.y, q.x * q.z, q.y * q.z
        wx, wy, wz = q.w * q.x, q.w * q.y, q.w * q.z
        rot = np.array(
            [
                [1 - 2 * (yy + zz), 2 * (xy - wz), 2 * (xz + wy)],
                [2 * (xy + wz), 1 - 2 * (xx + zz), 2 * (yz - wx)],
                [2 * (xz - wy), 2 * (yz + wx), 1 - 2 * (xx + yy)],
            ]
        )
        out = pts @ rot.T + np.array([t.x, t.y, t.z])

        out_angle = np.arctan2(out[:, 1], out[:, 0])
        out_range = np.hypot(out[:, 0], out[:, 1])
        bins = np.rint((out_angle - angle_min_out) / angle_increment_out).astype(int)
        bins %= ranges_out.size
        # Two beams can land in one bin; np.minimum.at keeps the nearer, as the
        # element-wise comparison it replaces did.
        np.minimum.at(ranges_out, bins, out_range)

    def _merge_and_publish(self):
        front = self._latest_front
        rear = self._latest_rear
        if front is None or rear is None:
            return

        now = self.get_clock().now()
        front_age = (
            now - rclpy.time.Time.from_msg(front.header.stamp)
        ).nanoseconds / 1e9
        rear_age = (now - rclpy.time.Time.from_msg(rear.header.stamp)).nanoseconds / 1e9
        if front_age > self.max_scan_age or rear_age > self.max_scan_age:
            self.get_logger().warn(
                "Dropping merge cycle: scan_front/scan_rear too stale "
                f"(front={front_age:.2f}s, rear={rear_age:.2f}s)",
                throttle_duration_sec=2.0,
            )
            return

        n = self.num_output_beams
        angle_increment_out = 2.0 * math.pi / n
        angle_min_out = -math.pi
        ranges_out = np.full(n, math.inf)

        self._project_scan_into_bins(
            front, ranges_out, angle_min_out, angle_increment_out
        )
        self._project_scan_into_bins(
            rear, ranges_out, angle_min_out, angle_increment_out
        )

        out = LaserScan()
        out.header.stamp = (
            front.header.stamp if front_age >= rear_age else rear.header.stamp
        )
        out.header.frame_id = self.target_frame
        out.angle_min = angle_min_out
        out.angle_max = angle_min_out + (n - 1) * angle_increment_out
        out.angle_increment = angle_increment_out
        out.time_increment = 0.0
        out.scan_time = 1.0 / self.get_parameter("merge_rate_hz").value
        # What this scan can contain, not what the sources could: the projection drops everything
        # below min_source_range, so the sources' range_min would advertise an empty band.
        out.range_min = max(min(front.range_min, rear.range_min), self.min_source_range)
        out.range_max = max(front.range_max, rear.range_max) + self._mount_offset
        out.ranges = ranges_out.astype(np.float32).tolist()

        self.scan_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ScanMerger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
