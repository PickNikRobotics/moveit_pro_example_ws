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

# Based on hangar_sim's script/lidar_flattener.py: the same flattening logic, with the docstring
# written for this package's scanner and ExternalShutdownException added to the spin guard.

"""Turns each MuJoCo 3D-lidar point cloud into the LaserScan the rest of the stack expects.

Both simulated SICK nanoScan3s are <camera> sensors with DEPTH_TYPE=THREE_D_LIDAR, so
picknik_mujoco_ros publishes them as an organized PointCloud2 rather than as a LaserScan.
Everything downstream of /scan_front and /scan_rear is built on LaserScan: the two relays that
feed beluga_amcl on /scan_interleaved, and both Nav2 costmap obstacle layers. The parameters
are set in launch/sim/robot_drivers_to_persist_sim.launch.py.

The cloud is `resolution` beams wide and 3 rows tall. Row 1 is the elevation-0 row and is
the only one that belongs in a 2D scan; rows 0 and 2 sit at +/-fovy/2 and would put floor
and ceiling returns into the scan plane. Columns run from azimuth -fov_x/2 to +fov_x/2
about the camera's optical Z, which is clockwise in the scan frame, so they are reversed to
give the counter-clockwise LaserScan convention.

Column order and the scan frame are fixed by the MJCF: the sweep, beam count and range
limits belong to the <camera> elements in mjcf/mobile_fr3_duo.xml, whose comment records
where they were measured. Beam 0 lands on the X axis of each lidar_*_ROS frame, which the
launch file publishes statically. A change to either camera or its optical site moves the
columns, and the launch parameters have to follow.
"""

import math

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from sensor_msgs.msg import LaserScan, PointCloud2


class LidarFlattener(Node):
    """Publishes one LaserScan per configured 3D-lidar point cloud."""

    def __init__(self):
        super().__init__("lidar_flattener")

        self.declare_parameter("lidars", ["lidar_front", "lidar_rear"])
        # The window the launch file sets must equal the camera's user[FOV_X] and the beam
        # count its resolution width in mjcf/mobile_fr3_duo.xml, or these angles stop
        # describing the columns published. angle_min is 0 because beam 0 is the frame's X.
        self.declare_parameter("angle_min", 0.0)
        self.declare_parameter("angle_max", math.radians(270.0))
        # Trimmed beams are dropped from each end of the published sweep. This package has
        # no laser_filters chain, so this node is the only place that happens; the launch
        # file sets both ends, and trimming moves angle_min off 0.
        self.declare_parameter("trim_low_deg", 0.0)
        self.declare_parameter("trim_high_deg", 0.0)
        self.declare_parameter("range_min", 0.05)
        self.declare_parameter("range_max", 25.0)
        # MuJoCo's rendered ranges carry no sensor noise, so the launch file injects the
        # real scanner's statistical error here. Off by default, publishing the render as-is.
        self.declare_parameter("range_noise_stddev", 0.0)
        # The sensor reports to the millimetre. Rounding to the same grid keeps a consumer
        # from reading precision out of the simulation that the hardware cannot give it.
        self.declare_parameter("range_quantum", 0.001)

        lidars = self.get_parameter("lidars").value
        self.angle_min = float(self.get_parameter("angle_min").value)
        self.angle_max = float(self.get_parameter("angle_max").value)
        self.trim_low_deg = float(self.get_parameter("trim_low_deg").value)
        self.trim_high_deg = float(self.get_parameter("trim_high_deg").value)
        self.range_min = float(self.get_parameter("range_min").value)
        self.range_max = float(self.get_parameter("range_max").value)
        self.noise = float(self.get_parameter("range_noise_stddev").value)
        self.quantum = float(self.get_parameter("range_quantum").value)
        self.rng = np.random.default_rng()
        self._last_stamp = {}
        self._logged_window = set()

        # BEST_EFFORT to match the plugin's SensorDataQoS on the cloud, and the sensor-data
        # QoS the Nav2 costmap obstacle layers subscribe to /scan_{front,rear} with.
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.publishers_by_name = {}
        for name in lidars:
            self.declare_parameter(f"{name}.points_topic", f"/{name}/points")
            suffix = name.rsplit("_", 1)[-1]
            self.declare_parameter(f"{name}.scan_topic", f"/scan_{suffix}")
            # Scans are stamped in <name>_ROS, a z-up frame with X at beam 0. The camera
            # sensor does not broadcast it, so the launch file publishes it statically.
            self.declare_parameter(f"{name}.frame_id", f"{name}_ROS")
            points_topic = self.get_parameter(f"{name}.points_topic").value
            scan_topic = self.get_parameter(f"{name}.scan_topic").value
            frame_id = self.get_parameter(f"{name}.frame_id").value

            pub = self.create_publisher(LaserScan, scan_topic, qos)
            self.publishers_by_name[name] = (pub, frame_id)
            self.create_subscription(
                PointCloud2,
                points_topic,
                lambda msg, n=name: self._cloud_callback(msg, n),
                qos,
            )
            self.get_logger().info(
                f"{name}: {points_topic} -> {scan_topic} in {frame_id}"
            )

    def _cloud_callback(self, msg, name):
        pub, frame_id = self.publishers_by_name[name]

        # The plugin sizes the cloud from the camera's resolution and publishes it on a
        # timer that can fire before the first render, so an empty message is normal at
        # startup rather than a fault.
        if msg.height < 1 or msg.width < 2 or not msg.data:
            return
        if msg.point_step != 12 or len(msg.data) != msg.width * msg.height * 12:
            self.get_logger().warn(
                f"{name}: unexpected cloud layout "
                f"({msg.width}x{msg.height}, point_step {msg.point_step}); ignoring",
                throttle_duration_sec=5.0,
            )
            return

        xyz = np.frombuffer(msg.data, dtype=np.float32).reshape(
            msg.height, msg.width, 3
        )
        ranges = np.linalg.norm(xyz[msg.height // 2], axis=1).astype(np.float64)

        # Increment comes from the sensor's full declared sweep and the cloud's own width,
        # so trimming moves the window without disturbing the beam spacing.
        increment = (self.angle_max - self.angle_min) / (msg.width - 1)
        n_low = int(math.ceil(math.radians(self.trim_low_deg) / increment - 1e-9))
        n_high = int(math.ceil(math.radians(self.trim_high_deg) / increment - 1e-9))
        if n_low + n_high >= msg.width - 1:
            self.get_logger().error(
                f"{name}: trim_low_deg + trim_high_deg leaves nothing to publish; ignoring",
                throttle_duration_sec=10.0,
            )
            return

        # A beam that hit nothing, or hit outside [range_min, range_max], is left as NaN
        # by the plugin's stitcher. LaserScan wants those out of range, not NaN, because
        # AMCL and both costmap layers only skip a reading they can compare against
        # range_max.
        valid = np.isfinite(ranges)
        if self.noise > 0.0:
            ranges[valid] += self.rng.normal(0.0, self.noise, int(valid.sum()))
        if self.quantum > 0.0:
            ranges[valid] = np.round(ranges[valid] / self.quantum) * self.quantum
        valid &= (ranges >= self.range_min) & (ranges <= self.range_max)
        ranges = np.where(valid, ranges, math.inf)

        # Columns run clockwise in the scan frame, so reverse first, then trim the ends of
        # the published order.
        ranges = ranges[::-1]
        last = msg.width - n_high
        ranges = ranges[n_low:last]

        scan = LaserScan()
        scan.header.stamp = msg.header.stamp
        scan.header.frame_id = frame_id
        scan.angle_min = self.angle_min + n_low * increment
        scan.angle_max = scan.angle_min + (len(ranges) - 1) * increment
        scan.angle_increment = increment
        # The whole cloud is rendered at one instant, so there is no per-beam offset to
        # report. The real sensor sweeps at 15 Hz and reports a beam every 82 us.
        scan.time_increment = 0.0
        # Measured from the gap to the previous cloud rather than assumed, because the
        # rate is whatever point_cloud_publish_rate delivers and that is capped by the
        # render timer this scene shares with the stereo pair and the wrist camera.
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        prev = self._last_stamp.get(name)
        self._last_stamp[name] = stamp
        scan.scan_time = (
            float(stamp - prev) if prev is not None and stamp > prev else 0.0
        )
        scan.range_min = self.range_min
        scan.range_max = self.range_max
        scan.ranges = ranges.astype(np.float32).tolist()
        pub.publish(scan)

        if name not in self._logged_window:
            self._logged_window.add(name)
            self.get_logger().info(
                f"{name}: publishing {len(ranges)} of {msg.width} beams, "
                f"{math.degrees(scan.angle_min):+.3f} to "
                f"{math.degrees(scan.angle_max):+.3f} deg, "
                f"increment {math.degrees(increment):.4f} deg"
            )


def main(args=None):
    rclpy.init(args=args)
    node = LidarFlattener()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
