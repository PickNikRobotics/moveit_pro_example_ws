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

"""Capture what `calibrate_scan_match_gate` needs to re-measure the fit-to-map gate.

Writes two files:

  grid.txt   the occupancy grid exactly as `map_server` published it on /map.
  scans.txt  merged scans, each stamped with the pose the robot truly held.

Capturing the published grid is deliberate. The alternative -- re-deriving the grid from the .pgm
with a second loader -- leaves you owing a proof that the two agree cell for cell, which is a real
piece of work and easy to get subtly wrong across a threshold or a row order. Reading /map removes
the question rather than answering it.

Ground truth comes from the virtual-rail joints (`linear_x_joint`, `linear_y_joint`,
`rotational_yaw_joint`). In simulation those carry MuJoCo's own body pose through
`joint_state_broadcaster`, and the `mj_world` -> `map` static transform is the identity, so their
values ARE the base pose in the map frame -- independent of anything the localizer believes. Do not
substitute a TF lookup of map -> ridgeback_base_link here: that chain runs through AMCL's own
map -> odom estimate, so it would be scoring the localizer against itself.

PRECONDITION FOR A DRIVEN MULTI-POSE CAMPAIGN -- read this before capturing on the move. A sample
pairs the latest cached /scan_merged with the latest cached virtual-rail joint state. There is no
stamp comparison between the two and no stationarity check, so the pairing is exact only while the
base is standing still. That is why the shipped one-pose calibration is unaffected. It does not
survive driving: at ~0.5 m/s a scan one 10 Hz frame plus one timer period old puts the robot tens
of centimetres from the pose recorded beside it -- well outside the 0.15 m inlier band -- so the
true-pose score comes back depressed and whoever reads the report sets the threshold LOWER than the
map warrants, which is the direction that lets a wrong pose through.

The driven multi-pose campaign the gate still owes must therefore either stop the base at each
sample and capture only while stationary, or add stamp synchronisation here first: gate capture on
the virtual-rail joint velocities being near zero, or reject a sample whose scan stamp and
joint-state stamp differ by more than one frame. Do not run it against this script as written.

    ros2 run hangar_sim dump_localization_calibration_data.py --output-dir /tmp/calib --samples 40
"""

import argparse
import math
import os
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)

from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import JointState, LaserScan

# The virtual-rail joints, in the order the URDF chains them: x, then y, then yaw.
GROUND_TRUTH_JOINTS = ("linear_x_joint", "linear_y_joint", "rotational_yaw_joint")


class CalibrationCapture(Node):
    """Collects one grid and a series of (scan, ground-truth pose) samples."""

    def __init__(self, args):
        super().__init__("dump_localization_calibration_data")
        self._args = args
        self._grid = None
        self._scan = None
        self._truth = None
        self._samples = []
        self._last_sample_position = None

        # /map is latched, so the subscription must be transient_local or a late joiner sees
        # nothing at all -- forever, silently.
        map_qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(OccupancyGrid, args.map_topic, self._on_map, map_qos)
        self.create_subscription(
            LaserScan, args.scan_topic, self._on_scan, qos_profile_sensor_data
        )
        self.create_subscription(JointState, "/joint_states", self._on_joint_state, 10)
        self.create_timer(args.interval, self._maybe_capture)

    def _on_map(self, message):
        self._grid = message

    def _on_scan(self, message):
        self._scan = message

    def _on_joint_state(self, message):
        try:
            values = [
                message.position[message.name.index(joint)]
                for joint in GROUND_TRUTH_JOINTS
            ]
        except (ValueError, IndexError):
            # Other publishers put partial joint states on this topic; a message without the
            # virtual-rail joints is not an error, it just is not the one we want.
            return
        self._truth = tuple(values)

    def _maybe_capture(self):
        if self._grid is None or self._scan is None or self._truth is None:
            self.get_logger().info(
                "waiting for map=%s scan=%s joint_states=%s"
                % (
                    self._grid is not None,
                    self._scan is not None,
                    self._truth is not None,
                ),
                throttle_duration_sec=5.0,
            )
            return
        if len(self._samples) >= self._args.samples:
            return

        x, y, yaw = self._truth
        # Spread the samples out. A dozen scans from one spot calibrate one spot; the threshold has
        # to hold everywhere the robot is allowed to be seeded.
        if self._last_sample_position is not None:
            moved = math.hypot(
                x - self._last_sample_position[0], y - self._last_sample_position[1]
            )
            turned = abs(
                math.atan2(
                    math.sin(yaw - self._last_sample_position[2]),
                    math.cos(yaw - self._last_sample_position[2]),
                )
            )
            if moved < self._args.min_separation and turned < self._args.min_turn:
                return

        self._last_sample_position = (x, y, yaw)
        self._samples.append((self._scan, (x, y, yaw)))
        self.get_logger().info(
            "captured sample %d/%d at (%.3f, %.3f) yaw %.1f deg"
            % (
                len(self._samples),
                self._args.samples,
                x,
                y,
                math.degrees(yaw),
            )
        )

    @property
    def done(self):
        return len(self._samples) >= self._args.samples

    def write(self, output_dir):
        os.makedirs(output_dir, exist_ok=True)

        grid_path = os.path.join(output_dir, "grid.txt")
        info = self._grid.info
        # A quaternion about z only; anything else would mean a rotated map, which the reader
        # handles but this workspace has never shipped.
        yaw = 2.0 * math.atan2(info.origin.orientation.z, info.origin.orientation.w)
        with open(grid_path, "w") as handle:
            handle.write(
                "# hangar_sim occupancy grid, captured from %s\n" % self._args.map_topic
            )
            handle.write("width %d\n" % info.width)
            handle.write("height %d\n" % info.height)
            handle.write("resolution %.10g\n" % info.resolution)
            handle.write("origin_x %.10g\n" % info.origin.position.x)
            handle.write("origin_y %.10g\n" % info.origin.position.y)
            handle.write("origin_yaw %.10g\n" % yaw)
            handle.write("data\n")
            handle.write(" ".join(str(int(cell)) for cell in self._grid.data))
            handle.write("\n")

        scans_path = os.path.join(output_dir, "scans.txt")
        with open(scans_path, "w") as handle:
            handle.write("# hangar_sim scan samples with MuJoCo ground-truth poses\n")
            for index, (scan, truth) in enumerate(self._samples):
                handle.write("scan sample_%02d\n" % index)
                handle.write("angle_min %.10g\n" % scan.angle_min)
                handle.write("angle_increment %.10g\n" % scan.angle_increment)
                handle.write("range_min %.10g\n" % scan.range_min)
                handle.write("range_max %.10g\n" % scan.range_max)
                handle.write("truth_x %.10g\n" % truth[0])
                handle.write("truth_y %.10g\n" % truth[1])
                handle.write("truth_yaw %.10g\n" % truth[2])
                # inf and nan are written literally: a no-return that round-trips as a large finite
                # number would become a scored beam pointing at empty space.
                handle.write(
                    "ranges %d %s\n"
                    % (
                        len(scan.ranges),
                        " ".join("%.6g" % value for value in scan.ranges),
                    )
                )
        return grid_path, scans_path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", default="/tmp/hangar_localization_calibration")
    parser.add_argument("--map-topic", default="/map")
    parser.add_argument("--scan-topic", default="/scan_merged")
    parser.add_argument("--samples", type=int, default=20)
    parser.add_argument(
        "--interval", type=float, default=0.5, help="seconds between capture attempts"
    )
    parser.add_argument(
        "--min-separation",
        type=float,
        default=0.75,
        help="metres the base must have moved before another sample is taken",
    )
    parser.add_argument(
        "--min-turn",
        type=float,
        default=0.35,
        help="radians the base must have turned before another sample is taken",
    )
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = CalibrationCapture(args)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass

    if not node._samples or node._grid is None:
        node.get_logger().error(
            "captured nothing; is the sim up and localization running?"
        )
        node.destroy_node()
        rclpy.shutdown()
        return 1

    grid_path, scans_path = node.write(args.output_dir)
    node.get_logger().info("wrote %s and %s" % (grid_path, scans_path))
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
