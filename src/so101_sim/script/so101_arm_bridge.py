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

"""Publish SO-101 follower joint positions into the mock joint trajectory controller.

The MoveIt Pro side of this demo is pure `mock_components/GenericSystem`: it
never opens a serial port. This node is the only thing that knows about the arm,
and it feeds the twin by publishing one-point `JointTrajectory` messages, which
the mock hardware echoes straight back out as `/joint_states`.

Two sources are foreseen:

* ``--fake`` (this phase) - a slow sine, so the twin moves with no arm plugged in.
* ``--real`` (phase two) - the Feetech STS3215 bus over USB, read through
  LeRobot's ``SO101Follower``. Stubbed out here behind the same interface.

Mirroring is off until something asks for it. The `Mirror SO101 Follower`
Objective ticks this node's ``~/mirror`` Trigger service in a loop; mirroring
runs while those ticks keep arriving and stops on its own when the Objective is
stopped. That is what keeps mirroring and planning from fighting over the
trajectory controller, which has one owner at a time.
"""

import argparse
import math
import sys

import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# LeRobot's joint order for the SO-101, and the order every downstream consumer
# (the controller, the dataset, the policy) expects.
JOINT_NAMES = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
]

# Radians. Comfortably inside the URDF limits so the sine never trips a
# joint-limit rejection, and centered on an extended, arm-out pose: a wider
# swing on shoulder_lift/elbow_flex folds the wrist back onto the shoulder,
# which is a self-collision, and MoveIt then refuses to plan from the twin's
# current state for as long as the sine sits in that region.
FAKE_AMPLITUDE = [0.8, 0.2, 0.25, 0.35, 1.0, 0.6]
FAKE_CENTER = [0.0, 0.6, -0.6, 0.0, 0.0, 0.7]


def to_radians(degrees, signs, offsets):
    """Convert a bus reading in degrees to URDF radians.

    The Feetech bus reports degrees and does not agree with the URDF on which
    way is positive or where zero is, so each joint carries a sign and an offset
    that a real arm has to be calibrated for. Offsets are in degrees, applied
    before the sign, so they can be read straight off a calibration sheet.
    """
    if not (len(degrees) == len(signs) == len(offsets)):
        raise ValueError(
            f"expected matching lengths, got {len(degrees)}, {len(signs)}, {len(offsets)}"
        )
    return [math.radians((d - o) * s) for d, o, s in zip(degrees, offsets, signs)]


def order_like(names, values):
    """Reorder ``values`` from ``names`` into JOINT_NAMES order.

    The bus and the controller both speak in joint names, but not necessarily in
    the same order, and a silently transposed pair is the kind of bug that only
    shows up as a robot bending the wrong way.
    """
    if len(names) != len(values):
        raise ValueError(f"{len(names)} names but {len(values)} values")
    lookup = dict(zip(names, values))
    missing = [n for n in JOINT_NAMES if n not in lookup]
    if missing:
        raise KeyError(f"missing joints: {missing}")
    return [lookup[n] for n in JOINT_NAMES]


def fake_positions(elapsed_s, period_s):
    """A slow sine, phase-shifted per joint so the whole arm visibly moves.

    Every joint shares one period, so the motion is a single closed curve that
    one period of sampling covers completely. Each joint's constant phase term
    is subtracted, which puts ``elapsed_s == 0`` exactly at FAKE_CENTER, where
    the mock hardware sits before anything drives it, and shifts that joint's
    envelope by ``-amplitude * sin(phase)``.
    """
    return [
        center
        + amplitude
        * (
            math.sin(2.0 * math.pi * elapsed_s / period_s + i * 0.7)
            - math.sin(i * 0.7)
        )
        for i, (center, amplitude) in enumerate(zip(FAKE_CENTER, FAKE_AMPLITUDE))
    ]


class So101ArmBridge(Node):
    """Publish follower joint positions as single-point trajectories."""

    def __init__(self, source="fake"):
        super().__init__("so101_arm_bridge")

        self.source = source
        publish_rate_hz = self.declare_parameter("publish_rate_hz", 50.0).value
        # One controller period of lead time. Too small and the controller
        # discards the point as already in the past; too large and the twin
        # visibly lags the arm.
        self.point_dt_s = self.declare_parameter("point_dt_s", 0.04).value
        self.sine_period_s = self.declare_parameter("sine_period_s", 12.0).value
        # How long a single ~/mirror tick keeps mirroring alive. Long enough to
        # ride out a slow Behavior Tree tick, short enough that stopping the
        # Objective visibly stops the twin.
        self.mirror_timeout_s = self.declare_parameter("mirror_timeout_s", 1.0).value
        topic = self.declare_parameter(
            "joint_trajectory_topic", "/joint_trajectory_controller/joint_trajectory"
        ).value
        status_topic = self.declare_parameter(
            "trajectory_status_topic",
            "/joint_trajectory_controller/follow_joint_trajectory/_action/status",
        ).value
        # Calibration knobs for the real bus. Fake mode ignores them, but they
        # are declared now because a physical arm always needs them and finding
        # that out at bring-up time is a bad afternoon.
        self.port = self.declare_parameter("follower_port", "/dev/so101_follower").value
        self.joint_signs = list(
            self.declare_parameter("joint_signs", [1.0] * len(JOINT_NAMES)).value
        )
        self.joint_offsets_deg = list(
            self.declare_parameter("joint_offsets_deg", [0.0] * len(JOINT_NAMES)).value
        )

        self.publisher = self.create_publisher(JointTrajectory, topic, 10)
        # The trajectory controller has one owner at a time. A stream of topic
        # messages restarts its trajectory on every tick, so an action goal
        # from a plan would be accepted and then never converge. Yield the
        # controller while a goal is live and pick mirroring back up after.
        self.goal_active = False
        self.create_subscription(
            GoalStatusArray, status_topic, self.on_trajectory_status, 10
        )
        self.last_mirror_tick = None
        self.create_service(Trigger, "~/mirror", self.on_mirror_tick)
        self.start_time = self.get_clock().now()
        # The sine's phase is pinned to FAKE_CENTER once, for the first Mirror
        # start. Later restarts continue from wall clock: the twin is then
        # holding wherever the sine left it, and rewinding would snap it back.
        self.sine_phase_pinned = False
        self.timer = self.create_timer(1.0 / publish_rate_hz, self.publish_once)
        self.get_logger().info(
            f"so101_arm_bridge ready to publish {self.source} joint states to "
            f"{topic} at {publish_rate_hz} Hz; run the Mirror SO101 Follower "
            "Objective to start mirroring"
        )

    def read_positions(self):
        """Return the six joint positions in JOINT_NAMES order, in radians."""
        if self.source == "fake":
            elapsed = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9
            return fake_positions(elapsed, self.sine_period_s)
        # Phase two: open the Feetech bus through LeRobot's SO101Follower on
        # self.port, read get_observation(), then
        #   order_like(names, degrees) -> to_radians(..., self.joint_signs,
        #                                            self.joint_offsets_deg)
        raise NotImplementedError(
            "the real Feetech bus source lands in phase two; run with --fake"
        )

    def build_message(self, positions):
        message = JointTrajectory()
        message.header.stamp = self.get_clock().now().to_msg()
        message.joint_names = list(JOINT_NAMES)
        point = JointTrajectoryPoint()
        point.positions = [float(p) for p in positions]
        point.time_from_start = Duration(
            sec=int(self.point_dt_s),
            nanosec=int((self.point_dt_s % 1.0) * 1e9),
        )
        message.points = [point]
        return message

    def on_mirror_tick(self, request, response):
        del request
        now = self.get_clock().now()
        if not self.sine_phase_pinned:
            self.start_time = now
            self.sine_phase_pinned = True
        self.last_mirror_tick = now
        response.success = True
        response.message = "mirroring"
        return response

    def mirroring(self):
        if self.last_mirror_tick is None:
            return False
        age = (self.get_clock().now() - self.last_mirror_tick).nanoseconds * 1e-9
        return age < self.mirror_timeout_s

    def on_trajectory_status(self, message):
        live = {
            GoalStatus.STATUS_ACCEPTED,
            GoalStatus.STATUS_EXECUTING,
            GoalStatus.STATUS_CANCELING,
        }
        active = any(status.status in live for status in message.status_list)
        if active != self.goal_active:
            self.get_logger().info(
                "trajectory goal active, pausing mirroring"
                if active
                else "trajectory goal finished, resuming mirroring"
            )
        self.goal_active = active

    def publish_once(self):
        if self.goal_active or not self.mirroring():
            return
        self.publisher.publish(self.build_message(self.read_positions()))


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group()
    source.add_argument(
        "--fake",
        dest="source",
        action="store_const",
        const="fake",
        default="fake",
        help="publish a slow sine instead of reading an arm (the default)",
    )
    source.add_argument(
        "--real",
        dest="source",
        action="store_const",
        const="real",
        help="read the Feetech bus (not implemented until phase two)",
    )
    args, ros_args = parser.parse_known_args(argv)

    rclpy.init(args=ros_args)
    try:
        rclpy.spin(So101ArmBridge(source=args.source))
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
