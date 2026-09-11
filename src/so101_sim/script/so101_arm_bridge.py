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

* ``--fake`` (this phase) - a small sine about the arm's current pose, so the
  twin moves with no arm plugged in. Because it is small and centred on wherever
  the arm already is, it doubles as the wiggle test: a joint-by-joint "is
  everything alive and moving the right way" diagnostic that is safe to run on
  the powered follower, in the spirit of lab_sim's joint diagnostic.
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
from sensor_msgs.msg import JointState
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

# Radians. The sine is a wiggle about wherever the arm already is, not a sweep
# through a fixed pose: small enough to be safe on a powered follower, big
# enough to see. A wider swing would need the self-collision envelope thought
# through again - folding the wrist back over the shoulder makes MoveIt refuse
# to plan from the twin's current state - which is exactly what staying near
# the current pose avoids.
WIGGLE_AMPLITUDE_RAD = 0.1

# Per-joint phase step, so the joints do not move in lockstep and each one's
# motion is readable as its own during the wiggle test.
PHASE_STEP_RAD = 0.7

# The URDF's joint limits, mirrored here because the wiggle is centred on a
# measured pose: a joint already parked at its limit would otherwise be
# commanded past it. Keep in sync with description/so101.urdf.xacro.
JOINT_LIMITS = {
    "shoulder_pan": (-1.91986, 1.91986),
    "shoulder_lift": (-1.74533, 1.74533),
    "elbow_flex": (-1.69, 1.69),
    "wrist_flex": (-1.65806, 1.65806),
    "wrist_roll": (-2.74385, 2.84121),
    "gripper": (-0.174533, 1.74533),
}


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


def fake_positions(elapsed_s, period_s, center, amplitude=WIGGLE_AMPLITUDE_RAD):
    """A small sine about ``center``, phase-shifted per joint.

    ``center`` is the arm's measured pose when mirroring started, so the twin
    never jumps on the first published point and the motion stays a wiggle
    around wherever the arm is standing. The per-joint phase offset means the
    joints do not move in lockstep, which is what makes this readable as a
    diagnostic: each joint's motion is visibly its own.

    Every joint shares one period, so the motion is a single closed curve that
    one period of sampling covers completely. ``elapsed_s == 0`` sits at the
    phase offset rather than at ``center``, so callers pin ``elapsed_s`` to the
    moment they sampled ``center``.

    Results are clamped to JOINT_LIMITS: the centre is measured, so a joint
    parked at its limit would otherwise be commanded past it.
    """
    if len(center) != len(JOINT_NAMES):
        raise ValueError(f"expected {len(JOINT_NAMES)} centers, got {len(center)}")
    positions = []
    for i, (name, middle) in enumerate(zip(JOINT_NAMES, center)):
        lower, upper = JOINT_LIMITS[name]
        phase = i * PHASE_STEP_RAD
        # Subtracting sin(phase) makes every joint start at exactly `center` at
        # elapsed_s == 0 - a plain sin(wt + phase) starts a whole amplitude away
        # for most joints, which on a powered arm is a snap, not a wiggle. The
        # halving keeps the excursion within `amplitude` either side, since the
        # subtracted term widens the range to [-2, 2].
        raw = middle + amplitude * 0.5 * (
            math.sin(2.0 * math.pi * elapsed_s / period_s + phase) - math.sin(phase)
        )
        positions.append(min(max(raw, lower), upper))
    return positions


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
        # How far each joint swings either side of the pose it started from.
        # Small by default so the wiggle test is safe to run on the powered
        # follower; raise it for a more visible sim demo.
        self.wiggle_amplitude_rad = self.declare_parameter(
            "wiggle_amplitude_rad", WIGGLE_AMPLITUDE_RAD
        ).value
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

        joint_states_topic = self.declare_parameter(
            "joint_states_topic", "/joint_states"
        ).value
        # How old the last /joint_states sample may be and still be trusted as
        # the wiggle's centre. A stale sample means the broadcaster died or the
        # bus went quiet, and centring on where the arm was seconds ago is how
        # you get a lurch instead of a wiggle.
        self.joint_states_timeout_s = self.declare_parameter(
            "joint_states_timeout_s", 2.0
        ).value

        self.publisher = self.create_publisher(JointTrajectory, topic, 10)
        # The wiggle is centred on the arm's own pose, so the node has to know
        # where the arm is before it may command anything. Until a /joint_states
        # message carrying all six joints arrives, publish_once() stays quiet.
        self.latest_positions = None
        self.latest_positions_time = None
        self.wiggle_center = None
        self.create_subscription(
            JointState, joint_states_topic, self.on_joint_states, 10
        )
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
        self.timer = self.create_timer(1.0 / publish_rate_hz, self.publish_once)
        self.get_logger().info(
            f"so101_arm_bridge ready to publish {self.source} joint states to "
            f"{topic} at {publish_rate_hz} Hz, wiggling "
            f"{self.wiggle_amplitude_rad} rad about the pose the arm is in when "
            "mirroring starts; run the Mirror SO101 Follower Objective to start "
            "mirroring"
        )

    def on_joint_states(self, message):
        """Keep the latest complete measured pose, in JOINT_NAMES order."""
        try:
            self.latest_positions = order_like(
                list(message.name), list(message.position)
            )
            self.latest_positions_time = self.get_clock().now()
        except (KeyError, ValueError):
            # A broadcaster publishing a subset (or a differently sized message)
            # is not an error worth logging every tick - just keep the last
            # complete pose.
            return

    def read_positions(self):
        """Return the six joint positions in JOINT_NAMES order, in radians."""
        if self.source == "fake":
            elapsed = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9
            return fake_positions(
                elapsed,
                self.sine_period_s,
                self.wiggle_center,
                self.wiggle_amplitude_rad,
            )
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
        """Keep mirroring alive; re-centre the wiggle on each fresh start.

        Every start samples the arm's pose again, because the arm has usually
        moved since the last run - a plan executed, or a person pushed it. The
        clock restarts with it so the sine begins at that sampled pose.
        """
        del request
        now = self.get_clock().now()
        if not self.mirroring():
            if self.latest_positions is None:
                response.success = False
                response.message = (
                    "no /joint_states yet; cannot centre the wiggle on the "
                    "arm's current pose"
                )
                return response
            age_s = (now - self.latest_positions_time).nanoseconds * 1e-9
            if age_s > self.joint_states_timeout_s:
                response.success = False
                response.message = (
                    f"last /joint_states is {age_s:.1f} s old, older than "
                    f"joint_states_timeout_s ({self.joint_states_timeout_s} s); "
                    "refusing to centre the wiggle on a stale pose"
                )
                return response
            self.wiggle_center = list(self.latest_positions)
            self.start_time = now
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
        if self.goal_active or not self.mirroring() or self.wiggle_center is None:
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
        help="wiggle about the arm's current pose instead of reading an arm "
        "(the default)",
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
