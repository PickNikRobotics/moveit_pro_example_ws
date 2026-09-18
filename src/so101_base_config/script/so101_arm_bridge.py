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

"""Publish SO-101 follower joint positions into the joint trajectory controller.

On mock hardware this feeds `mock_components/GenericSystem`, which echoes the
points straight back out as `/joint_states` with no arm plugged in. On real
hardware (`hardware_interface: "real"`) the same points go through
`feetech_ros2_driver` to the physical servos.

Two sources:

* ``--fake`` - a small sine about the arm's current pose, so the twin moves
  with no arm plugged in on mock, and the real arm wiggles in place on real.
  Because it is small and centred on wherever the arm already is, it doubles
  as the wiggle test: a joint-by-joint "is everything alive and moving the
  right way" diagnostic, in the spirit of lab_sim's joint diagnostic.
* ``--real`` - reads the leader arm's Feetech bus directly (``leader_port``,
  default ``/dev/so101_leader``) and maps its six present positions onto the
  follower, so the leader drives the follower through the same
  ``Mirror SO101 Follower`` Objective (see "Leader-driven mirroring on real
  hardware" in the README) - not related to this node's own connection to the
  follower, which always goes through ros2_control. The runtime image has
  neither ``pyserial`` nor ``scservo_sdk``, so the READ_DATA packet is
  hand-rolled over a raw tty fd rather than through a bus SDK. The commanded
  target glides toward the leader's pose at ``real_slew_rate_rad_s`` rather
  than jumping to it, and ``leader_offset_rad`` corrects a leader servo homed
  off from the follower.

Mirroring is off until something asks for it. The `Mirror SO101 Follower`
Objective ticks this node's ``~/mirror`` Trigger service in a loop; mirroring
runs while those ticks keep arriving and stops on its own when the Objective is
stopped. That is what keeps mirroring and planning from fighting over the
trajectory controller, which has one owner at a time.
"""

import argparse
import math
import os
import sys
import termios
import time

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
# commanded past it. Keep in sync with so101_base_config/description/so101.urdf.xacro.
JOINT_LIMITS = {
    "shoulder_pan": (-1.91986, 1.91986),
    "shoulder_lift": (-1.74533, 1.74533),
    "elbow_flex": (-1.69, 1.69),
    "wrist_flex": (-1.65806, 1.65806),
    "wrist_roll": (-2.74385, 2.84121),
    "gripper": (-0.174533, 1.74533),
}


# --- Leader bus: hand-rolled Feetech READ_DATA over a raw tty fd -----------
#
# No pyserial/scservo_sdk in the runtime image (see the README), so the
# present-position register (addr 56, 2 bytes, little-endian) is read with a
# packet built and parsed by hand: FF FF id len=4 instr=0x02 addr=56 count=2
# checksum=~(sum)&0xFF, replied to with FF FF id len=4 error param_lo param_hi
# checksum=~(sum)&0xFF.

FEETECH_HEADER = b"\xff\xff"
PRESENT_POSITION_ADDR = 56
PRESENT_POSITION_LEN = 2
READ_INSTRUCTION = 0x02
STATUS_PACKET_LEN = len(FEETECH_HEADER) + 3 + PRESENT_POSITION_LEN + 1

# Both arms were LeRobot-calibrated with the same URDF-zero homing pose, so
# tick 2048 is URDF zero on both and every non-gripper joint shares one
# formula (identical to the follower driver's own calibration formula, see
# the README).
LEADER_TICK_ZERO = 2048
LEADER_TICKS_PER_TURN = 4096
LEADER_RAD_PER_TICK = 2.0 * math.pi / LEADER_TICKS_PER_TURN

# The leader trigger and the follower gripper are different mechanisms with
# different travel in ticks (see the bench calibration in the README), so the
# gripper is mapped by fraction of trigger travel onto the URDF gripper
# range instead of the plain tick formula above.
LEADER_TRIGGER_TICKS_CLOSED = 2034
LEADER_TRIGGER_TICKS_OPEN = 3259

# Leader servo ids, base outward, same order as JOINT_NAMES.
LEADER_JOINT_IDS = {name: index + 1 for index, name in enumerate(JOINT_NAMES)}

# The bench leader's wrist_roll is homed about 96 degrees (1.68 rad) off from
# the follower's homing - see README. Added to the leader reading, in
# radians, before clamping. The correct fix is re-homing that servo, then
# setting this back to 0.
LEADER_OFFSET_RAD_DEFAULT = [0.0, 0.0, 0.0, 0.0, 1.68, 0.0]

# Real-mode motion is otherwise a jump straight to wherever the leader is;
# this bounds how fast the commanded target may move per joint, so the
# follower glides to the leader's pose instead.
REAL_SLEW_RATE_RAD_S_DEFAULT = 4.0


def build_read_present_position_packet(servo_id):
    """A Feetech READ_DATA request for one servo's present-position register."""
    body = bytes(
        [
            servo_id,
            PRESENT_POSITION_LEN + 2,
            READ_INSTRUCTION,
            PRESENT_POSITION_ADDR,
            PRESENT_POSITION_LEN,
        ]
    )
    return FEETECH_HEADER + body + bytes([(~sum(body)) & 0xFF])


def parse_present_position_response(data, expected_id):
    """Parse a present-position status packet, or None if anything is wrong.

    None covers every way a real bus reply can go bad - too short, a bad
    header, a stale reply from a different servo, a servo-reported error, a
    checksum mismatch - so the caller (``LeaderBus``) can treat them all the
    same way: skip this sample.
    """
    if len(data) != STATUS_PACKET_LEN or data[0:2] != FEETECH_HEADER:
        return None
    servo_id, length, error = data[2], data[3], data[4]
    if servo_id != expected_id or length != PRESENT_POSITION_LEN + 2 or error != 0:
        return None
    body = data[2:-1]
    if (~sum(body)) & 0xFF != data[-1]:
        return None
    param_low, param_high = data[5], data[6]
    return param_low | (param_high << 8)


def leader_tick_to_radians(tick):
    """Tick 2048 is URDF zero on both arms; see LEADER_TICK_ZERO above."""
    return (tick - LEADER_TICK_ZERO) * LEADER_RAD_PER_TICK


def leader_trigger_to_gripper_radians(tick):
    """Scale the leader trigger's travel onto the follower gripper's URDF range."""
    fraction = (tick - LEADER_TRIGGER_TICKS_CLOSED) / (
        LEADER_TRIGGER_TICKS_OPEN - LEADER_TRIGGER_TICKS_CLOSED
    )
    lower, upper = JOINT_LIMITS["gripper"]
    return lower + fraction * (upper - lower)


class LeaderBus:
    """Reads present-position ticks from the leader's Feetech bus.

    ``read_fn``/``write_fn`` are injected (rather than opening the port here)
    so the read loop and the packet functions above can be unit-tested
    against a pipe instead of a real port - see test_so101_arm_bridge.py.
    """

    def __init__(self, read_fn, write_fn, timeout_s=0.05):
        self.read_fn = read_fn
        self.write_fn = write_fn
        self.timeout_s = timeout_s

    def read_present_position(self, servo_id):
        """Return one servo's present-position tick, or None on a bad reply.

        None also covers a bus I/O error - an unplugged port, or a servo
        fault that knocks the bus out, raises ``OSError`` from the raw fd
        read/write - so the caller treats every failure mode the same way:
        skip this sample.
        """
        try:
            self.write_fn(build_read_present_position_packet(servo_id))
            deadline = time.monotonic() + self.timeout_s
            buffer = b""
            while len(buffer) < STATUS_PACKET_LEN and time.monotonic() < deadline:
                chunk = self.read_fn(STATUS_PACKET_LEN - len(buffer))
                if chunk:
                    buffer += chunk
        except OSError:
            return None
        if len(buffer) != STATUS_PACKET_LEN:
            return None
        return parse_present_position_response(buffer, servo_id)


def read_leader_pose_rad(bus, offset_rad=None):
    """Read all six leader joints, in JOINT_NAMES order, in radians.

    None if any single joint's read fails - a bad reply from one servo means
    skip this whole sample rather than publish a pose with one stale or
    made-up joint in it. ``offset_rad`` (by joint name) corrects a leader
    servo homed off from the follower - see LEADER_OFFSET_RAD_DEFAULT.
    """
    offset_rad = offset_rad or {}
    positions = []
    for name in JOINT_NAMES:
        tick = bus.read_present_position(LEADER_JOINT_IDS[name])
        if tick is None:
            return None
        rad = (
            leader_trigger_to_gripper_radians(tick)
            if name == "gripper"
            else leader_tick_to_radians(tick)
        )
        rad += offset_rad.get(name, 0.0)
        lower, upper = JOINT_LIMITS[name]
        positions.append(min(max(rad, lower), upper))
    return positions


def slew_toward(current, target, max_step):
    """Move each of ``current``'s values toward ``target`` by at most ``max_step``.

    Never overshoots: each result stays between that joint's current and
    target value, so a target already within JOINT_LIMITS keeps the result
    there too without a separate clamp.
    """
    result = []
    for c, t in zip(current, target):
        delta = t - c
        if delta > max_step:
            delta = max_step
        elif delta < -max_step:
            delta = -max_step
        result.append(c + delta)
    return result


def open_leader_port(port):
    """Open the leader's tty in raw mode at 1,000,000 baud.

    Needs a real character device, so this is not covered by unit tests -
    LeaderBus's read_fn/write_fn injection is what makes the rest testable.
    """
    fd = os.open(port, os.O_RDWR | os.O_NOCTTY)
    iflag, oflag, cflag, lflag, ispeed, ospeed, cc = termios.tcgetattr(fd)
    iflag = 0
    oflag = 0
    cflag = (cflag & ~(termios.PARENB | termios.CSTOPB | termios.CSIZE)) | (
        termios.CS8 | termios.CLOCAL | termios.CREAD
    )
    lflag = 0
    ispeed = ospeed = termios.B1000000
    cc[termios.VMIN] = 0
    cc[termios.VTIME] = 1  # 0.1 s per read() call; LeaderBus bounds the total wait
    termios.tcsetattr(
        fd, termios.TCSANOW, [iflag, oflag, cflag, lflag, ispeed, ospeed, cc]
    )
    return fd


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

    def __init__(self, source="fake", leader_bus=None):
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
        # The leader bus that --real reads. Not the follower's port: the
        # follower is always driven through ros2_control, never opened here.
        self.port = self.declare_parameter("leader_port", "/dev/so101_leader").value
        leader_poll_rate_hz = self.declare_parameter("leader_poll_rate_hz", 25.0).value
        # How long the leader bus may go without a good read before the
        # follower stops moving. A dropped read keeps the last good sample
        # (see poll_leader), which is fine for one bad packet, but a leader
        # servo overload or an unplugged bus means every read fails from then
        # on - without this, read_positions() would keep slewing the follower
        # toward that last, now-stale sample forever.
        self.leader_timeout_s = self.declare_parameter("leader_timeout_s", 0.5).value
        self.leader_offset_rad = dict(
            zip(
                JOINT_NAMES,
                self.declare_parameter(
                    "leader_offset_rad", LEADER_OFFSET_RAD_DEFAULT
                ).value,
            )
        )
        real_slew_rate_rad_s = self.declare_parameter(
            "real_slew_rate_rad_s", REAL_SLEW_RATE_RAD_S_DEFAULT
        ).value
        self.real_slew_max_step_rad = real_slew_rate_rad_s / publish_rate_hz

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

        self.leader_bus = None
        self.latest_leader_positions = None
        self.latest_leader_positions_time = None
        self.leader_stale = False
        self.real_target_positions = None
        if self.source == "real":
            self._last_leader_error_log_s = -math.inf
            if leader_bus is not None:
                self.leader_bus = leader_bus
            else:
                fd = open_leader_port(self.port)
                self.leader_bus = LeaderBus(
                    read_fn=lambda n, _fd=fd: os.read(_fd, n),
                    write_fn=lambda data, _fd=fd: os.write(_fd, data),
                )
            self.create_timer(1.0 / leader_poll_rate_hz, self.poll_leader)

        self.timer = self.create_timer(1.0 / publish_rate_hz, self.publish_once)
        source_description = (
            f"wiggling {self.wiggle_amplitude_rad} rad about the pose the arm is "
            "in when mirroring starts"
            if self.source == "fake"
            else f"mirroring the leader on {self.port}"
        )
        self.get_logger().info(
            f"so101_arm_bridge ready to publish {self.source} joint states to "
            f"{topic} at {publish_rate_hz} Hz, {source_description}; run the "
            "Mirror SO101 Follower Objective to start mirroring"
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
        """Return the six joint positions in JOINT_NAMES order, in radians.

        On "real" this glides real_target_positions toward whatever
        poll_leader() last read successfully, at most real_slew_max_step_rad
        per call, so the follower does not jump straight to the leader's
        pose. None until the first good leader read, per the same "don't
        command a guess" principle as the fake source's wiggle_center gate,
        and again once that read goes stale (see leader_fresh()) - a leader
        bus that stopped answering must stop the follower, not leave it
        creeping toward the last pose it heard.
        """
        if self.source == "fake":
            elapsed = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9
            return fake_positions(
                elapsed,
                self.sine_period_s,
                self.wiggle_center,
                self.wiggle_amplitude_rad,
            )
        if self.latest_leader_positions is None or not self.leader_fresh():
            return None
        self.real_target_positions = slew_toward(
            self.real_target_positions,
            self.latest_leader_positions,
            self.real_slew_max_step_rad,
        )
        return self.real_target_positions

    def poll_leader(self):
        """Read the leader's six joints; keep the last good sample on failure.

        A timeout, bad checksum, or bus I/O error means skip this sample, not
        publish a guess or a pose with one stale joint in it - the previous
        good latest_leader_positions is left untouched. But only for so long:
        once the last good sample is older than leader_timeout_s, stop
        follower motion outright (see leader_fresh()/read_positions()) rather
        than keep slewing toward a target that is no longer trustworthy - a
        leader overload or an unplugged bus means every read fails from then
        on, not just this one.
        """
        positions = read_leader_pose_rad(self.leader_bus, self.leader_offset_rad)
        if positions is None:
            self._log_leader_error_throttled(
                "leader read failed (timeout, bad checksum, or bus I/O "
                "error); keeping last known pose"
            )
            if not self.leader_fresh() and not self.leader_stale:
                self.leader_stale = True
                self.get_logger().error(
                    "no good leader read in over leader_timeout_s "
                    f"({self.leader_timeout_s} s); stopping follower motion "
                    "until leader reads recover"
                )
            return
        if self.leader_stale:
            self.get_logger().info("leader reads recovered; resuming mirroring")
            self.leader_stale = False
        self.latest_leader_positions = positions
        self.latest_leader_positions_time = self.get_clock().now()

    def leader_fresh(self):
        """Whether the last good leader read is within leader_timeout_s."""
        if self.latest_leader_positions_time is None:
            return False
        age = (
            self.get_clock().now() - self.latest_leader_positions_time
        ).nanoseconds * 1e-9
        return age < self.leader_timeout_s

    def _log_leader_error_throttled(self, message):
        now = time.monotonic()
        if now - self._last_leader_error_log_s >= 1.0:
            self.get_logger().error(message)
            self._last_leader_error_log_s = now

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
            self.real_target_positions = list(self.latest_positions)
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
            if self.goal_active and not active:
                # The goal may have moved the arm. Force the next mirror tick
                # to validate /joint_states again and re-centre on the pose
                # the arm is actually in now, instead of resuming from
                # wherever it was when the goal took over.
                self.last_mirror_tick = None
                self.wiggle_center = None
        self.goal_active = active

    def publish_once(self):
        if self.goal_active or not self.mirroring() or self.wiggle_center is None:
            return
        positions = self.read_positions()
        if positions is None:
            return
        self.publisher.publish(self.build_message(positions))


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
        help="mirror the leader arm's Feetech bus onto the follower",
    )
    args, ros_args = parser.parse_known_args(argv)

    rclpy.init(args=ros_args)
    try:
        node = So101ArmBridge(source=args.source)
    except OSError as exc:
        # Fail visibly rather than leaving the Mirror Objective's service
        # call to time out with no explanation: the leader must be plugged
        # in and powered before the instance starts on "real".
        print(f"so101_arm_bridge: cannot open leader port: {exc}", file=sys.stderr)
        rclpy.try_shutdown()
        sys.exit(1)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
