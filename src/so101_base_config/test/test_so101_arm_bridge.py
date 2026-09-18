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
"""Check the bridge's fake and real sources and its unit-conversion helpers."""

from pathlib import Path
import math
import os
import sys
import time

import pytest
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory

sys.path.insert(0, str(Path(__file__).parents[1] / "script"))
from so101_arm_bridge import (  # noqa: E402
    JOINT_LIMITS,
    JOINT_NAMES,
    LEADER_OFFSET_RAD_DEFAULT,
    LEADER_TRIGGER_TICKS_CLOSED,
    LEADER_TRIGGER_TICKS_OPEN,
    WIGGLE_AMPLITUDE_RAD,
    LeaderBus,
    So101ArmBridge,
    build_read_present_position_packet,
    fake_positions,
    leader_tick_to_radians,
    leader_trigger_to_gripper_radians,
    order_like,
    parse_present_position_response,
    read_leader_pose_rad,
    slew_toward,
)


def mock_start_positions():
    """The pose so101_base_config's config/initial_positions.yaml puts the mock hardware in."""
    path = (
        Path(get_package_share_directory("so101_base_config"))
        / "config"
        / "initial_positions.yaml"
    )
    initial = yaml.safe_load(path.read_text())["initial_positions"]
    return [initial[name] for name in JOINT_NAMES]


def test_order_like_reorders_by_name():
    shuffled = list(reversed(JOINT_NAMES))
    values = [float(i) for i in range(len(shuffled))]
    ordered = order_like(shuffled, values)
    assert ordered == list(reversed(values))
    assert order_like(JOINT_NAMES, values) == values


def test_order_like_rejects_a_missing_joint():
    with pytest.raises(KeyError):
        order_like(JOINT_NAMES[:-1] + ["jaw"], [0.0] * len(JOINT_NAMES))
    with pytest.raises(ValueError):
        order_like(JOINT_NAMES, [0.0])


def test_fake_positions_wiggle_about_the_measured_center():
    """The wiggle stays within one amplitude of wherever the arm was."""
    period = 12.0
    center = mock_start_positions()
    moved = [False] * len(JOINT_NAMES)
    for step in range(241):
        positions = fake_positions(step * period / 240.0, period, center)
        for index, name in enumerate(JOINT_NAMES):
            offset = positions[index] - center[index]
            assert abs(offset) <= WIGGLE_AMPLITUDE_RAD + 1e-9, name
            # A quarter amplitude is a "this joint is clearly alive" bar that
            # does not pin the test to the exact phase geometry; the smallest
            # per-joint peak is half an amplitude.
            if abs(offset) > WIGGLE_AMPLITUDE_RAD / 4.0:
                moved[index] = True
    assert all(moved), "every joint should visibly move over one sine period"


def test_fake_positions_start_exactly_at_the_measured_center():
    """The first published point must BE the measured pose, not merely near it.

    A plain sin(wt + phase) starts a whole amplitude away for every joint whose
    phase offset is not zero, which on a powered arm is a snap rather than a
    wiggle. Equality here is the no-jump guarantee; "within an amplitude" is
    not good enough and silently permitted the bug this test now pins.
    """
    center = mock_start_positions()
    assert fake_positions(0.0, 12.0, center) == pytest.approx(center, abs=1e-12)


def test_fake_positions_start_at_center_for_any_period_and_amplitude():
    center = [0.3] * len(JOINT_NAMES)
    for period in (4.0, 12.0, 30.0):
        for amplitude in (0.01, 0.1, 0.5):
            assert fake_positions(0.0, period, center, amplitude) == pytest.approx(
                center, abs=1e-12
            )


def test_fake_positions_clamp_a_center_at_the_joint_limit():
    """A joint parked on its limit must not be commanded past it."""
    center = [JOINT_LIMITS[name][1] for name in JOINT_NAMES]
    for step in range(241):
        positions = fake_positions(step * 12.0 / 240.0, 12.0, center)
        for index, name in enumerate(JOINT_NAMES):
            lower, upper = JOINT_LIMITS[name]
            assert lower <= positions[index] <= upper, name


def test_fake_positions_reject_a_wrong_sized_center():
    with pytest.raises(ValueError):
        fake_positions(0.0, 12.0, [0.0] * (len(JOINT_NAMES) - 1))


def test_joint_limits_cover_every_joint():
    assert set(JOINT_LIMITS) == set(JOINT_NAMES)


@pytest.fixture
def ros_context():
    rclpy.init()
    yield
    rclpy.try_shutdown()


def tick_mirroring(bridge):
    """Stand in for the Mirror Objective's ~/mirror service tick."""
    return bridge.on_mirror_tick(Trigger.Request(), Trigger.Response())


def feed_joint_states(bridge, positions=None):
    """Stand in for joint_state_broadcaster, which centres the wiggle."""
    message = JointState()
    message.name = list(JOINT_NAMES)
    message.position = list(mock_start_positions() if positions is None else positions)
    bridge.on_joint_states(message)


def test_the_bridge_is_silent_until_mirroring_is_requested(ros_context):
    received = []
    bridge = So101ArmBridge(source="fake")
    # Feed a pose, so what this proves is the mirror gate rather than the
    # bridge simply not knowing where the arm is.
    feed_joint_states(bridge)
    listener = Node("test_silence_listener")
    listener.create_subscription(
        JointTrajectory,
        "/joint_trajectory_controller/joint_trajectory",
        received.append,
        10,
    )
    executor = SingleThreadedExecutor()
    executor.add_node(bridge)
    executor.add_node(listener)
    deadline = time.monotonic() + 1.0
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.02)
    executor.shutdown()
    listener.destroy_node()
    bridge.destroy_node()
    assert received == [], "mirroring must be off until the Objective asks for it"


def test_fake_mode_publishes_a_usable_trajectory(ros_context):
    received = []
    bridge = So101ArmBridge(source="fake")
    listener = Node("test_listener")
    listener.create_subscription(
        JointTrajectory,
        "/joint_trajectory_controller/joint_trajectory",
        received.append,
        10,
    )
    executor = SingleThreadedExecutor()
    executor.add_node(bridge)
    executor.add_node(listener)

    feed_joint_states(bridge)
    deadline = time.monotonic() + 5.0
    while time.monotonic() < deadline and len(received) < 2:
        tick_mirroring(bridge)
        executor.spin_once(timeout_sec=0.05)

    executor.shutdown()
    listener.destroy_node()
    bridge.destroy_node()

    assert len(received) >= 2, "the bridge should publish at its configured rate"
    message = received[0]
    assert message.joint_names == JOINT_NAMES
    assert len(message.points) == 1
    assert len(message.points[0].positions) == len(JOINT_NAMES)
    # A zero lead time would land the point in the past and the controller
    # would drop it.
    lead = message.points[0].time_from_start
    assert lead.sec + lead.nanosec * 1e-9 > 0.0


def test_bridge_yields_while_a_trajectory_goal_is_active(ros_context):
    """A live action goal must silence the topic stream, or the goal never ends.

    The controller restarts its trajectory on every topic message, so a plan's
    goal is accepted and then hangs forever while the bridge keeps publishing.
    """
    from action_msgs.msg import GoalStatus, GoalStatusArray

    received = []
    bridge = So101ArmBridge(source="fake")
    listener = Node("test_yield_listener")
    listener.create_subscription(
        JointTrajectory,
        "/joint_trajectory_controller/joint_trajectory",
        received.append,
        10,
    )
    status_publisher = listener.create_publisher(
        GoalStatusArray,
        "/joint_trajectory_controller/follow_joint_trajectory/_action/status",
        10,
    )
    executor = SingleThreadedExecutor()
    executor.add_node(bridge)
    executor.add_node(listener)
    feed_joint_states(bridge)

    def spin(seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            tick_mirroring(bridge)
            executor.spin_once(timeout_sec=0.02)

    def publish_status(status):
        message = GoalStatusArray()
        entry = GoalStatus()
        entry.status = status
        message.status_list = [entry]
        status_publisher.publish(message)

    tick_mirroring(bridge)
    publish_status(GoalStatus.STATUS_EXECUTING)
    spin(0.5)
    received.clear()
    spin(0.5)
    assert received == [], "the bridge must not publish while a goal is executing"

    # The goal moved the arm - a plan executed while the bridge yielded.
    moved_pose = [p + 0.2 for p in mock_start_positions()]
    feed_joint_states(bridge, moved_pose)

    publish_status(GoalStatus.STATUS_SUCCEEDED)
    spin(0.5)
    assert received, "the bridge must resume once the goal finishes"
    assert received[0].points[0].positions == pytest.approx(moved_pose, abs=1e-3), (
        "the first resumed trajectory must start at the post-goal pose, not "
        "wherever the wiggle was centred before the goal took over"
    )

    executor.shutdown()
    listener.destroy_node()
    bridge.destroy_node()


def test_mirroring_starts_the_wiggle_at_the_measured_pose(ros_context):
    """The twin must not snap when the Mirror Objective starts.

    The wiggle is centred on the pose measured at that moment, so however long
    the bridge has been up, the first point published after a mirror start is
    within one amplitude of where the arm actually is.
    """
    received = []
    bridge = So101ArmBridge(source="fake")
    listener = Node("test_mirror_start_listener")
    listener.create_subscription(
        JointTrajectory,
        "/joint_trajectory_controller/joint_trajectory",
        received.append,
        10,
    )
    executor = SingleThreadedExecutor()
    executor.add_node(bridge)
    executor.add_node(listener)

    # Age the node, so a start that failed to reset the clock would sit at an
    # arbitrary point of the sine rather than at the pose just measured.
    deadline = time.monotonic() + 3.0
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.02)
    assert received == []

    pose = mock_start_positions()
    feed_joint_states(bridge, pose)
    tick_mirroring(bridge)
    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline and not received:
        tick_mirroring(bridge)
        executor.spin_once(timeout_sec=0.01)

    executor.shutdown()
    listener.destroy_node()
    bridge.destroy_node()

    assert received, "mirroring should start publishing once ticked"
    first = received[0].points[0].positions
    for index, name in enumerate(JOINT_NAMES):
        assert abs(first[index] - pose[index]) <= WIGGLE_AMPLITUDE_RAD + 1e-6, name


def test_mirroring_refuses_without_a_measured_pose(ros_context):
    """No /joint_states means no centre, and commanding a guess is unsafe."""
    bridge = So101ArmBridge(source="fake")
    response = tick_mirroring(bridge)
    assert not response.success
    assert bridge.wiggle_center is None
    bridge.publish_once()  # must be a no-op rather than raise
    bridge.destroy_node()


def test_mirroring_refuses_a_stale_pose(ros_context):
    """A sample older than joint_states_timeout_s must not become the centre.

    A stale sample means the broadcaster died or the bus went quiet; centring
    on where the arm was seconds ago is how a wiggle becomes a lurch.
    """
    bridge = So101ArmBridge(source="fake")
    feed_joint_states(bridge)
    # Backdate the sample well past the default timeout.
    bridge.latest_positions_time = bridge.get_clock().now() - Duration(
        seconds=int(bridge.joint_states_timeout_s) + 5
    )
    response = tick_mirroring(bridge)
    assert not response.success
    assert "stale" in response.message
    assert bridge.wiggle_center is None
    bridge.destroy_node()


def test_mirroring_recenters_on_a_later_start(ros_context):
    """A second run centres on wherever the arm ended up, not the first pose."""
    bridge = So101ArmBridge(source="fake")
    first_pose = mock_start_positions()
    feed_joint_states(bridge, first_pose)
    tick_mirroring(bridge)
    assert bridge.wiggle_center == pytest.approx(first_pose)

    # Let the mirror tick go stale, so the next tick counts as a fresh start.
    bridge.last_mirror_tick = None
    moved = [value + 0.3 for value in first_pose]
    feed_joint_states(bridge, moved)
    tick_mirroring(bridge)
    assert bridge.wiggle_center == pytest.approx(moved)
    bridge.destroy_node()


def encode_status_packet(servo_id, tick, error=0):
    """Build a Feetech present-position status (reply) packet by hand.

    Independent of build_read_present_position_packet (which builds the
    request, not the reply), so this is not a tautological check of the
    parser against its own encoder.
    """
    body = bytes([servo_id, 4, error, tick & 0xFF, (tick >> 8) & 0xFF])
    return b"\xff\xff" + body + bytes([(~sum(body)) & 0xFF])


def test_build_read_present_position_packet_roundtrips_with_a_status_reply():
    request = build_read_present_position_packet(3)
    assert request == bytes(
        [0xFF, 0xFF, 3, 4, 0x02, 56, 2, (~(3 + 4 + 0x02 + 56 + 2)) & 0xFF]
    )
    assert parse_present_position_response(encode_status_packet(3, 2500), 3) == 2500


def test_parse_present_position_response_rejects_a_bad_checksum():
    packet = bytearray(encode_status_packet(3, 2500))
    packet[-1] ^= 0xFF
    assert parse_present_position_response(bytes(packet), 3) is None


def test_parse_present_position_response_rejects_the_wrong_id():
    assert parse_present_position_response(encode_status_packet(3, 2500), 4) is None


def test_parse_present_position_response_rejects_a_servo_error():
    assert (
        parse_present_position_response(encode_status_packet(3, 2500, error=1), 3)
        is None
    )


def test_parse_present_position_response_rejects_a_short_packet():
    assert (
        parse_present_position_response(encode_status_packet(3, 2500)[:-1], 3) is None
    )


def test_leader_tick_to_radians_zero_is_urdf_zero():
    assert leader_tick_to_radians(2048) == pytest.approx(0.0)
    assert leader_tick_to_radians(2048 + 1024) == pytest.approx(math.pi / 2.0)


def test_leader_trigger_to_gripper_radians_spans_the_urdf_range():
    lower, upper = JOINT_LIMITS["gripper"]
    assert leader_trigger_to_gripper_radians(
        LEADER_TRIGGER_TICKS_CLOSED
    ) == pytest.approx(lower)
    assert leader_trigger_to_gripper_radians(
        LEADER_TRIGGER_TICKS_OPEN
    ) == pytest.approx(upper)


def non_blocking_read_fn(fd):
    """A pipe's blocking read would hang forever with no writer; a real tty's
    VMIN=0/VTIME>0 setup (open_leader_port) never blocks like that, so this
    mirrors that instead of the fake fd hanging the test suite."""
    os.set_blocking(fd, False)

    def read(n):
        try:
            return os.read(fd, n)
        except BlockingIOError:
            return b""

    return read


def test_leader_bus_reads_present_position_over_a_fake_file_descriptor():
    """A real fd pair (os.pipe), not a mock, stands in for the leader's tty."""
    request_r, request_w = os.pipe()
    response_r, response_w = os.pipe()
    try:
        os.write(response_w, encode_status_packet(servo_id=3, tick=2500))
        bus = LeaderBus(
            read_fn=non_blocking_read_fn(response_r),
            write_fn=lambda data: os.write(request_w, data),
        )
        assert bus.read_present_position(3) == 2500
        assert os.read(request_r, 64) == build_read_present_position_packet(3)
    finally:
        for fd in (request_r, request_w, response_r, response_w):
            os.close(fd)


def test_leader_bus_times_out_on_no_reply():
    request_r, request_w = os.pipe()
    response_r, response_w = os.pipe()
    try:
        bus = LeaderBus(
            read_fn=non_blocking_read_fn(response_r),
            write_fn=lambda data: os.write(request_w, data),
            timeout_s=0.05,
        )
        assert bus.read_present_position(3) is None
    finally:
        for fd in (request_r, request_w, response_r, response_w):
            os.close(fd)


class FakeLeaderBus:
    """A leader bus with canned per-servo ticks, for testing above the wire protocol."""

    def __init__(self, ticks_by_id):
        self.ticks_by_id = ticks_by_id

    def read_present_position(self, servo_id):
        return self.ticks_by_id.get(servo_id)


def test_read_leader_pose_rad_maps_and_clamps_every_joint():
    ticks = {index + 1: 2048 for index in range(len(JOINT_NAMES) - 1)}
    ticks[len(JOINT_NAMES)] = LEADER_TRIGGER_TICKS_CLOSED  # gripper
    positions = read_leader_pose_rad(FakeLeaderBus(ticks))
    for name, position in zip(JOINT_NAMES, positions):
        lower, upper = JOINT_LIMITS[name]
        assert lower <= position <= upper
    assert positions[JOINT_NAMES.index("gripper")] == pytest.approx(
        JOINT_LIMITS["gripper"][0]
    )


def test_read_leader_pose_rad_skips_the_whole_sample_on_one_bad_joint():
    ticks = {index + 1: 2048 for index in range(len(JOINT_NAMES))}
    del ticks[2]  # shoulder_lift's read failed
    assert read_leader_pose_rad(FakeLeaderBus(ticks)) is None


def test_slew_toward_moves_by_at_most_max_step_without_overshoot():
    assert slew_toward([0.0], [1.0], 0.3) == pytest.approx([0.3])
    assert slew_toward([0.9], [1.0], 0.3) == pytest.approx(
        [1.0]
    )  # closes the gap exactly
    assert slew_toward([1.0], [0.0], 0.3) == pytest.approx([0.7])
    assert slew_toward([0.0, 1.0], [1.0, 0.0], 0.3) == pytest.approx([0.3, 0.7])


def test_read_leader_pose_rad_applies_the_leader_offset_before_clamping():
    ticks = {index + 1: 2048 for index in range(len(JOINT_NAMES))}
    without_offset = read_leader_pose_rad(FakeLeaderBus(ticks))
    with_offset = read_leader_pose_rad(FakeLeaderBus(ticks), {"wrist_roll": 0.5})
    index = JOINT_NAMES.index("wrist_roll")
    assert with_offset[index] == pytest.approx(without_offset[index] + 0.5)
    for i in range(len(JOINT_NAMES)):
        if i != index:
            assert with_offset[i] == pytest.approx(without_offset[i])


def test_real_source_mirrors_the_leader(ros_context):
    """With a leader_bus injected, --real publishes the leader's mapped pose.

    This checks the bridge's plumbing (poll -> latest_leader_positions ->
    slew -> publish), not the mapping math - covered by the
    read_leader_pose_rad tests above - so the follower is started already at
    the (offset-applied) target: with zero initial error, one slew step
    lands exactly on it, regardless of the configured slew rate.
    """
    received = []
    ticks = {index + 1: 2048 for index in range(len(JOINT_NAMES))}
    offset = dict(zip(JOINT_NAMES, LEADER_OFFSET_RAD_DEFAULT))
    expected = read_leader_pose_rad(FakeLeaderBus(ticks), offset)
    bridge = So101ArmBridge(source="real", leader_bus=FakeLeaderBus(ticks))
    listener = Node("test_real_listener")
    listener.create_subscription(
        JointTrajectory,
        "/joint_trajectory_controller/joint_trajectory",
        received.append,
        10,
    )
    executor = SingleThreadedExecutor()
    executor.add_node(bridge)
    executor.add_node(listener)

    feed_joint_states(bridge, expected)
    bridge.poll_leader()
    deadline = time.monotonic() + 3.0
    while time.monotonic() < deadline and not received:
        tick_mirroring(bridge)
        executor.spin_once(timeout_sec=0.02)

    executor.shutdown()
    listener.destroy_node()
    bridge.destroy_node()

    assert received, "real mode should publish once the leader has been polled"
    assert received[0].points[0].positions == pytest.approx(expected)


def test_real_source_glides_instead_of_jumping_to_the_leader_pose(ros_context):
    """One read_positions() call moves at most real_slew_max_step_rad per joint."""
    ticks = {index + 1: 2048 for index in range(len(JOINT_NAMES))}
    bridge = So101ArmBridge(source="real", leader_bus=FakeLeaderBus(ticks))
    start_pose = [0.5] * len(JOINT_NAMES)
    feed_joint_states(bridge, start_pose)
    tick_mirroring(bridge)
    bridge.poll_leader()
    target = bridge.latest_leader_positions
    assert all(
        abs(t - s) > bridge.real_slew_max_step_rad for s, t in zip(start_pose, target)
    ), "test needs every joint further from start than one slew step"

    first = bridge.read_positions()
    for s, t, f in zip(start_pose, target, first):
        assert f == pytest.approx(
            s + math.copysign(bridge.real_slew_max_step_rad, t - s)
        )

    for _ in range(500):  # far more steps than needed to close any of the gaps above
        bridge.read_positions()
    assert bridge.read_positions() == pytest.approx(target)
    bridge.destroy_node()


def test_real_source_publishes_nothing_before_the_first_leader_read(ros_context):
    bridge = So101ArmBridge(source="real", leader_bus=FakeLeaderBus({}))
    feed_joint_states(bridge)
    tick_mirroring(bridge)
    assert bridge.read_positions() is None
    bridge.publish_once()  # must be a no-op rather than raise
    bridge.destroy_node()


def test_poll_leader_keeps_the_last_good_target_on_a_failed_read(ros_context):
    good_ticks = {index + 1: 2048 for index in range(len(JOINT_NAMES))}
    bridge = So101ArmBridge(source="real", leader_bus=FakeLeaderBus(good_ticks))
    bridge.poll_leader()
    first_target = bridge.latest_leader_positions
    assert first_target is not None

    bridge.leader_bus = FakeLeaderBus({})  # every joint now fails to read
    bridge.poll_leader()
    assert bridge.latest_leader_positions == pytest.approx(first_target)
    bridge.destroy_node()


def test_poll_leader_stops_motion_once_reads_have_been_stale_too_long(ros_context):
    """A single failed read keeps slewing toward the last good sample, but a
    leader that stays unreadable past leader_timeout_s must stop the
    follower outright rather than keep chasing a stale target forever."""
    good_ticks = {index + 1: 2048 for index in range(len(JOINT_NAMES))}
    bridge = So101ArmBridge(source="real", leader_bus=FakeLeaderBus(good_ticks))
    bridge.leader_timeout_s = 0.05
    feed_joint_states(bridge)
    tick_mirroring(bridge)
    bridge.poll_leader()
    assert bridge.read_positions() is not None
    assert bridge.leader_stale is False

    bridge.leader_bus = FakeLeaderBus({})  # every joint now fails to read
    time.sleep(0.1)
    bridge.poll_leader()
    assert bridge.leader_stale is True
    assert bridge.read_positions() is None
    bridge.publish_once()  # must be a no-op, not publish the stale target

    bridge.leader_bus = FakeLeaderBus(good_ticks)  # leader recovers
    bridge.poll_leader()
    assert bridge.leader_stale is False
    assert bridge.read_positions() is not None
    bridge.destroy_node()


def test_poll_leader_survives_a_bus_os_error_after_motion_has_started(ros_context):
    """An unplugged port (or a leader servo fault that knocks the bus out)
    raises OSError from the raw fd read/write once motion is already
    underway. That must not crash poll_leader(), and it must count toward
    leader_timeout_s the same as a timed-out or bad-checksum reply."""
    good_ticks = {index + 1: 2048 for index in range(len(JOINT_NAMES))}
    bridge = So101ArmBridge(source="real", leader_bus=FakeLeaderBus(good_ticks))
    bridge.leader_timeout_s = 0.05
    feed_joint_states(bridge)
    tick_mirroring(bridge)
    bridge.poll_leader()
    assert bridge.latest_leader_positions is not None

    def raising_write(data):
        raise OSError(6, "no such device or address")

    bridge.leader_bus = LeaderBus(read_fn=lambda n: b"", write_fn=raising_write)
    time.sleep(0.1)
    bridge.poll_leader()  # must not raise
    assert bridge.leader_stale is True
    assert bridge.read_positions() is None
    bridge.destroy_node()


def test_leader_bus_read_present_position_returns_none_on_a_write_os_error():
    """An unplugged port raises OSError on write; the bus must treat that
    like any other bad reply rather than let it escape to the caller."""

    def raising_write(data):
        raise OSError(6, "no such device or address")

    bus = LeaderBus(read_fn=lambda n: b"", write_fn=raising_write)
    assert bus.read_present_position(3) is None


def test_leader_bus_read_present_position_returns_none_on_a_read_os_error():
    request_r, request_w = os.pipe()
    try:

        def raising_read(n):
            raise OSError(5, "input/output error")

        bus = LeaderBus(
            read_fn=raising_read,
            write_fn=lambda data: os.write(request_w, data),
        )
        assert bus.read_present_position(3) is None
    finally:
        os.close(request_r)
        os.close(request_w)
