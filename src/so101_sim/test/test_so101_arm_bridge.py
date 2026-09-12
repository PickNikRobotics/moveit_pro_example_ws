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
"""Check the bridge's fake source and its name/unit conversion helpers."""

from pathlib import Path
import math
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
    WIGGLE_AMPLITUDE_RAD,
    So101ArmBridge,
    fake_positions,
    order_like,
    to_radians,
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


def test_to_radians_applies_offset_then_sign():
    # 90 deg with a 10 deg offset and a flipped sign is -80 deg.
    assert to_radians([90.0], [-1.0], [10.0]) == pytest.approx([math.radians(-80.0)])
    assert to_radians([0.0] * 6, [1.0] * 6, [0.0] * 6) == [0.0] * 6


def test_to_radians_rejects_mismatched_calibration():
    with pytest.raises(ValueError):
        to_radians([0.0, 0.0], [1.0], [0.0, 0.0])


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

    publish_status(GoalStatus.STATUS_SUCCEEDED)
    spin(0.5)
    assert received, "the bridge must resume once the goal finishes"

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


def test_real_source_is_still_a_stub(ros_context):
    bridge = So101ArmBridge(source="real")
    with pytest.raises(NotImplementedError):
        bridge.read_positions()
    bridge.destroy_node()
