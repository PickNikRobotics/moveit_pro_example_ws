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
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory

sys.path.insert(0, str(Path(__file__).parents[1] / "script"))
from so101_arm_bridge import (  # noqa: E402
    JOINT_NAMES,
    So101ArmBridge,
    fake_positions,
    order_like,
    to_radians,
)


def mock_start_positions():
    """The pose config/initial_positions.yaml puts the mock hardware in."""
    path = Path(__file__).parents[1] / "config" / "initial_positions.yaml"
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


def test_fake_positions_stay_inside_the_urdf_limits():
    limits = {
        "shoulder_pan": (-1.91986, 1.91986),
        "shoulder_lift": (-1.74533, 1.74533),
        "elbow_flex": (-1.69, 1.69),
        "wrist_flex": (-1.65806, 1.65806),
        "wrist_roll": (-2.74385, 2.84121),
        "gripper": (-0.174533, 1.74533),
    }
    period = 12.0
    moved = [False] * len(JOINT_NAMES)
    first = fake_positions(0.0, period)
    # The mock hardware is configured to start where the sine does, or the
    # twin jumps on the first published point.
    assert first == pytest.approx(mock_start_positions())
    for step in range(241):
        positions = fake_positions(step * period / 240.0, period)
        for index, name in enumerate(JOINT_NAMES):
            lower, upper = limits[name]
            assert lower <= positions[index] <= upper, name
            if abs(positions[index] - first[index]) > 0.1:
                moved[index] = True
    assert all(moved), "every joint should visibly move over one sine period"


def test_fake_positions_stay_out_of_the_self_collision_fold():
    """Guard the envelope that made MoveIt refuse to plan from the twin's state.

    Folding the wrist back over the shoulder is a self-collision, and while the
    sine sits in that region every plan from the current state is rejected. The
    fold needs shoulder_lift and elbow_flex to swing to the same side; keeping
    them on opposite sides keeps the arm reaching outward.
    """
    period = 12.0
    for step in range(241):
        positions = fake_positions(step * period / 240.0, period)
        shoulder_lift = positions[JOINT_NAMES.index("shoulder_lift")]
        elbow_flex = positions[JOINT_NAMES.index("elbow_flex")]
        assert shoulder_lift > 0.1, shoulder_lift
        assert elbow_flex < -0.1, elbow_flex


@pytest.fixture
def ros_context():
    rclpy.init()
    yield
    rclpy.try_shutdown()


def tick_mirroring(bridge):
    """Stand in for the Mirror Objective's ~/mirror service tick."""
    bridge.on_mirror_tick(Trigger.Request(), Trigger.Response())


def test_the_bridge_is_silent_until_mirroring_is_requested(ros_context):
    received = []
    bridge = So101ArmBridge(source="fake")
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


def test_mirroring_starts_the_sine_at_the_mock_start_state(ros_context):
    """The twin must not snap when the Mirror Objective starts.

    The mock hardware sits at its configured start pose until something drives
    it, so the first point published after the first ever mirror start has to be
    that same pose however long the bridge has been up before it.
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

    # Age the node up to where an unpinned sine sits near its peak, so a
    # missing reset misses the start pose by far more than the tolerance below.
    deadline = time.monotonic() + 3.0
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.02)
    assert received == []

    tick_mirroring(bridge)
    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline and not received:
        tick_mirroring(bridge)
        executor.spin_once(timeout_sec=0.01)

    executor.shutdown()
    listener.destroy_node()
    bridge.destroy_node()

    assert received, "mirroring should start publishing once ticked"
    assert received[0].points[0].positions == pytest.approx(
        mock_start_positions(), abs=0.1
    )


def test_real_source_is_still_a_stub(ros_context):
    bridge = So101ArmBridge(source="real")
    with pytest.raises(NotImplementedError):
        bridge.read_positions()
    bridge.destroy_node()
