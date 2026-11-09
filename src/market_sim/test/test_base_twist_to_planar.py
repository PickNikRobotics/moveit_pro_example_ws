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

"""Duo velocity conversion and fail-closed command callbacks using real ROS messages."""

import math
import sys
import time
from pathlib import Path

import yaml

import pytest
import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from moveit_pro_controllers_msgs.msg import VelocityForceCommand
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64MultiArray
from std_srvs.srv import Trigger

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

from base_twist_to_planar import (
    BaseTwistToPlanar,
    planar_velocities,
    safe_command,
)  # noqa: E402


def twist(vx=0.0, vy=0.0, wz=0.0):
    """Construct a body-frame command with no unsupported axes."""
    msg = Twist()
    msg.linear.x, msg.linear.y, msg.angular.z = vx, vy, wz
    return msg


@pytest.mark.parametrize("value", [math.nan, math.inf, -math.inf])
def test_nonfinite_conversion_is_rejected(value) -> None:
    """Invalid commanded velocity or heading cannot reach an actuator."""
    with pytest.raises(ValueError, match="non-finite"):
        planar_velocities(value, 0.0, 0.0, 0.0)
    assert safe_command(twist(vy=value), 0.0, 0.0, 0.0, 0.5, 0.35, 0.5) == [0.0] * 3
    assert safe_command(twist(vx=0.2), value, 0.0, 0.0, 0.5, 0.35, 0.5) == [0.0] * 3


@pytest.mark.parametrize(
    "command_age, feedback_age",
    [(-1e-6, 0.0), (0.500001, 0.0), (0.0, -1e-6), (0.0, 0.500001), (math.nan, 0.0)],
)
def test_stale_or_future_command_and_feedback_stop(command_age, feedback_age) -> None:
    """Both timestamps must fall within the inclusive watchdog window."""
    assert (
        safe_command(twist(vx=0.2), 0.0, command_age, feedback_age, 0.5, 0.35, 0.5)
        == [0.0] * 3
    )


def test_missing_command_or_feedback_stops() -> None:
    """A valid command alone or feedback alone cannot produce motion."""
    assert safe_command(None, 0.0, 0.0, 0.0, 0.5, 0.35, 0.5) == [0.0] * 3
    assert safe_command(twist(vx=0.2), None, 0.0, 0.0, 0.5, 0.35, 0.5) == [0.0] * 3


def test_nonplanar_command_stops() -> None:
    """Unsupported degrees of freedom must not be silently discarded."""
    msg = twist(vx=0.2)
    msg.angular.x = 0.1
    assert safe_command(msg, 0.0, 0.0, 0.0, 0.5, 0.35, 0.5) == [0.0] * 3


@pytest.mark.parametrize("age", [0.0, 0.5 - 1e-6, 0.5])
def test_fresh_command_at_watchdog_boundary_is_accepted(age) -> None:
    """The exact watchdog boundary is accepted; later samples are rejected separately."""
    assert safe_command(twist(vx=0.2), 0.0, age, age, 0.5, 0.35, 0.5) == [0.2, 0.0, 0.0]


def test_vector_speed_and_yaw_rate_are_bounded() -> None:
    """Diagonal commands retain direction without exceeding the planar speed limit."""
    command = safe_command(twist(3.0, 4.0, -2.0), 0.0, 0.0, 0.0, 0.5, 0.35, 0.5)
    assert command == pytest.approx([0.21, 0.28, -0.5])
    assert math.hypot(*command[:2]) == pytest.approx(0.35)


def test_forward_command_follows_the_heading() -> None:
    """A body-forward command follows the raw actuator heading."""
    assert planar_velocities(0.3, 0.0, 0.0, math.pi / 2) == pytest.approx(
        [0.0, 0.3, 0.0]
    )


def test_sideways_command_is_rotated_and_turn_passes_through() -> None:
    """Lateral and yaw commands remain independent for the omnidirectional base."""
    assert planar_velocities(0.0, 0.2, 0.5, math.pi) == pytest.approx([0.0, -0.2, 0.5])


@pytest.mark.parametrize("wz", [0.0, 0.4])
@pytest.mark.parametrize("sx,sy", [(1, 1), (-1, 1), (1, -1), (-1, -1)])
def test_regression_mppi_diagonal_feasible_commands_match_applied_velocity(
    wz, sx, sy
) -> None:
    """MPPI's feasible rectangle must never request a vector that the bridge scales down."""
    package = Path(__file__).resolve().parents[1]
    params = yaml.safe_load((package / "params/nav2_params.yaml").read_text())
    controller = params["controller_server"]["ros__parameters"]["FollowPath"]
    limits = yaml.safe_load(
        (package / "config/control/mobile_fr3_duo.ros2_control.yaml").read_text()
    )["base_jvc"]["ros__parameters"]["max_joint_velocity"]
    vx = controller["vx_max"] if sx > 0 else controller["vx_min"]
    vy = sy * controller["vy_max"]
    assert math.hypot(vx, vy) <= min(limits[:2])
    smoother = params["velocity_smoother"]["ros__parameters"]
    assert smoother["max_velocity"][:2] == [controller["vx_max"], controller["vy_max"]]
    assert smoother["min_velocity"][:2] == [controller["vx_min"], -controller["vy_max"]]
    for yaw in (0.0, math.pi / 3):
        predicted = planar_velocities(vx, vy, wz, yaw)
        applied = safe_command(
            twist(vx, vy, wz), yaw, 0.0, 0.0, 0.5, min(limits[:2]), limits[2]
        )
        assert applied == pytest.approx(predicted)


def test_regression_undock_owns_competing_commands_until_release() -> None:
    """Real competing publishers cannot overwrite reverse/zero while undock owns the command."""
    rclpy.init()
    node = BaseTwistToPlanar(
        parameter_overrides=[
            Parameter("max_linear_velocity", value=0.35),
            Parameter("max_angular_velocity", value=0.5),
        ]
    )
    peer = Node("undock_competitor_test")
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(peer)
    acquire = peer.create_client(Trigger, "/market_sim/acquire_undock")
    release = peer.create_client(Trigger, "/market_sim/release_undock")
    nav = peer.create_publisher(Twist, "/nav2_cmd_vel", 10)
    stamped_pub = peer.create_publisher(TwistStamped, "/cmd_vel", 10)
    undock = peer.create_publisher(
        VelocityForceCommand, "/market_sim/undock_velocity_command", 10
    )
    feedback = peer.create_publisher(
        JointState, "/base_state_broadcaster/joint_states", qos_profile_sensor_data
    )
    ready = peer.create_publisher(Bool, "/market_sim/localization_ready", 10)
    reset_hold = peer.create_publisher(
        Bool,
        "/market_sim/reset_controller_hold",
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
    )
    output = []
    peer.create_subscription(
        Float64MultiArray,
        "/base_jgvc/commands",
        lambda msg: output.append(list(msg.data)),
        10,
    )
    velocity = -0.25
    age = 0
    frame = "base_link"
    send_undock = True
    send_external = True

    def publish():
        state = JointState(name=["planar_theta"], position=[0.0])
        state.header.stamp = peer.get_clock().now().to_msg()
        feedback.publish(state)
        ready.publish(Bool(data=True))
        reset_hold.publish(Bool(data=False))
        if send_undock:
            command = VelocityForceCommand()
            command.header.stamp = peer.get_clock().now().to_msg()
            command.header.stamp.sec -= age
            command.control_frame_id = frame
            command.velocity_controlled_axes.x = True
            command.velocity_controlled_axes.y = True
            command.velocity_controlled_axes.rz = True
            command.twist = twist(velocity)
            undock.publish(command)
        if send_external:
            nav.publish(twist(0.2))
            stamped = TwistStamped()
            stamped.header.frame_id = "base_link"
            stamped.header.stamp = peer.get_clock().now().to_msg()
            stamped.twist = twist(0.2)
            stamped_pub.publish(stamped)

    peer.create_timer(0.01, publish)

    def until(predicate):
        deadline = time.monotonic() + 2.0
        while not predicate() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.005)
        assert (
            predicate()
        ), "Undock command interface did not reach its expected observable state"

    def call(client):
        until(client.service_is_ready)
        result = client.call_async(Trigger.Request())
        until(result.done)
        return result.result()

    def holds(expected):
        until(
            lambda: len(output) >= 2
            and output[-1] == pytest.approx(expected)
            and output[-2] == pytest.approx(expected)
        )
        output.clear()
        deadline = time.monotonic() + 0.15
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.005)
        assert output and all(command == pytest.approx(expected) for command in output)

    try:
        until(
            lambda: nav.get_subscription_count() > 0
            and stamped_pub.get_subscription_count() > 0
            and undock.get_subscription_count() > 0
        )
        holds([0.2, 0.0, 0.0])
        assert call(acquire).success
        assert not call(acquire).success
        holds([-0.25, 0.0, 0.0])
        velocity = 0.0
        holds([0.0] * 3)
        frame = "map"
        velocity = -0.25
        holds([0.0] * 3)
        frame = "base_link"
        age = 2
        holds([0.0] * 3)
        send_undock = False
        # Ownership stays held when the publisher is silent, rather than yielding on watchdog expiry.
        deadline = time.monotonic() + 0.55
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.005)
        holds([0.0] * 3)
        send_external = False
        assert call(release).success
        holds([0.0] * 3)
        send_undock = True
        age = 0
        holds([0.0] * 3)
        send_external = True
        holds([0.2, 0.0, 0.0])
    finally:
        executor.shutdown()
        node.destroy_node()
        peer.destroy_node()
        rclpy.shutdown()


def test_command_callbacks_gate_reset_reject_frames_and_expire_feedback() -> None:
    """The deployed node holds zero until localization is ready and rejects unusable messages."""
    rclpy.init()
    node = BaseTwistToPlanar(
        parameter_overrides=[
            Parameter("max_linear_velocity", value=0.35),
            Parameter("max_angular_velocity", value=0.5),
        ]
    )
    peer = Node("base_command_test_peer")
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(peer)
    output = []
    peer.create_subscription(
        Float64MultiArray,
        "/base_jgvc/commands",
        lambda msg: output.append(list(msg.data)),
        10,
    )
    command_pub = peer.create_publisher(Twist, "/nav2_cmd_vel", 10)
    stamped_pub = peer.create_publisher(TwistStamped, "/cmd_vel", 10)
    undock_pub = peer.create_publisher(
        VelocityForceCommand, "/market_sim/undock_velocity_command", 10
    )
    feedback_pub = peer.create_publisher(
        JointState, "/base_state_broadcaster/joint_states", qos_profile_sensor_data
    )
    ready_pub = peer.create_publisher(Bool, "/market_sim/localization_ready", 10)
    acquire = peer.create_client(Trigger, "/market_sim/acquire_undock")
    reset_hold_pub = peer.create_publisher(
        Bool,
        "/market_sim/reset_controller_hold",
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
    )

    def exercise(
        *,
        ready,
        stamped_frame=None,
        feedback_age=0,
        missing_joint=False,
        reset_hold=False,
        undock=False,
        command_age=0,
        invalidate_undock=None,
    ):
        output.clear()
        deadline = time.monotonic() + 0.15
        while time.monotonic() < deadline:
            state = JointState()
            state.name = ["wrong_joint" if missing_joint else "planar_theta"]
            state.position = [math.pi / 2]
            state.header.stamp = peer.get_clock().now().to_msg()
            state.header.stamp.sec -= feedback_age
            feedback_pub.publish(state)
            if reset_hold is not None:
                reset_hold_pub.publish(Bool(data=reset_hold))
            if ready is not None:
                ready_pub.publish(Bool(data=ready))
            if undock:
                msg = VelocityForceCommand()
                msg.header.stamp = peer.get_clock().now().to_msg()
                msg.header.stamp.sec -= command_age
                msg.control_frame_id = "base_link"
                msg.control_frame_offset.rotation.w = 1.0
                msg.velocity_controlled_axes.x = True
                msg.velocity_controlled_axes.y = True
                msg.velocity_controlled_axes.rz = True
                msg.twist = twist(vy=0.2)
                if invalidate_undock is not None:
                    invalidate_undock(msg)
                undock_pub.publish(msg)
            elif stamped_frame is None:
                command_pub.publish(twist(vy=0.2))
            else:
                msg = TwistStamped()
                msg.header.frame_id = stamped_frame
                msg.header.stamp = peer.get_clock().now().to_msg()
                msg.twist = twist(vy=0.2)
                stamped_pub.publish(msg)
            executor.spin_once(timeout_sec=0.005)
        assert (
            output
        ), "The command timer must publish a zero command even when inputs fail"
        return output[-1]

    try:
        assert exercise(ready=None) == [0.0] * 3
        assert exercise(ready=False) == [0.0] * 3
        assert exercise(ready=True) == pytest.approx([-0.2, 0.0, 0.0])
        assert exercise(ready=True, stamped_frame="map") == [0.0] * 3
        assert exercise(ready=True, stamped_frame="base_link") == pytest.approx(
            [-0.2, 0.0, 0.0]
        )
        assert exercise(ready=True, missing_joint=True) == [0.0] * 3
        assert exercise(ready=True, feedback_age=2) == [0.0] * 3
        assert exercise(ready=True, reset_hold=True) == [0.0] * 3
        assert exercise(ready=True, reset_hold=False) == pytest.approx([-0.2, 0.0, 0.0])
        assert exercise(ready=False) == [0.0] * 3
        owned = acquire.call_async(Trigger.Request())
        deadline = time.monotonic() + 2.0
        while not owned.done() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.01)
        assert owned.done() and owned.result().success
        assert exercise(ready=False, undock=True) == [0.0] * 3
        assert exercise(ready=True, undock=True) == pytest.approx([-0.2, 0.0, 0.0])
        assert exercise(ready=True, undock=True, command_age=2) == [0.0] * 3
        assert exercise(ready=True, undock=True, command_age=-2) == [0.0] * 3
        assert exercise(ready=True, undock=True, feedback_age=2) == [0.0] * 3
        for invalidate in (
            lambda msg: setattr(msg, "control_frame_id", "map"),
            lambda msg: setattr(msg.force_controlled_axes, "x", True),
            lambda msg: setattr(msg.velocity_controlled_axes, "rz", False),
            lambda msg: setattr(msg.control_frame_offset.rotation, "z", 0.1),
            lambda msg: setattr(msg.control_frame.rotation, "x", 0.1),
            lambda msg: setattr(msg.control_frame_offset.translation, "x", 0.1),
            lambda msg: setattr(msg.wrench.force, "x", 0.1),
            lambda msg: setattr(msg, "wrench_gain", 0.1),
            lambda msg: setattr(msg, "nullspace_component", [0.1]),
            lambda msg: setattr(msg.twist.linear, "x", math.nan),
            lambda msg: setattr(msg.twist.angular, "x", 0.1),
        ):
            assert (
                exercise(ready=True, undock=True, invalidate_undock=invalidate)
                == [0.0] * 3
            )
        assert (
            exercise(
                ready=True,
                undock=True,
                invalidate_undock=lambda msg: setattr(msg, "twist", Twist()),
            )
            == [0.0] * 3
        )
        assert exercise(ready=False, undock=True) == [0.0] * 3
    finally:
        executor.shutdown()
        node.destroy_node()
        peer.destroy_node()
        rclpy.shutdown()
