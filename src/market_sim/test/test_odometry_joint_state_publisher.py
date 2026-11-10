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

"""Planar actuator and fused odometry contracts using real ROS messages."""

import math
import sys
from pathlib import Path

import pytest
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

from odometry_joint_state_publisher import (  # noqa: E402
    JOINT_NAMES,
    encoder_odometry,
    fused_joint_state,
    twist_in_base,
    yaw_from_quaternion,
)


def sample(time, position):
    """Create stamped planar feedback with deliberately irrelevant qvel values."""
    msg = JointState()
    msg.header.stamp.sec = int(time)
    msg.header.stamp.nanosec = round((time - int(time)) * 1e9)
    msg.name = list(JOINT_NAMES)
    msg.position = list(position)
    msg.velocity = [100.0, 100.0, 100.0]
    return msg


@pytest.mark.parametrize("time", [0.0, -1.0])
def test_encoder_rejects_nonincreasing_time(time) -> None:
    """Duplicate and rewound stamps cannot yield usable velocities."""
    with pytest.raises(ValueError, match="increasing timestamps"):
        encoder_odometry(sample(0.0, (0.0, 0.0, 0.0)), sample(time, (0.0, 0.0, 0.0)))


@pytest.mark.parametrize("value", [math.nan, math.inf, -math.inf])
def test_encoder_rejects_nonfinite_positions(value) -> None:
    """A bad encoder must not inject invalid state into Fuse."""
    with pytest.raises(ValueError, match="finite positions"):
        encoder_odometry(sample(0.0, (0.0, 0.0, 0.0)), sample(1.0, (value, 0.0, 0.0)))


@pytest.mark.parametrize("missing", ["name", "position"])
def test_encoder_rejects_incomplete_feedback(missing) -> None:
    """All three planar position interfaces are required."""
    msg = sample(1.0, (0.0, 0.0, 0.0))
    setattr(msg, missing, list(getattr(msg, missing))[:-1])
    with pytest.raises((ValueError, IndexError)):
        encoder_odometry(sample(0.0, (0.0, 0.0, 0.0)), msg)


@pytest.mark.parametrize(
    "quaternion", [Quaternion(x=0.0, y=0.0, z=0.0, w=0.0), Quaternion(z=math.nan)]
)
def test_invalid_quaternion_is_rejected(quaternion) -> None:
    """A zero or nonfinite orientation must not become a fabricated yaw."""
    with pytest.raises(ValueError, match="finite, nonzero quaternion"):
        yaw_from_quaternion(quaternion)


def test_fused_state_rejects_incorrect_frames() -> None:
    """A pose in map or simulator truth is not a planar odom joint value."""
    msg = Odometry()
    msg.header.frame_id = "map"
    msg.child_frame_id = "base_link"
    with pytest.raises(ValueError, match="odom -> base_link"):
        fused_joint_state(msg)


def test_fused_state_rejects_nonfinite_velocity() -> None:
    """MoveIt must not receive invalid fused velocities."""
    msg = encoder_odometry(sample(0.0, (0.0, 0.0, 0.0)), sample(1.0, (0.0, 0.0, 0.0)))
    msg.twist.twist.linear.y = math.nan
    with pytest.raises(ValueError, match="non-finite"):
        fused_joint_state(msg)


@pytest.mark.parametrize(
    "position", [(0.500001, 0.0, 0.0), (0.0, 0.500001, 0.0), (0.0, 0.0, 0.500001)]
)
def test_encoder_teleport_interval_is_rejected(position) -> None:
    """A reset must not inject teleport velocity into Fuse."""
    with pytest.raises(ValueError, match="teleport interval discarded"):
        encoder_odometry(sample(0.0, (0.0, 0.0, 0.0)), sample(0.02, position))


def test_encoder_after_teleport_resumes_at_new_position() -> None:
    """The first stationary post-reset interval carries no residual teleport velocity."""
    msg = encoder_odometry(
        sample(0.02, (20.0, 3.0, 1.2)), sample(0.04, (20.0, 3.0, 1.2))
    )
    assert (
        msg.twist.twist.linear.x,
        msg.twist.twist.linear.y,
        msg.twist.twist.angular.z,
    ) == (0.0, 0.0, 0.0)


def test_stationary_encoder_remains_stationary() -> None:
    """Holding encoder position yields exactly zero velocity, without synthetic drift."""
    msg = encoder_odometry(sample(0.0, (2.0, 3.0, 0.7)), sample(1.0, (2.0, 3.0, 0.7)))
    assert (
        msg.twist.twist.linear.x,
        msg.twist.twist.linear.y,
        msg.twist.twist.angular.z,
    ) == (0.0, 0.0, 0.0)
    assert (msg.pose.pose.position.x, msg.pose.pose.position.y) == (2.0, 3.0)


@pytest.mark.parametrize("duration", [0.02, 0.05, 0.1, 1.0])
def test_encoder_distance_uses_stamped_positions_not_simulation_velocity(
    duration,
) -> None:
    """Wall-time odometry integrates to the same distance at different simulation rates."""
    current = sample(duration, (0.01, 0.0, 0.0))
    current.header.frame_id = "private_encoder_frame"
    msg = encoder_odometry(sample(0.0, (0.0, 0.0, 0.0)), current)
    assert msg.twist.twist.linear.x * duration == pytest.approx(0.01)
    assert msg.twist.twist.linear.y == 0.0
    assert msg.header.frame_id == "odom"
    assert msg.child_frame_id == "base_link"
    assert msg.header.stamp == current.header.stamp
    assert current.header.frame_id == "private_encoder_frame"


def test_encoder_order_is_resolved_by_joint_name() -> None:
    """A broadcaster can reorder interfaces without swapping x, y and yaw."""
    previous = sample(0.0, (0.0, 0.0, math.pi / 2))
    current = sample(1.0, (0.1, 0.2, math.pi / 2))
    current.name.reverse()
    current.position.reverse()
    msg = encoder_odometry(previous, current)
    assert (msg.twist.twist.linear.x, msg.twist.twist.linear.y) == pytest.approx(
        (0.2, -0.1)
    )


def test_heading_wrap_is_a_small_turn() -> None:
    """Crossing pi must not inject a full revolution into angular velocity."""
    msg = encoder_odometry(
        sample(0.0, (0.0, 0.0, math.pi - 0.01)),
        sample(0.1, (0.0, 0.0, -math.pi + 0.01)),
    )
    assert msg.twist.twist.angular.z == pytest.approx(0.2)


def test_fused_joint_state_preserves_stamp_and_rotates_lateral_velocity() -> None:
    """Body-forward and lateral velocities both reach world-axis planning joints."""
    msg = encoder_odometry(
        sample(0.0, (2.0, 3.0, math.pi / 2)), sample(1.0, (2.1, 3.2, math.pi / 2))
    )
    state = fused_joint_state(msg)
    assert state.name == list(JOINT_NAMES)
    assert state.position == pytest.approx((2.1, 3.2, math.pi / 2))
    assert state.velocity == pytest.approx((0.1, 0.2, 0.0))
    assert state.header.stamp == msg.header.stamp


def test_twist_in_base_rotates_world_velocity_into_the_body_frame() -> None:
    """Facing +y makes world +x a negative lateral velocity."""
    assert twist_in_base(0.0, 0.3, math.pi / 2) == pytest.approx((0.3, 0.0))
    assert twist_in_base(0.3, 0.0, math.pi / 2) == pytest.approx((0.0, -0.3))


def test_yaw_normalizes_quaternion_before_conversion() -> None:
    """Equivalent scaled quaternions must represent the same heading."""
    yaw = 0.7
    assert yaw_from_quaternion(
        Quaternion(z=2 * math.sin(yaw / 2), w=2 * math.cos(yaw / 2))
    ) == pytest.approx(yaw)
