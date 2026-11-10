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

"""Bridge the Duo's planar actuator feedback to Fuse and its fused pose to MoveIt.

The synchro drive is simulated as three planar actuators, not driven wheels. Their private joint-state topic supplies encoder odometry; no ground-truth pose topic is read. Position differences use message time so a slow simulation does not inflate travel. Only /odom_filtered supplies the planar joints on /joint_states, where robot_state_publisher owns odom -> base_link.
"""

import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState

JOINT_NAMES = ("planar_x", "planar_y", "planar_theta")
# A reset step exceeds normal travel between the 50 Hz encoder samples.
TELEPORT_DISTANCE = 0.5
TELEPORT_ANGLE = 0.5


def is_jump(previous, current, max_step=TELEPORT_DISTANCE, max_turn=TELEPORT_ANGLE):
    """Detect a planar reset while treating the heading wrap as a continuous turn."""
    turn = math.atan2(
        math.sin(current[2] - previous[2]), math.cos(current[2] - previous[2])
    )
    return (
        math.hypot(current[0] - previous[0], current[1] - previous[1]) > max_step
        or abs(turn) > max_turn
    )


def yaw_from_quaternion(q):
    """Yaw of a finite, nonzero geometry_msgs Quaternion, in [-pi, pi]."""
    values = (q.x, q.y, q.z, q.w)
    norm = math.hypot(*values)
    if not math.isfinite(norm) or norm == 0:
        raise ValueError("Odometry orientation requires a finite, nonzero quaternion.")
    x, y, z, w = (value / norm for value in values)
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def twist_in_base(vx, vy, yaw):
    """Rotate world-axis planar velocity into the body frame."""
    c, s = math.cos(yaw), math.sin(yaw)
    return c * vx + s * vy, -s * vx + c * vy


def encoder_odometry(previous, current):
    """Convert consecutive encoder joint states to odometry; reject invalid time or state."""
    indices = [current.name.index(name) for name in JOINT_NAMES]
    values = [current.position[i] for i in indices]
    old_values = [previous.position[previous.name.index(name)] for name in JOINT_NAMES]
    before, now = previous.header.stamp, current.header.stamp
    dt = now.sec - before.sec + (now.nanosec - before.nanosec) * 1e-9
    if dt <= 0 or not all(math.isfinite(v) for v in values + old_values):
        raise ValueError(
            "Planar encoder samples require finite positions and increasing timestamps."
        )
    if is_jump(old_values, values):
        raise ValueError(
            "Planar encoder teleport interval discarded; wait for reset localization synchronization."
        )
    x, y, yaw = values
    px, py, pyaw = old_values
    turn = math.atan2(math.sin(yaw - pyaw), math.cos(yaw - pyaw))
    vx, vy = twist_in_base((x - px) / dt, (y - py) / dt, pyaw + turn / 2)
    odom = Odometry()
    odom.header.stamp = current.header.stamp
    odom.header.frame_id = "odom"
    odom.child_frame_id = "base_link"
    odom.pose.pose.position.x = x
    odom.pose.pose.position.y = y
    odom.pose.pose.orientation.z = math.sin(yaw / 2)
    odom.pose.pose.orientation.w = math.cos(yaw / 2)
    odom.twist.twist.linear.x = vx
    odom.twist.twist.linear.y = vy
    odom.twist.twist.angular.z = turn / dt
    for index in (0, 7, 35):
        odom.pose.covariance[index] = 0.01
        odom.twist.covariance[index] = 0.001
    return odom


def fused_joint_state(odom):
    """Represent fused planar pose and world-axis velocity as MoveIt joint state."""
    if odom.header.frame_id != "odom" or odom.child_frame_id != "base_link":
        raise ValueError(
            "Fused odometry must describe odom -> base_link; check Fuse publisher frame parameters."
        )
    yaw = yaw_from_quaternion(odom.pose.pose.orientation)
    c, s = math.cos(yaw), math.sin(yaw)
    twist = odom.twist.twist
    if not all(
        math.isfinite(value)
        for value in (
            odom.pose.pose.position.x,
            odom.pose.pose.position.y,
            twist.linear.x,
            twist.linear.y,
            twist.angular.z,
        )
    ):
        raise ValueError(
            "Fused odometry contains non-finite planar pose or velocity; check state_estimator inputs."
        )
    state = JointState()
    state.header.stamp = odom.header.stamp
    state.name = list(JOINT_NAMES)
    state.position = [odom.pose.pose.position.x, odom.pose.pose.position.y, yaw]
    state.velocity = [
        c * twist.linear.x - s * twist.linear.y,
        s * twist.linear.x + c * twist.linear.y,
        twist.angular.z,
    ]
    return state


class OdometryJointStatePublisher(Node):
    """Keep raw actuator feedback separate from the fused planning-state stream."""

    def __init__(self):
        super().__init__("odometry_joint_state_publisher")
        self._previous = None
        self._period = float(self.declare_parameter("sample_period", 0.02).value)
        self._odom = self.create_publisher(Odometry, "/odom", 10)
        self._joints = self.create_publisher(JointState, "/joint_states", 10)
        self.create_subscription(
            JointState,
            "/base_state_broadcaster/joint_states",
            self._on_encoder,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Odometry,
            "/odom_filtered",
            self._on_filtered,
            qos_profile_sensor_data,
        )

    def _on_encoder(self, msg):
        if self._previous is None:
            self._previous = msg
            return
        before = self._previous.header.stamp
        now = msg.header.stamp
        elapsed = now.sec - before.sec + (now.nanosec - before.nanosec) * 1e-9
        if 0 <= elapsed < self._period:
            return
        try:
            odom = encoder_odometry(self._previous, msg)
        except (ValueError, IndexError) as error:
            self.get_logger().error(
                f"Cannot publish planar odometry: {error} Check base_state_broadcaster joints and clock."
            )
        else:
            self._odom.publish(odom)
        self._previous = msg

    def _on_filtered(self, msg):
        try:
            state = fused_joint_state(msg)
        except ValueError as error:
            self.get_logger().error(
                f"Cannot publish fused planar joint state: {error}",
                throttle_duration_sec=5.0,
            )
            return
        self._joints.publish(state)


def main():
    rclpy.init()
    node = OdometryJointStatePublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
