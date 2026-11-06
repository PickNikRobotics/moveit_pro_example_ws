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

"""Convert body-frame velocity to the Duo simulation's world-axis planar actuators.

The private planar encoder heading supplies the actuator coordinate conversion, not navigation localization. Nav2 sends Twist on /nav2_cmd_vel; external clients send TwistStamped in base_link on /cmd_vel. The undock Objective sends a velocity-only VelocityForceCommand on /market_sim/undock_velocity_command. All three inputs share the readiness, timestamp and encoder watchdog. No ground-truth pose topic is consumed.
"""

import math

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from moveit_pro_controllers_msgs.msg import VelocityForceCommand
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64MultiArray


def planar_velocities(vx, vy, wz, yaw):
    """World-axis actuator velocities for a finite body-frame twist and heading."""
    if not all(math.isfinite(value) for value in (vx, vy, wz, yaw)):
        raise ValueError(
            "Cannot convert a non-finite base velocity or encoder heading."
        )
    c, s = math.cos(yaw), math.sin(yaw)
    return [vx * c - vy * s, vx * s + vy * c, wz]


def safe_command(
    twist, yaw, command_age, feedback_age, timeout, max_linear, max_angular
):
    """Stop on missing/stale data; bound planar speed before rotating into actuator axes."""
    if (
        twist is None
        or yaw is None
        or not 0 <= command_age <= timeout
        or not 0 <= feedback_age <= timeout
    ):
        return [0.0, 0.0, 0.0]
    vx, vy, wz = twist.linear.x, twist.linear.y, twist.angular.z
    if not all(math.isfinite(v) for v in (vx, vy, wz, yaw)) or any(
        value != 0.0 for value in (twist.linear.z, twist.angular.x, twist.angular.y)
    ):
        return [0.0, 0.0, 0.0]
    scale = max(1.0, math.hypot(vx, vy) / max_linear)
    return planar_velocities(
        vx / scale, vy / scale, max(-max_angular, min(max_angular, wz)), yaw
    )


class BaseTwistToPlanar(Node):
    """Fail closed when a velocity command cannot be placed in the actuator frame."""

    def __init__(self, **kwargs):
        super().__init__("base_twist_to_planar", **kwargs)
        self._timeout = float(self.declare_parameter("twist_timeout_sec", 0.5).value)
        # Launch supplies the limits from the robot's base_jvc controller configuration.
        self._max_linear = float(
            self.declare_parameter("max_linear_velocity", 0.0).value
        )
        self._max_angular = float(
            self.declare_parameter("max_angular_velocity", 0.0).value
        )
        if not all(
            math.isfinite(v) and v > 0
            for v in (self._timeout, self._max_linear, self._max_angular)
        ):
            raise ValueError(
                "Base command limits and twist_timeout_sec must be finite and positive; check the market_sim driver launch parameters."
            )
        self._twist = None
        self._yaw = None
        self._command_at = self._feedback_at = self._ready_at = 0
        self._localization_ready = False
        self.create_subscription(
            Bool, "/market_sim/localization_ready", self._on_ready, 10
        )
        self.create_subscription(Twist, "/nav2_cmd_vel", self._on_twist, 10)
        self.create_subscription(TwistStamped, "/cmd_vel", self._on_stamped_twist, 10)
        self.create_subscription(
            VelocityForceCommand,
            "/market_sim/undock_velocity_command",
            self._on_undock_command,
            10,
        )
        self.create_subscription(
            JointState,
            "/base_state_broadcaster/joint_states",
            self._on_encoder,
            qos_profile_sensor_data,
        )
        self._pub = self.create_publisher(Float64MultiArray, "/base_jgvc/commands", 10)
        self.create_timer(0.02, self._tick)

    def _on_ready(self, msg):
        self._localization_ready = msg.data
        self._ready_at = self.get_clock().now().nanoseconds
        if not msg.data:
            self._twist = None

    def _on_twist(self, msg):
        self._twist = msg
        self._command_at = self.get_clock().now().nanoseconds

    def _on_stamped_twist(self, msg):
        if msg.header.frame_id != "base_link":
            self._twist = None
            self.get_logger().error(
                f"Rejected /cmd_vel frame '{msg.header.frame_id}'; publish the Duo twist in base_link."
            )
            return
        self._twist = msg.twist
        self._command_at = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec

    def _on_undock_command(self, msg):
        axes = ("x", "y", "z", "rx", "ry", "rz")
        selected = tuple(getattr(msg.velocity_controlled_axes, axis) for axis in axes)
        rotation = msg.control_frame_offset.rotation
        legacy_rotation = msg.control_frame.rotation
        wrench = (
            msg.wrench.force.x,
            msg.wrench.force.y,
            msg.wrench.force.z,
            msg.wrench.torque.x,
            msg.wrench.torque.y,
            msg.wrench.torque.z,
        )
        if (
            msg.control_frame_id != "base_link"
            or selected != (True, True, False, False, False, True)
            or any(getattr(msg.force_controlled_axes, axis) for axis in axes)
            or any(value != 0.0 for value in wrench)
            or msg.wrench_gain != 0.0
            or msg.nullspace_component
            or any(value != 0.0 for value in (rotation.x, rotation.y, rotation.z))
            or rotation.w not in (0.0, 1.0)
            or any(
                value != 0.0
                for value in (
                    legacy_rotation.x,
                    legacy_rotation.y,
                    legacy_rotation.z,
                    msg.control_frame_offset.translation.x,
                    msg.control_frame_offset.translation.y,
                    msg.control_frame_offset.translation.z,
                    msg.control_frame.translation.x,
                    msg.control_frame.translation.y,
                    msg.control_frame.translation.z,
                )
            )
            or legacy_rotation.w not in (0.0, 1.0)
        ):
            self._twist = None
            self.get_logger().error(
                "Rejected /market_sim/undock_velocity_command: requires base_link, unrotated planar velocity axes and no force/nullspace control; check Undock From D4 ports."
            )
            return
        self._twist = msg.twist
        self._command_at = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec

    def _on_encoder(self, msg):
        try:
            self._yaw = msg.position[msg.name.index("planar_theta")]
        except (ValueError, IndexError):
            self._yaw = None
            self.get_logger().error(
                "Missing planar_theta position on /base_state_broadcaster/joint_states; check its joints and interfaces."
            )
            return
        self._feedback_at = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec

    def _tick(self):
        now = self.get_clock().now().nanoseconds
        ready = (
            self._localization_ready
            and 0 <= (now - self._ready_at) / 1e9 <= self._timeout
        )
        command = safe_command(
            self._twist if ready else None,
            self._yaw,
            (now - self._command_at) / 1e9,
            (now - self._feedback_at) / 1e9,
            self._timeout,
            self._max_linear,
            self._max_angular,
        )
        self._pub.publish(Float64MultiArray(data=command))


def main():
    rclpy.init()
    node = BaseTwistToPlanar()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    if rclpy.ok():
        node._pub.publish(Float64MultiArray(data=[0.0, 0.0, 0.0]))
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
