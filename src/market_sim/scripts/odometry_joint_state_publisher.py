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

"""Publish the planar base joints and /odom from simulated odometry, with drift added.

    ros2 run market_sim odometry_joint_state_publisher.py

The robot description is rooted at ``odom``, so these joint values ARE ``odom -> base_link``,
which robot_state_publisher publishes; this node is their only publisher. The input is the
simulator's ground-truth base pose. Drift grows with distance driven and angle turned, never
with time, so AMCL has a real correction to make and a parked base stays still. The same
drifted pose goes to Nav2 on /odom, with the twist expressed in ``base_link``.

"""

import math
import random

JOINT_NAMES = ("planar_x", "planar_y", "planar_theta")
TURN = 2.0 * math.pi
# Increments below these are applied exactly, so contact jitter never accumulates drift.
MIN_NOISY_STEP_M = 1.0e-3
MIN_NOISY_STEP_RAD = 1.0e-3


def wrap(angle):
    """Wrap an angle into [-pi, pi)."""
    return (angle + math.pi) % TURN - math.pi


class NoisyOdometry:
    """Integrates true pose increments into a pose that drifts like wheel odometry.

    Each increment is taken in the previous true body frame and applied in the current
    drifted one, so a heading error bends every later translation, as on a real base.
    Variances are linear in motion, so the error does not depend on the message rate.
    """

    def __init__(
        self,
        distance_error_at_10m,
        heading_error_at_10m,
        heading_error_per_turn,
        teleport_distance,
        teleport_angle,
        noise_enabled=True,
        seed=0,
    ):
        """@param distance_error_at_10m 1-sigma error along the path after 10 m, metres.
        @param heading_error_at_10m 1-sigma heading error after 10 m of travel, radians.
        @param heading_error_per_turn 1-sigma heading error per full turn, radians.
        @param teleport_distance Step above which the input is treated as a jump.
        @param teleport_angle Heading step above which the input is treated as a jump.
        @param noise_enabled False passes the input through unchanged.
        @param seed Seed of the noise stream, for repeatable runs.
        """
        self._var_distance_per_m = distance_error_at_10m**2 / 10.0
        self._var_heading_per_m = heading_error_at_10m**2 / 10.0
        self._var_heading_per_rad = heading_error_per_turn**2 / TURN
        self._teleport_distance = teleport_distance
        self._teleport_angle = teleport_angle
        self._noise_enabled = noise_enabled
        self._rng = random.Random(seed)
        self._truth = None
        self._pose = None

    def update(self, x, y, yaw):
        """Feed one true pose; return the drifted pose and whether the input jumped."""
        if self._truth is None:
            self._truth = self._pose = (x, y, wrap(yaw))
            return self._pose, False

        px, py, pyaw = self._truth
        cos_p, sin_p = math.cos(pyaw), math.sin(pyaw)
        step_x = cos_p * (x - px) + sin_p * (y - py)
        step_y = -sin_p * (x - px) + cos_p * (y - py)
        step_yaw = wrap(yaw - pyaw)
        self._truth = (x, y, wrap(yaw))

        distance = math.hypot(step_x, step_y)
        # A keyframe reset moves the base faster than it can drive: restart at the new pose.
        if distance > self._teleport_distance or abs(step_yaw) > self._teleport_angle:
            self._pose = self._truth
            return self._pose, True

        moving = distance > MIN_NOISY_STEP_M or abs(step_yaw) > MIN_NOISY_STEP_RAD
        if self._noise_enabled and moving:
            along = self._rng.gauss(0.0, math.sqrt(self._var_distance_per_m * distance))
            if distance > 0.0:
                step_x += along * step_x / distance
                step_y += along * step_y / distance
            step_yaw += self._rng.gauss(
                0.0,
                math.sqrt(
                    self._var_heading_per_m * distance
                    + self._var_heading_per_rad * abs(step_yaw)
                ),
            )

        nx, ny, nyaw = self._pose
        cos_n, sin_n = math.cos(nyaw), math.sin(nyaw)
        self._pose = (
            nx + cos_n * step_x - sin_n * step_y,
            ny + sin_n * step_x + cos_n * step_y,
            wrap(nyaw + step_yaw),
        )
        return self._pose, False


def yaw_from_quaternion(q):
    """Yaw of a geometry_msgs Quaternion, in [-pi, pi]."""
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    )


def twist_in_base(vx, vy, yaw):
    """Rotate a world-axis planar velocity into the base frame at heading yaw."""
    cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
    return vx * cos_yaw + vy * sin_yaw, -vx * sin_yaw + vy * cos_yaw


def main():
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import JointState

    class OdometryJointStatePublisher(Node):
        def __init__(self):
            super().__init__("odometry_joint_state_publisher")
            p = self.declare_parameter
            truth_topic = p("ground_truth_topic", "/ground_truth/odom").value
            self._odom_frame = p("odom_frame", "odom").value
            self._base_frame = p("base_frame", "base_link").value
            self._model = NoisyOdometry(
                distance_error_at_10m=p("distance_error_at_10m", 0.05).value,
                heading_error_at_10m=p("heading_error_at_10m", 0.0175).value,
                heading_error_per_turn=p("heading_error_per_turn", 0.0175).value,
                teleport_distance=p("teleport_distance", 0.5).value,
                teleport_angle=p("teleport_angle", 0.5).value,
                noise_enabled=p("noise_enabled", True).value,
                seed=p("seed", 0).value,
            )
            self._joints = self.create_publisher(JointState, "/joint_states", 10)
            self._odom = self.create_publisher(
                Odometry, p("odom_topic", "/odom").value, 10
            )
            self.create_subscription(
                Odometry, truth_topic, self._on_truth, qos_profile_sensor_data
            )

        def _on_truth(self, truth):
            pose = truth.pose.pose
            true_yaw = yaw_from_quaternion(pose.orientation)
            (x, y, yaw), jumped = self._model.update(
                pose.position.x, pose.position.y, true_yaw
            )
            if jumped:
                self.get_logger().info("Odometry jumped; drift reset to the new pose.")
            # One message per input, moving or not: MoveIt judges state freshness by the stamp.
            joints = JointState()
            joints.header.stamp = truth.header.stamp
            joints.name = list(JOINT_NAMES)
            joints.position = [x, y, yaw]
            self._joints.publish(joints)

            odom = Odometry()
            odom.header.stamp = truth.header.stamp
            odom.header.frame_id = self._odom_frame
            odom.child_frame_id = self._base_frame
            odom.pose.pose.position.x = x
            odom.pose.pose.position.y = y
            odom.pose.pose.orientation.z = math.sin(yaw / 2.0)
            odom.pose.pose.orientation.w = math.cos(yaw / 2.0)
            # The simulator reports the twist along world axes; Nav2 reads it in base_link.
            linear = truth.twist.twist.linear
            vx, vy = twist_in_base(linear.x, linear.y, true_yaw)
            odom.twist.twist.linear.x = vx
            odom.twist.twist.linear.y = vy
            odom.twist.twist.angular.z = truth.twist.twist.angular.z
            self._odom.publish(odom)

    rclpy.init()
    node = OdometryJointStatePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
