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

"""Turn Nav2's base-frame velocity command into world-axis planar joint velocities.

    ros2 run mobile_fr3_duo_mock base_twist_to_planar.py

The base moves through velocity commands on world-axis slides (planar_x, planar_y) and a
hinge (planar_theta), while /cmd_vel is in base_link:

    planar_x_vel = vx * cos(yaw) - vy * sin(yaw)
    planar_y_vel = vx * sin(yaw) + vy * cos(yaw)
    planar_theta_vel = wz

The yaw is the base's TRUE heading: the simulator's ground-truth odometry in mobile_fr3_duo_sim,
not the drifted planar joint values, and the exact /odom on mock hardware. A real base executes
a body-frame command in its true body frame.
The velocity controller holds its last command, so a quiet /cmd_vel sends zeros.

"""

import math

JOINTS = ("planar_x", "planar_y", "planar_theta")


def planar_velocities(vx, vy, wz, yaw):
    """World-axis planar joint velocities for a base-frame twist at heading yaw."""
    cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
    return [vx * cos_yaw - vy * sin_yaw, vx * sin_yaw + vy * cos_yaw, wz]


def main():
    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from std_msgs.msg import Float64MultiArray

    class BaseTwistToPlanar(Node):
        def __init__(self):
            super().__init__("base_twist_to_planar")
            p = self.declare_parameter
            self._timeout = float(p("twist_timeout_sec", 0.5).value)
            self._twist = None
            self._twist_at = None
            self._yaw = None
            self.create_subscription(
                Twist, p("twist_topic", "/cmd_vel").value, self._on_twist, 10
            )
            self.create_subscription(
                Odometry,
                p("ground_truth_topic", "/ground_truth/odom").value,
                self._on_truth,
                qos_profile_sensor_data,
            )
            self._pub = self.create_publisher(
                Float64MultiArray,
                p("command_topic", "/base_jgvc/commands").value,
                10,
            )
            self.create_timer(1.0 / float(p("command_rate_hz", 50.0).value), self._tick)

        def _on_twist(self, msg):
            self._twist = msg
            self._twist_at = self.get_clock().now().nanoseconds

        def _on_truth(self, msg):
            q = msg.pose.pose.orientation
            self._yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            )

        def _tick(self):
            stale = (
                self._twist is None
                or (self.get_clock().now().nanoseconds - self._twist_at) / 1e9
                > self._timeout
            )
            command = Float64MultiArray()
            if stale or self._yaw is None:
                command.data = [0.0, 0.0, 0.0]
            else:
                command.data = planar_velocities(
                    self._twist.linear.x,
                    self._twist.linear.y,
                    self._twist.angular.z,
                    self._yaw,
                )
            self._pub.publish(command)

    rclpy.init()
    node = BaseTwistToPlanar()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
