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

"""Publish Nav2's /odom from the mock base's planar joint states.

    ros2 run mobile_fr3_duo_mock joint_state_odometry.py

Mock hardware moves the base exactly as commanded, so the planar joints (planar_x, planar_y,
planar_theta) are the true base pose in odom. This node turns them into nav_msgs/Odometry:
the pose from the joint positions, and the twist rotated into base_link from the joint velocities.
It keeps the latest planar state and publishes it at a fixed rate (odom_rate_hz), stamped with
that joint state's time. robot_state_publisher already publishes odom -> base_link from the same
joints, so this node publishes no TF.

"""

import math

JOINTS = ("planar_x", "planar_y", "planar_theta")


def planar_odometry(positions, velocities):
    """Pose (x, y, yaw) and base-frame twist (vx, vy, wz) from planar joint positions and velocities."""
    x, y, yaw = positions
    vx_world, vy_world, wz = velocities
    cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
    return (x, y, yaw), (
        vx_world * cos_yaw + vy_world * sin_yaw,
        -vx_world * sin_yaw + vy_world * cos_yaw,
        wz,
    )


def main():
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from sensor_msgs.msg import JointState

    class JointStateOdometry(Node):
        def __init__(self):
            super().__init__("joint_state_odometry")
            p = self.declare_parameter
            self._odom_frame = p("odom_frame_id", "odom").value
            self._base_frame = p("base_frame_id", "base_link").value
            self._pub = self.create_publisher(
                Odometry, p("odom_topic", "/odom").value, 10
            )
            self._state = None
            self.create_subscription(
                JointState, "/joint_states", self._on_joint_states, 10
            )
            self.create_timer(1.0 / float(p("odom_rate_hz", 50.0).value), self._tick)

        def _on_joint_states(self, msg):
            index = {name: i for i, name in enumerate(msg.name)}
            if not all(j in index for j in JOINTS):
                return
            positions = [msg.position[index[j]] for j in JOINTS]
            velocities = [
                msg.velocity[index[j]] if len(msg.velocity) > index[j] else 0.0
                for j in JOINTS
            ]
            self._state = (msg.header.stamp, positions, velocities)

        def _tick(self):
            if self._state is None:
                return
            stamp, positions, velocities = self._state
            (x, y, yaw), (vx, vy, wz) = planar_odometry(positions, velocities)
            odom = Odometry()
            odom.header.stamp = stamp
            odom.header.frame_id = self._odom_frame
            odom.child_frame_id = self._base_frame
            odom.pose.pose.position.x = x
            odom.pose.pose.position.y = y
            odom.pose.pose.orientation.z = math.sin(yaw / 2.0)
            odom.pose.pose.orientation.w = math.cos(yaw / 2.0)
            odom.twist.twist.linear.x = vx
            odom.twist.twist.linear.y = vy
            odom.twist.twist.angular.z = wz
            self._pub.publish(odom)

    rclpy.init()
    node = JointStateOdometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
