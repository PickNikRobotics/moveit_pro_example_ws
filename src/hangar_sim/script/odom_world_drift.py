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

"""Publish odom -> world so that odom -> base equals fuse's (drifty) estimate,
while robot_state_publisher keeps world -> base as ground truth (#19667).

    odom -> world = fuse_estimate(odom->base) @ inverse(world->base_true)
makes the TF lookup odom -> base resolve to fuse's estimate; AMCL then has real
drift to correct. Replaces the static odom -> world identity when use_fuse:=true.

The drift lives in the ground plane, so everything here is 2D pose algebra
(x, y, yaw) done with the standard library -- deliberately no numpy. numpy pulls
in OpenBLAS, whose worker threads spin-wait and burned ~2 CPU cores on the sim's
high-rate topics. The ground-truth base pose is read straight from the
virtual-rail joints on /joint_states rather than a tf2 listener (the sim floods
/tf at ~700 Hz across ~140 frames; buffering that in Python was the other core).
"""

import math

import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped

# Virtual-rail joints encoding the ground-truth base pose in `world`.
RAIL_JOINTS = ("linear_x_joint", "linear_y_joint", "rotational_yaw_joint")


def yaw_of(q):
    """Yaw (rad) from a quaternion (x, y, z, w); ignores any roll/pitch."""
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    )


def invert(p):
    """Inverse of a planar pose (x, y, yaw)."""
    x, y, yaw = p
    c, s = math.cos(yaw), math.sin(yaw)
    return (-c * x - s * y, s * x - c * y, -yaw)


def compose(a, b):
    """Planar pose composition a (+) b, each (x, y, yaw)."""
    ax, ay, ayaw = a
    bx, by, byaw = b
    c, s = math.cos(ayaw), math.sin(ayaw)
    return (ax + c * bx - s * by, ay + s * bx + c * by, ayaw + byaw)


class OdomWorldDrift(Node):
    def __init__(self):
        super().__init__("odom_world_drift")
        self.br = TransformBroadcaster(self)
        self.est = None  # fuse estimate odom->base as (x, y, yaw)
        self.base = None  # ground-truth world->base as (x, y, yaw)
        self.idx = None  # cached indices of RAIL_JOINTS within /joint_states
        self.names = None  # the msg.name the cached indices were resolved against
        self.create_subscription(Odometry, "/odom_filtered", self._est, 10)
        self.create_subscription(JointState, "/joint_states", self._joints, 10)
        self.create_timer(0.02, self._pub)  # 50 Hz, keeps odom->world fresh for AMCL

    def _est(self, m):
        p = m.pose.pose
        self.est = (p.position.x, p.position.y, yaw_of(p.orientation))

    def _joints(self, msg):
        # /joint_states is multi-publisher: messages can omit the rail joints or
        # order them differently, so re-resolve the indices whenever the name list
        # changes rather than trusting a stale cache (a bad index would raise or
        # read the wrong joint).
        if msg.name != self.names:
            try:
                self.idx = tuple(msg.name.index(j) for j in RAIL_JOINTS)
            except ValueError:
                self.idx = None
                return  # rail joints not present in this message
            self.names = list(msg.name)
        x, y, yaw = (msg.position[i] for i in self.idx)
        self.base = (x, y, yaw)

    def _pub(self):
        if self.est is None or self.base is None:
            return
        # odom->world = odom->base(est) (+) inverse(world->base(true))
        x, y, yaw = compose(self.est, invert(self.base))

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "world"
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.rotation.z = math.sin(yaw / 2.0)
        t.transform.rotation.w = math.cos(yaw / 2.0)
        self.br.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdomWorldDrift()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
