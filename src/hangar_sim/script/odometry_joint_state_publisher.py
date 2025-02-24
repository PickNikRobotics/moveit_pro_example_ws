#!/usr/bin/env python3

# Copyright 2024 PickNik Inc.
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

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState

from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
)
from math import atan2


class OdometryJointStateRepublisher(Node):
    """! The Odometry to JointState republisher node.

    Subscribes to Odometry messages, copies the data into JointState messages,
    which are published to '/joint_states'.
    """

    def __init__(self, odom_topic, joint_states_topic):
        """! Initialize Odometry subscriber and JointState publisher.

        @param odom_topic topic name to use for Odometry subscription
        @param joint_states_topic topic name to use for JointState publishing

        @return Instance of the OdometryJointStateRepublisher node
        """

        super().__init__("odometry_joint_state_republisher")

        # Define a QoS profile
        qos_profile_sub = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,  # Ensure reliability
            durability=QoSDurabilityPolicy.VOLATILE,  # Non-persistent messages
            history=QoSHistoryPolicy.KEEP_LAST,  # Keep only the last few messages
            depth=1,  # Queue size
        )

        # Define a QoS profile
        qos_profile_pub = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,  # Ensure reliability
            durability=QoSDurabilityPolicy.VOLATILE,  # Non-persistent messages
            history=QoSHistoryPolicy.KEEP_LAST,  # Keep only the last few messages
            depth=1,  # Queue size
        )

        self.odom_sub_ = self.create_subscription(
            Odometry, odom_topic, self.odom_callback, qos_profile_sub
        )
        self.joint_states_pub_ = self.create_publisher(
            JointState, joint_states_topic, qos_profile_pub
        )

    def odom_callback(self, odom_msg):
        """! Subscription callback to run for incoming Odometry messages

        Each Odometry message is copied into JointState messages and republished
        to /joint_states.

        """

        # Get yaw angle from quaternion orientation,
        # see https://stackoverflow.com/questions/5782658/extracting-yaw-from-a-quaternion
        q = odom_msg.pose.pose.orientation
        rotation_yaw = atan2(
            2.0 * (q.z * q.w + q.x * q.y), -1.0 + 2.0 * (q.w * q.w + q.x * q.x)
        )

        # Populate joint state message with x,y,yaw joint dimensions
        joint_state_msg = JointState()
        joint_state_msg.name = [
            "linear_x_joint",
            "linear_y_joint",
            "rotational_yaw_joint",
        ]
        joint_state_msg.position = [
            odom_msg.pose.pose.position.x,
            odom_msg.pose.pose.position.y,
            rotation_yaw,
        ]
        self.joint_states_pub_.publish(joint_state_msg)


def main(args=None):
    rclpy.init(args=args)

    odometry_repub = OdometryJointStateRepublisher("/odom", "/joint_states")

    rclpy.spin(odometry_repub)

    odometry_repub.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
