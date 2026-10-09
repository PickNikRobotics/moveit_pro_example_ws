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

"""Re-seed AMCL at the simulator's true base pose after a keyframe reset teleports the base.

    ros2 run market_sim reset_relocalizer.py

A reset moves the base farther in one step than it can drive. The odometry bridge restarts
at the new pose, but AMCL reads that as a long, noisy motion and loses the robot. This node
watches the true pose; after a jump it waits until AMCL has absorbed the odometry step, then
publishes the true pose on /initialpose. The map frame equals the simulator's world here.
"""

import math


def is_jump(previous, current, max_step, max_turn):
    """True when two consecutive (x, y, yaw) poses are farther apart than the base can move."""
    turn = math.atan2(
        math.sin(current[2] - previous[2]), math.cos(current[2] - previous[2])
    )
    step = math.hypot(current[0] - previous[0], current[1] - previous[1])
    return step > max_step or abs(turn) > max_turn


def main():
    import rclpy
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data

    class ResetRelocalizer(Node):
        def __init__(self):
            super().__init__("reset_relocalizer")
            p = self.declare_parameter
            self._max_step = float(p("teleport_distance", 0.5).value)
            self._max_turn = float(p("teleport_angle", 0.5).value)
            # Seconds after a jump to seed; two seeds cover a slow odometry update.
            self._delays = [float(d) for d in p("seed_delays_sec", [3.0, 5.0]).value]
            self._variance = float(p("variance", 0.01).value)
            self._frame = p("map_frame", "map").value
            self._truth = None
            self._pending = []
            self._pub = self.create_publisher(
                PoseWithCovarianceStamped,
                p("initial_pose_topic", "/initialpose").value,
                1,
            )
            self.create_subscription(
                Odometry,
                p("ground_truth_topic", "/ground_truth/odom").value,
                self._on_truth,
                qos_profile_sensor_data,
            )
            self.create_timer(0.1, self._tick)

        def _on_truth(self, msg):
            pose = msg.pose.pose
            q = pose.orientation
            yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            )
            current = (pose.position.x, pose.position.y, yaw)
            if self._truth is not None and is_jump(
                self._truth[0], current, self._max_step, self._max_turn
            ):
                now = self.get_clock().now().nanoseconds / 1e9
                self._pending = [now + d for d in self._delays]
                self.get_logger().info("Base teleported; re-seeding AMCL shortly.")
            self._truth = (current, pose)

        def _tick(self):
            now = self.get_clock().now().nanoseconds / 1e9
            if not self._pending or now < self._pending[0] or self._truth is None:
                return
            self._pending.pop(0)
            seed = PoseWithCovarianceStamped()
            seed.header.frame_id = self._frame
            seed.header.stamp = self.get_clock().now().to_msg()
            seed.pose.pose = self._truth[1]
            seed.pose.covariance[0] = seed.pose.covariance[7] = self._variance
            seed.pose.covariance[35] = self._variance
            self._pub.publish(seed)
            self.get_logger().info("Re-seeded AMCL at the true base pose.")

    rclpy.init()
    node = ResetRelocalizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
