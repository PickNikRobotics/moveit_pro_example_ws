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

"""Synchronize Fuse, planar TF and AMCL after a simulation reset, never during ordinary driving.

Reset Objectives call /market_sim/synchronize_localization before restoring controllers. Teleports from the MuJoCo viewer also trigger synchronization. The command bridge requires this node's readiness heartbeat and holds zero throughout a reset or failure. Ground truth seeds the estimators only at these explicit simulation discontinuities; subsequent navigation uses encoders, IMU and lidar. Outside a reset, missing or invalid ground truth holds commands until it is continuous again, without reseeding; a failed reset stays held until the next explicit synchronization.
"""

import copy
import math
import time

import rclpy
from fuse_msgs.srv import SetPose
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.task import Future
from rclpy.time import Time
from std_msgs.msg import Bool
from std_srvs.srv import Empty, Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from odometry_joint_state_publisher import is_jump, yaw_from_quaternion

RESET_PLANAR_VARIANCE = 0.01
WAITING = "waiting_for_truth"
IDLE_STATES = ("ready", "failed", WAITING)


def planar_pose(pose):
    """Read a finite planar pose, rejecting invalid positions and orientations."""
    if not all(
        math.isfinite(value)
        for value in (pose.position.x, pose.position.y, pose.position.z)
    ):
        raise ValueError("Reset localization requires finite pose coordinates.")
    return pose.position.x, pose.position.y, yaw_from_quaternion(pose.orientation)


class ResetRelocalizer(Node):
    """Bound one reset handshake and fail closed if its observed state does not converge."""

    def __init__(self, **kwargs):
        super().__init__("reset_relocalizer", **kwargs)
        self._timeout = float(self.declare_parameter("reset_timeout_sec", 10.0).value)
        self._max_age = float(self.declare_parameter("sample_max_age_sec", 0.5).value)
        self._tolerance = float(
            self.declare_parameter("reset_pose_tolerance", 0.15).value
        )
        self._use_amcl = bool(self.declare_parameter("use_amcl", True).value)
        if not all(
            math.isfinite(v) and v > 0
            for v in (self._timeout, self._max_age, self._tolerance)
        ):
            raise ValueError(
                "Reset timeout, sample age and pose tolerance must be finite and positive; check reset_relocalizer parameters."
            )
        self._truth = None
        self._truth_at = None
        self._valid_since = None
        self._filtered = None
        self._amcl = None
        self._state = WAITING
        self._deadline = 0.0
        self._target = None
        self._requested_at = None
        self._seeded_at = None
        self._future = None
        self._waiters = []
        self._fuse = self.create_client(SetPose, "/state_estimator/set_pose")
        self._nomotion = self.create_client(Empty, "/request_nomotion_update")
        self._next_nomotion_update = 0.0
        self._seed = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 10
        )
        self._ready = self.create_publisher(Bool, "/market_sim/localization_ready", 10)
        self._tf = Buffer()
        self._listener = TransformListener(self._tf, self)
        self.create_subscription(
            Odometry, "/ground_truth/odom", self._on_truth, qos_profile_sensor_data
        )
        self.create_subscription(
            Odometry, "/odom_filtered", self._on_filtered, qos_profile_sensor_data
        )
        self.create_subscription(PoseWithCovarianceStamped, "/pose", self._on_amcl, 10)
        # Awaiting the handshake must not exclude the subscriptions and timer that complete it.
        self.create_service(
            Trigger,
            "/market_sim/synchronize_localization",
            self._synchronize,
            callback_group=ReentrantCallbackGroup(),
        )
        self.create_timer(0.05, self._tick)

    def _fresh(self, stamp):
        age = (self.get_clock().now() - Time.from_msg(stamp)).nanoseconds / 1e9
        return 0 <= age <= self._max_age

    def _on_truth(self, msg):
        try:
            if msg.header.frame_id != "mj_world" or msg.child_frame_id != "base_link":
                raise ValueError(
                    "/ground_truth/odom must describe mj_world -> base_link."
                )
            current = planar_pose(msg.pose.pose)
            if not self._fresh(msg.header.stamp):
                raise ValueError(
                    "/ground_truth/odom is stale or future-dated; check simulator timestamps."
                )
        except ValueError as error:
            self._truth_invalid(str(error))
            return
        jumped = self._truth is not None and is_jump(
            planar_pose(self._truth.pose.pose), current
        )
        now = time.monotonic()
        if (
            self._valid_since is None
            or self._truth_at is None
            or now - self._truth_at > self._max_age
        ):
            self._valid_since = now
        self._truth = msg
        self._truth_at = now
        if jumped:
            self._begin()
        elif self._state == WAITING:
            # Startup and a transient truth outage resume without reseeding once truth is continuous again.
            if now - self._valid_since >= self._max_age:
                self._state = "ready"
                self.get_logger().info(
                    "/ground_truth/odom is continuous again; releasing Nav2 commands."
                )
        elif (
            self._state == "truth"
            and Time.from_msg(msg.header.stamp) > self._requested_at
        ):
            self._target = copy.deepcopy(msg.pose.pose)
            self._state = "fuse_service"

    def _truth_invalid(self, message):
        self._valid_since = None
        if self._state not in IDLE_STATES:
            self._finish(
                False,
                f"{message} Nav2 commands remain held; restore simulator odometry and run Reset MuJoCo Sim.",
            )
        elif self._state == "ready":
            self._state = WAITING
            self._ready.publish(Bool(data=False))
            self.get_logger().warn(
                f"{message} Nav2 commands are held until /ground_truth/odom is continuous again.",
                throttle_duration_sec=5.0,
            )

    def _on_filtered(self, msg):
        if msg.header.frame_id == "odom" and msg.child_frame_id == "base_link":
            self._filtered = msg

    def _on_amcl(self, msg):
        if msg.header.frame_id == "map":
            self._amcl = msg

    def _begin(self):
        if self._future is not None:
            self._future.cancel()
        self._future = None
        self._target = None
        self._filtered = self._amcl = None
        self._requested_at = self.get_clock().now()
        self._deadline = time.monotonic() + self._timeout
        self._state = "truth"
        self._ready.publish(Bool(data=False))
        self.get_logger().info(
            "Simulation reset: holding Nav2 commands while synchronizing Fuse, planar TF and localization."
        )

    async def _synchronize(self, _request, response):
        if self._waiters:
            response.success = False
            response.message = "A localization synchronization request is already pending; wait for its result."
            return response
        # Require a new post-request sample even if an automatic teleport reset just completed.
        if self._state in IDLE_STATES:
            self._begin()
        waiter = Future()
        self._waiters.append(waiter)
        success, message = await waiter
        response.success, response.message = success, message
        return response

    def _finish(self, success, message):
        self._state = "ready" if success else "failed"
        if self._future is not None:
            self._future.cancel()
            self._future = None
        if success:
            self.get_logger().info(message)
        else:
            self.get_logger().error(message, throttle_duration_sec=5.0)
        for waiter in self._waiters:
            waiter.set_result((success, message))
        self._waiters.clear()
        self._ready.publish(Bool(data=success))

    def _matches(self, pose):
        try:
            return not is_jump(
                planar_pose(self._target),
                planar_pose(pose),
                self._tolerance,
                self._tolerance,
            )
        except ValueError:
            return False

    def _tf_matches(self, frame, after):
        try:
            # Pose and TF callbacks have different stamps. Check the latest common TF against the stationary reset target, but never accept a pre-reset transform.
            latest = self._tf.lookup_transform(frame, "base_link", Time())
            if (
                not self._fresh(latest.header.stamp)
                or Time.from_msg(latest.header.stamp) <= after
            ):
                return False
            transform = latest.transform
            pose = copy.deepcopy(self._target)
            pose.position.x, pose.position.y, pose.position.z = (
                transform.translation.x,
                transform.translation.y,
                transform.translation.z,
            )
            pose.orientation = transform.rotation
            return self._matches(pose)
        except TransformException:
            return False

    def _tick(self):
        if (
            self._truth_at is not None
            and time.monotonic() - self._truth_at > self._max_age
        ):
            self._truth_invalid("Reset monitor lost /ground_truth/odom.")
        self._ready.publish(Bool(data=self._state == "ready"))
        if self._state in IDLE_STATES:
            return
        if time.monotonic() >= self._deadline:
            self._finish(
                False,
                f"Localization reset timed out waiting for {self._state}; Nav2 commands remain held. Check /state_estimator/set_pose, /odom_filtered, TF and Beluga /pose, then run Reset MuJoCo Sim.",
            )
            return
        if self._state == "fuse_service" and self._fuse.service_is_ready():
            request = SetPose.Request()
            request.pose.header.frame_id = "odom"
            request.pose.header.stamp = self.get_clock().now().to_msg()
            request.pose.pose.pose = copy.deepcopy(self._target)
            for index, variance in (
                (0, RESET_PLANAR_VARIANCE),
                (7, RESET_PLANAR_VARIANCE),
                (14, 1e-6),
                (21, 1e-4),
                (28, 1e-4),
                (35, RESET_PLANAR_VARIANCE),
            ):
                request.pose.pose.covariance[index] = variance
            self._requested_at = Time.from_msg(request.pose.header.stamp)
            self._future = self._fuse.call_async(request)
            self._state = "fuse_response"
        if self._state == "fuse_response" and self._future.done():
            try:
                result = self._future.result()
                if not result.success:
                    raise RuntimeError(result.message)
            except Exception as error:
                self._finish(
                    False,
                    f"Localization reset: /state_estimator/set_pose failed ({error}); Nav2 commands remain held. Check Fuse and run Reset MuJoCo Sim.",
                )
                return
            self._future = None
            self._state = "fused_pose_and_tf"
        if self._state == "fused_pose_and_tf":
            msg = self._filtered
            if (
                msg is None
                or not self._fresh(msg.header.stamp)
                or Time.from_msg(msg.header.stamp) <= self._requested_at
            ):
                return
            if not self._matches(msg.pose.pose) or not self._tf_matches(
                "odom", self._requested_at
            ):
                return
            if not self._use_amcl:
                self._finish(
                    True,
                    "Simulation reset: Fuse and planar TF synchronized; SLAM retains its own map state.",
                )
                return
            seed = PoseWithCovarianceStamped()
            seed.header.frame_id = "map"
            seed.header.stamp = self.get_clock().now().to_msg()
            seed.pose.pose = copy.deepcopy(self._target)
            seed.pose.covariance[0] = seed.pose.covariance[7] = seed.pose.covariance[
                35
            ] = RESET_PLANAR_VARIANCE
            self._seeded_at = Time.from_msg(seed.header.stamp)
            self._seed.publish(seed)
            self._next_nomotion_update = 0.0
            self._state = "amcl_pose_and_tf"
        if self._state == "amcl_pose_and_tf":
            # A queued pre-seed scan can publish first; request fresh stationary observations until pose and TF acknowledge the seed.
            if self._future is not None and self._future.done():
                try:
                    self._future.result()
                except Exception as error:
                    self._finish(
                        False,
                        f"Localization reset: /request_nomotion_update failed ({error}); Nav2 commands remain held. Check Beluga AMCL and retry Reset MuJoCo Sim.",
                    )
                    return
                self._future = None
            if (
                self._future is None
                and self._nomotion.service_is_ready()
                and time.monotonic() >= self._next_nomotion_update
            ):
                self._future = self._nomotion.call_async(Empty.Request())
                self._next_nomotion_update = time.monotonic() + 0.2
            msg = self._amcl
            if (
                msg is not None
                and self._fresh(msg.header.stamp)
                and Time.from_msg(msg.header.stamp) > self._seeded_at
            ):
                if self._matches(msg.pose.pose) and self._tf_matches(
                    "map", self._seeded_at
                ):
                    self._finish(
                        True,
                        "Simulation reset: Fuse, planar TF and AMCL synchronized with the reset pose.",
                    )


def main():
    rclpy.init()
    node = ResetRelocalizer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
