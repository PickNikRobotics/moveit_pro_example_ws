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

"""Reset handshake tests through ROS topics, TF and real service callbacks."""

import math
import sys
import time
from pathlib import Path

import pytest
import rclpy
from fuse_msgs.srv import SetPose
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool
from std_srvs.srv import Empty, Trigger
from tf2_ros import TransformBroadcaster

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

from reset_relocalizer import ResetRelocalizer, is_jump, planar_pose  # noqa: E402


class ResetHarness:
    """An isolated simulation/Fuse/AMCL peer exercising the deployed node's interface."""

    def __init__(self):
        self.node = ResetRelocalizer(
            parameter_overrides=[Parameter("reset_timeout_sec", value=1.0)]
        )
        self.peer = Node("reset_test_peer")
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.executor.add_node(self.peer)
        self.truth = self.peer.create_publisher(
            Odometry, "/ground_truth/odom", qos_profile_sensor_data
        )
        self.filtered = self.peer.create_publisher(
            Odometry, "/odom_filtered", qos_profile_sensor_data
        )
        self.amcl = self.peer.create_publisher(PoseWithCovarianceStamped, "/pose", 10)
        self.tf = TransformBroadcaster(self.peer)
        self.seeds = []
        self.ready = []
        self.requests = []
        self.accept = True
        self.amcl_age = 0
        self.update_requests = []
        self.nomotion_service = self.peer.create_service(
            Empty, "/request_nomotion_update", self._nomotion_update
        )
        self.peer.create_subscription(
            PoseWithCovarianceStamped, "/initialpose", self.seeds.append, 10
        )
        self.peer.create_subscription(
            Bool,
            "/market_sim/localization_ready",
            lambda msg: self.ready.append(msg.data),
            10,
        )
        self.service = self.peer.create_service(
            SetPose, "/state_estimator/set_pose", self._set_pose
        )
        self.client = self.peer.create_client(
            Trigger, "/market_sim/synchronize_localization"
        )
        self.x = 0.0
        self.fused_x = None
        self.amcl_x = None
        self.publish_tf = True
        self.truth_age = 0.0
        self.fused_age = 0.0
        self.timer = self.peer.create_timer(0.02, self.publish)
        self.until(lambda: self.ready and self.ready[-1])

    def _set_pose(self, request, response):
        self.requests.append(request.pose)
        response.success = self.accept
        response.message = "test rejection" if not self.accept else "accepted"
        return response

    def _nomotion_update(self, request, response):
        self.update_requests.append(request)
        return response

    def publish(self):
        """Publish the selected state and frame chain with advancing timestamps."""
        stamp = self.peer.get_clock().now().to_msg()
        truth = Odometry()
        truth.header.stamp = stamp
        truth.header.stamp.sec -= int(self.truth_age)
        truth.header.frame_id = "mj_world"
        truth.child_frame_id = "base_link"
        truth.pose.pose.position.x = self.x
        truth.pose.pose.orientation.w = 1.0
        self.truth.publish(truth)
        if self.fused_x is not None:
            filtered = Odometry()
            filtered.header.frame_id = "odom"
            filtered.child_frame_id = "base_link"
            filtered.header.stamp = self.peer.get_clock().now().to_msg()
            filtered.header.stamp.sec -= int(self.fused_age)
            filtered.pose.pose.position.x = self.fused_x
            filtered.pose.pose.orientation.w = 1.0
            self.filtered.publish(filtered)
            if self.publish_tf:
                transform = TransformStamped()
                transform.header.stamp = filtered.header.stamp
                transform.header.frame_id = "odom"
                transform.child_frame_id = "base_link"
                transform.transform.translation.x = self.fused_x
                transform.transform.rotation.w = 1.0
                self.tf.sendTransform(transform)
        if self.amcl_x is not None:
            pose = PoseWithCovarianceStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = self.peer.get_clock().now().to_msg()
            pose.header.stamp.sec -= self.amcl_age
            pose.pose.pose.position.x = self.amcl_x
            pose.pose.pose.orientation.w = 1.0
            self.amcl.publish(pose)
            if self.publish_tf:
                transform = TransformStamped()
                transform.header = pose.header
                transform.child_frame_id = "odom"
                transform.transform.rotation.w = 1.0
                self.tf.sendTransform(transform)

    def until(self, predicate, timeout=2.0):
        """Spin without fixed startup sleeps until the observable condition is met."""
        deadline = time.monotonic() + timeout
        while not predicate() and time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.01)
        assert (
            predicate()
        ), "ROS reset handshake did not reach the expected observable state"

    def spin_for(self, duration):
        """Exercise callbacks over a bounded interval to prove a state remains held."""
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.01)

    def close(self):
        """Release every test endpoint before the next isolated case."""
        self.executor.shutdown()
        self.node.destroy_node()
        self.peer.destroy_node()


@pytest.fixture
def reset():
    """Create the real reset node with a peer; never touch an external simulator."""
    rclpy.init()
    harness = ResetHarness()
    try:
        yield harness
    finally:
        harness.close()
        rclpy.shutdown()


@pytest.mark.parametrize("value", [math.nan, math.inf, -math.inf])
def test_reset_rejects_nonfinite_pose(value) -> None:
    """A malformed simulator pose cannot seed either estimator."""
    pose = Odometry().pose.pose
    pose.position.x = value
    with pytest.raises(ValueError, match="finite pose"):
        planar_pose(pose)


def test_rejected_fuse_reset_is_reported_and_holds_commands(reset) -> None:
    """A service rejection reaches the caller and never seeds AMCL or releases the gate."""
    reset.accept = False
    result = reset.client.call_async(Trigger.Request())
    reset.until(result.done)
    assert not result.result().success
    assert "/state_estimator/set_pose" in result.result().message
    assert "test rejection" in result.result().message
    reset.until(lambda: reset.ready and not reset.ready[-1])
    assert reset.seeds == []


@pytest.mark.parametrize("missing", ["service", "fused", "tf", "amcl"])
def test_missing_reset_dependency_times_out_without_releasing_commands(
    reset, missing
) -> None:
    """Unavailable service, pose or TF keeps the reset incomplete and exposes the missing stage."""
    if missing == "service":
        reset.peer.destroy_service(reset.service)
    if missing in ("tf", "amcl"):
        reset.fused_x = 0.0
    if missing == "tf":
        reset.publish_tf = False
    result = reset.client.call_async(Trigger.Request())
    reset.until(result.done)
    response = result.result()
    assert not response.success
    assert "timed out" in response.message
    assert "Nav2 commands remain held" in response.message
    reset.until(lambda: reset.ready and not reset.ready[-1])
    if missing != "amcl":
        assert reset.seeds == []


def test_stale_fused_pose_cannot_acknowledge_a_reset(reset) -> None:
    """Even a matching pose is unusable when its stamp predates the Fuse reset."""
    reset.fused_x = 0.0
    reset.fused_age = 2.0
    result = reset.client.call_async(Trigger.Request())
    reset.until(result.done)
    assert not result.result().success
    assert "fused_pose_and_tf" in result.result().message
    assert reset.seeds == []


def test_stale_truth_fails_with_actionable_response(reset) -> None:
    """The service fails rather than seeding from an old simulator sample."""
    result = reset.client.call_async(Trigger.Request())
    reset.truth_age = 2.0
    reset.until(result.done)
    assert not result.result().success
    assert "/ground_truth/odom" in result.result().message
    assert "timestamps" in result.result().message
    assert reset.requests == []


def test_explicit_reset_waits_for_fused_pose_tf_then_amcl(reset) -> None:
    """Controllers remain gated until both estimators and their observed TF match the reset."""
    reset.x = 4.0
    result = reset.client.call_async(Trigger.Request())
    reset.until(lambda: bool(reset.requests))
    assert reset.requests[-1].header.frame_id == "odom"
    assert reset.requests[-1].pose.pose.position.x == 4.0
    reset.fused_x = 0.0
    reset.spin_for(0.1)
    assert reset.seeds == []
    reset.fused_x = 4.0
    reset.until(lambda: bool(reset.seeds))
    assert not result.done()
    assert reset.seeds[-1].header.frame_id == "map"
    assert reset.seeds[-1].pose.pose.position.x == 4.0
    reset.amcl_x = 4.0
    reset.until(result.done)
    assert result.result().success
    reset.until(lambda: reset.ready[-1])


def test_viewer_teleport_resets_but_ordinary_motion_does_not(reset) -> None:
    """Only a discontinuity triggers truth seeding; regular navigation remains lidar/fused."""
    reset.x = 0.01
    reset.spin_for(0.1)
    assert reset.requests == []
    reset.x = 3.0
    reset.until(lambda: bool(reset.requests))
    reset.fused_x = 3.0
    reset.until(lambda: bool(reset.seeds))
    reset.amcl_x = 3.0
    reset.until(lambda: reset.ready[-1])
    request_count, seed_count = len(reset.requests), len(reset.seeds)
    reset.x = 3.02
    reset.spin_for(0.1)
    assert (len(reset.requests), len(reset.seeds)) == (request_count, seed_count)


def test_simultaneous_reset_request_is_rejected_without_cancelling_the_first(
    reset,
) -> None:
    """A second caller cannot steal the first reset's completion or extend its deadline."""
    first = reset.client.call_async(Trigger.Request())
    reset.until(lambda: bool(reset.requests))
    second = reset.client.call_async(Trigger.Request())
    reset.until(second.done)
    assert not second.result().success
    assert "already pending" in second.result().message
    reset.fused_x = reset.amcl_x = 0.0
    reset.until(first.done)
    assert first.result().success


def test_lost_truth_during_reset_fails_closed(reset) -> None:
    """A disappearing simulator topic cannot complete a reset using cached truth."""
    first = reset.client.call_async(Trigger.Request())
    reset.until(lambda: bool(reset.requests))
    reset.timer.cancel()
    reset.until(first.done)
    assert not first.result().success
    assert "lost /ground_truth/odom" in first.result().message
    assert reset.seeds == []


def test_retry_after_rejection_can_complete(reset) -> None:
    """A failed reset stays held until a subsequent explicit, successful handshake."""
    reset.accept = False
    first = reset.client.call_async(Trigger.Request())
    reset.until(first.done)
    assert not first.result().success
    reset.accept = True
    reset.fused_x = reset.amcl_x = 0.0
    second = reset.client.call_async(Trigger.Request())
    reset.until(second.done)
    assert second.result().success


@pytest.mark.parametrize(
    "step, expected", [(0.5 - 1e-6, False), (0.5, False), (0.5 + 1e-6, True)]
)
def test_jump_threshold(step, expected) -> None:
    """Distance and rotation discontinuities use a strict 0.5 threshold."""
    assert is_jump((0.0, 0.0, 0.0), (step, 0.0, 0.0)) == expected
    assert is_jump((0.0, 0.0, 0.0), (0.0, 0.0, step)) == expected


def test_heading_wrap_is_not_a_jump() -> None:
    """A continuous heading crossing pi is not a reset."""
    assert not is_jump((0.0, 0.0, math.pi - 0.01), (0.0, 0.0, -math.pi + 0.01))


def test_beluga_reset_requests_a_post_seed_stationary_pose(reset) -> None:
    """Beluga publishes on /pose and needs a no-motion update after an older queued scan."""
    reset.fused_x = reset.amcl_x = 0.0
    reset.amcl_age = 2
    result = reset.client.call_async(Trigger.Request())
    reset.until(lambda: bool(reset.seeds))
    reset.until(lambda: bool(reset.update_requests))
    assert not result.done()
    reset.amcl_age = 0
    reset.until(result.done)
    assert result.result().success
    reset.until(lambda: reset.ready[-1])


def interrupt_truth(reset, fault):
    """Make the simulated ground truth unusable in one of the ways a loaded host produces."""
    if fault == "gap":
        reset.timer.cancel()
    elif fault == "stale":
        reset.truth_age = 2.0
    elif fault == "future":
        reset.truth_age = -2.0
    else:
        reset.x = math.nan


def restore_truth(reset, fault):
    """Return to fresh, finite truth at the robot's unchanged pose."""
    reset.truth_age = 0.0
    reset.x = 0.0
    if fault == "gap":
        reset.timer.reset()


@pytest.mark.parametrize("fault", ["gap", "stale", "future", "nonfinite"])
def test_transient_truth_loss_holds_then_releases_without_reseeding(
    reset, fault
) -> None:
    """Outside a reset, unusable truth holds commands only until it is continuous again."""
    interrupt_truth(reset, fault)
    reset.until(lambda: not reset.ready[-1])
    reset.spin_for(0.8)
    assert not reset.ready[-1]
    restore_truth(reset, fault)
    held = len(reset.ready)
    reset.spin_for(0.2)
    assert not any(reset.ready[held:])
    reset.until(lambda: reset.ready[-1])
    assert reset.requests == []
    assert reset.seeds == []
    assert reset.update_requests == []


def test_interrupted_recovery_restarts_the_continuity_window(reset) -> None:
    """A single bad sample during recovery re-arms the hold rather than releasing early."""
    reset.truth_age = 2.0
    reset.until(lambda: not reset.ready[-1])
    reset.truth_age = 0.0
    reset.spin_for(0.35)
    reset.truth_age = 2.0
    reset.spin_for(0.05)
    reset.truth_age = 0.0
    held = len(reset.ready)
    reset.spin_for(0.35)
    assert not any(reset.ready[held:])
    reset.until(lambda: reset.ready[-1])


def test_failed_reset_stays_held_after_truth_returns(reset) -> None:
    """A reset that lost truth is not released by fresh samples, only by a new synchronization."""
    first = reset.client.call_async(Trigger.Request())
    reset.until(lambda: bool(reset.requests))
    reset.timer.cancel()
    reset.until(first.done)
    assert not first.result().success
    reset.timer.reset()
    reset.spin_for(1.5)
    assert not reset.ready[-1]
    assert reset.seeds == []
    reset.fused_x = reset.amcl_x = 0.0
    second = reset.client.call_async(Trigger.Request())
    reset.until(second.done)
    assert second.result().success
    reset.until(lambda: reset.ready[-1])
