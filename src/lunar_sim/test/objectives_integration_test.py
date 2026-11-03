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

"""Integration tests for the lunar_sim objective library and its navigation stack."""

import math
import time

import pytest
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rcl_interfaces.srv import GetParameters
from rclpy.action import ActionClient
from rclpy.qos import DurabilityPolicy, QoSProfile
from tf2_msgs.msg import TFMessage
from moveit_pro_test_utils.objective_test_fixture import (
    ExecuteObjectiveResource,
    MUJOCO_RESET_HOOK,
    SIM_RESETTER,
    execute_objective_resource as execute_objective_resource,
    get_objective_pytest_params,
    reset_simulation_before_test as reset_simulation_before_test,
    run_objective,
    wait_for_action_servers,
)

SIM_RESETTER.register("lunar_sim", MUJOCO_RESET_HOOK)

cancel_objectives: set[str] = set()

# Non-runnable objectives are filtered out by the fixture and need not be listed here.
skip_objectives = {
    # User input required: no primary UI is attached in headless CI.
    "Navigate to Clicked Point",  # GetPoseFromUser + WaitForUserPathApproval.
    "Navigate to Clicked Point with Replanning",  # GetPoseFromUser.
    "Request Teleoperation",  # DoTeleoperateAction rejects the goal with no UI subscribed.
    # Core-library objectives aggregated into every config.
    "Teleoperate",  # DoTeleoperateAction rejects the goal with no UI subscribed.
    "Marker Visualization Example",  # GetTextFromUser server unavailable headless.
    "Vector Subtrees Example",  # Looks up the `world` frame, which lunar_sim's TF tree does not have.
    # Hand-eye calibration needs the ChArUco board only hand_eye_calibration_sim provides.
    "Calibrate Camera Intrinsics",
    "Calibrate Eye In Hand Camera",
    "Calibrate Eye To Hand Camera",
    "Calibrate Multiple Cameras",
    "Generate Calibration Waypoints",
}

TF_FIRST_SIGHTING_TIMEOUT_S = 120.0
TF_PARENT_SAMPLE_DURATION_S = 4.0


def _tf_parents(
    resource: ExecuteObjectiveResource, child_frames: set[str]
) -> dict[str, set[str]]:
    """Collect every /tf and /tf_static parent of each child frame: wait for all to appear, then keep sampling."""
    node = resource.node
    parents: dict[str, set[str]] = {child: set() for child in child_frames}

    def collect(msg: TFMessage) -> None:
        for transform in msg.transforms:
            if transform.child_frame_id in parents:
                parents[transform.child_frame_id].add(transform.header.frame_id)

    tf_static_qos = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    subscriptions = [
        node.create_subscription(TFMessage, "/tf", collect, 100),
        node.create_subscription(TFMessage, "/tf_static", collect, tf_static_qos),
    ]
    deadline = time.monotonic() + TF_FIRST_SIGHTING_TIMEOUT_S
    while time.monotonic() < deadline and not all(parents.values()):
        rclpy.spin_once(node, timeout_sec=0.1)
    deadline = time.monotonic() + TF_PARENT_SAMPLE_DURATION_S
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    for subscription in subscriptions:
        node.destroy_subscription(subscription)
    return parents


def test_odom_and_footprint_each_have_one_tf_parent(
    execute_objective_resource: ExecuteObjectiveResource,
) -> None:
    """map alone parents odom, fuse alone publishes odom -> footprint, MujocoSystem publishes no mj_world -> odom, and only the URDF places imu_link."""
    node = execute_objective_resource.node
    client = node.create_client(
        GetParameters, "/platform_velocity_controller/get_parameters"
    )
    try:
        assert client.wait_for_service(
            timeout_sec=TF_FIRST_SIGHTING_TIMEOUT_S
        ), "platform_velocity_controller is not running."
        future = client.call_async(GetParameters.Request(names=["enable_odom_tf"]))
        rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
        assert (
            future.done()
        ), "platform_velocity_controller did not return its parameters."
        # Its odom -> footprint would duplicate fuse's edge, so the parent check below cannot see it.
        assert (
            future.result().values[0].bool_value is False
        ), "platform_velocity_controller publishes odom -> footprint alongside fuse."
    finally:
        node.destroy_client(client)

    parents = _tf_parents(execute_objective_resource, {"odom", "footprint", "imu_link"})
    assert parents == {
        "odom": {"map"},
        "footprint": {"odom"},
        "imu_link": {"base_link"},
    }, f"Expected map -> odom -> footprint with one publisher each, saw parents: {parents}"


# Map frame, whose origin is the spawn: 2.2 m out, about 0.7 m from the nearest boulder's surface.
NAV_GOAL_XY = (2.0, -1.0)
# goal checker xy_goal_tolerance (0.4 m) plus the odometry drift measured over this distance.
NAV_GOAL_TRUTH_TOLERANCE_M = 0.6
NAV_GOAL_TIMEOUT_S = 120.0
NAV_ACTIVATION_TIMEOUT_S = 60.0


def test_navigate_to_pose_reaches_goal_in_ground_truth(
    execute_objective_resource: ExecuteObjectiveResource,
) -> None:
    """Nav2 drives the rover to a goal, and MuJoCo ground truth, not the estimate, confirms it arrived.

    pytest runs tests in file order, so this runs before the objective sweep, while the rover is at
    the spawn: a keyframe reset elsewhere moves the rover without resetting fuse.
    """
    resource = execute_objective_resource
    node = resource.node
    wait_for_action_servers(node, ["/navigate_to_pose"])
    resource.lookup_object_pose("map", "footprint", timeout_sec=30.0)

    goal = NavigateToPose.Goal()
    goal.pose = PoseStamped()
    goal.pose.header.frame_id = "map"
    goal.pose.pose.position.x, goal.pose.pose.position.y = NAV_GOAL_XY
    goal.pose.pose.orientation.w = 1.0
    client = ActionClient(node, NavigateToPose, "/navigate_to_pose")
    try:
        # The action server exists once bt_navigator is configured, but rejects goals until the lifecycle manager activates it.
        deadline = time.monotonic() + NAV_ACTIVATION_TIMEOUT_S
        while True:
            send = client.send_goal_async(goal)
            rclpy.spin_until_future_complete(
                node, send, timeout_sec=max(deadline - time.monotonic(), 0.1)
            )
            handle = send.result()
            if handle is not None and handle.accepted:
                break
            assert (
                time.monotonic() < deadline
            ), f"Nav2 rejected the goal for {NAV_ACTIVATION_TIMEOUT_S:.0f} s; bt_navigator never activated."
            time.sleep(1.0)
        result = handle.get_result_async()
        rclpy.spin_until_future_complete(node, result, timeout_sec=NAV_GOAL_TIMEOUT_S)
        if not result.done():
            handle.cancel_goal_async()
            pytest.fail(f"Nav2 did not finish within {NAV_GOAL_TIMEOUT_S:.0f} s.")
        assert (
            result.result().status == GoalStatus.STATUS_SUCCEEDED
        ), f"Nav2 ended with status {result.result().status}, not SUCCEEDED."
    finally:
        client.destroy()

    truth = resource.lookup_object_pose(
        "map", "chassis_link_ground_truth", timeout_sec=10.0
    ).position
    miss = math.hypot(truth.x - NAV_GOAL_XY[0], truth.y - NAV_GOAL_XY[1])
    assert (
        miss < NAV_GOAL_TRUTH_TOLERANCE_M
    ), f"Rover stopped {miss:.2f} m from the goal in ground truth, at ({truth.x:.2f}, {truth.y:.2f})."


@pytest.mark.parametrize(
    "objective_id, should_cancel",
    get_objective_pytest_params("lunar_sim", cancel_objectives, skip_objectives),
)
def test_all_objectives(
    objective_id: str,
    should_cancel: bool,
    execute_objective_resource: ExecuteObjectiveResource,
) -> None:
    """Run (or cancel) each lunar_sim objective and assert it completes without error."""
    try:
        run_objective(objective_id, should_cancel, execute_objective_resource)
    except AssertionError as e:
        mode = "cancel" if should_cancel else "execute"
        pytest.fail(f"Objective '{objective_id}' failed to {mode}: {e}")
    except Exception as e:
        mode = "cancel" if should_cancel else "execute"
        pytest.fail(
            f"Objective '{objective_id}' hit an unexpected error during {mode}: "
            f"{type(e).__name__}: {e}"
        )
