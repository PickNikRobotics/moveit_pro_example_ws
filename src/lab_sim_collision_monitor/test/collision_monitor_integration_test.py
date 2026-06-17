# Copyright 2026 PickNik Inc.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the PickNik Inc. nor the names of its contributors
#      may be used to endorse or promote products derived from this software
#      without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DAMAGES ARISING IN ANY WAY
# OUT OF THE USE OF THIS SOFTWARE.

"""Layer-2 parallel-monitor integration test for lab_sim.

Runs the lab_sim MoveIt Pro backend (via the shared
``execute_objective_resource`` fixture) and the licensed, vendor-neutral
``collision_monitor`` node *together*, then asserts BOTH directions of
correctness the rearchitecture contract requires:

  (a) NO false trips while a valid lab_sim objective runs (the monitor must
      not spuriously stop a known-safe motion); and
  (b) a DELIBERATE self-collision pose MUST trip (proves the monitor catches a
      real, ACM-allowed collision).

The deliberate scenario is a *self-collision* pose, not a scene-obstacle
contact, because the generic monitor node initializes an empty collision world
(no param to inject scene obstacles); robot-link-vs-robot-link self-collisions
are exactly what it can see for this robot, and the contract explicitly allows
a self-collision pose. The pose folds the wrist/gripper into the upper arm /
base, a pair that is NOT disabled in the SRDF ACM.

LICENSE GATE: the monitor's engine constructor is license-gated. This test
only passes when a valid ``MOVEIT_LICENSE_KEY`` is present in the environment
(example_ws ships one in ``.env``; CI maps the ``STUDIO_CI_LICENSE_KEY``
secret). Without a key the monitor node dies on init, ``/status`` never
publishes, and ``test_monitor_becomes_active`` fails loudly — the intended IP
protection, surfaced as a test failure rather than a silent skip.

The monitor's TripSink in production is wired to a lab_sim stop (controller
halt). The deliberate-trip test demonstrates that wiring: on the first trip it
deactivates ``joint_trajectory_controller`` via ``switch_controller``. The
trip/no-trip *assertions* themselves read the monitor's own ``trip`` (String)
and ``status`` (Bool) outputs, which are the authoritative monitor signals.
"""

from __future__ import annotations

import multiprocessing
import os
import signal
import time
from typing import Optional

import pytest
import rclpy
from launch import LaunchDescription, LaunchService
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from rclpy.action import ActionClient
from rclpy.node import Node as ROSNode
from rclpy.qos import QoSProfile

from builtin_interfaces.msg import Duration as DurationMsg
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import SwitchController
from std_msgs.msg import Bool, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# ``execute_objective_resource`` (brings up the lab_sim MoveIt Pro backend) and
# the autouse ``reset_simulation_before_test`` are provided via conftest.py;
# the test references them through pytest fixture injection.
from moveit_pro_test_utils.objective_test_fixture import (
    ExecuteObjectiveResource,
    run_objective,
)

# Monitor topic surface (see collision_monitor_component.cpp). The launch puts
# the node at namespace "collision_monitor" with node name "collision_monitor",
# and the component declares ~/trip and ~/status, hence the doubled segment.
MONITOR_NS = "collision_monitor"
TRIP_TOPIC = f"/{MONITOR_NS}/collision_monitor/trip"
STATUS_TOPIC = f"/{MONITOR_NS}/collision_monitor/status"

# lab_sim model-DOF order for the joint_trajectory_controller.
ARM_JOINTS = [
    "linear_rail_joint",
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]
JTC_ACTION = "/joint_trajectory_controller/follow_joint_trajectory"
SWITCH_CONTROLLER_SRV = "/controller_manager/switch_controller"

# A known-safe arm motion objective shipped by lab_sim. Exercises real arm
# movement without driving into self-collision; the monitor must stay clear.
SAFE_OBJECTIVE = "Move Along Square"

# Deliberate self-collision target. Curls the wrist back over the upper arm so
# the gripper/wrist links collide with upper_arm_link / base (NOT disabled in
# the SRDF ACM). rail at 0; shoulder folded; elbow + wrists curled hard.
#   [linear_rail, shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3]
SELF_COLLISION_POSE = [0.0, 0.0, -2.6, 2.7, -2.8, 0.0, 0.0]

# Monitor needs URDF + SRDF (transient_local) + a complete joint_state before it
# flips to "monitoring active" and starts publishing /status. Generous on CI.
MONITOR_ACTIVE_TIMEOUT_S = 120.0
SAFE_RUN_TIMEOUT_S = 180.0
TRIP_TIMEOUT_S = 60.0


def _monitor_launch() -> None:
    """Run only the collision_monitor launch in this subprocess."""
    launch_service = LaunchService()
    launch_service.include_launch_description(
        LaunchDescription(
            [
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        [
                            FindPackageShare("lab_sim_collision_monitor"),
                            "/launch/collision_monitor.launch.py",
                        ]
                    )
                )
            ]
        )
    )
    launch_service.run()


class MonitorProbe:
    """Subscribes to the monitor's trip/status outputs and tracks them.

    Also owns the production-style TripSink demonstration: a controller halt
    via ``switch_controller`` fired once on the first observed trip.
    """

    def __init__(self, node: ROSNode) -> None:
        self.node = node
        self.latest_status: Optional[bool] = None
        self.trip_messages: list[str] = []
        self._halt_fired = False
        self._switch_client = node.create_client(
            SwitchController, SWITCH_CONTROLLER_SRV
        )
        node.create_subscription(
            Bool, STATUS_TOPIC, self._on_status, QoSProfile(depth=10)
        )
        node.create_subscription(
            String, TRIP_TOPIC, self._on_trip, QoSProfile(depth=10)
        )

    def _on_status(self, msg: Bool) -> None:
        self.latest_status = msg.data

    def _on_trip(self, msg: String) -> None:
        self.trip_messages.append(msg.data)
        # TripSink -> lab_sim stop: halt the arm controller on the first trip.
        # Clearly marked as the deliberate stop action; best-effort (the
        # assertions below key off the monitor's own outputs, not this halt).
        if not self._halt_fired and self._switch_client.service_is_ready():
            self._halt_fired = True
            req = SwitchController.Request()
            req.deactivate_controllers = ["joint_trajectory_controller"]
            req.strictness = SwitchController.Request.BEST_EFFORT
            self._switch_client.call_async(req)

    def tripped(self) -> bool:
        return bool(self.trip_messages) or self.latest_status is True


def _spin(node: ROSNode, duration_s: float) -> None:
    deadline = time.monotonic() + duration_s
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)


@pytest.fixture(scope="module")
def monitor_probe(execute_objective_resource: ExecuteObjectiveResource):
    """Launch the monitor in parallel with the (already-up) lab_sim backend.

    Depends on ``execute_objective_resource`` so the MoveIt Pro backend — and
    therefore /robot_description(_semantic) and /joint_states — is live before
    the monitor starts. Yields a :class:`MonitorProbe`.
    """
    proc = multiprocessing.Process(target=_monitor_launch)
    proc.start()
    print(f"[monitor] launched pid={proc.pid}", flush=True)

    node = ROSNode("collision_monitor_test_probe")
    probe = MonitorProbe(node)
    try:
        yield probe
    finally:
        node.destroy_node()
        if proc.pid is not None:
            os.kill(proc.pid, signal.SIGINT)
        proc.join(timeout=20)
        if proc.is_alive():
            proc.kill()
            proc.join(timeout=10)


def _wait_for_monitor_active(probe: MonitorProbe, timeout_s: float) -> bool:
    """Block until the monitor publishes its first /status (== active)."""
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        rclpy.spin_once(probe.node, timeout_sec=0.2)
        if probe.latest_status is not None:
            return True
    return False


def _send_arm_trajectory(node: ROSNode, positions: list[float], move_time_s: float):
    """Command the JTC directly to ``positions`` (bypasses MoveIt planning).

    Returns the goal-handle future; the caller spins. Used to drive the robot
    into the deliberate self-collision pose without MoveIt refusing the goal.
    """
    client = ActionClient(node, FollowJointTrajectory, JTC_ACTION)
    if not client.wait_for_server(timeout_sec=30.0):
        raise TimeoutError(f"{JTC_ACTION} action server not available")

    goal = FollowJointTrajectory.Goal()
    goal.trajectory = JointTrajectory()
    goal.trajectory.joint_names = ARM_JOINTS
    point = JointTrajectoryPoint()
    point.positions = positions
    secs = int(move_time_s)
    point.time_from_start = DurationMsg(
        sec=secs, nanosec=int((move_time_s - secs) * 1e9)
    )
    goal.trajectory.points = [point]
    return client.send_goal_async(goal)


# === Test (preconditions) ===


def test_monitor_becomes_active(monitor_probe: MonitorProbe):
    """The monitor must come up and publish /status.

    This is also the LICENSE GATE assertion: if MOVEIT_LICENSE_KEY is missing
    or invalid the engine ctor throws, the node dies, /status never publishes,
    and this fails — the example refuses to run unlicensed, loudly.
    """
    assert _wait_for_monitor_active(monitor_probe, MONITOR_ACTIVE_TIMEOUT_S), (
        f"collision_monitor did not publish {STATUS_TOPIC} within "
        f"{MONITOR_ACTIVE_TIMEOUT_S:.0f}s. Either the licensed monitor bundle "
        "is not on the overlay, or MOVEIT_LICENSE_KEY is missing/invalid "
        "(engine ctor is license-gated)."
    )
    # Fresh start: not tripped.
    assert monitor_probe.latest_status is False, (
        "Monitor reported tripped at startup on a safe home configuration."
    )


# === Test (a): no false trips on a valid objective ===


def test_no_false_trip_during_safe_objective(
    monitor_probe: MonitorProbe,
    execute_objective_resource: ExecuteObjectiveResource,
):
    """A known-safe objective must run without tripping the monitor."""
    assert _wait_for_monitor_active(monitor_probe, MONITOR_ACTIVE_TIMEOUT_S), (
        "Monitor not active before safe-objective run."
    )
    monitor_probe.trip_messages.clear()

    run_objective(
        SAFE_OBJECTIVE,
        should_cancel=False,
        execute_objective_resource=execute_objective_resource,
        objective_wait_time=SAFE_RUN_TIMEOUT_S,
    )
    # Drain any in-flight status/trip messages.
    _spin(monitor_probe.node, 2.0)

    assert not monitor_probe.tripped(), (
        f"Monitor FALSE-TRIPPED during the safe objective "
        f"'{SAFE_OBJECTIVE}'. Trip messages: {monitor_probe.trip_messages}"
    )


# === Test (b): deliberate self-collision MUST trip ===


def test_deliberate_self_collision_trips(
    monitor_probe: MonitorProbe,
    execute_objective_resource: ExecuteObjectiveResource,
):
    """Driving into a self-collision pose MUST trip the monitor.

    The reset fixture restores a safe keyframe before this test, so we start
    clean, then command the JTC straight to a self-colliding configuration and
    assert the monitor trips (its forward-projected braking stop collides).
    """
    assert _wait_for_monitor_active(monitor_probe, MONITOR_ACTIVE_TIMEOUT_S), (
        "Monitor not active before deliberate-trip run."
    )
    monitor_probe.trip_messages.clear()

    goal_future = _send_arm_trajectory(
        monitor_probe.node, SELF_COLLISION_POSE, move_time_s=4.0
    )
    rclpy.spin_until_future_complete(
        monitor_probe.node, goal_future, timeout_sec=30.0
    )

    # Wait for the robot to traverse toward / arrive at the colliding pose and
    # for the monitor to trip.
    deadline = time.monotonic() + TRIP_TIMEOUT_S
    while time.monotonic() < deadline and not monitor_probe.tripped():
        rclpy.spin_once(monitor_probe.node, timeout_sec=0.1)

    assert monitor_probe.tripped(), (
        "Monitor did NOT trip while the arm was driven into a self-collision "
        f"pose {SELF_COLLISION_POSE}. Expected a PROJECTED_COLLISION trip. "
        f"latest_status={monitor_probe.latest_status}, "
        f"trips={monitor_probe.trip_messages}"
    )
    # The trip reason should reflect a projected collision (not, say, a stale
    # state). Accept any trip but surface the reason for diagnostics.
    if monitor_probe.trip_messages:
        assert any(
            "PROJECTED_COLLISION" in m for m in monitor_probe.trip_messages
        ), (
            "Monitor tripped but not for PROJECTED_COLLISION: "
            f"{monitor_probe.trip_messages}"
        )
