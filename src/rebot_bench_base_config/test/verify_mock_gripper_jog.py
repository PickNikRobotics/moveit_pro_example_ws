#!/usr/bin/env python3
# Copyright 2026 PickNik Inc.
# SPDX-License-Identifier: BSD-3-Clause
"""Exercise J7 through JointJog after selecting gripper in the Joint teleop tab.

Keep Teleoperate running with collision checking enabled and do not send other
commands during this check. Requires the isolated reBot GenericSystem instance.
Checks intermediate hold/release, both bounds, and absence of arm drift.
"""
import json
import time
import xml.etree.ElementTree as ET

from control_msgs.msg import JointJog
from controller_manager_msgs.srv import ListControllers
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import String


def main():
    rclpy.init()
    node = rclpy.create_node("verify_rebot_mock_gripper_jog")
    description, samples = [], []
    node.create_subscription(
        String,
        "/robot_description",
        lambda m: description.append(m.data),
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
    )
    node.create_subscription(
        JointState,
        "/joint_states",
        lambda m: samples.append(dict(zip(m.name, m.position))),
        10,
    )
    deadline = time.monotonic() + 30
    while (not description or not samples) and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    if not description or not samples:
        raise RuntimeError("Missing model or joint states")
    robot = ET.fromstring(description[-1])
    if robot.get("name") != "rebot_b601_rs":
        raise RuntimeError("Unexpected robot")
    if [p.text for p in robot.findall("ros2_control/hardware/plugin")] != [
        "mock_components/GenericSystem"
    ]:
        raise RuntimeError("Refusing non-mock hardware")
    limit = robot.find("joint[@name='joint7']/limit")
    lower, upper = float(limit.get("lower")), float(limit.get("upper"))
    client = node.create_client(ListControllers, "/controller_manager/list_controllers")
    assert client.wait_for_service(timeout_sec=10)
    future = client.call_async(ListControllers.Request())
    rclpy.spin_until_future_complete(node, future, timeout_sec=10)
    assert future.done(), "Controller query timed out"
    controllers = {c.name: c for c in future.result().controller}
    gripper = controllers["gripper_joint_velocity_controller"]
    assert (
        gripper.state == "active"
    ), "Select gripper JointJog and start Teleoperate first"
    assert gripper.claimed_interfaces == ["joint7/position"]
    assert controllers["joint_trajectory_controller"].state == "inactive"
    # The manager may leave disjoint arm controllers active; only J7 ownership
    # must be exclusive. The measured arm-drift assertion below guards motion.
    assert all(
        "joint7/position" not in c.claimed_interfaces
        for name, c in controllers.items()
        if name != "gripper_joint_velocity_controller"
    )
    initial = samples[-1]
    publisher = node.create_publisher(JointJog, "/joint_jog/gripper", 10)
    results = []

    def hold(velocity, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            msg = JointJog()
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.joint_names = ["joint7"]
            msg.velocities = [velocity]
            publisher.publish(msg)
            rclpy.spin_once(node, timeout_sec=0.02)
            # Bound the command rate even when joint-state callbacks are frequent.
            time.sleep(0.02)

    def release():
        hold(0.0, 0.6)
        first = len(samples)
        end = time.monotonic() + 0.6
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
        stopped = [s["joint7"] for s in samples[first:]]
        assert len(stopped) > 2, "Joint state stream stopped"
        drift = max(stopped) - min(stopped)
        assert drift < 0.001, ("release drift", drift)
        results.append(
            {"joint7_rad": samples[-1]["joint7"], "release_drift_rad": drift}
        )

    def reach_bound(velocity, target):
        # Predictive stopping leaves clearance before the hard limit. At this
        # command speed the observed stop margin is about 0.04 rad; independently
        # check every measured sample against the exact URDF bounds below.
        deadline = time.monotonic() + 90
        while abs(samples[-1]["joint7"] - target) > 0.05:
            assert time.monotonic() < deadline, (
                "Failed to approach bound",
                samples[-1],
            )
            hold(velocity, 0.2)
        hold(velocity, 1.0)
        release()
        assert abs(samples[-1]["joint7"] - target) < 0.05

    try:
        reach_bound(-0.15, lower)
        start = samples[-1]["joint7"]
        hold(0.15, 1.5)
        release()
        assert start + 0.05 < samples[-1]["joint7"] < upper - 0.05
        reach_bound(0.15, upper)
        start = samples[-1]["joint7"]
        hold(-0.15, 1.5)
        release()
        assert lower + 0.05 < samples[-1]["joint7"] < start - 0.05
        reach_bound(-0.15, lower)
        assert all(lower - 1e-6 <= s["joint7"] <= upper + 1e-6 for s in samples)
        arm_drift = max(
            abs(s[j] - initial[j])
            for s in samples
            for j in [f"joint{i}" for i in range(1, 7)]
        )
        assert arm_drift < 0.001, ("arm drift", arm_drift)
        print(
            json.dumps(
                {
                    "checks": results,
                    "sample_count": len(samples),
                    "min_joint7_rad": min(s["joint7"] for s in samples),
                    "max_joint7_rad": max(s["joint7"] for s in samples),
                    "max_arm_drift_rad": arm_drift,
                }
            ),
            flush=True,
        )
    finally:
        hold(0.0, 0.3)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
