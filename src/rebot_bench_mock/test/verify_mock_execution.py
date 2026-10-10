#!/usr/bin/env python3
# Copyright 2026 PickNik Inc.
# SPDX-License-Identifier: BSD-3-Clause
"""Run after launching rebot_bench_mock in an isolated ROS graph.

Refuses to send any goal unless the latched robot description identifies this
model and its sole hardware plugin is GenericSystem. Prints JSON evidence.
"""
import json
from math import radians
import time
import xml.etree.ElementTree as ET

from moveit_msgs.srv import GetPlanningScene
from moveit_studio_sdk_msgs.action import DoObjectiveSequence
from moveit_studio_sdk_msgs.msg import BehaviorParameter
import rclpy
from rclpy.action import ActionClient
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import String


def main():
    rclpy.init()
    node = rclpy.create_node("verify_rebot_mock_execution")
    description = []
    state = {}
    node.create_subscription(
        String,
        "/robot_description",
        lambda msg: description.append(msg.data),
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
    )
    node.create_subscription(
        JointState,
        "/joint_states",
        lambda msg: state.update(zip(msg.name, msg.position)),
        10,
    )
    deadline = time.monotonic() + 30
    while (not description or not state) and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    assert description and state, "No robot description/joint states"
    robot = ET.fromstring(description[-1])
    if robot.get("name") != "rebot_b601_rs":
        raise RuntimeError("Unexpected robot")
    plugins = [e.text for e in robot.findall("ros2_control/hardware/plugin")]
    if plugins != ["mock_components/GenericSystem"]:
        raise RuntimeError("Refusing non-mock hardware")
    # The Objective action can appear before move_group finishes initialization.
    planning_scene = node.create_client(GetPlanningScene, "/get_planning_scene")
    if not planning_scene.wait_for_service(timeout_sec=60.0):
        raise RuntimeError("Planning scene service unavailable after 60 seconds")
    node.destroy_client(planning_scene)
    client = ActionClient(node, DoObjectiveSequence, "/do_objective")
    assert client.wait_for_server(timeout_sec=30), "Objective server unavailable"

    def wait(future, timeout=100):
        rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
        assert future.done(), "Objective request timed out"
        return future.result()

    raised = dict(zip([f"joint{i}" for i in range(1, 7)], [0, 0.65, 0.85, 0.3, 0, 0]))
    cases = [
        ("Move reBot to Waypoint", "Raised", raised),
        ("Open Gripper", None, {"joint7": radians(310)}),
        ("Close Gripper", None, {"joint7": 0}),
        ("Move reBot to Waypoint", "Zeroed Rest", dict.fromkeys(raised, 0)),
    ]
    for objective, waypoint, target in cases:
        goal = DoObjectiveSequence.Goal()
        goal.objective_name = objective
        goal.caller.type = goal.caller.SCRIPT
        goal.caller.name = "rebot_mock_acceptance"
        if waypoint:
            param = BehaviorParameter()
            param.description.name = "waypoint_name"
            param.description.type = param.description.TYPE_STRING
            param.string_value = waypoint
            goal.parameter_overrides = [param]
        started = time.monotonic()
        handle = wait(client.send_goal_async(goal))
        assert handle.accepted, f"Rejected: {objective} {waypoint}"
        result = wait(handle.get_result_async())
        assert result.result.error_code.val == 1, result.result
        for _ in range(5):
            rclpy.spin_once(node, timeout_sec=0.1)
        max_error = max(abs(state[j] - value) for j, value in target.items())
        assert max_error < 0.01, (objective, state, target)
        print(
            json.dumps(
                {
                    "objective": objective,
                    "waypoint": waypoint,
                    "error_code": result.result.error_code.val,
                    "elapsed_seconds": round(time.monotonic() - started, 3),
                    "max_joint_error_rad": max_error,
                    "joint_positions_rad": state,
                }
            ),
            flush=True,
        )
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
