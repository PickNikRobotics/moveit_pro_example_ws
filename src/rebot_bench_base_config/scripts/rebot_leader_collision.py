# Copyright 2026 PickNik Inc.
# SPDX-License-Identifier: BSD-3-Clause
"""Check position targets with MoveIt Pro's native model and collision engine."""
import math
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from moveit_pro_base import (
    collision_detection,
    planning_scene,
    robot_model,
    robot_state,
)

from rebot_leader import JOINT_NAMES

# Path sample spacing per joint, rad: joint7's 0.008 m/rad mimic moves each finger
# 0.4 mm per 0.05 rad.
PATH_SPACING = (0.005,) * 6 + (0.05,)


class LeaderCollision:
    def __init__(self, urdf):
        self.directory = tempfile.TemporaryDirectory(prefix="rebot-leader-")
        description = Path(self.directory.name) / "robot.urdf"
        description.write_text(urdf)
        share = Path(get_package_share_directory("rebot_bench_base_config"))
        root = ET.fromstring(urdf)
        self.bounds = [
            (
                float(root.find(f"joint[@name='{name}']/limit").get("lower")),
                float(root.find(f"joint[@name='{name}']/limit").get("upper")),
            )
            for name in JOINT_NAMES
        ]
        self.model = robot_model.RobotModel(
            str(description), str(share / "config/moveit/rebot.srdf")
        )
        self.scene = planning_scene.PlanningScene(self.model)
        self.scene.set_link_padding(0.0)
        self.state = robot_state.RobotState(self.model)
        self.state.set_to_default_values()
        self.request = collision_detection.CollisionRequest()
        self.request.joint_model_group_name = "manipulator_and_gripper"
        self.request.contacts = True
        self.request.max_contacts = 100
        self.request.max_contacts_per_pair = 1
        self.request.pad_self_collisions = True
        self.request.pad_environment_collisions = True

    def update_world(self, message):
        if message.world.octomap.octomap.data:
            raise ValueError(
                "Octomap collision checking is unavailable in the position bridge"
            )
        if not message.is_diff:
            self.scene.remove_all_collision_objects()
        for obj in message.world.collision_objects:
            self.scene.apply_collision_object(obj)
        for attached in message.robot_state.attached_collision_objects:
            self.scene.apply_attached_collision_object_msg(attached)

    def check_pose(self, positions):
        self.state.joint_positions = dict(zip(JOINT_NAMES, positions))
        self.state.update()
        result = collision_detection.CollisionResult()
        self.scene.check_collision(self.request, result, self.state)
        if result.collision:
            return "Collision: " + ", ".join(
                " / ".join(pair) for pair in result.contacts
            )
        return ""

    def check_path(self, measured, target):
        if not all(
            lower - 1e-9 <= value <= upper + 1e-9
            for value, (lower, upper) in zip(target, self.bounds)
        ):
            return "Target exceeds robot joint bounds"
        steps = max(
            1,
            math.ceil(
                max(
                    abs(b - a) / spacing
                    for a, b, spacing in zip(measured, target, PATH_SPACING)
                )
            ),
        )
        for step in range(steps + 1):
            fraction = step / steps
            reason = self.check_pose(
                [a + fraction * (b - a) for a, b in zip(measured, target)]
            )
            if reason:
                return reason
        return ""
