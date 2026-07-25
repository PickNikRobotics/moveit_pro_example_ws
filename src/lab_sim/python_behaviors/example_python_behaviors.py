#!/usr/bin/env python3

# Copyright 2026 PickNik Inc.
# All rights reserved.
#
# Unauthorized copying of this code base via any medium is strictly prohibited.
# Proprietary and confidential.

"""Example Behaviors written in Python.

Copy this directory into a robot config package (as ``python_behaviors/``) and add
``moveit_pro::behaviors::PythonBehaviorsLoader`` to that config's ``behavior_loader_plugins``
to see these in the Behavior catalog.
"""

from _behavior_math import joint_positions, largest_absolute_value

from moveit_pro_python_behavior import (
    Behavior,
    InputPort,
    NodeStatus,
    OutputPort,
)


class ScaleValue(Behavior):
    """Multiply a number by a factor and write the result to the blackboard."""

    subcategory = "Python Examples"

    @classmethod
    def provided_ports(cls):
        return [
            InputPort("value", float, description="The number to scale."),
            InputPort("factor", float, default=2.0, description="What to multiply by."),
            OutputPort("result", float, description="value multiplied by factor."),
        ]

    def on_tick(self):
        result = self.get_input("value") * self.get_input("factor")
        self.log_info(f"Scaled to {result}.")
        self.set_output("result", result)
        return NodeStatus.SUCCESS


class SummarizeJointState(Behavior):
    """Report the largest joint position magnitude in a JointState message.

    Shows two things at once: an ``any`` port receives a ROS message as a nested dict, and a
    Behavior can use ordinary third-party packages (here numpy, via a helper module that sits
    next to this file).
    """

    subcategory = "Python Examples"

    @classmethod
    def provided_ports(cls):
        return [
            InputPort(
                "joint_state",
                dict,
                description="A JointState or RobotJointState, e.g. from RetrieveWaypoint.",
            ),
            OutputPort(
                "largest_position",
                float,
                description="Largest absolute joint position, in radians.",
            ),
        ]

    def on_tick(self):
        positions = joint_positions(self.get_input("joint_state"))
        if not positions:
            self.publish_failure("The joint state carries no joint positions.")
            return NodeStatus.FAILURE

        largest = largest_absolute_value(list(positions.values()))
        self.log_info(
            f"{len(positions)} joints, largest absolute position {largest:.4f} rad."
        )
        self.set_output("largest_position", largest)
        return NodeStatus.SUCCESS


class MakePose(Behavior):
    """Build a PoseStamped from a position and write it to the blackboard.

    A port declared with a message type name is registered with that ROS message type, the same
    as a C++ Behavior's port. The dict written here is built into a real `PoseStamped`, so any
    Behavior downstream reads a message rather than text.
    """

    subcategory = "Python Examples"

    @classmethod
    def provided_ports(cls):
        return [
            InputPort(
                "x", float, default=0.0, description="Position along x, in meters."
            ),
            InputPort(
                "y", float, default=0.0, description="Position along y, in meters."
            ),
            InputPort(
                "z", float, default=0.0, description="Position along z, in meters."
            ),
            InputPort(
                "frame_id", str, default="world", description="Frame the pose is in."
            ),
            OutputPort(
                "pose",
                "geometry_msgs/msg/PoseStamped",
                description="The resulting pose.",
            ),
        ]

    def on_tick(self):
        self.set_output(
            "pose",
            {
                "header": {"frame_id": self.get_input("frame_id")},
                "pose": {
                    "position": {
                        "x": self.get_input("x"),
                        "y": self.get_input("y"),
                        "z": self.get_input("z"),
                    },
                    "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
                },
            },
        )
        return NodeStatus.SUCCESS


class CountTicks(Behavior):
    """Stay RUNNING for a number of ticks, then succeed.

    A Behavior that cannot finish in one tick returns ``RUNNING`` and picks up where it left
    off on the next tick. Sleeping instead would block the whole Objective's tree.
    """

    subcategory = "Python Examples"

    @classmethod
    def provided_ports(cls):
        return [
            InputPort(
                "ticks", int, default=5, description="How many ticks to run for."
            ),
        ]

    def on_start(self):
        self._remaining = self.get_input("ticks")
        return self.on_running()

    def on_running(self):
        if self._remaining <= 0:
            return NodeStatus.SUCCESS
        self._remaining -= 1
        return NodeStatus.RUNNING

    def on_halt(self):
        self.log_warn(f"Halted with {self._remaining} tick(s) left.")
