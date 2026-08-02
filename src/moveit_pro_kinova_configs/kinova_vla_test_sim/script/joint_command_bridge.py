#!/usr/bin/env python3
"""Publish /joint_commands, which Forge records as the training `action`.

Forge labels `action` from a joint command topic and falls back to next-state
labels when there is none; no MoveIt Pro Behavior publishes one. This node
assembles the trajectory controller's reference setpoint with the latched
gripper command and republishes both as a `sensor_msgs/JointState`.

    ros2 run kinova_vla_test_sim joint_command_bridge.py \\
        --ros-args -p publish_rate:=10.0
"""

from __future__ import annotations

import rclpy
from control_msgs.msg import JointTrajectoryControllerState
from rclpy.node import Node
from sensor_msgs.msg import JointState

# joint_trajectory_admittance_controller publishes ~/controller_state only while
# a goal executes, so a reference older than this means no trajectory is running
# and the arm should be labelled with the pose it is holding.
DEFAULT_REFERENCE_TIMEOUT_S = 0.5


def is_reference_fresh(now: float, stamp: float, timeout: float) -> bool:
    return (now - stamp) < timeout


def assemble_joint_command(
    joint_order: list[str],
    gripper_joint: str | None,
    reference_names: list[str] | None,
    reference_positions: list[float] | None,
    reference_fresh: bool,
    measured: dict[str, float],
    gripper_cmd: float | None,
    gripper_fallback: float,
) -> list[float] | None:
    """One position per name in ``joint_order``, or None if a joint has no source.

    Arm joints take the controller's setpoint while it is fresh, otherwise their
    measured position. The gripper takes the latched command: its
    ``GripperCommand`` goal is not observable on any topic, and it shares the
    arm's radian scale through the ``split`` tendon in
    ``description/mujoco/gen3_7dof.xml``.
    """
    reference_map: dict[str, float] = {}
    if (
        reference_fresh
        and reference_names
        and reference_positions
        and len(reference_names) == len(reference_positions)
    ):
        reference_map = dict(zip(reference_names, reference_positions))

    positions: list[float] = []
    for joint in joint_order:
        if gripper_joint is not None and joint == gripper_joint:
            positions.append(
                float(gripper_cmd)
                if gripper_cmd is not None
                else float(gripper_fallback)
            )
        elif joint in reference_map:
            positions.append(float(reference_map[joint]))
        elif joint in measured:
            positions.append(float(measured[joint]))
        else:
            return None
    return positions


class JointCommandBridge(Node):
    def __init__(self) -> None:
        super().__init__("joint_command_bridge")

        self._controller_state_topic = self._param(
            "controller_state_topic",
            "/joint_trajectory_admittance_controller/controller_state",
        )
        self._joint_states_topic = self._param("joint_states_topic", "/joint_states")
        self._joint_command_topic = self._param(
            "joint_command_topic", "/joint_commands"
        )
        self._gripper_joint = self._param(
            "gripper_joint_name", "robotiq_85_left_knuckle_joint"
        )
        self._gripper_fallback = float(self._param("gripper_rest_position", 0.0))
        # float(): an integer in YAML would declare this INTEGER and reject every
        # SetROS2Parameter double at collection time.
        self.declare_parameter("gripper_command_position", self._gripper_fallback)
        # Forge resamples every stream onto the dataset's 1/fps grid.
        self._publish_rate = float(self._param("publish_rate", 10.0))
        if self._publish_rate <= 0.0:
            raise ValueError(f"publish_rate must be > 0, got {self._publish_rate}")
        self._reference_timeout = float(
            self._param("reference_timeout", DEFAULT_REFERENCE_TIMEOUT_S)
        )

        # One tuple: a torn read could zip new names against old positions of the
        # same length, which the length guard above cannot catch.
        self._reference: tuple[list[str], list[float]] | None = None
        self._reference_stamp = 0.0
        self._measured: dict[str, float] = {}
        self._joint_order: list[str] | None = None

        self._publisher = self.create_publisher(
            JointState, self._joint_command_topic, 10
        )
        self.create_subscription(
            JointTrajectoryControllerState,
            self._controller_state_topic,
            self._on_controller_state,
            10,
        )
        self.create_subscription(
            JointState, self._joint_states_topic, self._on_joint_states, 10
        )
        self.create_timer(1.0 / self._publish_rate, self._publish)
        self.get_logger().info(
            f"bridging {self._controller_state_topic} -> {self._joint_command_topic} "
            f"at {self._publish_rate:g} Hz"
        )

    def _param(self, name: str, default):
        return self.declare_parameter(name, default).value

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_controller_state(self, msg: JointTrajectoryControllerState) -> None:
        self._reference = (list(msg.joint_names), list(msg.reference.positions))
        self._reference_stamp = self._now()
        if self._joint_order is None and msg.joint_names:
            # ExecutePolicy's layout: the planning group's order, gripper last.
            order = list(msg.joint_names)
            if self._gripper_joint:
                order.append(self._gripper_joint)
            self._joint_order = order

    def _on_joint_states(self, msg: JointState) -> None:
        self._measured = dict(zip(msg.name, msg.position))

    def _publish(self) -> None:
        if self._joint_order is None:
            self.get_logger().warning(
                f"no message yet on {self._controller_state_topic}; not publishing "
                f"{self._joint_command_topic}. The trajectory controller publishes "
                "only while a goal runs, so this is expected before the first "
                "trajectory and a misconfiguration after it.",
                throttle_duration_sec=10.0,
            )
            return
        if (
            self._gripper_joint
            and self._measured
            and self._gripper_joint not in self._measured
        ):
            self.get_logger().warning(
                f"gripper_joint_name '{self._gripper_joint}' is not in "
                f"{self._joint_states_topic}; not publishing "
                f"{self._joint_command_topic}. Check the configured joint name.",
                throttle_duration_sec=30.0,
            )
            return

        reference_names, reference_positions = self._reference or (None, None)
        positions = assemble_joint_command(
            joint_order=self._joint_order,
            gripper_joint=self._gripper_joint or None,
            reference_names=reference_names,
            reference_positions=reference_positions,
            reference_fresh=is_reference_fresh(
                self._now(), self._reference_stamp, self._reference_timeout
            ),
            measured=self._measured,
            gripper_cmd=self.get_parameter("gripper_command_position").value,
            gripper_fallback=self._gripper_fallback,
        )
        if positions is None:
            self.get_logger().warning(
                f"no setpoint or measured position for some joint in "
                f"{self._joint_order}; not publishing {self._joint_command_topic}.",
                throttle_duration_sec=10.0,
            )
            return

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self._joint_order)
        msg.position = positions
        self._publisher.publish(msg)


def main() -> None:
    rclpy.init()
    node = JointCommandBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
