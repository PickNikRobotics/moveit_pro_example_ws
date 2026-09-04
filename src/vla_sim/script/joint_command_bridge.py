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

"""Publish the commanded joint vector Trainer labels `action` from.

Conversion labels `action` from a `sensor_msgs/JointState` command topic, and
nothing in the MoveIt Pro tree publishes one outside of teleoperation. Without
it a scripted collection Objective converts only with **Action source: Next
state**, which labels each frame with the next frame's measured position rather
than with what the Objective actually commanded.

This node assembles the trajectory controller's reference setpoint with the
latched gripper command and republishes the pair as `/joint_commands`.
"""

from __future__ import annotations

import rclpy
from control_msgs.msg import JointTrajectoryControllerState
from rclpy.node import Node
from sensor_msgs.msg import JointState

# joint_trajectory_admittance_controller publishes ~/controller_state only while
# a goal executes, so a reference older than this means no trajectory is running
# and the arm should be labelled with the pose it is holding.
REFERENCE_TIMEOUT_S = 0.5

# The arm joints, in the order the trajectory controller reports them. Fixing the
# layout here rather than adopting the controller's keeps the recorded channel
# order stable across a session, whatever order the controller reports in.
DEFAULT_ARM_JOINT_NAMES = [f"joint_{index}" for index in range(1, 8)]


def assemble_joint_command(
    joint_order: list[str],
    gripper_joint: str,
    reference: dict[str, float],
    measured: dict[str, float],
    gripper_command: float,
) -> list[float] | None:
    """One position per name in ``joint_order``, or None if a joint has no source.

    Arm joints take the controller's setpoint when the caller passes a fresh one,
    otherwise their measured position, so a dwell is labelled with the pose the
    arm is holding. The gripper takes the latched command: its ``GripperCommand``
    goal is not observable on any topic, and it shares the arm's radian scale
    through the ``split`` tendon in ``description/mujoco/gen3_7dof.xml``.
    """
    positions: list[float] = []
    for joint in joint_order:
        if joint == gripper_joint:
            positions.append(float(gripper_command))
        elif joint in reference:
            positions.append(float(reference[joint]))
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
        self._arm_joint_names = list(
            self._param("arm_joint_names", DEFAULT_ARM_JOINT_NAMES)
        )
        self._gripper_joint = self._param(
            "gripper_joint_name", "robotiq_85_left_knuckle_joint"
        )
        # Latched by the gripper Objectives through SetRos2Parameter. float()
        # keeps the declared type DOUBLE, which is what SetRos2Parameter sends.
        self.declare_parameter("gripper_command_position", float(0.0))
        # Conversion nearest-time-syncs this stream against the camera stream
        # within 50 ms before resampling, so publish well inside that window
        # rather than at a 10 Hz period whose worst case sits exactly on it.
        publish_rate = float(self._param("publish_rate", 30.0))
        if publish_rate <= 0.0:
            raise ValueError(f"publish_rate must be > 0, got {publish_rate}")

        # ExecutePolicy's layout: the planning group's order, gripper last.
        self._joint_order = self._arm_joint_names + [self._gripper_joint]
        self._reference: dict[str, float] = {}
        self._reference_stamp = 0.0
        self._measured: dict[str, float] = {}
        self._measured_stamp = 0.0

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
        self.create_timer(1.0 / publish_rate, self._publish)
        self.get_logger().info(
            f"bridging {self._controller_state_topic} -> {self._joint_command_topic} "
            f"at {publish_rate:g} Hz for {self._joint_order}"
        )

    def _param(self, name: str, default):
        return self.declare_parameter(name, default).value

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _is_fresh(self, stamp: float) -> bool:
        return self._now() - stamp < REFERENCE_TIMEOUT_S

    def _on_controller_state(self, msg: JointTrajectoryControllerState) -> None:
        self._reference = dict(zip(msg.joint_names, msg.reference.positions))
        self._reference_stamp = self._now()

    def _on_joint_states(self, msg: JointState) -> None:
        self._measured = dict(zip(msg.name, msg.position))
        self._measured_stamp = self._now()

    def _publish(self) -> None:
        if not self._reference:
            # No trajectory has run, so there is no commanded stream to mirror.
            # Falling back to the measured position here would hand a
            # hand-teleoperated recording an `action` column that is a copy of
            # its own observations; conversion refuses an empty action topic
            # precisely so that mislabelling cannot happen silently.
            return
        positions = assemble_joint_command(
            joint_order=self._joint_order,
            gripper_joint=self._gripper_joint,
            reference=self._reference if self._is_fresh(self._reference_stamp) else {},
            # Held to the same freshness as the setpoint: the fallback labels a
            # dwell with the pose the arm is holding, which a stale reading is not.
            measured=self._measured if self._is_fresh(self._measured_stamp) else {},
            gripper_command=self.get_parameter("gripper_command_position").value,
        )
        if positions is None:
            self.get_logger().warning(
                "no fresh setpoint or measured position for some joint in "
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
