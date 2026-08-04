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

"""Publish the training `action` and `observation.state` streams.

Forge labels `action` from a joint command topic, which no MoveIt Pro Behavior
publishes. This node assembles the trajectory controller's reference setpoint
with the latched gripper command and republishes both as a
`sensor_msgs/JointState`.

It publishes the same eight joints as `observation.state`, which is the width
the policies are trained on. Point MOVEIT_PRO_TRAIN_JOINT_STATES_TOPIC at that
topic; the default `/joint_states` carries all fifteen, eight of them passive
Robotiq linkage.

    ros2 run vla_sim joint_command_bridge.py \\
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

# The arm joints, in the order the trajectory controller reports them. The layout
# is fixed here so both streams publish from the first tick; the controller stays
# silent until a trajectory runs, several frames into an episode.
DEFAULT_ARM_JOINT_NAMES = [f"joint_{index}" for index in range(1, 8)]


def is_reference_fresh(now: float, stamp: float, timeout: float) -> bool:
    """True while ``stamp`` is within ``timeout`` of ``now``, and not ahead of it.

    A stamp in the future means the clock jumped backwards, which sim time does on a scene
    reset. Reading the elapsed time as a small positive number would carry the previous
    episode's setpoint into the next recording.
    """
    elapsed = now - stamp
    return 0.0 <= elapsed < timeout


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
        self._observation_state_topic = self._param(
            "observation_state_topic", "/observed_joint_states"
        )
        # Resolved, not compared as written: observation_state_topic comes from
        # MOVEIT_PRO_TRAIN_JOINT_STATES_TOPIC, where a relative "joint_states" names the same
        # topic as "/joint_states". Publishing there would put this node's eight joints on the
        # topic it reads all fifteen from.
        resolved_observation = self.resolve_topic_name(self._observation_state_topic)
        if resolved_observation == self.resolve_topic_name(self._joint_states_topic):
            raise ValueError(
                f"observation_state_topic '{self._observation_state_topic}' and "
                f"joint_states_topic '{self._joint_states_topic}' both resolve to "
                f"'{resolved_observation}'. Record the raw stream by pointing "
                "MOVEIT_PRO_TRAIN_JOINT_STATES_TOPIC at it and disabling this node, not by "
                "republishing onto it."
            )
        self._arm_joint_names = list(
            self._param("arm_joint_names", DEFAULT_ARM_JOINT_NAMES)
        )
        self._gripper_joint = self._param(
            "gripper_joint_name", "robotiq_85_left_knuckle_joint"
        )
        self._gripper_fallback = float(self._param("gripper_rest_position", 0.0))
        # float(): keeps the declared type DOUBLE, which is what SetROS2Parameter
        # sends at collection time.
        self.declare_parameter("gripper_command_position", self._gripper_fallback)
        # Forge resamples every stream onto the dataset's 1/fps grid.
        self._publish_rate = float(self._param("publish_rate", 10.0))
        if self._publish_rate <= 0.0:
            raise ValueError(f"publish_rate must be > 0, got {self._publish_rate}")
        self._reference_timeout = float(
            self._param("reference_timeout", DEFAULT_REFERENCE_TIMEOUT_S)
        )

        # One tuple, so names and positions are always read from the same message.
        self._reference: tuple[list[str], list[float]] | None = None
        self._reference_stamp = 0.0
        self._measured: dict[str, float] = {}
        self._measured_stamp = 0.0
        # ExecutePolicy's layout: the planning group's order, gripper last.
        self._joint_order = self._arm_joint_names + (
            [self._gripper_joint] if self._gripper_joint else []
        )

        self._publisher = self.create_publisher(
            JointState, self._joint_command_topic, 10
        )
        self._state_publisher = self.create_publisher(
            JointState, self._observation_state_topic, 10
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
            f"and {self._joint_states_topic} -> {self._observation_state_topic} "
            f"at {self._publish_rate:g} Hz for {self._joint_order}"
        )

    def _param(self, name: str, default):
        return self.declare_parameter(name, default).value

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_controller_state(self, msg: JointTrajectoryControllerState) -> None:
        self._reference = (list(msg.joint_names), list(msg.reference.positions))
        self._reference_stamp = self._now()
        if msg.joint_names and list(msg.joint_names) != self._arm_joint_names:
            # The configured layout stands for the whole episode; report and keep
            # the channel order stable.
            self.get_logger().error(
                f"{self._controller_state_topic} reports {list(msg.joint_names)}, not "
                f"the configured arm_joint_names {self._arm_joint_names}; recorded "
                "channels will not match the controller. Fix arm_joint_names.",
                throttle_duration_sec=30.0,
            )

    def _on_joint_states(self, msg: JointState) -> None:
        self._measured = dict(zip(msg.name, msg.position))
        self._measured_stamp = self._now()

    def _publish(self) -> None:
        # One reading for both streams, so they agree on what counts as fresh this tick.
        now = self._now()
        self._publish_action(now)
        self._publish_observation(now)

    def _publish_action(self, now: float) -> None:
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
                now, self._reference_stamp, self._reference_timeout
            ),
            # Held to the same freshness as the observation stream: the fallback labels a
            # dwell with the pose the arm is holding, which a stale reading is not.
            measured=(
                self._measured
                if is_reference_fresh(
                    now, self._measured_stamp, self._reference_timeout
                )
                else {}
            ),
            gripper_cmd=self.get_parameter("gripper_command_position").value,
            gripper_fallback=self._gripper_fallback,
        )
        if positions is None:
            self.get_logger().warning(
                f"no fresh setpoint or measured position for some joint in "
                f"{self._joint_order}; not publishing {self._joint_command_topic}.",
                throttle_duration_sec=10.0,
            )
            return
        self._publisher.publish(self._joint_state(positions))

    def _publish_observation(self, now: float) -> None:
        # Stale positions are dropped rather than restamped, so a frozen arm stays
        # distinguishable from a still one.
        if not is_reference_fresh(now, self._measured_stamp, self._reference_timeout):
            self.get_logger().warning(
                f"no {self._joint_states_topic} within {self._reference_timeout:g}s; "
                f"not publishing {self._observation_state_topic}.",
                throttle_duration_sec=10.0,
            )
            return
        missing = [j for j in self._joint_order if j not in self._measured]
        if missing:
            self.get_logger().warning(
                f"{self._joint_states_topic} carries no {missing}; not publishing "
                f"{self._observation_state_topic}.",
                throttle_duration_sec=10.0,
            )
            return
        self._state_publisher.publish(
            self._joint_state([self._measured[j] for j in self._joint_order])
        )

    def _joint_state(self, positions: list[float]) -> JointState:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self._joint_order)
        msg.position = positions
        return msg


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
