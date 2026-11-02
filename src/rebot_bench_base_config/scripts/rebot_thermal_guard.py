#!/usr/bin/env python3
# Copyright 2026 PickNik Inc.
# SPDX-License-Identifier: BSD-3-Clause
"""Thermal latch -> measured-pose anchor -> configured gains -> cancel/HOME/OFF.

One backend process owns thermal recovery even when the leader is disabled.
It never opens CAN/USB, enables torque, or changes lifecycle. After successful
recovery it attests fresh HOME/OFF to the driver's cool-temperature clear check.
"""
import math
import signal
import time
import xml.etree.ElementTree as ET

import rclpy
from action_msgs.msg import GoalStatus
from action_msgs.srv import CancelGoal
from control_msgs.msg import JointTrajectoryControllerState
from moveit_pro_controllers_msgs.action import FollowJointTrajectoryWithAdmittance
from moveit_studio_sdk_msgs.action import DoObjectiveSequence
from rclpy.action import ActionClient
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from robstride_interfaces.msg import RobstrideState
from robstride_interfaces.srv import SetDataToRobstride
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String, UInt64
from std_srvs.srv import SetBool
from trajectory_msgs.msg import JointTrajectoryPoint

from rebot_leader import JOINT_NAMES

# Each stage has its own bound, so time spent canceling never shortens native HOME.
STAGE_LIMITS = {
    "anchor": (3.0, "Measured-pose anchor unavailable or not accepted"),
    "anchor_reference": (1.0, "Measured-pose controller reference not confirmed"),
    "restore": (7.0, "Configured spring gains not restored"),
    "cancel": (3.0, "Objective cancellation unavailable"),
    "home": (3.0, "Native HOME server unavailable or goal not accepted"),
    "result": (120.0, "Native HOME did not finish"),
    "verify": (5.0, "Fresh stationary HOME not confirmed"),
    "disable": (3.0, "HOME-only OFF did not respond"),
    "confirm": (3.0, "Motors did not report OFF"),
}


class ThermalGuard:
    def __init__(self, node):
        self.node = node
        self.stage = "idle"
        self.started = None
        self.future = None
        self.home_handle = None
        self.hardware = None
        self.hardware_time = -math.inf
        self.measured = None
        self.measured_time = -math.inf
        self.home_since = None
        self.hardware_stamp_ns = self.measured_stamp_ns = -1
        self.next_clear = -math.inf
        self.gain_commands = None
        self.gain_index = 0
        self.gain_started = None
        self.anchor_handle = self.anchor_result = None
        self.anchor_positions = self.reference_positions = None
        self.anchor_sent_ns = self.reference_stamp_ns = -1
        self.reference_time = -math.inf
        self.reference_samples = 0
        self.anchor = ActionClient(
            node,
            FollowJointTrajectoryWithAdmittance,
            "/joint_trajectory_admittance_controller/follow_joint_trajectory",
        )
        self.gains = node.create_client(
            SetDataToRobstride, "/robstride_hardware_interface/set_data_to_robstride"
        )
        node.create_subscription(
            String,
            "/robot_description",
            self.observe_description,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        node.create_subscription(
            JointTrajectoryControllerState,
            "/joint_trajectory_admittance_controller/controller_state",
            self.observe_reference,
            qos_profile_sensor_data,
        )
        self.cancel = node.create_client(
            CancelGoal, "/do_objective/_action/cancel_goal"
        )
        self.home = ActionClient(node, DoObjectiveSequence, "/do_objective")
        self.torque = node.create_client(
            SetBool, "/robstride_hardware_interface/set_torque"
        )
        self.clear = node.create_publisher(
            UInt64, "/robstride_hardware_interface/thermal_rest_verified", 1
        )
        node.create_subscription(
            Bool,
            "/robstride_hardware_interface/thermal_tripped",
            self.trip,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        node.create_subscription(
            JointState,
            "/joint_states",
            self.observe_joints,
            qos_profile_sensor_data,
        )
        node.create_subscription(
            RobstrideState,
            "/robstride_hardware_interface/robstride_state",
            self.observe_hardware,
            qos_profile_sensor_data,
        )
        node.create_timer(0.02, self.tick)

    def trip(self, message):
        if not message.data and self.stage == "cooling":
            self.stage, self.future, self.started = "idle", None, None
            self.home_since = None
            self.node.get_logger().info(
                "Thermal latch cleared; torque stays OFF, explicitly restart the Objective at HOME"
            )
        if message.data and self.stage == "idle":
            self.stage, self.future = "anchor", None
            self.started = time.monotonic()
            self.anchor_handle = self.anchor_result = None
            self.anchor_positions = self.reference_positions = None
            self.anchor_sent_ns = self.reference_stamp_ns = -1
            self.reference_time = -math.inf
            self.reference_samples = self.gain_index = 0
            self.gain_started = None
            self.node.get_logger().error(
                "Driver thermal trip: re-anchoring measured pose and restoring spring gains "
                "before cancel/HOME/torque OFF"
            )

    def observe_description(self, message):
        """Keep initial driver configuration, not float-modified service values."""
        if self.gain_commands is not None:
            return
        try:
            controls = [
                element
                for element in ET.fromstring(message.data).findall("ros2_control")
                if element.findtext("hardware/plugin")
                == "robstride_hardware_interface/RobstrideHardware"
            ]
            if len(controls) != 1:
                raise ValueError("expected one Robstride hardware component")
            control = controls[0]
            gains = {}
            for motor_id, name in enumerate(JOINT_NAMES[:6], 1):
                joints = control.findall(f"joint[@name='{name}']")
                if len(joints) != 1 or joints[0].findtext("param[@name='id']") != str(
                    motor_id
                ):
                    raise ValueError(f"{name} must map to motor {motor_id}")
                gpios = [
                    gpio
                    for gpio in control.findall("gpio")
                    if gpio.findtext("param[@name='ID']") == str(motor_id)
                ]
                if len(gpios) != 1:
                    raise ValueError(f"motor {motor_id} needs one GPIO configuration")
                for gain in ("kd", "kp"):
                    value = float(gpios[0].findtext(f"param[@name='{gain}']"))
                    if not math.isfinite(value) or not 0 <= value <= 3.402823e38:
                        raise ValueError(f"motor {motor_id} has invalid {gain}")
                    gains[motor_id, gain] = value
            # One outstanding service request; damping first, jaw gains untouched.
            self.gain_commands = tuple(
                (motor_id, gain, gains[motor_id, gain])
                for gain in ("kd", "kp")
                for motor_id in range(1, 7)
            )
        except (ET.ParseError, TypeError, ValueError) as exc:
            self.node.get_logger().error(
                f"Thermal spring configuration unavailable: {exc}"
            )

    def observe_reference(self, message):
        if self.stage not in ("anchor_reference", "restore"):
            return
        if len(message.joint_names) != len(message.reference.positions) or len(
            set(message.joint_names)
        ) != len(message.joint_names):
            return
        lookup = dict(zip(message.joint_names, message.reference.positions))
        if not all(
            name in lookup and math.isfinite(lookup[name]) for name in JOINT_NAMES
        ):
            return
        age = self.source_age(
            message, max(self.reference_stamp_ns, self.anchor_sent_ns), 0.1
        )
        if age is None:
            return
        self.reference_stamp_ns = Time.from_msg(message.header.stamp).nanoseconds
        self.reference_time = time.monotonic() - age
        self.reference_positions = [lookup[name] for name in JOINT_NAMES]
        if self.reference_matches_anchor():
            self.reference_samples += 1
        else:
            self.reference_samples = 0

    def reference_matches_anchor(self):
        return (
            self.anchor_positions is not None
            and self.reference_positions is not None
            and all(
                abs(a - b) <= 0.002
                for a, b in zip(self.anchor_positions, self.reference_positions)
            )
        )

    def source_age(self, message, last_stamp_ns, maximum_age, now_ns=None):
        stamp = Time.from_msg(message.header.stamp)
        if now_ns is None:
            now_ns = self.node.get_clock().now().nanoseconds
        age = (now_ns - stamp.nanoseconds) / 1e9
        if stamp.nanoseconds <= last_stamp_ns or not 0 <= age < maximum_age:
            return None
        return age

    def observe_hardware(self, message):
        # ros2_control passes its steady update clock to the driver's read().
        # JointState stamps below use ROS time; motor-state stamps use CLOCK_MONOTONIC.
        age = self.source_age(
            message, self.hardware_stamp_ns, 0.5, now_ns=time.monotonic_ns()
        )
        if age is None:
            return
        self.hardware_stamp_ns = Time.from_msg(message.header.stamp).nanoseconds
        self.hardware = message
        self.hardware_time = time.monotonic() - age

    def observe_joints(self, message):
        if (
            len(message.name) != len(message.position)
            or len(message.name) != len(message.velocity)
            or len(set(message.name)) != len(message.name)
        ):
            return
        lookup = dict(zip(message.name, zip(message.position, message.velocity)))
        if not all(
            name in lookup and all(math.isfinite(v) for v in lookup[name])
            for name in JOINT_NAMES
        ):
            return
        age = self.source_age(message, self.measured_stamp_ns, 0.1)
        if age is None:
            return
        self.measured_stamp_ns = Time.from_msg(message.header.stamp).nanoseconds
        self.measured = [lookup[name] for name in JOINT_NAMES]
        self.measured_time = time.monotonic() - age

    def hardware_fresh(self, now, *, ignore_history=False):
        state = self.hardware
        return (
            state is not None
            and 0 <= now - self.hardware_time < 0.5
            and list(state.id) == list(range(1, 8))
            and len(state.enabled) == 7
            and len(state.fault_bits) == 7
            and state.comm_state == 0
            and (ignore_history or not any(state.fault_bits))
        )

    def at_home(self, now):
        return (
            self.measured is not None
            and 0 <= now - self.measured_time < 0.1
            and all(abs(q) <= 0.08 for q, v in self.measured[:6])
            and all(abs(v) <= 0.01 for q, v in self.measured)
        )

    def fail(self, reason):
        self.stage = "failed"
        self.node.get_logger().error(
            reason
            + "; no further torque commands, thermal latch remains, intervention required"
        )

    def tick(self):
        if self.stage in ("idle", "failed", "off"):
            return
        now = time.monotonic()
        try:
            self.advance(now)
        except Exception as exc:
            self.fail(f"Thermal recovery unavailable: {exc}")

    def advance(self, now):
        if self.stage == "cooling":
            if (
                now >= self.next_clear
                # Healthy Type-2 clears current faults, not diagnostic history.
                and self.hardware_fresh(now, ignore_history=True)
                and not any(self.hardware.enabled)
                and self.at_home(now)
            ):
                # Same-host steady clock as motor feedback. Carry the measured
                # HOME sample's time, so even a delayed DDS delivery expires.
                message = UInt64()
                message.data = int(self.measured_time * 1e9)
                self.clear.publish(message)
                self.next_clear = now + 1.0
            return
        if (
            self.stage in ("anchor", "anchor_reference", "restore", "cancel", "home")
            and self.hardware_fresh(now)
            and not any(self.hardware.enabled)
        ):
            # A hot arm detected during initial probing stays OFF; no cancel,
            # HOME motion or Enable is needed or permitted in this case.
            self.stage = "off"
            self.node.get_logger().error(
                "Thermal latch retained; all motors already OFF"
            )
            return
        limit, reason = STAGE_LIMITS[self.stage]
        if now - self.started > limit:
            self.fail(reason)
            return
        if self.stage in ("anchor", "anchor_reference", "restore"):
            if self.stage == "anchor" and self.future is None and self.hardware is None:
                # The latched notification can arrive before the first motor sample.
                # Wait within the anchor bound, preserving the hot/all-OFF startup path.
                return
            if not self.hardware_fresh(now) or not all(self.hardware.enabled):
                self.fail(
                    "Motor state stale, faulted or partly OFF before spring restore"
                )
                return
            if self.measured is None or not 0 <= now - self.measured_time < 0.1:
                self.fail("Measured pose stale before spring restore")
                return
            self.restore_spring(now)
        elif self.stage == "cancel":
            if self.future is None:
                if self.cancel.service_is_ready():
                    self.future = self.cancel.call_async(CancelGoal.Request())
            elif self.future.done():
                result = self.future.result()
                if result is None or result.return_code != 0:
                    self.fail("Objective cancellation rejected")
                    return
                self.stage, self.future, self.started = "home", None, now
        elif self.stage == "home":
            # Let the canceled Objective and leader bridge relinquish control.
            if now - self.started < 0.5:
                return
            if not self.hardware_fresh(now):
                self.fail("Motor state stale or faulted")
                return
            if not all(self.hardware.enabled):
                self.fail("Some motors are OFF; HOME cannot safely recover")
                return
            if self.future is None:
                if self.home.server_is_ready():
                    goal = DoObjectiveSequence.Goal()
                    goal.objective_name = "Return reBot Leader Home"
                    goal.caller.type = goal.caller.SCRIPT
                    goal.caller.name = "rebot_thermal_guard"
                    self.future = self.home.send_goal_async(goal)
            elif self.future.done():
                self.home_handle = self.future.result()
                if self.home_handle is None or not self.home_handle.accepted:
                    self.fail("Native HOME rejected")
                    return
                self.stage, self.started = "result", now
                self.future = self.home_handle.get_result_async()
        elif self.stage == "result" and self.future.done():
            result = self.future.result()
            if (
                result is None
                or result.status != 4
                or result.result.error_code.val != 1
            ):
                self.fail("Native HOME failed")
                return
            self.stage, self.future, self.started = "verify", None, now
        elif self.stage == "verify":
            if not self.hardware_fresh(now) or not self.at_home(now):
                self.home_since = None
                return
            if self.home_since is None:
                self.home_since = now
            if now - self.home_since < 0.5:
                return
            if not self.torque.service_is_ready():
                self.fail("Torque service unavailable at HOME")
                return
            request = SetBool.Request()
            request.data = False
            self.future = self.torque.call_async(request)
            self.stage, self.started = "disable", now
        elif self.stage == "disable" and self.future.done():
            result = self.future.result()
            if result is None or not result.success:
                self.fail("HOME-only OFF rejected")
                return
            self.stage, self.started = "confirm", now
        elif (
            self.stage == "confirm"
            and self.hardware_fresh(now)
            and not any(self.hardware.enabled)
        ):
            self.stage = "cooling"
            self.node.get_logger().error(
                "Thermal recovery complete: HOME and all seven motors OFF; waiting for cooling"
            )

    def restore_spring(self, now):
        if self.stage == "anchor":
            if self.future is None:
                if self.gain_commands is None or not self.anchor.server_is_ready():
                    return
                stamp = self.node.get_clock().now()
                if stamp.nanoseconds <= 0:
                    self.fail(
                        "A nonzero ROS stamp is required for the measured-pose anchor"
                    )
                    return
                self.anchor_positions = [q for q, _ in self.measured]
                self.anchor_sent_ns = stamp.nanoseconds
                goal = FollowJointTrajectoryWithAdmittance.Goal()
                goal.trajectory.header.stamp = stamp.to_msg()
                goal.trajectory.joint_names = list(JOINT_NAMES)
                # Keep the stationary goal alive through the bounded gain writes so
                # reference/goal status remain observable. Native HOME supersedes it.
                for seconds in (0, 10):
                    point = JointTrajectoryPoint(
                        positions=self.anchor_positions,
                        velocities=[0.0] * 7,
                        accelerations=[0.0] * 7,
                    )
                    point.time_from_start.sec = seconds
                    goal.trajectory.points.append(point)
                self.future = self.anchor.send_goal_async(goal)
            elif self.future.done():
                self.anchor_handle = self.future.result()
                if self.anchor_handle is None or not self.anchor_handle.accepted:
                    self.fail("Measured-pose anchor rejected; gains unchanged")
                    return
                self.anchor_result = self.anchor_handle.get_result_async()
                self.stage, self.future, self.started = "anchor_reference", None, now
            return
        if (
            self.anchor_handle is not None
            and self.anchor_handle.status
            not in (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING)
        ) or (self.anchor_result is not None and self.anchor_result.done()):
            self.fail(
                "Measured-pose anchor ended or was preempted before spring restore"
            )
            return
        confirmed = (
            self.reference_samples >= 2
            and 0 <= now - self.reference_time < 0.1
            and self.reference_matches_anchor()
        )
        if self.stage == "anchor_reference":
            # A reference is published before the command write in installed JTAC.
            # Two advancing matching samples witness an intervening controller cycle;
            # action acceptance or a single desired-state publication is not enough.
            if confirmed:
                self.stage, self.started = "restore", now
            return
        if not confirmed:
            self.fail(
                "Measured-pose controller reference changed or stale before gain write"
            )
            return
        if self.future is None:
            if not self.gains.service_is_ready():
                return
            motor_id, gain, value = self.gain_commands[self.gain_index]
            request = SetDataToRobstride.Request()
            request.header.stamp = self.node.get_clock().now().to_msg()
            request.id, request.item_name, request.item_data = motor_id, gain, value
            self.future = self.gains.call_async(request)
            self.gain_started = now
        elif now - self.gain_started > 0.5:
            self.fail(
                "Spring gain service timed out; earlier queued gains are not rolled back"
            )
        elif self.future.done():
            response = self.future.result()
            if response is None or not response.result:
                self.fail(
                    "Spring gain service rejected the write; earlier gains remain"
                )
                return
            self.gain_index += 1
            self.future = None
            if self.gain_index == len(self.gain_commands):
                self.node.get_logger().info(
                    "Measured-pose anchor confirmed; all 12 configured arm gains queued, "
                    "continuing cancel/HOME/OFF (ACK is not motor gain readback)"
                )
                self.stage, self.started = "cancel", now


def main():
    # rclpy's own handler would shut the context down under the cleanup below.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)

    def request_shutdown(signum, frame):
        del signum, frame
        # One stop request is enough; a launch forwards its own SIGINT after
        # the terminal's.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, request_shutdown)
    signal.signal(signal.SIGTERM, request_shutdown)
    node = rclpy.create_node("rebot_thermal_guard")
    try:
        guard = ThermalGuard(node)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
