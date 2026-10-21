#!/usr/bin/env python3
# Copyright 2026 PickNik Inc.
# SPDX-License-Identifier: BSD-3-Clause
"""Driver thermal latch -> cancel -> native HOME -> fresh stationary HOME-only OFF.

One backend process owns thermal recovery even when the leader is disabled.
It never opens CAN/USB, enables torque, resets the latch, or changes lifecycle.
"""
import math
import signal
import time

import rclpy
from action_msgs.srv import CancelGoal
from moveit_studio_sdk_msgs.action import DoObjectiveSequence
from rclpy.action import ActionClient
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from robstride_interfaces.msg import RobstrideState
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_srvs.srv import SetBool

from rebot_leader import JOINT_NAMES

# Each stage has its own bound, so time spent canceling never shortens native HOME.
STAGE_LIMITS = {
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
        self.cancel = node.create_client(
            CancelGoal, "/do_objective/_action/cancel_goal"
        )
        self.home = ActionClient(node, DoObjectiveSequence, "/do_objective")
        self.torque = node.create_client(
            SetBool, "/robstride_hardware_interface/set_torque"
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
        if message.data and self.stage == "idle":
            self.stage = "cancel"
            self.started = time.monotonic()
            self.node.get_logger().error(
                "Driver thermal trip: canceling task, returning HOME before torque OFF"
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

    def hardware_fresh(self, now):
        state = self.hardware
        return (
            state is not None
            and 0 <= now - self.hardware_time < 0.5
            and list(state.id) == list(range(1, 8))
            and len(state.enabled) == 7
            and len(state.fault_bits) == 7
            and state.comm_state == 0
            and not any(state.fault_bits)
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
        if (
            self.stage in ("cancel", "home")
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
        if self.stage == "cancel":
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
            self.stage = "off"
            self.node.get_logger().error(
                "Thermal recovery complete: HOME and all seven motors OFF; latch retained"
            )


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
