#!/usr/bin/env python3
# Copyright 2026 PickNik Inc.
# SPDX-License-Identifier: BSD-3-Clause
"""Stream calibrated positions through a native controller and FCL collision checks."""
import argparse
import json
import math
import os
import signal
from pathlib import Path
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

import rclpy
from moveit_studio_sdk_msgs.action import DoObjectiveSequence
from rclpy.action import ActionClient
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String, Float64MultiArray
from moveit_msgs.msg import PlanningScene, PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene
from std_srvs.srv import Trigger

from rebot_leader import JOINT_NAMES, LIMITS_DEG
from rebot_leader_state import LeaderState, joint_label, operator_fault
from rebot_leader_collision import LeaderCollision


class LeaderBridge:
    """One position-command owner; native FCL checks every target and path."""

    def __init__(self, node, stream):
        self.node = node
        self.stream = stream
        self.state = LeaderState()
        self.real_hardware = False
        self.thermal_tripped = False
        self.hardware_state = None
        self.hardware_time = -math.inf
        from robstride_interfaces.msg import RobstrideState

        node.create_subscription(
            RobstrideState,
            "/robstride_hardware_interface/robstride_state",
            self.observe_hardware,
            qos_profile_sensor_data,
        )
        node.create_subscription(
            Bool,
            "/robstride_hardware_interface/thermal_tripped",
            self.observe_thermal,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self.start_checked = False
        self.prepared_at = None
        self.warmup_heartbeat = -math.inf
        self.collider = None
        self.scene_ready = False
        self.scene_future = None
        self.scene_client = node.create_client(GetPlanningScene, "/get_planning_scene")
        self.warmup_reference = None
        self.verified = False
        self.buffer = b""
        self.reader_exited = False
        self.recovery_started = None
        self.recovery_future = None
        self.recovery_result = None
        self.home_failed = False
        self.commands = node.create_publisher(
            Float64MultiArray, "/leader_position_controller/commands", 1
        )
        node.create_subscription(
            JointState, "/joint_states", self.observe, qos_profile_sensor_data
        )
        node.create_subscription(
            PlanningScene, "/monitored_planning_scene", self.scene_update, 1
        )
        node.create_subscription(
            String,
            "/robot_description",
            self.description,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self.home_client = ActionClient(node, DoObjectiveSequence, "/do_objective")
        node.create_service(Trigger, "~/prepare", self.prepare)
        node.create_service(Trigger, "~/mirror", self.mirror)
        node.create_service(Trigger, "~/clutch", self.clutch)
        os.set_blocking(stream.fileno(), False)
        publish_rate = float(node.declare_parameter("publish_rate", 100.0).value)
        if not math.isfinite(publish_rate) or not 20.0 <= publish_rate <= 100.0:
            raise ValueError("publish_rate must be between 20 and 100 Hz")
        node.create_timer(1.0 / publish_rate, self.tick)

    def observe_thermal(self, message):
        self.thermal_tripped = bool(message.data)
        if message.data:
            self.state.stop("Driver thermal trip; backend owns HOME recovery")
            # The thermal recovery owner cancels the task and sends the one
            # native HOME goal. Do not send a competing goal or position hold.
            self.state.recover = False
            self.state.fault = "Driver thermal trip latched"
            self.prepared_at = None
        # False releases only the driver's thermal flag. The stop fault,
        # preparation and explicit HOME/start checks still gate following.

    def observe_hardware(self, message):
        self.hardware_state = message
        self.hardware_time = time.monotonic()

    def hardware_healthy(self, now):
        state = self.hardware_state
        return (
            state is not None
            and now - self.hardware_time < 0.5
            and list(state.id) == list(range(1, 8))
            and state.comm_state == 0
            and len(state.fault_bits) == 7
            and not any(state.fault_bits)
            and len(state.enabled) == 7
            and all(state.enabled)
        )

    def prepare(self, request, response):
        del request
        now = time.monotonic()
        ready = (
            not self.thermal_tripped
            and not self.state.active
            and not self.home_failed
            and self.verified
            and self.scene_ready
            and self.state.fresh(now)
        )
        if self.real_hardware:
            ready = ready and self.hardware_healthy(now)
        # Reset clears the fault latch, so all readiness checks must pass first.
        ready = ready and self.state.reset()
        if ready:
            self.start_checked = False
            self.prepared_at = now
            self.warmup_heartbeat = -math.inf
            self.warmup_reference = None
            self.hold(fixed=True)
        response.success = bool(ready)
        response.message = (
            "HOME ready; start continuous following"
            if ready
            else self.refusal_reason(now, preparing=True)
        )
        return response

    def refusal_reason(self, now, preparing=False):
        """Explain the first unmet prerequisite without clearing any stop latch."""
        if self.thermal_tripped:
            return operator_fault("Driver thermal trip latched")
        if self.home_failed:
            return (
                "Following refused: automatic HOME recovery failed. Torque remains on to support the arm. "
                "Do not push reBot or disable torque away from HOME. Get operator help to recover HOME "
                "through the supported motion procedure before running Mirror reBot Leader again."
            )
        if self.reader_exited:
            return (
                "Following refused: the Star Arm reader has exited, so there is no leader data. "
                "It exits when the Star Arm is unplugged or unpowered, is not at rebot_leader_port, "
                "or its rebot_leader_calibration_file is rejected; the backend log shows its error. "
                "Connect and power the Star Arm and check those config.yaml settings. Once reBot is at HOME, "
                "restart the backend (moveit_pro run); the reader starts only with the backend. "
                "Then run Mirror reBot Leader again."
            )
        if not self.verified:
            return operator_fault(
                "the reBot robot model is not available or does not match this configuration"
            )
        if not self.scene_ready:
            return operator_fault(
                "the collision scene is not ready; wait for the backend collision scene"
            )
        if self.state.fault and not preparing:
            return operator_fault(self.state.fault)
        if not self.state.fresh(now):
            return operator_fault(
                self.state.rejection or "leader/follower samples are stale or missing"
            )
        if preparing:
            if self.state.active:
                return operator_fault(
                    "Mirror is already following; stop it before starting another Mirror Objective"
                )
            if self.state.recover:
                return operator_fault("HOME recovery is still running")
            off_home = [
                f"{joint_label(i)} {math.degrees(q):+.1f} deg ({q:+.3f} rad)"
                for i, q in enumerate(self.state.measured[:6])
                if abs(q) > 0.08
            ]
            if off_home:
                return (
                    "Following refused: reBot must be at HOME before starting Mirror. "
                    f"Away from HOME: {'; '.join(off_home)}. Run Return reBot Leader Home "
                    "to return reBot; do not move it by hand or disable torque away from HOME. "
                    "Then match the Star Arm to reBot and run Mirror reBot Leader again."
                )
            if self.real_hardware and not self.hardware_healthy(now):
                return operator_fault(
                    "motor feedback is stale, faulted, or torque is disabled; "
                    "check the owning driver and use the supported HOME enable procedure"
                )
        return operator_fault(
            "Mirror preparation is required or a stop is still latched"
        )

    def observe(self, message):
        self.state.observe(message.name, message.position, time.monotonic())

    def scene_update(self, message):
        if self.collider is None:
            return
        try:
            self.collider.update_world(message)
        except (ValueError, RuntimeError) as exc:
            self.scene_ready = False
            self.state.stop(str(exc))
            self.state.fault = str(exc)
            self.node.get_logger().error(str(exc))

    def description(self, message):
        robot = ET.fromstring(message.data)
        plugins = [p.text for p in robot.findall("ros2_control/hardware/plugin")]
        real_plugin = ["robstride_hardware_interface/RobstrideHardware"]
        self.real_hardware = plugins == real_plugin
        allowed = [
            ["picknik_mujoco_ros/MujocoSystem"],
            ["mock_components/GenericSystem"],
            real_plugin,
        ]
        self.verified = robot.get("name") == "rebot_b601_rs" and plugins in allowed
        if self.verified and self.collider is None:
            self.collider = LeaderCollision(message.data)
        if not self.verified:
            self.state.stop("Hardware model is not authorized")
            self.node.get_logger().error("Bridge refuses this hardware model")

    def mirror(self, request, response):
        del request
        now = time.monotonic()
        if not self.start_checked:
            response.success = (
                not self.thermal_tripped
                and self.verified
                and self.scene_ready
                and not self.home_failed
                and not self.state.fault
                and self.prepared_at is not None
                and self.state.fresh(now)
            )
            if response.success:
                self.warmup_heartbeat = now
            response.message = (
                "Holding measured position"
                if response.success
                else self.refusal_reason(now)
            )
        else:
            response.success = (
                not self.thermal_tripped
                and self.verified
                and self.scene_ready
                and not self.home_failed
                and self.state.engage(now)
            )
            response.message = (
                "Following" if response.success else self.refusal_reason(now)
            )
        return response

    def clutch(self, request, response):
        del request
        self.state.stop("Clutched; returning HOME")
        self.state.fault = "Clutched; returning HOME"
        self.hold()
        response.success = True
        response.message = "Leader commands stopped; planned HOME return requested"
        return response

    def publish(self, positions):
        message = Float64MultiArray()
        message.data = list(positions)
        self.commands.publish(message)

    def hold(self, fixed=False):
        now = time.monotonic()
        if self.state.measured is not None and now - self.state.measured_time < 0.1:
            positions = [
                max(math.radians(lower), min(math.radians(upper), position))
                for position, (lower, upper) in zip(self.state.measured, LIMITS_DEG)
            ]
            if fixed:
                if self.warmup_reference is None:
                    self.warmup_reference = positions
                positions = self.warmup_reference
            self.publish(positions)

    def tick(self):
        if self.thermal_tripped:
            return
        was_active = self.state.active
        now = time.monotonic()
        try:
            chunk = os.read(self.stream.fileno(), 65536)
        except BlockingIOError:
            chunk = None
        if chunk == b"":
            if not self.reader_exited:
                self.node.get_logger().error(
                    "Star Arm reader exited; leader following is unavailable until the backend restarts"
                )
            self.reader_exited = True
            self.state.stop("Leader disconnected")
        elif chunk:
            self.buffer += chunk
            if len(self.buffer) > 65536:
                self.buffer = b""
                self.state.stop("Oversized leader stream")
            while b"\n" in self.buffer:
                line, self.buffer = self.buffer.split(b"\n", 1)
                try:
                    sample = json.loads(line)
                except (ValueError, UnicodeDecodeError):
                    sample = {}
                self.state.accept(sample, time.monotonic())
        now = time.monotonic()
        if self.collider is not None and not self.scene_ready:
            if self.scene_future is None and self.scene_client.service_is_ready():
                request = GetPlanningScene.Request()
                request.components.components = (
                    PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
                    | PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
                )
                self.scene_future = self.scene_client.call_async(request)
            elif self.scene_future is not None and self.scene_future.done():
                result = self.scene_future.result()
                if result is not None:
                    try:
                        self.collider.update_world(result.scene)
                        self.scene_ready = True
                        self.node.get_logger().info("Native collision scene ready")
                    except (ValueError, RuntimeError) as exc:
                        self.state.stop(str(exc))
                        self.state.fault = str(exc)
                        self.node.get_logger().error(str(exc))
                    self.scene_future = None
        if not self.start_checked:
            ready = (
                self.verified
                and self.scene_ready
                and not self.state.fault
                and self.prepared_at is not None
                and self.state.fresh(now)
                and now - self.warmup_heartbeat < 0.5
            )
            if ready:
                self.hold(fixed=True)
                self.node.get_logger().info(
                    "WARMUP READY: position reference seeded from measured state"
                )
                self.start_checked = self.state.engage(now)
                if self.start_checked:
                    self.prepared_at = None
                    self.node.get_logger().info("POSITION FOLLOWING STARTED")
            elif self.prepared_at is not None and (
                self.state.fault or now - self.prepared_at >= 3.0
            ):
                self.start_checked = True
                self.prepared_at = None
                self.state.fault = (
                    self.state.fault or "Following did not start; returning HOME"
                )
                self.state.recover = True
        self.state.check_deadman(now)
        if was_active and not self.state.active:
            self.hold()
        if self.state.active:
            if self.real_hardware and not self.hardware_healthy(now):
                reason = "Robstride status stale, faulted or not all motors enabled"
            else:
                try:
                    reason = self.collider.check_path(
                        self.state.measured, self.state.target
                    )
                except (ValueError, RuntimeError) as exc:
                    reason = f"Collision check unavailable: {exc}"
            if reason:
                self.state.stop(reason)
                self.hold()
            elif self.state.fresh(time.monotonic()):
                self.publish(self.state.target)
            else:
                self.state.stop("Samples expired during collision check")
                self.hold()
        elif not self.start_checked:
            self.hold(fixed=True)
        if self.state.recover:
            self.recover_home(now)

    def recover_home(self, now):
        if self.home_failed:
            return
        if self.recovery_started is None:
            self.recovery_started = now
            self.node.get_logger().warn(
                f"{self.state.fault}; requesting native HOME recovery"
            )
        # Let the mirror Objective observe its failed heartbeat and halt the
        # controller ownership before starting a new Objective on the same server.
        if now - self.recovery_started < 0.5:
            return
        if self.recovery_future is None:
            if not self.home_client.server_is_ready():
                if now - self.recovery_started > 3.0:
                    self.fail_home("Objective server unavailable; torque stays on")
                return
            goal = DoObjectiveSequence.Goal()
            goal.objective_name = "Return reBot Leader Home"
            goal.caller.type = goal.caller.SCRIPT
            goal.caller.name = "rebot_leader_bridge"
            self.recovery_future = self.home_client.send_goal_async(goal)
        elif self.recovery_future.done() and self.recovery_result is None:
            handle = self.recovery_future.result()
            if handle is None or not handle.accepted:
                self.fail_home("Native HOME goal rejected; torque stays on")
            else:
                self.recovery_result = handle.get_result_async()
        elif self.recovery_result is not None and self.recovery_result.done():
            result = self.recovery_result.result()
            if (
                result is None
                or result.status != 4
                or result.result.error_code.val != 1
            ):
                self.fail_home("Native HOME recovery failed; torque stays on")
                return
            self.state.recover = False
            self.node.get_logger().info(
                "Native HOME recovery succeeded; normal controller ready, torque retained; restart Mirror to re-engage"
            )
            self.recovery_started = self.recovery_future = self.recovery_result = None

    def fail_home(self, reason):
        self.home_failed = True
        self.node.get_logger().error(reason)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--port", required=True, help="Configured FashionStar USB serial port"
    )
    parser.add_argument(
        "--calibration-file",
        type=Path,
        required=True,
        help="Configured existing leader calibration",
    )
    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args, signal_handler_options=SignalHandlerOptions.NO)

    def request_shutdown(signum, frame):
        del signum, frame
        # One stop request is enough. A non-interactive launch forwards its own
        # SIGINT after the terminal's, then escalates to SIGTERM; neither may
        # interrupt the HOME return below.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, request_shutdown)
    signal.signal(signal.SIGTERM, request_shutdown)
    node = rclpy.create_node("rebot_leader_bridge")
    child = bridge = None
    try:
        command = [
            sys.executable,
            str(Path(__file__).with_name("rebot_leader.py")),
            "--port",
            args.port,
            "--calibration-file",
            str(args.calibration_file),
        ]
        child = subprocess.Popen(
            command,
            stdout=subprocess.PIPE,
            start_new_session=True,
        )
        bridge = LeaderBridge(node, child.stdout)
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Keep the process alive through its home return; SIGKILL cannot run
        # cleanup and is never an operator stop mechanism.
        if bridge is None:
            return
        bridge.state.stop("Bridge shutdown requested")
        bridge.hold()
        deadline = time.monotonic() + 120
        while (
            bridge.state.recover
            and not bridge.home_failed
            and time.monotonic() < deadline
        ):
            rclpy.spin_once(node, timeout_sec=0.02)
    finally:
        if child is not None:
            child.terminate()
            try:
                child.wait(timeout=5)
            except subprocess.TimeoutExpired:
                child.kill()
                child.wait()
            child.stdout.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
