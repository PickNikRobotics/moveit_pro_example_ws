#!/usr/bin/env python3
"""Fail the drivers sidecar if the Pi's read-only joint-state stream stops."""

from __future__ import annotations

import math
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState

from external_driver_contract import DriverHeartbeat, validate_joint_sample


class ExternalDriverWatchdog(Node):
    """Track the Pi driver through valid joint states without publishing commands."""

    def __init__(self) -> None:
        super().__init__("so101_external_driver_watchdog")
        self.declare_parameter("startup_timeout_seconds", 30.0)
        self.declare_parameter("stale_after_seconds", 5.0)
        startup_timeout = float(self.get_parameter("startup_timeout_seconds").value)
        stale_after = float(self.get_parameter("stale_after_seconds").value)
        if not math.isfinite(startup_timeout) or startup_timeout <= 0.0:
            raise ValueError("startup_timeout_seconds must be finite and positive")
        if not math.isfinite(stale_after) or stale_after <= 0.0:
            raise ValueError("stale_after_seconds must be finite and positive")

        self._heartbeat = DriverHeartbeat(
            started_at=time.monotonic(),
            startup_timeout=startup_timeout,
            stale_after=stale_after,
        )
        self._announced_ready = False
        self._last_invalid_reason: str | None = None
        self._subscription = self.create_subscription(
            JointState,
            "/joint_states",
            self._on_joint_state,
            qos_profile_sensor_data,
        )
        self._timer = self.create_timer(0.25, self._check_liveness)
        self.get_logger().info(
            "Waiting for the Pi's read-only SO-101 joint-state heartbeat"
        )

    def _on_joint_state(self, message: JointState) -> None:
        reason = validate_joint_sample(message.name, message.position)
        if reason is not None:
            if reason != self._last_invalid_reason:
                self.get_logger().error(
                    f"Ignoring invalid SO-101 joint state: {reason}"
                )
                self._last_invalid_reason = reason
            return
        self._last_invalid_reason = None
        self._heartbeat.record_valid_sample(time.monotonic())
        if not self._announced_ready:
            self.get_logger().info(
                "Pi ros2_control joint-state heartbeat is live; this watchdog "
                "does not publish robot commands"
            )
            self._announced_ready = True

    def _check_liveness(self) -> None:
        reason = self._heartbeat.failure_reason(time.monotonic())
        if reason is None:
            return
        self.get_logger().fatal(f"External Pi driver unavailable: {reason}")
        raise RuntimeError(reason)


def main(args: list[str] | None = None) -> None:
    """Run the watchdog until launch requests a clean shutdown."""
    rclpy.init(args=args)
    node: ExternalDriverWatchdog | None = None
    try:
        node = ExternalDriverWatchdog()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
