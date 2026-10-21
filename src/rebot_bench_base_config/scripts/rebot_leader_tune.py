#!/usr/bin/env python3
# Copyright 2026 PickNik Inc.
# SPDX-License-Identifier: BSD-3-Clause
"""Explicit, HOME/OFF-only volatile motor tuning; never save motor parameters."""
import argparse
import json
import math
from pathlib import Path
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from robstride_interfaces.msg import RobstrideState
from robstride_interfaces.srv import SetDataToRobstride
from sensor_msgs.msg import JointState


def settings(path):
    values = yaml.safe_load(Path(path).read_text())
    bounds = {"jaw_torque_limit_nm": (0, 14), "joint4_speed_limit_rad_s": (0, 33)}
    if not isinstance(values, dict) or set(values) != set(bounds):
        raise ValueError("Tuning file must contain exactly the two documented settings")
    for key, (lower, upper) in bounds.items():
        value = values[key]
        if (
            isinstance(value, bool)
            or not isinstance(value, (int, float))
            or not math.isfinite(value)
            or not lower < value <= upper
        ):
            raise ValueError(f"{key} must be finite and in ({lower}, {upper}]")
    return [
        (7, "0x700B", values["jaw_torque_limit_nm"]),
        (4, "0x7017", values["joint4_speed_limit_rad_s"]),
    ]


def safe_home(health, joint, now):
    if (
        health is None
        or joint is None
        or now - health[0] >= 0.5
        or now - joint[0] >= 0.1
    ):
        return False
    h, j = health[1], joint[1]
    positions = dict(zip(j.name, j.position))
    return (
        list(h.id) == list(range(1, 8))
        and len(h.enabled) == 7
        and not any(h.enabled)
        and len(h.fault_bits) == 7
        and not any(h.fault_bits)
        and h.comm_state == 0
        and all(
            math.isfinite(positions.get(f"joint{i}", math.nan))
            and abs(positions[f"joint{i}"]) <= 0.08
            for i in range(1, 7)
        )
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config",
        type=Path,
        default=Path(get_package_share_directory("rebot_bench_base_config"))
        / "config/rebot_leader_tuning.yaml",
    )
    parser.add_argument(
        "--apply",
        action="store_true",
        help="Explicitly write volatile settings after physical commissioning approval",
    )
    args = parser.parse_args()
    writes = settings(args.config)
    print(
        json.dumps({"writes": writes, "apply": args.apply, "persistent_save": False}),
        flush=True,
    )
    if not args.apply:
        return
    rclpy.init()
    node = rclpy.create_node("rebot_leader_tune")
    latest = {"health": None, "joint": None}

    def observe(key, message):
        latest[key] = (time.monotonic(), message)

    node.create_subscription(
        RobstrideState,
        "/robstride_hardware_interface/robstride_state",
        lambda m: observe("health", m),
        1,
    )
    node.create_subscription(
        JointState, "/joint_states", lambda m: observe("joint", m), 1
    )
    client = node.create_client(
        SetDataToRobstride, "/robstride_hardware_interface/set_data_to_robstride"
    )
    try:
        if not client.wait_for_service(timeout_sec=3):
            raise RuntimeError("Motor tuning service unavailable; no writes")
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline and not safe_home(
            latest["health"], latest["joint"], time.monotonic()
        ):
            rclpy.spin_once(node, timeout_sec=0.02)
        for motor, item, value in writes:
            rclpy.spin_once(node, timeout_sec=0.01)
            if not safe_home(latest["health"], latest["joint"], time.monotonic()):
                raise RuntimeError(
                    "Fresh healthy HOME/all OFF required; remaining writes refused"
                )
            future = client.call_async(
                SetDataToRobstride.Request(
                    id=motor, item_name=item, item_data=float(value)
                )
            )
            rclpy.spin_until_future_complete(node, future, timeout_sec=2)
            if (
                not future.done()
                or future.result() is None
                or not future.result().result
            ):
                raise RuntimeError(f"Motor {motor} {item} write unconfirmed; no retry")
            print(
                f"ACK id={motor} item={item} value={value}; verify register readback before enable",
                flush=True,
            )
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
