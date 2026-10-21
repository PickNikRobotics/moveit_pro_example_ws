#!/usr/bin/env python3
# Copyright 2026 PickNik Inc.
# SPDX-License-Identifier: BSD-3-Clause
"""Fresh FashionStar samples for the reBot ROS bridge (JSON lines on stdout)."""
import argparse
import json
import math
from pathlib import Path
import signal
import sys
import time

LEADER_NAMES = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_yaw",
    "wrist_roll",
    "gripper",
)
JOINT_NAMES = tuple(f"joint{i}" for i in range(1, 8))
DIRECTIONS = (1, 1, -1, -1, -1, 1, 6)
LIMITS_DEG = (
    (-145, 145),
    (0, 203),
    (0, 239),
    (-106, 86),
    (-101, 108),
    (-175, 175),
    (0, 310),
)
LEADER_RANGES = (
    (-150, 150),
    (-1, 203),
    (-239, 1),
    (-86, 106),
    (-108, 101),
    (-175, 175),
    (0, 270),
)


def map_positions(raw, unwrap):
    """Calibrated leader degrees -> motor-frame URDF radians, in joint1..7 order."""
    if set(raw) != set(LEADER_NAMES):
        raise ValueError("A sample must contain all seven leader joints")
    positions = []
    for name, direction, limits, leader_range in zip(
        LEADER_NAMES, DIRECTIONS, LIMITS_DEG, LEADER_RANGES
    ):
        angle = float(raw[name])
        if not math.isfinite(angle):
            raise ValueError(f"Non-finite leader angle: {name}")
        angle, _ = unwrap(angle, *leader_range)
        angle = max(leader_range[0], min(leader_range[1], angle))
        positions.append(
            math.radians(max(limits[0], min(limits[1], direction * angle)))
        )
    return positions


def load_calibration(path):
    """Validate Seeed's stored-origin calibration without changing its meaning.

    The Seeed plugin sets the origin in the servo, writes zero homing offsets,
    and uses joint_ranges (not the JSON's +/-90 placeholder) for unwrapping.
    Applying those placeholder ranges to samples would clip approved motion.
    Unknown offset/drive conventions must be commissioned, not guessed here.
    """
    data = json.loads(path.read_text())
    if not isinstance(data, dict) or set(data) != set(LEADER_NAMES):
        raise ValueError("Existing calibration must contain all seven leader joints")
    for motor_id, name in enumerate(LEADER_NAMES):
        item = data[name]
        if (
            not isinstance(item, dict)
            or type(item.get("id")) is not int
            or item["id"] != motor_id
        ):
            raise ValueError(f"Calibration motor id mismatch: {name}")
        for field in ("homing_offset", "range_min", "range_max", "drive_mode"):
            if type(item.get(field)) not in (int, float) or not math.isfinite(
                item[field]
            ):
                raise ValueError(f"Invalid calibration {field}: {name}")
        if item["range_min"] >= item["range_max"]:
            raise ValueError(f"Invalid calibration range: {name}")
        if item["homing_offset"] != 0 or item["drive_mode"] != 0:
            raise ValueError(
                f"Unsupported FashionStar stored-origin calibration: {name}"
            )
    return data


def unwrap_angle(value, lower, upper):
    """Same nearest-turn window as Seeed's plugin, including +/-180 ties."""
    center = (lower + upper) / 2.0
    low, high = center - 180, center + 180
    if low <= value <= high:
        return value, 0
    turns = (
        math.ceil((value - high) / 360)
        if value > high
        else math.floor((value - low) / 360)
    )
    return value - turns * 360, abs(turns)


def read_positions(bus, calibration):
    # Never use sync_monitor's held angles or the plugin's stale-pose fallback.
    raw = {
        name: bus.read_angle(calibration[name]["id"]).raw_deg for name in LEADER_NAMES
    }
    return map_positions(raw, unwrap_angle)


def run_reader(
    port, calibration, bus_factory, output, clock=time.monotonic, sleep=time.sleep
):
    """One serial owner, closed on failed startup, EOF, or any stop signal."""
    bus = bus_factory(port, baudrate=1_000_000)
    try:
        for name in LEADER_NAMES:
            if not bus.ping(calibration[name]["id"]):
                raise RuntimeError(f"Leader servo not found: {name}")
        # Match the Seeed plugin's unloaded leader. No origin/calibration writes,
        # multi-turn reset, angle target, or follower motor access.
        for name in LEADER_NAMES:
            bus.unlock(calibration[name]["id"])
        while True:
            started = clock()
            try:
                positions = read_positions(bus, calibration)
                print(
                    json.dumps({"sample_time": started, "positions": positions}),
                    file=output,
                    flush=True,
                )
            except Exception as exc:
                print(f"Leader sample rejected: {exc}", file=sys.stderr, flush=True)
                print(
                    json.dumps({"sample_time": started, "error": str(exc)}),
                    file=output,
                    flush=True,
                )
            sleep(max(0, 0.02 - (clock() - started)))
    finally:
        bus.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--port", default="/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"
    )
    parser.add_argument(
        "--calibration-file",
        type=Path,
        required=True,
        help="Existing workspace calibration",
    )
    args = parser.parse_args()
    calibration = load_calibration(args.calibration_file)
    from motorbridge_smart_servo import FashionStarServo

    def stop(signum, frame):
        del signum, frame
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, stop)
    run_reader(args.port, calibration, FashionStarServo, sys.stdout)


if __name__ == "__main__":
    main()
