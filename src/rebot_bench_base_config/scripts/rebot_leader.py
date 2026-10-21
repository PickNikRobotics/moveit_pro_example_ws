#!/usr/bin/env python3
# Copyright 2026 PickNik Inc.
# SPDX-License-Identifier: BSD-3-Clause
"""Fresh FashionStar samples for the reBot ROS bridge (JSON lines on stdout)."""
import argparse
import json
import math
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


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--port", default="/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"
    )
    parser.add_argument("--id", default="rebot_arm_102_leader")
    args = parser.parse_args()
    from lerobot_teleoperator_rebot_arm_102.config_rebot_arm_102_leader import (
        RebotArm102LeaderConfig,
    )
    from lerobot_teleoperator_rebot_arm_102.rebot_arm_102_leader import (
        RebotArm102Leader,
    )

    leader = RebotArm102Leader(
        RebotArm102LeaderConfig(
            port=args.port,
            id=args.id,
            joint_ranges=dict(zip(LEADER_NAMES, LEADER_RANGES)),
        )
    )
    if not leader.is_calibrated:
        raise RuntimeError("Existing leader calibration required; calibrate separately")
    # The stock get_action catches errors and returns old positions. Read the
    # SDK's raw angle directly: no monitor zero deadband and no stale fallback.
    try:
        leader.connect(calibrate=False)
        while True:
            started = time.monotonic()
            try:
                raw = {
                    name: leader.bus.read_angle(leader.config.joint_ids[name]).raw_deg
                    for name in LEADER_NAMES
                }
                positions = map_positions(raw, leader._round_to_valid_range)
                print(
                    json.dumps({"sample_time": started, "positions": positions}),
                    flush=True,
                )
            except Exception as exc:
                # No sample at all: the consumer freezes immediately and its
                # watchdog requests a planned home return. Never replay a pose.
                print(f"Leader sample rejected: {exc}", file=sys.stderr, flush=True)
                print(
                    json.dumps({"sample_time": started, "error": str(exc)}), flush=True
                )
            time.sleep(max(0, 0.02 - (time.monotonic() - started)))
    finally:
        if leader.is_connected:
            leader.disconnect()


if __name__ == "__main__":
    main()
