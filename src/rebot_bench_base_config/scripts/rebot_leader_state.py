# Copyright 2026 PickNik Inc.
# SPDX-License-Identifier: BSD-3-Clause
"""Leader deadman and fresh-sample contract, independent of ROS and the SDK."""
import math

from rebot_leader import DIRECTIONS, JOINT_NAMES, LEADER_NAMES, LIMITS_DEG

UNPLUGGED = (
    "The reader does not reopen its port: if the Star Arm USB was unplugged, reconnect it and, "
    "once reBot is at HOME, restart the backend (moveit_pro run). "
)


def joint_label(index):
    return f"{LEADER_NAMES[index].replace('_', ' ')} ({JOINT_NAMES[index]})"


def alignment_fault(target, measured, tolerance=0.1):
    """Explain mismatches and the signed correction in the Star Arm's calibrated frame."""
    offsets = []
    corrections = []
    for index, (leader, follower, direction) in enumerate(
        zip(target, measured, DIRECTIONS)
    ):
        delta = leader - follower
        if abs(delta) <= tolerance:
            continue
        offsets.append(
            f"{joint_label(index)}: {math.degrees(delta):+.1f} deg ({delta:+.3f} rad)"
        )
        correction = -delta / direction
        corrections.append(
            f"{joint_label(index)} toward a {'higher' if correction > 0 else 'lower'} calibrated angle "
            f"by {abs(math.degrees(correction)):.1f} deg ({abs(correction):.3f} rad)"
        )
    if not offsets:
        return ""
    return (
        "Following refused to avoid a jump: the Star Arm's pose does not match reBot. "
        f"Leader minus reBot offset: {'; '.join(offsets)}. "
        f"Each joint must be within {math.degrees(tolerance):.1f} deg ({tolerance:g} rad). "
        f"Move only the Star Arm: {'; '.join(corrections)}, toward reBot's matching pose. "
        "Then run Mirror reBot Leader again."
    )


def operator_fault(reason):
    """Add an operator recovery step without hiding the original diagnostic."""
    if reason.startswith("Following refused") or reason.startswith("Following stopped"):
        return reason
    if "thermal" in reason.lower():
        return (
            "Following refused: a motor reached the configured temperature limit and thermal protection is latched. "
            "The backend handles HOME recovery and turns torque off only at HOME. "
            "If HOME recovery fails, torque stays on: stop and get operator help. "
            "Wait for cooling and confirmed HOME with all seven motors OFF, then restart the backend (moveit_pro run): "
            "only a restart clears the latch, and startup enables torque at HOME. "
            "Then run Mirror reBot Leader again."
        )
    if "did not start" in reason.lower():
        return (
            "Following refused: it did not start within 3 seconds. reBot is returning HOME; "
            "wait for recovery to finish. Check the Star Arm USB connection, fresh joint feedback and collision scene. "
            + UNPLUGGED
            + "Then match the Star Arm to reBot and run Mirror reBot Leader again."
        )
    if any(word in reason.lower() for word in ("sample", "usb", "stream")):
        return (
            "Following stopped: fresh Star Arm or reBot joint data is missing or invalid. "
            f"Detail: {reason}. Check the Star Arm USB connection and backend joint feedback; "
            "wait for HOME recovery to finish. "
            + UNPLUGGED
            + "Then match the Star Arm to reBot and run Mirror reBot Leader again."
        )
    return (
        f"Following refused: {reason}. Wait for any HOME recovery to finish; "
        "check the backend diagnostics and match the Star Arm to reBot before running Mirror reBot Leader again."
    )


class LeaderState:
    """A fault requires explicit reset at home; recovered USB never re-engages."""

    def __init__(self, sample_timeout=0.1, heartbeat_timeout=0.5):
        self.sample_timeout = sample_timeout
        self.heartbeat_timeout = heartbeat_timeout
        self.target = None
        self.sample_time = -math.inf
        self.measured = None
        self.measured_time = -math.inf
        self.heartbeat = -math.inf
        self.active = False
        self.fault = ""
        self.recover = False
        self.rejection = ""

    def accept(self, sample, now):
        try:
            if "error" in sample:
                raise ValueError(f"Star Arm USB read failed: {sample['error']}")
            values = [float(v) for v in sample["positions"]]
            stamp = float(sample["sample_time"])
            if len(values) != 7 or not all(math.isfinite(v) for v in values):
                raise ValueError("Incomplete/non-finite leader sample")
            if not 0 <= now - stamp < self.sample_timeout or stamp <= self.sample_time:
                raise ValueError("Stale/out-of-order leader sample")
            for value, (lower, upper) in zip(values, LIMITS_DEG):
                if not math.radians(lower) <= value <= math.radians(upper):
                    raise ValueError("Leader target exceeds approved motor limits")
        except (KeyError, TypeError, ValueError) as exc:
            self.target = None
            self.rejection = str(exc)
            self.stop(self.rejection)
            return False
        self.target, self.sample_time = values, stamp
        self.rejection = ""
        return True

    def observe(self, names, positions, now):
        if len(names) != len(positions) or len(set(names)) != len(names):
            return
        lookup = dict(zip(names, positions))
        if not all(j in lookup and math.isfinite(lookup[j]) for j in JOINT_NAMES):
            return
        self.measured = [lookup[j] for j in JOINT_NAMES]
        self.measured_time = now

    def fresh(self, now):
        return (
            self.target is not None
            and self.measured is not None
            and 0 <= now - self.sample_time < self.sample_timeout
            and 0 <= now - self.measured_time < self.sample_timeout
        )

    def engage(self, now):
        if self.fault or self.recover or not self.fresh(now):
            return False
        if not self.active:
            self.fault = alignment_fault(self.target, self.measured)
            if self.fault:
                return False
        self.active = True
        self.heartbeat = now
        return True

    def stop(self, reason):
        if self.active:
            self.recover = True
            self.fault = reason
        self.active = False

    def reset(self):
        if (
            self.recover
            or self.measured is None
            or any(abs(q) > 0.08 for q in self.measured[:6])
        ):
            return False
        self.fault = ""
        return True

    def check_deadman(self, now):
        if self.active and (
            not self.fresh(now) or now - self.heartbeat >= self.heartbeat_timeout
        ):
            self.stop("Leader/follower sample or Objective heartbeat expired")
        return self.active
