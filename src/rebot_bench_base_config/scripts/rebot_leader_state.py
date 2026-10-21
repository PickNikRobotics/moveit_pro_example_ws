# Copyright 2026 PickNik Inc.
# SPDX-License-Identifier: BSD-3-Clause
"""Leader deadman and fresh-sample contract, independent of ROS and the SDK."""
import math

from rebot_leader import JOINT_NAMES, LIMITS_DEG


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

    def accept(self, sample, now):
        try:
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
            self.stop(str(exc))
            return False
        self.target, self.sample_time = values, stamp
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
