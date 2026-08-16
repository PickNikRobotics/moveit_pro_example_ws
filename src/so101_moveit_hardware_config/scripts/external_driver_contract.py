#!/usr/bin/env python3
"""Dependency-free SO-101 external-driver heartbeat checks."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence


REQUIRED_JOINTS = frozenset(
    {
        "Rotation_R",
        "Pitch_R",
        "Elbow_R",
        "Wrist_Pitch_R",
        "Wrist_Roll_R",
        "Jaw_R",
    }
)


def validate_joint_sample(
    names: Sequence[str], positions: Sequence[float]
) -> str | None:
    """Return a diagnostic for an invalid sample, or ``None`` when valid."""
    if len(names) != len(positions):
        return "joint-state names and positions have different lengths"
    missing = sorted(REQUIRED_JOINTS.difference(names))
    if missing:
        return f"joint-state sample is missing: {', '.join(missing)}"
    if not all(math.isfinite(position) for position in positions):
        return "joint-state sample contains a non-finite position"
    return None


@dataclass
class DriverHeartbeat:
    """Track whether valid external-driver samples arrive on time."""

    started_at: float
    startup_timeout: float
    stale_after: float
    last_valid_at: float | None = None

    def record_valid_sample(self, received_at: float) -> None:
        """Record the monotonic arrival time of a validated sample."""
        self.last_valid_at = received_at

    def failure_reason(self, now: float) -> str | None:
        """Return why the driver is unavailable, or ``None`` while healthy."""
        if self.last_valid_at is None:
            if now - self.started_at > self.startup_timeout:
                return (
                    "no valid SO-101 joint state arrived within "
                    f"{self.startup_timeout:g} seconds"
                )
            return None
        if now - self.last_valid_at > self.stale_after:
            return (
                "SO-101 joint states stopped for more than "
                f"{self.stale_after:g} seconds"
            )
        return None
