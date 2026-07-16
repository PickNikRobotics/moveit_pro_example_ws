# Copyright 2026 PickNik Inc.
# All rights reserved.
#
# Unauthorized copying of this code base via any medium is strictly prohibited.
# Proprietary and confidential.

"""TartanAir-style stereo sequence discovery and ROS playback node."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np


@dataclass(frozen=True)
class StereoFrame:
    """One timestamped benchmark stereo pair."""

    left: Path
    right: Path
    timestamp: float


def stereo_frame_key(path: Path) -> str:
    """Normalize common TartanAir side suffixes to one stereo frame identity."""
    stem = path.stem
    for suffix in ("_left", "_right"):
        if stem.endswith(suffix):
            return stem[: -len(suffix)]
    return stem


def validate_stereo_frame_keys(left: list[Path], right: list[Path]) -> None:
    """Require unambiguous one-to-one normalized stereo frame identities."""
    left_keys = [stereo_frame_key(path) for path in left]
    right_keys = [stereo_frame_key(path) for path in right]
    if len(set(left_keys)) != len(left_keys) or len(set(right_keys)) != len(right_keys):
        raise ValueError("duplicate stereo frame keys are not allowed")
    if left_keys != right_keys:
        raise ValueError("left and right image frame keys must match")


def discover_tartanair_sequence(root: Path) -> list[StereoFrame]:
    """Discover exact stereo pairs with timestamps; never silently truncate."""
    left = sorted((root / "image_left").glob("*.png"))
    right = sorted((root / "image_right").glob("*.png"))
    timestamp_path = root / "timestamps.txt"
    if not timestamp_path.is_file():
        raise ValueError(f"missing source timestamps: {timestamp_path}")
    timestamps = [
        float(line)
        for line in timestamp_path.read_text(encoding="utf-8").splitlines()
        if line.strip()
    ]
    if not left or len(left) != len(right) or len(left) != len(timestamps):
        raise ValueError(
            "image_left, image_right, and timestamps.txt must have equal counts"
        )
    validate_stereo_frame_keys(left, right)
    if not np.all(np.isfinite(timestamps)) or any(value < 0.0 for value in timestamps):
        raise ValueError("source timestamps must be finite and nonnegative")
    if any(value >= 2147483648.0 for value in timestamps):
        raise ValueError("source timestamps must fit ROS Time seconds")
    if any(
        current <= previous for previous, current in zip(timestamps, timestamps[1:])
    ):
        raise ValueError("source timestamps must be strictly increasing")
    return [
        StereoFrame(left_path, right_path, timestamp)
        for left_path, right_path, timestamp in zip(left, right, timestamps)
    ]


def load_tartanair_poses(root: Path, expected_count: int) -> np.ndarray:
    """Load TartanAir xyz+xyzw poses with exact frame-count validation."""
    pose_path = root / "pose_left.txt"
    if not pose_path.is_file():
        raise ValueError(f"missing ground-truth poses: {pose_path}")
    poses = np.loadtxt(pose_path, dtype=np.float64, ndmin=2)
    if poses.shape != (expected_count, 7):
        raise ValueError(
            f"pose count/shape must be ({expected_count}, 7), got {poses.shape}"
        )
    if not np.all(np.isfinite(poses)):
        raise ValueError("ground-truth poses must be finite")
    quaternions = poses[:, 3:7]
    quaternion_scales = np.max(np.abs(quaternions), axis=1)
    if np.any(quaternion_scales == 0.0):
        raise ValueError("ground-truth pose quaternions must be nonzero")
    scaled_quaternions = quaternions / quaternion_scales[:, None]
    quaternion_norms = np.linalg.norm(scaled_quaternions, axis=1)
    normalized_quaternions = scaled_quaternions / quaternion_norms[:, None]
    if not np.all(np.isfinite(normalized_quaternions)):
        raise ValueError(
            "ground-truth pose quaternions must normalize to finite values"
        )
    poses[:, 3:7] = normalized_quaternions
    return poses


def main() -> None:
    """Run the ROS benchmark publisher; ROS imports remain lazy for CPU tests."""
    from .playback_node import run_playback_node

    run_playback_node()
