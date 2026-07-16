#!/usr/bin/env python3

# Copyright 2026 PickNik Inc.
# All rights reserved.
#
# Unauthorized copying of this code base via any medium is strictly prohibited.
# Proprietary and confidential.

"""Tests for ROS-facing diagnostic and benchmark playback contracts."""

from pathlib import Path

import numpy as np
import pytest

from vo_vslam_evaluation_harness.core import TrackingResult, TrackingState
from vo_vslam_evaluation_harness.playback import (
    discover_tartanair_sequence,
    load_tartanair_poses,
)
from vo_vslam_evaluation_harness.ros_contract import diagnostic_values


def test_diagnostics_include_measured_health_and_failure_reason() -> None:
    """Diagnostic confidence v0 exposes measured fields without covariance claims."""
    result = TrackingResult(
        timestamp=12.5,
        state=TrackingState.DEGRADED,
        world_from_camera=np.eye(4),
        failure_reason="insufficient PnP points",
        keypoints_left=100,
        keypoints_right=90,
        stereo_matches=70,
        temporal_matches=60,
        pnp_points=5,
        pnp_inliers=0,
        stage_latency_ms={"extract": 4.0, "total": 7.0},
    )

    values = diagnostic_values(
        result, unsynchronized_messages=2, adapter_name="candidate"
    )

    assert values["tracking_state"] == "degraded"
    assert values["failure_reason"] == "insufficient PnP points"
    assert values["unsynchronized_messages"] == "2"
    assert values["pnp_points"] == "5"
    assert values["latency_total_ms"] == "7.000"
    assert values["covariance_status"] == "unknown"


def test_tartanair_sequence_requires_equal_timestamped_stereo_frames(
    tmp_path: Path,
) -> None:
    """Playback discovery pairs sorted stereo files and source timestamps."""
    left = tmp_path / "image_left"
    right = tmp_path / "image_right"
    left.mkdir()
    right.mkdir()
    for index in (1, 0):
        (left / f"{index:06d}_left.png").touch()
        (right / f"{index:06d}_right.png").touch()
    (tmp_path / "timestamps.txt").write_text("10.0\n10.1\n", encoding="utf-8")

    frames = discover_tartanair_sequence(tmp_path)

    assert [frame.timestamp for frame in frames] == pytest.approx([10.0, 10.1])
    assert frames[0].left.name == "000000_left.png"


def test_tartanair_sequence_fails_on_mismatched_inputs(tmp_path: Path) -> None:
    """Missing stereo partners or timestamps fail instead of truncating data."""
    (tmp_path / "image_left").mkdir()
    (tmp_path / "image_right").mkdir()
    (tmp_path / "image_left" / "000000.png").touch()
    (tmp_path / "timestamps.txt").write_text("10.0\n", encoding="utf-8")

    with pytest.raises(ValueError, match="equal counts"):
        discover_tartanair_sequence(tmp_path)


def test_tartanair_sequence_rejects_duplicate_normalized_keys(tmp_path: Path) -> None:
    """Side-suffixed aliases cannot create ambiguous duplicate stereo pairs."""
    (tmp_path / "image_left").mkdir()
    (tmp_path / "image_right").mkdir()
    for name in ("000000.png", "000000_left.png"):
        (tmp_path / "image_left" / name).touch()
    for name in ("000000.png", "000000_right.png"):
        (tmp_path / "image_right" / name).touch()
    (tmp_path / "timestamps.txt").write_text("0.0\n0.1\n", encoding="utf-8")

    with pytest.raises(ValueError, match="duplicate stereo frame keys"):
        discover_tartanair_sequence(tmp_path)


def test_tartanair_pose_count_must_match_stereo_frames(tmp_path: Path) -> None:
    """Ground-truth playback rejects poses that cannot align one-to-one with frames."""
    (tmp_path / "pose_left.txt").write_text("0 0 0 0 0 0 1\n", encoding="utf-8")

    with pytest.raises(ValueError, match="pose count"):
        load_tartanair_poses(tmp_path, expected_count=2)


@pytest.mark.parametrize("timestamp", ["nan", "inf", "-1.0"])
def test_tartanair_sequence_rejects_invalid_ros_timestamps(
    tmp_path: Path, timestamp: str
) -> None:
    """Playback rejects timestamps that cannot represent valid ROS source time."""
    (tmp_path / "image_left").mkdir()
    (tmp_path / "image_right").mkdir()
    (tmp_path / "image_left" / "000000.png").touch()
    (tmp_path / "image_right" / "000000.png").touch()
    (tmp_path / "timestamps.txt").write_text(timestamp + "\n", encoding="utf-8")

    with pytest.raises(ValueError, match="finite and nonnegative"):
        discover_tartanair_sequence(tmp_path)


def test_tartanair_sequence_rejects_unrepresentable_ros_timestamp(
    tmp_path: Path,
) -> None:
    """Playback timestamps must fit builtin_interfaces/Time seconds."""
    (tmp_path / "image_left").mkdir()
    (tmp_path / "image_right").mkdir()
    (tmp_path / "image_left" / "000000.png").touch()
    (tmp_path / "image_right" / "000000.png").touch()
    (tmp_path / "timestamps.txt").write_text("1e20\n", encoding="utf-8")

    with pytest.raises(ValueError, match="ROS Time"):
        discover_tartanair_sequence(tmp_path)


def test_tartanair_poses_reject_zero_quaternion(tmp_path: Path) -> None:
    """Invalid ground-truth orientation fails before any frame is published."""
    (tmp_path / "pose_left.txt").write_text("0 0 0 0 0 0 0\n", encoding="utf-8")

    with pytest.raises(ValueError, match="quaternion"):
        load_tartanair_poses(tmp_path, expected_count=1)


def test_tartanair_poses_normalize_huge_finite_quaternion(tmp_path: Path) -> None:
    """Overflow cannot turn a finite orientation into a published zero quaternion."""
    (tmp_path / "pose_left.txt").write_text(
        "0 0 0 1e308 1e308 1e308 1e308\n", encoding="utf-8"
    )

    poses = load_tartanair_poses(tmp_path, expected_count=1)

    assert np.all(np.isfinite(poses[0, 3:7]))
    assert np.linalg.norm(poses[0, 3:7]) == pytest.approx(1.0)
