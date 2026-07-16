#!/usr/bin/env python3

# Copyright 2026 PickNik Inc.
# All rights reserved.
#
# Unauthorized copying of this code base via any medium is strictly prohibited.
# Proprietary and confidential.

"""Tests for ROS-independent node failure contracts."""

import numpy as np
import pytest

from vo_vslam_evaluation_harness.core import TrackingState
from vo_vslam_evaluation_harness.node import (
    _quaternion_from_rotation,
    input_failure_result,
    unknown_covariance,
    validate_frame_contract,
    validate_frame_continuity,
    validate_timestamp_continuity,
)
from vo_vslam_evaluation_harness.playback_node import validate_playback_parameters


def test_input_failure_retains_last_valid_pose() -> None:
    """Invalid ROS input reports LOST without publishing an artificial pose jump."""
    last_pose = np.eye(4)
    last_pose[0, 3] = 2.5

    result = input_failure_result(12.5, last_pose, ValueError("bad calibration"))

    assert result.state is TrackingState.LOST
    assert result.timestamp == 12.5
    assert result.failure_reason == "ValueError: bad calibration"
    np.testing.assert_allclose(result.world_from_camera, last_pose)


def test_unknown_odometry_covariance_uses_nan_not_negative_variance() -> None:
    """Unknown nav_msgs covariance is explicit without claiming negative variance."""
    covariance = unknown_covariance()

    assert len(covariance) == 36
    assert np.all(np.isnan(covariance))


def test_frame_contract_rejects_odometry_child_mismatch() -> None:
    """An optical-camera pose cannot be published under a different child frame."""
    with pytest.raises(ValueError, match="odometry child frame"):
        validate_frame_contract(
            "left_camera_optical",
            "left_camera_optical",
            "right_camera_optical",
            "right_camera_optical",
            "camera_left_optical",
        )


def test_frame_continuity_rejects_runtime_stereo_frame_change() -> None:
    """Motion cannot accumulate across differently named camera coordinate frames."""
    with pytest.raises(ValueError, match="changed"):
        validate_frame_continuity(
            ("camera_left_optical", "camera_right_optical"),
            ("new_left_optical", "new_right_optical"),
        )


def test_timestamp_continuity_survives_tracker_replacement() -> None:
    """Calibration re-priming cannot admit reordered synchronized input."""
    with pytest.raises(ValueError, match="strictly increasing"):
        validate_timestamp_continuity(2.0, 1.0)


@pytest.mark.parametrize(
    "values",
    [
        (float("nan"), 320.0, 320.0, 320.0, 240.0, 0.25),
        (10.0, float("inf"), 320.0, 320.0, 240.0, 0.25),
        (10.0, 320.0, 320.0, 320.0, 240.0, float("nan")),
    ],
)
def test_playback_parameters_must_be_finite(values: tuple[float, ...]) -> None:
    """ROS timers and CameraInfo never receive non-finite configuration."""
    with pytest.raises(ValueError):
        validate_playback_parameters(*values)


def test_quaternion_conversion_rejects_huge_invalid_rotation() -> None:
    """Overflow-sized matrices cannot produce NaN ROS quaternions."""
    with pytest.raises(ValueError, match="rotation"):
        _quaternion_from_rotation(np.full((3, 3), 1e308))


def test_playback_rejects_nonfinite_derived_values() -> None:
    """Finite scalar inputs cannot overflow CameraInfo or timer values."""
    with pytest.raises(ValueError, match="derived"):
        validate_playback_parameters(10.0, 1e308, 320.0, 320.0, 240.0, 1e308)
    with pytest.raises(ValueError, match="timer period"):
        validate_playback_parameters(
            float.fromhex("0x0.0000000000001p-1022"),
            320.0,
            320.0,
            320.0,
            240.0,
            0.25,
        )
    with pytest.raises(ValueError, match="one nanosecond"):
        validate_playback_parameters(1e308, 320.0, 320.0, 320.0, 240.0, 0.25)
