#!/usr/bin/env python3

# Copyright 2026 PickNik Inc.
# All rights reserved.
#
# Unauthorized copying of this code base via any medium is strictly prohibited.
# Proprietary and confidential.

"""Tests for stereo camera calibration parsing."""

import pytest

from vo_vslam_evaluation_harness.calibration import StereoCalibration


def test_projection_matrices_define_metric_stereo_calibration() -> None:
    """Projection matrices provide intrinsics and the metric stereo baseline."""
    left_projection = (
        320.0,
        0.0,
        320.0,
        0.0,
        0.0,
        320.0,
        240.0,
        0.0,
        0.0,
        0.0,
        1.0,
        0.0,
    )
    right_projection = (
        320.0,
        0.0,
        320.0,
        -80.0,
        0.0,
        320.0,
        240.0,
        0.0,
        0.0,
        0.0,
        1.0,
        0.0,
    )

    calibration = StereoCalibration.from_projection_matrices(
        left_projection, right_projection
    )

    assert calibration.fx == pytest.approx(320.0)
    assert calibration.fy == pytest.approx(320.0)
    assert calibration.cx == pytest.approx(320.0)
    assert calibration.cy == pytest.approx(240.0)
    assert calibration.baseline_m == pytest.approx(0.25)


@pytest.mark.parametrize(
    ("left", "right", "message"),
    [
        ((1.0,) * 11, (1.0,) * 12, "12 elements"),
        (
            (0.0, 0.0, 320.0, 0.0, 0.0, 320.0, 240.0, 0.0, 0.0, 0.0, 1.0, 0.0),
            (320.0, 0.0, 320.0, -80.0, 0.0, 320.0, 240.0, 0.0, 0.0, 0.0, 1.0, 0.0),
            "focal length",
        ),
        (
            (320.0, 0.0, 320.0, 0.0, 0.0, 320.0, 240.0, 0.0, 0.0, 0.0, 1.0, 0.0),
            (321.0, 0.0, 320.0, -80.0, 0.0, 320.0, 240.0, 0.0, 0.0, 0.0, 1.0, 0.0),
            "matching intrinsics",
        ),
        (
            (320.0, 0.0, 320.0, 0.0, 0.0, 320.0, 240.0, 0.0, 0.0, 0.0, 1.0, 0.0),
            (320.0, 0.0, 320.0, 80.0, 0.0, 320.0, 240.0, 0.0, 0.0, 0.0, 1.0, 0.0),
            "right camera center",
        ),
        (
            (320.0, 1.0, 320.0, 0.0, 0.0, 320.0, 240.0, 0.0, 0.0, 0.0, 1.0, 0.0),
            (320.0, 0.0, 320.0, -80.0, 0.0, 320.0, 240.0, 0.0, 0.0, 0.0, 1.0, 0.0),
            "canonical rectified",
        ),
        (
            (320.0, 0.0, 320.0, 0.0, 0.0, 320.0, 240.0, 1.0, 0.0, 0.0, 1.0, 0.0),
            (320.0, 0.0, 320.0, -80.0, 0.0, 320.0, 240.0, 0.0, 0.0, 0.0, 1.0, 0.0),
            "canonical rectified",
        ),
        (
            (320.0, 0.0, 320.0, 0.0, 0.0, 320.0, 240.0, 0.0, 0.0, 0.0, 2.0, 0.0),
            (320.0, 0.0, 320.0, -80.0, 0.0, 320.0, 240.0, 0.0, 0.0, 0.0, 1.0, 0.0),
            "canonical rectified",
        ),
    ],
)
def test_invalid_projection_matrices_fail_explicitly(
    left: tuple[float, ...], right: tuple[float, ...], message: str
) -> None:
    """Malformed or inconsistent rectified projection matrices are rejected."""
    with pytest.raises(ValueError, match=message):
        StereoCalibration.from_projection_matrices(left, right)


def test_direct_calibration_rejects_invalid_geometry() -> None:
    """Offline tools cannot bypass finite positive calibration validation."""
    with pytest.raises(ValueError, match="finite and positive"):
        StereoCalibration(320.0, 320.0, 320.0, 240.0, float("nan"))
