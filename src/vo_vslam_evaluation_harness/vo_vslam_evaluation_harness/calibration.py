# Copyright 2026 PickNik Inc.
# All rights reserved.
#
# Unauthorized copying of this code base via any medium is strictly prohibited.
# Proprietary and confidential.

"""Stereo camera calibration types."""

from __future__ import annotations

from dataclasses import dataclass
from math import isfinite
from typing import Sequence


@dataclass(frozen=True)
class StereoCalibration:
    """Rectified pinhole stereo calibration in metric units."""

    fx: float
    fy: float
    cx: float
    cy: float
    baseline_m: float

    def __post_init__(self) -> None:
        """Reject non-finite intrinsics and nonpositive metric geometry."""
        values = (self.fx, self.fy, self.cx, self.cy, self.baseline_m)
        if not all(isfinite(value) for value in values):
            raise ValueError(
                "calibration values must be finite and positive where required"
            )
        if self.fx <= 0.0 or self.fy <= 0.0 or self.baseline_m <= 0.0:
            raise ValueError(
                "calibration values must be finite and positive where required"
            )

    @classmethod
    def from_projection_matrices(
        cls, left: Sequence[float], right: Sequence[float]
    ) -> StereoCalibration:
        """Build calibration from row-major 3x4 rectified projection matrices."""
        if len(left) != 12 or len(right) != 12:
            raise ValueError("projection matrices must contain exactly 12 elements")
        left = tuple(float(value) for value in left)
        right = tuple(float(value) for value in right)
        if not all(isfinite(value) for value in (*left, *right)):
            raise ValueError("projection matrices must be finite")
        canonical_zero_indices = (1, 4, 7, 8, 9, 11)
        if (
            any(left[index] != 0.0 for index in canonical_zero_indices)
            or any(right[index] != 0.0 for index in canonical_zero_indices)
            or left[3] != 0.0
            or left[10] != 1.0
            or right[10] != 1.0
        ):
            raise ValueError("projection matrices must use canonical rectified form")
        if float(left[0]) <= 0.0 or float(left[5]) <= 0.0 or float(right[0]) <= 0.0:
            raise ValueError("projection matrices must have positive focal length")
        intrinsic_indices = (0, 2, 5, 6)
        if any(
            float(left[index]) != float(right[index]) for index in intrinsic_indices
        ):
            raise ValueError(
                "rectified projection matrices must have matching intrinsics"
            )
        fx = float(left[0])
        left_x_m = -float(left[3]) / fx
        right_x_m = -float(right[3]) / float(right[0])
        baseline_m = right_x_m - left_x_m
        if baseline_m <= 0.0:
            raise ValueError("right camera center must lie at positive stereo baseline")
        return cls(
            fx=fx,
            fy=float(left[5]),
            cx=float(left[2]),
            cy=float(left[6]),
            baseline_m=baseline_m,
        )
