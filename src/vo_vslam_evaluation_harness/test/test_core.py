#!/usr/bin/env python3

# Copyright 2026 PickNik Inc.
# All rights reserved.
#
# Unauthorized copying of this code base via any medium is strictly prohibited.
# Proprietary and confidential.

"""Tests for the candidate-neutral stereo visual-odometry core."""

import numpy as np
import pytest
from time import sleep

from vo_vslam_evaluation_harness.calibration import StereoCalibration
from vo_vslam_evaluation_harness.core import (
    FeatureSet,
    PnPFailure,
    PnPResult,
    StereoVisualOdometry,
    TrackingState,
    VOConfig,
    accumulate_camera_motion,
    triangulate_temporal_correspondences,
    validate_pnp_result,
)

CALIBRATION = StereoCalibration(320.0, 320.0, 320.0, 240.0, 0.25)


def test_invalid_disparity_is_filtered_from_triangulation() -> None:
    """Nonpositive, too-small, and over-depth disparities produce no PnP point."""
    left = np.array([[100.0, 100.0], [120.0, 100.0], [140.0, 100.0]])
    right = np.array([[100.0, 100.0], [119.8, 100.0], [139.0, 100.0]])
    current = np.array([[101.0, 100.0], [121.0, 100.0], [141.0, 100.0]])

    points_3d, points_2d = triangulate_temporal_correspondences(
        left,
        right,
        current,
        np.array([[0, 0], [1, 1], [2, 2]]),
        np.array([[0, 0], [1, 1], [2, 2]]),
        CALIBRATION,
        min_disparity_px=0.5,
        max_depth_m=100.0,
    )

    assert points_3d.shape == (1, 3)
    assert points_2d.shape == (1, 2)
    assert points_3d[0] == pytest.approx([-45.0, -35.0, 80.0], abs=1e-6)


def test_camera_motion_is_inverted_before_world_pose_accumulation() -> None:
    """PnP previous-camera to current-camera motion is inverted into world pose."""
    world_from_previous = np.eye(4)
    current_from_previous = np.eye(4)
    current_from_previous[0, 3] = -1.0

    world_from_current = accumulate_camera_motion(
        world_from_previous, current_from_previous
    )

    assert world_from_current[0, 3] == pytest.approx(1.0)


class DeterministicAdapter:
    """Feature adapter returning image-supplied keypoints and fixed matches."""

    name = "deterministic"

    def extract(self, image: np.ndarray) -> FeatureSet:
        """Treat each image row as one keypoint and descriptor."""
        points = np.asarray(image, dtype=np.float32)
        return FeatureSet(points, points.copy(), np.ones(len(points), dtype=np.float32))

    def match(self, first: FeatureSet, second: FeatureSet) -> np.ndarray:
        """Match features by stable array index."""
        count = min(len(first.keypoints), len(second.keypoints))
        return np.column_stack((np.arange(count), np.arange(count)))


class SequencePnPSolver:
    """PnP solver returning configured outcomes."""

    def __init__(self, outcomes: list[PnPResult | Exception]) -> None:
        self._outcomes = iter(outcomes)

    def solve(
        self,
        points_3d: np.ndarray,
        points_2d: np.ndarray,
        calibration: StereoCalibration,
    ) -> PnPResult:
        """Return or raise the next configured result."""
        outcome = next(self._outcomes)
        if isinstance(outcome, Exception):
            raise outcome
        return outcome


def _stereo_points(offset: float = 0.0) -> tuple[np.ndarray, np.ndarray]:
    left = np.array([[330.0 + i * 4.0 + offset, 240.0] for i in range(6)])
    right = left.copy()
    right[:, 0] -= 8.0
    return left, right


def test_tracking_failure_resets_reference_and_next_frame_recovers() -> None:
    """A failed PnP is explicit, re-primes the reference, and permits recovery."""
    motion = np.eye(4)
    motion[0, 3] = -0.1
    solver = SequencePnPSolver(
        [PnPFailure("RANSAC found no model"), PnPResult(motion, 6, 0.4, 0.8)]
    )
    odometry = StereoVisualOdometry(
        DeterministicAdapter(),
        CALIBRATION,
        VOConfig(min_pnp_points=6, reset_after_failures=1),
        pnp_solver=solver,
    )
    left0, right0 = _stereo_points()
    left1, right1 = _stereo_points(1.0)
    left2, right2 = _stereo_points(2.0)
    left3, right3 = _stereo_points(3.0)

    primed = odometry.process(left0, right0, 1.0)
    failed = odometry.process(left1, right1, 2.0)
    reprime = odometry.process(left2, right2, 3.0)
    recovered = odometry.process(left3, right3, 4.0)

    assert primed.state is TrackingState.INITIALIZING
    assert failed.state is TrackingState.LOST
    assert failed.failure_reason == "RANSAC found no model"
    assert reprime.state is TrackingState.INITIALIZING
    assert recovered.state is TrackingState.TRACKING
    assert recovered.world_from_camera[0, 3] == pytest.approx(0.1)
    assert recovered.timestamp == pytest.approx(4.0)


def test_insufficient_matches_reports_degraded_without_calling_pnp() -> None:
    """Too few metric correspondences produce an explicit degraded result."""
    odometry = StereoVisualOdometry(
        DeterministicAdapter(), CALIBRATION, VOConfig(min_pnp_points=6)
    )
    left = np.array([[330.0, 240.0], [334.0, 240.0]])
    right = left.copy()
    right[:, 0] -= 8.0

    odometry.process(left, right, 1.0)
    result = odometry.process(left + 1.0, right + 1.0, 2.0)

    assert result.state is TrackingState.DEGRADED
    assert "insufficient PnP points" in result.failure_reason
    assert result.pnp_points == 2


def test_failed_pnp_keeps_pose_reference_until_recovery() -> None:
    """Recovery estimates motion from the pose reference, not the failed frame."""
    motion = np.eye(4)
    motion[0, 3] = -0.2
    solver = SequencePnPSolver(
        [PnPFailure("transient failure"), PnPResult(motion, 6, 0.2, 0.4)]
    )
    odometry = StereoVisualOdometry(
        DeterministicAdapter(),
        CALIBRATION,
        VOConfig(min_pnp_points=6, reset_after_failures=3),
        pnp_solver=solver,
    )
    left0, right0 = _stereo_points()
    left1, right1 = _stereo_points(1.0)
    left2, right2 = _stereo_points(2.0)

    odometry.process(left0, right0, 1.0)
    failed = odometry.process(left1, right1, 2.0)
    recovered = odometry.process(left2, right2, 3.0)

    assert failed.state is TrackingState.LOST
    assert recovered.state is TrackingState.TRACKING
    assert recovered.world_from_camera[0, 3] == pytest.approx(0.2)


class SlowFailingPnPSolver:
    """PnP solver that proves attempted failure latency is measured."""

    def solve(self, points_3d, points_2d, calibration):
        sleep(0.002)
        raise PnPFailure("measured failure")


def test_failed_pnp_reports_attempt_latency() -> None:
    """An attempted PnP failure reports elapsed PnP time instead of zero."""
    odometry = StereoVisualOdometry(
        DeterministicAdapter(),
        CALIBRATION,
        VOConfig(min_pnp_points=6),
        pnp_solver=SlowFailingPnPSolver(),
    )
    left, right = _stereo_points()
    odometry.process(left, right, 1.0)

    result = odometry.process(left + 1.0, right + 1.0, 2.0)

    assert result.stage_latency_ms["pnp"] > 0.0


def test_invalid_pnp_output_cannot_poison_accumulated_pose() -> None:
    """Non-finite solver output fails before tracker state mutation."""
    invalid_motion = np.eye(4)
    invalid_motion[0, 3] = float("nan")
    odometry = StereoVisualOdometry(
        DeterministicAdapter(),
        CALIBRATION,
        VOConfig(min_pnp_points=6),
        pnp_solver=SequencePnPSolver([PnPResult(invalid_motion, 6, 0.1, 0.2)]),
    )
    left, right = _stereo_points()
    odometry.process(left, right, 1.0)

    result = odometry.process(left + 1.0, right + 1.0, 2.0)

    assert result.state is TrackingState.LOST
    assert "PnP result" in result.failure_reason
    np.testing.assert_allclose(result.world_from_camera, np.eye(4))


@pytest.mark.parametrize("inlier_count", [0, 5])
def test_insufficient_pnp_inliers_cannot_commit_pose(inlier_count: int) -> None:
    """A solver model below the configured support threshold is rejected."""
    motion = np.eye(4)
    motion[0, 3] = -1.0
    odometry = StereoVisualOdometry(
        DeterministicAdapter(),
        CALIBRATION,
        VOConfig(min_pnp_points=6),
        pnp_solver=SequencePnPSolver([PnPResult(motion, inlier_count, 0.1, 0.2)]),
    )
    left, right = _stereo_points()
    odometry.process(left, right, 1.0)

    result = odometry.process(left + 1.0, right + 1.0, 2.0)

    assert result.state is TrackingState.LOST
    assert "inlier count" in result.failure_reason
    np.testing.assert_allclose(result.world_from_camera, np.eye(4))


@pytest.mark.parametrize(
    "result",
    [
        PnPResult(np.ones((3, 3)), 6, 0.1, 0.2),
        PnPResult(np.diag([1.0, 1.0, -1.0, 1.0]), 6, 0.1, 0.2),
        PnPResult(np.eye(4), 7, 0.1, 0.2),
        PnPResult(np.eye(4), 6, float("nan"), 0.2),
        PnPResult(np.eye(4), 6, 0.1, -1.0),
    ],
)
def test_pnp_result_contract_rejects_malformed_solver_output(result: PnPResult) -> None:
    """Transform structure, diagnostics, and inlier bounds are fail-closed."""
    with pytest.raises((ValueError, TypeError)):
        validate_pnp_result(result, 6, 6)


def test_tracker_can_reprime_without_resetting_accumulated_pose() -> None:
    """Calibration replacement can retain the last valid world pose."""
    initial_pose = np.eye(4)
    initial_pose[0, 3] = 2.5
    odometry = StereoVisualOdometry(
        DeterministicAdapter(), CALIBRATION, initial_world_from_camera=initial_pose
    )
    left, right = _stereo_points()

    result = odometry.process(left, right, 1.0)

    assert result.state is TrackingState.INITIALIZING
    np.testing.assert_allclose(result.world_from_camera, initial_pose)


def test_tracker_rejects_non_monotonic_timestamps() -> None:
    """Reordered input cannot be accumulated as continuous camera motion."""
    odometry = StereoVisualOdometry(DeterministicAdapter(), CALIBRATION, VOConfig())
    left, right = _stereo_points()
    odometry.process(left, right, 2.0)

    with pytest.raises(ValueError, match="strictly increasing"):
        odometry.process(left, right, 2.0)


@pytest.mark.parametrize(
    "configuration",
    [
        {"min_disparity_px": 0.0},
        {"max_depth_m": 0.0},
        {"min_pnp_points": 0},
        {"pnp_reprojection_error_px": 0.0},
        {"pnp_confidence": 1.0},
        {"pnp_iterations": 0},
        {"reset_after_failures": 0},
        {"min_disparity_px": float("nan")},
        {"max_depth_m": float("inf")},
        {"pnp_reprojection_error_px": float("nan")},
        {"pnp_confidence": float("nan")},
        {"min_pnp_points": float("nan")},
        {"pnp_iterations": float("nan")},
        {"reset_after_failures": 3.0},
    ],
)
def test_invalid_vo_configuration_is_rejected(configuration: dict[str, float]) -> None:
    """Invalid tuning cannot reach OpenCV or produce undefined reset behavior."""
    with pytest.raises(ValueError):
        VOConfig(**configuration)
