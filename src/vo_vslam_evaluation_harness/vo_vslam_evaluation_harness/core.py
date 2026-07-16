# Copyright 2026 PickNik Inc.
# All rights reserved.
#
# Unauthorized copying of this code base via any medium is strictly prohibited.
# Proprietary and confidential.

"""Candidate-neutral stereo frame-to-frame visual odometry."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from math import isfinite
from numbers import Integral
from time import perf_counter
from typing import Protocol

import cv2
import numpy as np

from .calibration import StereoCalibration


class TrackingState(str, Enum):
    """Observable state of the visual-odometry tracker."""

    INITIALIZING = "initializing"
    TRACKING = "tracking"
    DEGRADED = "degraded"
    LOST = "lost"


class PnPFailure(RuntimeError):
    """Raised when temporal pose estimation cannot produce a valid model."""


@dataclass(frozen=True)
class FeatureSet:
    """Normalized candidate output consumed by the shared matcher pipeline."""

    keypoints: np.ndarray
    descriptors: np.ndarray
    scores: np.ndarray
    image_size: tuple[int, int] | None = None

    def __post_init__(self) -> None:
        """Validate the stable candidate adapter contract."""
        keypoints = np.asarray(self.keypoints)
        descriptors = np.asarray(self.descriptors)
        scores = np.asarray(self.scores)
        if keypoints.ndim != 2 or keypoints.shape[1] != 2:
            raise ValueError("keypoints must have shape (N, 2)")
        if descriptors.ndim != 2 or descriptors.shape[0] != len(keypoints):
            raise ValueError("descriptors must have shape (N, D)")
        if scores.ndim != 1 or len(scores) != len(keypoints):
            raise ValueError("scores must have shape (N,)")
        if not (
            np.all(np.isfinite(keypoints))
            and np.all(np.isfinite(descriptors))
            and np.all(np.isfinite(scores))
        ):
            raise ValueError("keypoints, descriptors, and scores must be finite")
        if self.image_size is not None and (
            len(self.image_size) != 2 or min(self.image_size) <= 0
        ):
            raise ValueError("image_size must be a positive (width, height) pair")


class FeatureAdapter(Protocol):
    """Swappable feature extraction and matching runtime contract."""

    name: str

    def extract(self, image: np.ndarray) -> FeatureSet:
        """Extract image-space keypoints, descriptors, and scores."""

    def match(self, first: FeatureSet, second: FeatureSet) -> np.ndarray:
        """Return index pairs with shape (M, 2)."""


@dataclass(frozen=True)
class PnPResult:
    """Successful previous-camera to current-camera PnP estimate."""

    current_from_previous: np.ndarray
    inlier_count: int
    residual_mean_px: float
    residual_p95_px: float


class PnPSolver(Protocol):
    """Injectable temporal pose solver contract."""

    def solve(
        self,
        points_3d: np.ndarray,
        points_2d: np.ndarray,
        calibration: StereoCalibration,
    ) -> PnPResult:
        """Estimate current-camera from previous-camera transform."""


@dataclass(frozen=True)
class VOConfig:
    """Candidate-independent stereo and PnP tuning parameters."""

    min_disparity_px: float = 0.5
    max_depth_m: float = 100.0
    min_pnp_points: int = 6
    pnp_reprojection_error_px: float = 3.0
    pnp_confidence: float = 0.999
    pnp_iterations: int = 100
    reset_after_failures: int = 3

    def __post_init__(self) -> None:
        """Reject parameters that make geometry or reset behavior undefined."""
        for name in ("min_pnp_points", "pnp_iterations", "reset_after_failures"):
            value = getattr(self, name)
            if not isinstance(value, Integral) or isinstance(value, bool):
                raise ValueError(f"{name} must be an integer")
        if not isfinite(self.min_disparity_px) or self.min_disparity_px <= 0.0:
            raise ValueError("min_disparity_px must be positive")
        if not isfinite(self.max_depth_m) or self.max_depth_m <= 0.0:
            raise ValueError("max_depth_m must be positive")
        if self.min_pnp_points < 4:
            raise ValueError("min_pnp_points must be at least 4")
        if (
            not isfinite(self.pnp_reprojection_error_px)
            or self.pnp_reprojection_error_px <= 0.0
        ):
            raise ValueError("pnp_reprojection_error_px must be positive")
        if not isfinite(self.pnp_confidence) or not 0.0 < self.pnp_confidence < 1.0:
            raise ValueError("pnp_confidence must be between zero and one")
        if self.pnp_iterations <= 0:
            raise ValueError("pnp_iterations must be positive")
        if self.reset_after_failures <= 0:
            raise ValueError("reset_after_failures must be positive")


@dataclass(frozen=True)
class TrackingResult:
    """Pose, health, and measured diagnostic confidence for one stereo frame."""

    timestamp: float
    state: TrackingState
    world_from_camera: np.ndarray
    failure_reason: str = ""
    keypoints_left: int = 0
    keypoints_right: int = 0
    stereo_matches: int = 0
    temporal_matches: int = 0
    pnp_points: int = 0
    pnp_inliers: int = 0
    pnp_inlier_ratio: float = 0.0
    residual_mean_px: float = float("nan")
    residual_p95_px: float = float("nan")
    stage_latency_ms: dict[str, float] = field(default_factory=dict)


class OpenCVPnPSolver:
    """OpenCV RANSAC PnP implementation matching the research benchmark semantics."""

    def __init__(self, config: VOConfig) -> None:
        self._config = config

    def solve(
        self,
        points_3d: np.ndarray,
        points_2d: np.ndarray,
        calibration: StereoCalibration,
    ) -> PnPResult:
        """Solve metric camera motion and report measured reprojection residuals."""
        camera_matrix = np.array(
            [
                [calibration.fx, 0.0, calibration.cx],
                [0.0, calibration.fy, calibration.cy],
                [0.0, 0.0, 1.0],
            ],
            dtype=np.float64,
        )
        try:
            ok, rotation_vector, translation, inliers = cv2.solvePnPRansac(
                np.asarray(points_3d, dtype=np.float32),
                np.asarray(points_2d, dtype=np.float32),
                camera_matrix,
                np.zeros(4),
                flags=cv2.SOLVEPNP_ITERATIVE,
                reprojectionError=self._config.pnp_reprojection_error_px,
                confidence=self._config.pnp_confidence,
                iterationsCount=self._config.pnp_iterations,
            )
        except cv2.error as error:
            raise PnPFailure(f"OpenCV RANSAC PnP error: {error}") from error
        if not ok or inliers is None or len(inliers) < self._config.min_pnp_points:
            raise PnPFailure("RANSAC PnP failed or returned insufficient inliers")
        rotation, _ = cv2.Rodrigues(rotation_vector)
        transform = np.eye(4, dtype=np.float64)
        transform[:3, :3] = rotation
        transform[:3, 3] = translation.reshape(3)
        projected, _ = cv2.projectPoints(
            points_3d[inliers[:, 0]],
            rotation_vector,
            translation,
            camera_matrix,
            np.zeros(4),
        )
        residuals = np.linalg.norm(
            projected.reshape(-1, 2) - points_2d[inliers[:, 0]], axis=1
        )
        return PnPResult(
            transform,
            int(len(inliers)),
            float(np.mean(residuals)),
            float(np.percentile(residuals, 95)),
        )


def validate_rotation_matrix(
    rotation: np.ndarray, label: str = "rotation"
) -> np.ndarray:
    """Require a finite right-handed orthonormal 3x3 rotation."""
    result = np.asarray(rotation, dtype=np.float64)
    tolerance = 1e-5
    if result.shape != (3, 3) or not np.all(np.isfinite(result)):
        raise ValueError(f"{label} must be a finite 3x3 rotation")
    if np.max(np.abs(result)) > 1.0 + tolerance:
        raise ValueError(f"{label} must be orthonormal")
    if not np.allclose(result.T @ result, np.eye(3), atol=tolerance, rtol=0.0):
        raise ValueError(f"{label} must be orthonormal")
    if not np.isclose(np.linalg.det(result), 1.0, atol=tolerance, rtol=0.0):
        raise ValueError(f"{label} must be right-handed")
    return result


def validate_rigid_transform(
    transform: np.ndarray, label: str = "transform"
) -> np.ndarray:
    """Require a finite homogeneous SE(3) transform."""
    result = np.asarray(transform, dtype=np.float64)
    if result.shape != (4, 4) or not np.all(np.isfinite(result)):
        raise ValueError(f"{label} must be a finite 4x4 rigid transform")
    if not np.allclose(result[3], [0.0, 0.0, 0.0, 1.0], atol=1e-8, rtol=0.0):
        raise ValueError(f"{label} must have a homogeneous final row")
    validate_rotation_matrix(result[:3, :3], f"{label} rotation")
    return result


def validate_pnp_result(
    result: PnPResult, point_count: int, minimum_inlier_count: int
) -> PnPResult:
    """Validate solver output before committing odometry state."""
    validate_rigid_transform(result.current_from_previous, "PnP result transform")
    if (
        not isinstance(result.inlier_count, Integral)
        or isinstance(result.inlier_count, bool)
        or not minimum_inlier_count <= result.inlier_count <= point_count
    ):
        raise ValueError("PnP result inlier count is outside the input point set")
    if not (
        isfinite(result.residual_mean_px)
        and result.residual_mean_px >= 0.0
        and isfinite(result.residual_p95_px)
        and result.residual_p95_px >= 0.0
    ):
        raise ValueError("PnP result residuals must be finite and nonnegative")
    return result


def _validated_matches(
    matches: np.ndarray, first_count: int, second_count: int
) -> np.ndarray:
    result = np.asarray(matches, dtype=np.int64)
    if result.size == 0:
        return np.empty((0, 2), dtype=np.int64)
    if result.ndim != 2 or result.shape[1] != 2:
        raise ValueError("matches must have shape (M, 2)")
    if (
        np.any(result < 0)
        or np.any(result[:, 0] >= first_count)
        or np.any(result[:, 1] >= second_count)
    ):
        raise ValueError("match index is outside the feature set")
    return result


def triangulate_temporal_correspondences(
    previous_left: np.ndarray,
    previous_right: np.ndarray,
    current_left: np.ndarray,
    stereo_matches: np.ndarray,
    temporal_matches: np.ndarray,
    calibration: StereoCalibration,
    min_disparity_px: float,
    max_depth_m: float,
) -> tuple[np.ndarray, np.ndarray]:
    """Create previous-frame 3D/current-frame 2D correspondences from matches."""
    previous_left = np.asarray(previous_left, dtype=np.float64)
    previous_right = np.asarray(previous_right, dtype=np.float64)
    current_left = np.asarray(current_left, dtype=np.float64)
    stereo_matches = _validated_matches(
        stereo_matches, len(previous_left), len(previous_right)
    )
    temporal_matches = _validated_matches(
        temporal_matches, len(previous_left), len(current_left)
    )
    right_by_left = {int(left): int(right) for left, right in stereo_matches}
    points_3d: list[list[float]] = []
    points_2d: list[np.ndarray] = []
    for previous_index, current_index in temporal_matches:
        right_index = right_by_left.get(int(previous_index))
        if right_index is None:
            continue
        u_left, v_left = previous_left[previous_index]
        u_right = previous_right[right_index, 0]
        disparity = float(u_left - u_right)
        if not np.isfinite(disparity) or disparity <= min_disparity_px:
            continue
        depth = calibration.fx * calibration.baseline_m / disparity
        if not np.isfinite(depth) or depth <= 0.0 or depth > max_depth_m:
            continue
        points_3d.append(
            [
                (u_left - calibration.cx) * depth / calibration.fx,
                (v_left - calibration.cy) * depth / calibration.fy,
                depth,
            ]
        )
        points_2d.append(current_left[current_index])
    return (
        np.asarray(points_3d, dtype=np.float32).reshape(-1, 3),
        np.asarray(points_2d, dtype=np.float32).reshape(-1, 2),
    )


def accumulate_camera_motion(
    world_from_previous: np.ndarray, current_from_previous: np.ndarray
) -> np.ndarray:
    """Accumulate PnP camera motion into a world-from-current pose."""
    return np.asarray(world_from_previous, dtype=np.float64) @ np.linalg.inv(
        np.asarray(current_from_previous, dtype=np.float64)
    )


@dataclass
class _Frame:
    left: FeatureSet
    right: FeatureSet
    stereo_matches: np.ndarray


class StereoVisualOdometry:
    """Streaming stereo-PnP odometry with explicit reset and recovery behavior."""

    def __init__(
        self,
        adapter: FeatureAdapter,
        calibration: StereoCalibration,
        config: VOConfig | None = None,
        pnp_solver: PnPSolver | None = None,
        initial_world_from_camera: np.ndarray | None = None,
    ) -> None:
        self._adapter = adapter
        self._calibration = calibration
        self._config = config or VOConfig()
        self._solver = pnp_solver or OpenCVPnPSolver(self._config)
        self._previous: _Frame | None = None
        initial_pose = (
            np.eye(4, dtype=np.float64)
            if initial_world_from_camera is None
            else np.asarray(initial_world_from_camera, dtype=np.float64)
        )
        validate_rigid_transform(initial_pose, "initial_world_from_camera")
        self._world_from_camera = initial_pose.copy()
        self._consecutive_failures = 0
        self._last_timestamp: float | None = None

    def reset(self, reset_pose: bool = False) -> None:
        """Clear tracking history and optionally reset the accumulated origin."""
        self._previous = None
        self._consecutive_failures = 0
        if reset_pose:
            self._world_from_camera = np.eye(4, dtype=np.float64)

    def process(
        self, left_image: np.ndarray, right_image: np.ndarray, timestamp: float
    ) -> TrackingResult:
        """Process one synchronized rectified stereo pair."""
        timestamp = float(timestamp)
        if not isfinite(timestamp) or timestamp < 0.0:
            raise ValueError("timestamp must be finite and nonnegative")
        if self._last_timestamp is not None and timestamp <= self._last_timestamp:
            raise ValueError("timestamps must be strictly increasing")
        self._last_timestamp = timestamp
        total_start = perf_counter()
        extract_start = perf_counter()
        left = self._adapter.extract(left_image)
        right = self._adapter.extract(right_image)
        extract_ms = (perf_counter() - extract_start) * 1000.0
        match_start = perf_counter()
        stereo = _validated_matches(
            self._adapter.match(left, right), len(left.keypoints), len(right.keypoints)
        )
        stereo_ms = (perf_counter() - match_start) * 1000.0
        current = _Frame(left, right, stereo)
        base_fields = {
            "timestamp": timestamp,
            "world_from_camera": self._world_from_camera.copy(),
            "keypoints_left": len(left.keypoints),
            "keypoints_right": len(right.keypoints),
            "stereo_matches": len(stereo),
        }
        if self._previous is None:
            self._previous = current
            return TrackingResult(
                state=TrackingState.INITIALIZING,
                stage_latency_ms={
                    "extract": extract_ms,
                    "stereo_match": stereo_ms,
                    "total": (perf_counter() - total_start) * 1000.0,
                },
                **base_fields,
            )
        temporal_start = perf_counter()
        temporal = _validated_matches(
            self._adapter.match(self._previous.left, left),
            len(self._previous.left.keypoints),
            len(left.keypoints),
        )
        temporal_ms = (perf_counter() - temporal_start) * 1000.0
        points_3d, points_2d = triangulate_temporal_correspondences(
            self._previous.left.keypoints,
            self._previous.right.keypoints,
            left.keypoints,
            self._previous.stereo_matches,
            temporal,
            self._calibration,
            self._config.min_disparity_px,
            self._config.max_depth_m,
        )
        if len(points_3d) < self._config.min_pnp_points:
            return self._failure_result(
                base_fields,
                temporal,
                len(points_3d),
                f"insufficient PnP points: {len(points_3d)} < {self._config.min_pnp_points}",
                TrackingState.DEGRADED,
                total_start,
                extract_ms,
                stereo_ms,
                temporal_ms,
            )
        pnp_start = perf_counter()
        try:
            pnp = self._solver.solve(points_3d, points_2d, self._calibration)
            pnp = validate_pnp_result(pnp, len(points_3d), self._config.min_pnp_points)
            candidate_world_from_camera = accumulate_camera_motion(
                self._world_from_camera, pnp.current_from_previous
            )
            validate_rigid_transform(candidate_world_from_camera, "accumulated pose")
        except (
            PnPFailure,
            cv2.error,
            ValueError,
            TypeError,
            AttributeError,
            OverflowError,
        ) as error:
            pnp_ms = (perf_counter() - pnp_start) * 1000.0
            return self._failure_result(
                base_fields,
                temporal,
                len(points_3d),
                str(error),
                TrackingState.LOST,
                total_start,
                extract_ms,
                stereo_ms,
                temporal_ms,
                pnp_ms,
            )
        pnp_ms = (perf_counter() - pnp_start) * 1000.0
        self._world_from_camera = candidate_world_from_camera
        self._previous = current
        self._consecutive_failures = 0
        return TrackingResult(
            timestamp=float(timestamp),
            state=TrackingState.TRACKING,
            world_from_camera=self._world_from_camera.copy(),
            keypoints_left=len(left.keypoints),
            keypoints_right=len(right.keypoints),
            stereo_matches=len(stereo),
            temporal_matches=len(temporal),
            pnp_points=len(points_3d),
            pnp_inliers=pnp.inlier_count,
            pnp_inlier_ratio=pnp.inlier_count / len(points_3d),
            residual_mean_px=pnp.residual_mean_px,
            residual_p95_px=pnp.residual_p95_px,
            stage_latency_ms={
                "extract": extract_ms,
                "stereo_match": stereo_ms,
                "temporal_match": temporal_ms,
                "pnp": pnp_ms,
                "total": (perf_counter() - total_start) * 1000.0,
            },
        )

    def _failure_result(
        self,
        base_fields: dict[str, object],
        temporal: np.ndarray,
        pnp_points: int,
        reason: str,
        state: TrackingState,
        total_start: float,
        extract_ms: float,
        stereo_ms: float,
        temporal_ms: float,
        pnp_ms: float = 0.0,
    ) -> TrackingResult:
        self._consecutive_failures += 1
        if self._consecutive_failures >= self._config.reset_after_failures:
            self._previous = None
            self._consecutive_failures = 0
        return TrackingResult(
            state=state,
            failure_reason=reason,
            temporal_matches=len(temporal),
            pnp_points=pnp_points,
            stage_latency_ms={
                "extract": extract_ms,
                "stereo_match": stereo_ms,
                "temporal_match": temporal_ms,
                "pnp": pnp_ms,
                "total": (perf_counter() - total_start) * 1000.0,
            },
            **base_fields,
        )
