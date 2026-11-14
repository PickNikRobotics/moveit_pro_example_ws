#!/usr/bin/env python3

# Copyright 2026 PickNik Inc.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the PickNik Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""Metric, no-fit trajectory qualification for the lunar stereo VO example."""

from bisect import bisect_left
from dataclasses import dataclass
import math
from typing import Sequence

import numpy as np
from scipy.spatial.transform import Rotation, Slerp


@dataclass(frozen=True)
class PoseSample:
    """A source-stamped rigid pose with an xyzw quaternion."""

    stamp: float
    translation: np.ndarray
    quaternion: np.ndarray


@dataclass(frozen=True)
class RigidTransform:
    """A translation and xyzw quaternion."""

    translation: np.ndarray
    quaternion: np.ndarray


@dataclass(frozen=True)
class RouteInterval:
    """Source-clock bounds for the complete commanded route motion."""

    start_stamp: float
    end_stamp: float


@dataclass(frozen=True)
class QualificationCriteria:
    """Fixed acceptance limits for the 1.8 m lunar square."""

    minimum_samples: int = 40
    minimum_path_length_m: float = 5.5
    minimum_rotation_rad: float = 4.5
    maximum_ate_rmse_m: float = 0.35
    maximum_position_p95_m: float = 0.60
    maximum_final_drift_m: float = 0.45
    maximum_orientation_rmse_rad: float = math.radians(15.0)
    maximum_output_gap_s: float = 0.50
    maximum_truth_association_s: float = 0.05
    maximum_vo_step_m: float = 0.75
    maximum_vo_step_rad: float = 1.0


@dataclass(frozen=True)
class QualificationResult:
    """Metrics and deterministic acceptance violations."""

    accepted: bool
    violations: tuple[str, ...]
    metrics: dict[str, float | int]
    vo_anchored: tuple[PoseSample, ...]
    truth_anchored: tuple[PoseSample, ...]


def _quaternion(quaternion: np.ndarray, label: str) -> np.ndarray:
    value = np.asarray(quaternion, dtype=float)
    if value.shape != (4,) or not np.isfinite(value).all():
        raise ValueError(f"{label} quaternion must contain four finite values")
    norm = float(np.linalg.norm(value))
    if norm < 1e-12:
        raise ValueError(f"{label} quaternion has zero norm")
    return value / norm


def _translation(translation: np.ndarray, label: str) -> np.ndarray:
    value = np.asarray(translation, dtype=float)
    if value.shape != (3,) or not np.isfinite(value).all():
        raise ValueError(f"{label} translation must contain three finite values")
    return value


def _rotation(quaternion: np.ndarray, label: str) -> Rotation:
    return Rotation.from_quat(_quaternion(quaternion, label))


def _compose(lhs: RigidTransform, rhs: RigidTransform) -> RigidTransform:
    lhs_rotation = _rotation(lhs.quaternion, "left transform")
    rhs_rotation = _rotation(rhs.quaternion, "right transform")
    return RigidTransform(
        _translation(lhs.translation, "left transform")
        + lhs_rotation.apply(_translation(rhs.translation, "right transform")),
        (lhs_rotation * rhs_rotation).as_quat(),
    )


def _inverse(transform: RigidTransform) -> RigidTransform:
    rotation = _rotation(transform.quaternion, "inverse transform").inv()
    return RigidTransform(
        rotation.apply(-_translation(transform.translation, "inverse transform")),
        rotation.as_quat(),
    )


def _relative(anchor: RigidTransform, pose: RigidTransform) -> RigidTransform:
    return _compose(_inverse(anchor), pose)


def _angle(lhs: np.ndarray, rhs: np.ndarray) -> float:
    return float((_rotation(lhs, "angle").inv() * _rotation(rhs, "angle")).magnitude())


def _slerp(lhs: np.ndarray, rhs: np.ndarray, fraction: float) -> np.ndarray:
    rotations = Rotation.from_quat(
        np.vstack(
            [
                _quaternion(lhs, "interpolation"),
                _quaternion(rhs, "interpolation"),
            ]
        )
    )
    return Slerp([0.0, 1.0], rotations)([fraction]).as_quat()[0]


def _validated(samples: Sequence[PoseSample], label: str) -> tuple[PoseSample, ...]:
    if len(samples) < 2:
        raise ValueError(f"{label} trajectory needs at least two samples")
    result = []
    previous = -math.inf
    for index, sample in enumerate(samples):
        if not math.isfinite(sample.stamp):
            raise ValueError(f"{label} sample {index} has a non-finite timestamp")
        if sample.stamp <= previous:
            raise ValueError(
                f"{label} timestamps must be strictly increasing; sample {index} is duplicate or backward"
            )
        previous = sample.stamp
        result.append(
            PoseSample(
                sample.stamp,
                _translation(sample.translation, f"{label} sample {index}"),
                _quaternion(sample.quaternion, f"{label} sample {index}"),
            )
        )
    return tuple(result)


def _interpolate(
    truth: tuple[PoseSample, ...], stamp: float, maximum_association_s: float
) -> PoseSample:
    stamps = [sample.stamp for sample in truth]
    upper = bisect_left(stamps, stamp)
    if upper < len(truth) and truth[upper].stamp == stamp:
        return truth[upper]
    if upper == 0 or upper == len(truth):
        raise ValueError(f"VO stamp {stamp:.9f} is outside the truth interval")
    lower = upper - 1
    before = truth[lower]
    after = truth[upper]
    distance_before = stamp - before.stamp
    distance_after = after.stamp - stamp
    if (
        distance_before > maximum_association_s
        or distance_after > maximum_association_s
    ):
        raise ValueError(
            f"VO stamp {stamp:.9f} is bracketed by truth samples "
            f"{distance_before:.9f} s before and {distance_after:.9f} s after; "
            f"both must be within {maximum_association_s:.3f} s"
        )
    fraction = (stamp - before.stamp) / (after.stamp - before.stamp)
    return PoseSample(
        stamp,
        before.translation + fraction * (after.translation - before.translation),
        _slerp(before.quaternion, after.quaternion, fraction),
    )


def route_interval_from_commands(
    commands: Sequence[tuple[float, bool]],
) -> RouteInterval:
    """Bound all commanded motion using the first motion and post-final-motion STOP."""
    if not commands:
        raise ValueError("no same-run /cmd_vel commands were observed")
    previous = -math.inf
    for index, (stamp, _) in enumerate(commands):
        if not math.isfinite(stamp):
            raise ValueError(
                f"/cmd_vel command {index} has a non-finite source timestamp"
            )
        if stamp < previous:
            raise ValueError("/cmd_vel source timestamps moved backward")
        previous = stamp
    moving_indices = [index for index, (_, moving) in enumerate(commands) if moving]
    if not moving_indices:
        raise ValueError("no nonzero same-run /cmd_vel command was observed")
    first_motion = moving_indices[0]
    last_motion = moving_indices[-1]
    stop = next(
        (stamp for stamp, moving in commands[last_motion + 1 :] if not moving),
        None,
    )
    if stop is None:
        raise ValueError("no source-stamped STOP followed the final /cmd_vel motion")
    interval = RouteInterval(commands[first_motion][0], stop)
    if interval.end_stamp <= interval.start_stamp:
        raise ValueError("the final /cmd_vel STOP did not follow route motion")
    return interval


def evaluate_trajectory(
    vo_samples: Sequence[PoseSample],
    chassis_truth_samples: Sequence[PoseSample],
    chassis_to_camera: RigidTransform,
    route_interval: RouteInterval,
    criteria: QualificationCriteria = QualificationCriteria(),
) -> QualificationResult:
    """Score route-bounded VO against camera truth without trajectory fitting."""
    all_vo = _validated(vo_samples, "VO")
    truth = _validated(chassis_truth_samples, "truth")
    if not math.isfinite(route_interval.start_stamp) or not math.isfinite(
        route_interval.end_stamp
    ):
        raise ValueError("route interval must contain finite source timestamps")
    if route_interval.end_stamp <= route_interval.start_stamp:
        raise ValueError("route interval end must be after its start")
    vo = tuple(
        sample
        for sample in all_vo
        if route_interval.start_stamp <= sample.stamp <= route_interval.end_stamp
    )
    if len(vo) < 2:
        raise ValueError("route interval needs at least two VO samples")
    extrinsic = RigidTransform(
        _translation(chassis_to_camera.translation, "camera extrinsic"),
        _quaternion(chassis_to_camera.quaternion, "camera extrinsic"),
    )
    camera_truth = tuple(
        PoseSample(
            sample.stamp,
            *as_pose_tuple(
                _compose(
                    RigidTransform(sample.translation, sample.quaternion), extrinsic
                )
            ),
        )
        for sample in truth
    )
    matched_truth = tuple(
        _interpolate(camera_truth, sample.stamp, criteria.maximum_truth_association_s)
        for sample in vo
    )

    vo_anchor = RigidTransform(vo[0].translation, vo[0].quaternion)
    truth_anchor = RigidTransform(
        matched_truth[0].translation, matched_truth[0].quaternion
    )
    vo_anchored = tuple(_anchored_sample(sample, vo_anchor) for sample in vo)
    truth_anchored = tuple(
        _anchored_sample(sample, truth_anchor) for sample in matched_truth
    )

    gaps = np.diff([sample.stamp for sample in vo])
    duration = vo[-1].stamp - vo[0].stamp
    leading_output_gap = vo[0].stamp - route_interval.start_stamp
    trailing_output_gap = route_interval.end_stamp - vo[-1].stamp
    position_errors = np.array(
        [
            np.linalg.norm(candidate.translation - expected.translation)
            for candidate, expected in zip(vo_anchored, truth_anchored)
        ]
    )
    orientation_errors = np.array(
        [
            _angle(candidate.quaternion, expected.quaternion)
            for candidate, expected in zip(vo_anchored, truth_anchored)
        ]
    )
    truth_steps = np.array(
        [
            np.linalg.norm(current.translation - previous.translation)
            for previous, current in zip(truth_anchored, truth_anchored[1:])
        ]
    )
    truth_turns = np.array(
        [
            _angle(previous.quaternion, current.quaternion)
            for previous, current in zip(truth_anchored, truth_anchored[1:])
        ]
    )
    vo_steps = np.array(
        [
            np.linalg.norm(current.translation - previous.translation)
            for previous, current in zip(vo_anchored, vo_anchored[1:])
        ]
    )
    vo_turns = np.array(
        [
            _angle(previous.quaternion, current.quaternion)
            for previous, current in zip(vo_anchored, vo_anchored[1:])
        ]
    )
    metrics: dict[str, float | int] = {
        "sample_count": len(vo),
        "route_start_stamp": route_interval.start_stamp,
        "route_end_stamp": route_interval.end_stamp,
        "duration_s": duration,
        "leading_output_gap_s": leading_output_gap,
        "trailing_output_gap_s": trailing_output_gap,
        "truth_path_length_m": float(truth_steps.sum()),
        "truth_rotation_rad": float(truth_turns.sum()),
        "ate_rmse_m": float(np.sqrt(np.mean(np.square(position_errors)))),
        "position_p95_m": float(np.percentile(position_errors, 95)),
        "final_drift_m": float(position_errors[-1]),
        "orientation_rmse_rad": float(np.sqrt(np.mean(np.square(orientation_errors)))),
        "output_rate_hz": (len(vo) - 1) / duration,
        "maximum_output_gap_s": float(gaps.max()),
        "maximum_vo_step_m": float(vo_steps.max()),
        "maximum_vo_step_rad": float(vo_turns.max()),
    }
    checks = (
        (len(vo) >= criteria.minimum_samples, "insufficient accepted VO samples"),
        (
            metrics["truth_path_length_m"] >= criteria.minimum_path_length_m,
            "insufficient route translation coverage",
        ),
        (
            metrics["truth_rotation_rad"] >= criteria.minimum_rotation_rad,
            "insufficient route rotation coverage",
        ),
        (
            metrics["ate_rmse_m"] <= criteria.maximum_ate_rmse_m,
            "ATE RMSE exceeds limit",
        ),
        (
            metrics["position_p95_m"] <= criteria.maximum_position_p95_m,
            "position p95 exceeds limit",
        ),
        (
            metrics["final_drift_m"] <= criteria.maximum_final_drift_m,
            "final drift exceeds limit",
        ),
        (
            metrics["orientation_rmse_rad"] <= criteria.maximum_orientation_rmse_rad,
            "orientation RMSE exceeds limit",
        ),
        (
            metrics["maximum_output_gap_s"] <= criteria.maximum_output_gap_s,
            "output gap exceeds limit",
        ),
        (
            metrics["leading_output_gap_s"] <= criteria.maximum_output_gap_s,
            "leading route output gap exceeds limit",
        ),
        (
            metrics["trailing_output_gap_s"] <= criteria.maximum_output_gap_s,
            "trailing route output gap exceeds limit",
        ),
        (
            metrics["maximum_vo_step_m"] <= criteria.maximum_vo_step_m,
            "VO trajectory contains a translation discontinuity/reset",
        ),
        (
            metrics["maximum_vo_step_rad"] <= criteria.maximum_vo_step_rad,
            "VO trajectory contains a rotation discontinuity/reset",
        ),
    )
    violations = tuple(message for passed, message in checks if not passed)
    return QualificationResult(
        accepted=not violations,
        violations=violations,
        metrics=metrics,
        vo_anchored=vo_anchored,
        truth_anchored=truth_anchored,
    )


def _anchored_sample(sample: PoseSample, anchor: RigidTransform) -> PoseSample:
    relative = _relative(anchor, RigidTransform(sample.translation, sample.quaternion))
    return PoseSample(sample.stamp, relative.translation, relative.quaternion)


def as_pose_tuple(transform: RigidTransform) -> tuple[np.ndarray, np.ndarray]:
    """Return a transform's translation and quaternion for PoseSample construction."""
    return transform.translation, transform.quaternion
