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

"""Regression tests for the lunar stereo VO metric qualification gate."""

from dataclasses import replace
import math

import numpy as np
import pytest

from trajectory_metrics import (
    PoseSample,
    QualificationCriteria,
    RigidTransform,
    RouteInterval,
    evaluate_trajectory,
    route_interval_from_commands,
)


def _quaternion(yaw: float) -> np.ndarray:
    return np.array([0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)])


def _planar_pose(time_s: float) -> tuple[np.ndarray, float]:
    side = min(int(time_s // 3.0), 3)
    phase = time_s - 3.0 * side
    corners = (
        np.array([0.0, 0.0]),
        np.array([1.8, 0.0]),
        np.array([1.8, 1.8]),
        np.array([0.0, 1.8]),
    )
    directions = (
        np.array([1.0, 0.0]),
        np.array([0.0, 1.0]),
        np.array([-1.0, 0.0]),
        np.array([0.0, -1.0]),
    )
    if phase <= 2.0:
        position = corners[side] + directions[side] * 0.9 * phase
        yaw = 0.4 + side * math.pi / 2.0
    else:
        position = corners[(side + 1) % 4]
        yaw = 0.4 + side * math.pi / 2.0 + (phase - 2.0) * math.pi / 2.0
    return position, yaw


def _fixture() -> tuple[list[PoseSample], list[PoseSample], RigidTransform]:
    spawn = np.array([-0.3, 0.5, 0.15373])
    extrinsic = RigidTransform(np.array([0.45, 0.18, 0.62]), _quaternion(0.2))
    truth = []
    camera_world = []
    for time_s in np.arange(0.0, 12.0001, 0.05):
        planar, yaw = _planar_pose(float(time_s))
        chassis_position = spawn + np.array([planar[0], planar[1], 0.0])
        truth.append(PoseSample(float(time_s), chassis_position, _quaternion(yaw)))
        cosine, sine = math.cos(yaw), math.sin(yaw)
        camera_offset = np.array(
            [
                cosine * extrinsic.translation[0] - sine * extrinsic.translation[1],
                sine * extrinsic.translation[0] + cosine * extrinsic.translation[1],
                extrinsic.translation[2],
            ]
        )
        camera_world.append((chassis_position + camera_offset, yaw + 0.2))

    anchor_position, anchor_yaw = camera_world[0]
    cosine, sine = math.cos(-anchor_yaw), math.sin(-anchor_yaw)
    vo = []
    for index in range(0, len(truth), 2):
        position, yaw = camera_world[index]
        delta = position - anchor_position
        anchored = np.array(
            [
                cosine * delta[0] - sine * delta[1],
                sine * delta[0] + cosine * delta[1],
                delta[2],
            ]
        )
        vo.append(
            PoseSample(truth[index].stamp, anchored, _quaternion(yaw - anchor_yaw))
        )
    return vo, truth, extrinsic


def _evaluate(
    vo: list[PoseSample],
    truth: list[PoseSample],
    extrinsic: RigidTransform,
    criteria: QualificationCriteria = QualificationCriteria(),
    route_interval: RouteInterval = RouteInterval(0.0, 12.0),
):
    return evaluate_trajectory(vo, truth, extrinsic, route_interval, criteria)


def test_nonzero_spawn_nonidentity_motion_passes_without_fitting() -> None:
    """World spawn and camera mounting offsets must cancel only through the common anchor."""
    vo, truth, extrinsic = _fixture()
    result = _evaluate(vo, truth, extrinsic)
    assert result.accepted, result.violations
    assert result.metrics["truth_path_length_m"] == pytest.approx(10.24, abs=0.02)
    assert result.metrics["truth_rotation_rad"] == pytest.approx(2 * math.pi, abs=0.02)
    assert result.metrics["ate_rmse_m"] < 1e-9
    assert result.metrics["orientation_rmse_rad"] < 1e-9


def test_functional_qualification_accepts_measured_thor_rate() -> None:
    """A healthy 4.5 Hz trajectory must pass without an invented throughput minimum."""
    vo, truth, extrinsic = _fixture()
    sample_indices = np.linspace(0, len(vo) - 1, 55, dtype=int)
    measured_rate_vo = [vo[index] for index in sample_indices]

    result = _evaluate(measured_rate_vo, truth, extrinsic)

    assert result.metrics["output_rate_hz"] == pytest.approx(4.5)
    assert result.accepted, result.violations


def test_world_offset_on_vo_is_removed_only_by_common_timestamp_anchor() -> None:
    """A nonzero VO origin must not be confused with translation fitting over the trajectory."""
    vo, truth, extrinsic = _fixture()
    offset_vo = [
        PoseSample(
            sample.stamp,
            sample.translation + np.array([4.0, -3.0, 1.0]),
            sample.quaternion,
        )
        for sample in vo
    ]
    result = _evaluate(offset_vo, truth, extrinsic)
    assert result.accepted, result.violations
    assert np.allclose(result.vo_anchored[0].translation, 0.0)
    assert np.allclose(result.truth_anchored[0].translation, 0.0)


def test_wrong_scale_trajectory_is_rejected() -> None:
    """Metric acceptance must not fit scale to make monocular-style drift disappear."""
    vo, truth, extrinsic = _fixture()
    scaled = [
        PoseSample(sample.stamp, sample.translation * 0.5, sample.quaternion)
        for sample in vo
    ]
    result = _evaluate(scaled, truth, extrinsic)
    assert not result.accepted
    assert "ATE RMSE exceeds limit" in result.violations


def test_chassis_frame_mislabeled_as_camera_frame_is_rejected() -> None:
    """Ignoring the physical camera extrinsic must fail during the square's rotations."""
    vo, truth, extrinsic = _fixture()
    anchor = truth[0]
    anchor_yaw = 0.4
    cosine, sine = math.cos(-anchor_yaw), math.sin(-anchor_yaw)
    wrong_frame = []
    for sample in truth[::2]:
        delta = sample.translation - anchor.translation
        wrong_frame.append(
            PoseSample(
                sample.stamp,
                np.array(
                    [
                        cosine * delta[0] - sine * delta[1],
                        sine * delta[0] + cosine * delta[1],
                        delta[2],
                    ]
                ),
                sample.quaternion,
            )
        )
    result = _evaluate(wrong_frame, truth, extrinsic)
    assert not result.accepted
    assert (
        result.metrics["position_p95_m"]
        > QualificationCriteria().maximum_position_p95_m
    )


def test_output_gap_and_insufficient_coverage_are_rejected() -> None:
    """Sparse output and a partial route cannot qualify despite accurate retained poses."""
    vo, truth, extrinsic = _fixture()
    gapped = vo[:30] + vo[40:]
    gap_result = _evaluate(gapped, truth, extrinsic)
    assert not gap_result.accepted
    assert "output gap exceeds limit" in gap_result.violations

    partial_result = _evaluate(vo[:50], truth, extrinsic)
    assert not partial_result.accepted
    assert "insufficient route translation coverage" in partial_result.violations
    assert "insufficient route rotation coverage" in partial_result.violations


def test_reset_segment_is_not_joined_into_a_continuous_trajectory() -> None:
    """A pose reset with increasing source stamps must still fail the continuity contract."""
    vo, truth, extrinsic = _fixture()
    reset = [
        PoseSample(
            sample.stamp,
            sample.translation + (np.array([2.0, 0.0, 0.0]) if index >= 60 else 0.0),
            sample.quaternion,
        )
        for index, sample in enumerate(vo)
    ]
    result = _evaluate(reset, truth, extrinsic)
    assert not result.accepted
    assert (
        "VO trajectory contains a translation discontinuity/reset" in result.violations
    )


@pytest.mark.parametrize("fault", ["duplicate", "backward", "nonfinite"])
def test_invalid_source_timestamps_are_rejected(fault: str) -> None:
    """Duplicate, backward, and non-finite source stamps must never be sorted or joined."""
    vo, truth, extrinsic = _fixture()
    broken = list(vo)
    if fault == "duplicate":
        broken[10] = PoseSample(
            broken[9].stamp, broken[10].translation, broken[10].quaternion
        )
    elif fault == "backward":
        broken[10] = PoseSample(
            broken[9].stamp - 0.01, broken[10].translation, broken[10].quaternion
        )
    else:
        broken[10] = PoseSample(math.nan, broken[10].translation, broken[10].quaternion)
    with pytest.raises(ValueError, match="timestamp|timestamps"):
        _evaluate(broken, truth, extrinsic)


def test_nonfinite_pose_and_wide_truth_brackets_are_rejected() -> None:
    """Malformed poses and either distant truth bracket must fail closed."""
    vo, truth, extrinsic = _fixture()
    broken = list(vo)
    broken[10] = PoseSample(
        broken[10].stamp, np.array([math.nan, 0.0, 0.0]), broken[10].quaternion
    )
    with pytest.raises(ValueError, match="finite"):
        _evaluate(broken, truth, extrinsic)

    sparse_truth = [sample for sample in truth if not 4.0 < sample.stamp < 4.4]
    with pytest.raises(ValueError, match="both must be within"):
        _evaluate(vo, sparse_truth, extrinsic)

    near_edge_vo = [
        PoseSample(0.01, vo[0].translation, vo[0].quaternion),
        PoseSample(10.0, vo[100].translation, vo[100].quaternion),
    ]
    wide_bracket_truth = [truth[0], truth[200], truth[-1]]
    with pytest.raises(ValueError, match="9.990000000 s after"):
        _evaluate(
            near_edge_vo,
            wide_bracket_truth,
            extrinsic,
            route_interval=RouteInterval(0.0, 10.0),
        )


@pytest.mark.parametrize(
    ("truncated", "violation"),
    [
        (slice(10, None), "leading route output gap exceeds limit"),
        (slice(None, 90), "trailing route output gap exceeds limit"),
    ],
)
def test_route_boundary_gaps_reject_aggregate_coverage_passes(
    truncated: slice, violation: str
) -> None:
    """Front or tail loss must fail even when translation and turn totals pass."""
    vo, truth, extrinsic = _fixture()
    result = _evaluate(vo[truncated], truth, extrinsic)
    criteria = QualificationCriteria()
    assert result.metrics["truth_path_length_m"] >= criteria.minimum_path_length_m
    assert result.metrics["truth_rotation_rad"] >= criteria.minimum_rotation_rad
    assert not result.accepted
    assert violation in result.violations


@pytest.mark.parametrize(
    ("criterion", "metric", "violation"),
    [
        ("minimum_samples", "sample_count", "insufficient accepted VO samples"),
        (
            "minimum_path_length_m",
            "truth_path_length_m",
            "insufficient route translation coverage",
        ),
        (
            "minimum_rotation_rad",
            "truth_rotation_rad",
            "insufficient route rotation coverage",
        ),
        ("maximum_ate_rmse_m", "ate_rmse_m", "ATE RMSE exceeds limit"),
        ("maximum_position_p95_m", "position_p95_m", "position p95 exceeds limit"),
        ("maximum_final_drift_m", "final_drift_m", "final drift exceeds limit"),
        (
            "maximum_orientation_rmse_rad",
            "orientation_rmse_rad",
            "orientation RMSE exceeds limit",
        ),
        ("maximum_output_gap_s", "maximum_output_gap_s", "output gap exceeds limit"),
        (
            "maximum_output_gap_s",
            "leading_output_gap_s",
            "leading route output gap exceeds limit",
        ),
        (
            "maximum_output_gap_s",
            "trailing_output_gap_s",
            "trailing route output gap exceeds limit",
        ),
        (
            "maximum_vo_step_m",
            "maximum_vo_step_m",
            "VO trajectory contains a translation discontinuity/reset",
        ),
        (
            "maximum_vo_step_rad",
            "maximum_vo_step_rad",
            "VO trajectory contains a rotation discontinuity/reset",
        ),
    ],
)
def test_qualification_limits_include_equality_and_reject_outside(
    criterion: str, metric: str, violation: str
) -> None:
    """Every acceptance comparison must include equality and distinguish adjacent limits."""
    vo, truth, extrinsic = _fixture()
    interval = RouteInterval(-0.25, 12.25)
    measured = _evaluate(vo, truth, extrinsic, route_interval=interval).metrics[metric]
    minimum = criterion.startswith("minimum_")
    step = 1 if criterion == "minimum_samples" else max(abs(measured) * 1e-6, 1e-12)
    for offset in (-step, 0, step):
        criteria = replace(QualificationCriteria(), **{criterion: measured + offset})
        result = _evaluate(vo, truth, extrinsic, criteria, interval)
        outside = offset > 0 if minimum else offset < 0
        assert (violation in result.violations) is outside, (
            criterion,
            offset,
            result.metrics,
        )


def test_route_interval_uses_stop_after_final_motion() -> None:
    """Interior segment STOP commands must not shorten the full route interval."""
    interval = route_interval_from_commands(
        [
            (1.0, False),
            (2.0, True),
            (3.0, False),
            (4.0, True),
            (5.0, False),
        ]
    )
    assert interval == RouteInterval(2.0, 5.0)
