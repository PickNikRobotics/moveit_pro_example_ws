# Copyright 2026 PickNik Inc.
# SPDX-License-Identifier: BSD-3-Clause

"""Validate cycle timing extraction without running robot motion."""

from benchmark_bracket_profiles import profile_summary

from benchmark_parallel_pick import cycle_times, timing_summary

import pytest


def test_cycle_duration_excludes_planning_duration() -> None:
    """Only the duration paired with the cycle label contributes to the result."""
    messages = [
        {"message": "StopwatchEnd Info: Time elapsed: 1.25s"},
        {"message": "StopwatchEnd Info: PLANNING: vacuum pick from bin"},
        {"message": "StopwatchEnd Info: Time elapsed: 24.3s"},
        {"message": "StopwatchEnd Info: CYCLE: bracket lookahead"},
    ]
    assert cycle_times(messages) == [24.3]


def test_unpaired_cycle_does_not_reuse_previous_stage_duration() -> None:
    """A lost timing message cannot turn an earlier planning duration into a cycle."""
    messages = [
        {"message": "StopwatchEnd Info: Time elapsed: 1.25s"},
        {"message": "StopwatchEnd Info: PLANNING: vacuum pick from bin"},
        {"message": "StopwatchEnd Info: CYCLE: bracket lookahead"},
    ]
    with pytest.raises(RuntimeError, match="no preceding stopwatch duration"):
        cycle_times(messages)


def test_empty_timing_stream_has_no_cycles() -> None:
    """No cycle times are fabricated when execution has not started."""
    assert cycle_times([]) == []


def test_terminal_cycle_is_not_a_preparation_opportunity() -> None:
    """The final cycle has no next pick and cannot measure preparation overlap."""
    summary = timing_summary(
        [
            {"parallel": True, "cycle_times_s": [25.0, 18.0, 17.0], "duration_s": 78.0},
            {"parallel": True, "cycle_times_s": [26.0, 20.0, 17.0], "duration_s": 80.0},
            {
                "parallel": False,
                "cycle_times_s": [30.0, 25.0, 23.0],
                "duration_s": 95.0,
            },
        ],
        True,
    )
    assert summary["steady_preparation_cycles"] == 2
    assert summary["steady_preparation_median_s"] == 19.0
    assert summary["terminal_cycle_median_s"] == 17.0
    assert summary["batch_median_s"] == 79.0


def test_short_batch_has_no_steady_preparation_cycle() -> None:
    """A one-pick batch must not claim a steady preparation measurement."""
    summary = timing_summary(
        [
            {"parallel": True, "cycle_times_s": [22.0], "duration_s": 40.0},
        ],
        True,
    )
    assert summary["steady_preparation_cycles"] == 0
    assert summary["steady_preparation_median_s"] is None


def test_profile_summary_keeps_original_and_cached_batches_separate() -> None:
    """Unrelated profiles cannot alter the original baseline statistic."""
    runs = [
        {
            "profile": "original",
            "duration_s": 90.0,
            "cycle_times_s": [24.0, 25.0, 26.0],
        },
        {"profile": "cached", "duration_s": 70.0, "cycle_times_s": [21.0, 16.0, 17.0]},
        {
            "profile": "original",
            "duration_s": 92.0,
            "cycle_times_s": [24.0, 27.0, 26.0],
        },
    ]
    assert profile_summary(runs, "original")["batch_median_s"] == 91.0
    assert profile_summary(runs, "original")["middle_cycle_median_s"] == 26.0


def test_profile_summary_rejects_absent_profile() -> None:
    """A missing baseline cannot be reported as a zero-time run."""
    with pytest.raises(ValueError, match="No completed runs"):
        profile_summary([], "original")
