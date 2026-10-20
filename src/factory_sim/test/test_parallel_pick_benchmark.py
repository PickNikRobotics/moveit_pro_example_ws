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

"""Validate cycle timing extraction without running robot motion."""

from benchmark_bracket_profiles import profile_summary

from benchmark_parallel_pick import Benchmark, cycle_times, timing_summary
import benchmark_parallel_pick as benchmark_module
from concurrent.futures import Future
from types import SimpleNamespace

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


@pytest.fixture
def delayed_diagnostics(monkeypatch):
    """Complete the action before delivering its queued diagnostic callbacks."""
    clock = [0.0]
    queued = []
    node = SimpleNamespace(messages=[])
    response = SimpleNamespace(
        status=4,
        result=SimpleNamespace(error_code=SimpleNamespace(val=1), error_message=""),
    )
    sent = Future()
    result = Future()
    handle = SimpleNamespace(accepted=True, get_result_async=lambda: result)
    node.client = SimpleNamespace(
        wait_for_server=lambda timeout_sec: True,
        send_goal_async=lambda goal: sent,
    )
    node.wait_for_cycle_diagnostics = (
        lambda expected: Benchmark.wait_for_cycle_diagnostics(node, expected)
    )
    monkeypatch.setattr(benchmark_module.time, "monotonic", lambda: clock[0])

    def complete_action(node, future, timeout_sec):
        if future is sent:
            future.set_result(handle)
        else:
            clock[0] += 3.0
            future.set_result(response)

    def deliver_log(node, timeout_sec):
        clock[0] += timeout_sec
        if queued:
            node.messages.append(queued.pop(0))

    monkeypatch.setattr(
        benchmark_module.rclpy, "spin_until_future_complete", complete_action
    )
    monkeypatch.setattr(benchmark_module.rclpy, "spin_once", deliver_log)
    return node, queued, clock, response


def test_result_before_diagnostics_collects_all_cycles_without_inflating_time(
    delayed_diagnostics,
):
    """Queued labels remain collectible after the result future becomes ready."""
    node, queued, clock, _ = delayed_diagnostics
    for duration in [1.0, 2.0, 3.0]:
        queued.extend(
            [
                {"message": f"StopwatchEnd Info: Time elapsed: {duration}s"},
                {"message": "StopwatchEnd Info: CYCLE: bracket lookahead"},
            ]
        )
    result = Benchmark.run(node, object(), 10.0, expected_cycles=3)
    assert cycle_times(node.messages) == [1.0, 2.0, 3.0]
    assert result["cycle_diagnostics_complete"] is True
    assert result["duration_s"] == 3.0
    assert clock[0] > result["duration_s"]


def test_missing_diagnostics_stop_at_deadline_without_fabricating_cycles(
    delayed_diagnostics,
):
    """Missing labels cannot cause an unbounded wait or a successful full batch."""
    node, queued, clock, _ = delayed_diagnostics
    queued.extend(
        [
            {"message": "StopwatchEnd Info: Time elapsed: 1.0s"},
            {"message": "StopwatchEnd Info: CYCLE: bracket lookahead"},
        ]
    )
    result = Benchmark.run(node, object(), 10.0, expected_cycles=3)
    assert result["cycle_diagnostics_complete"] is False
    assert cycle_times(node.messages) == [1.0]
    assert clock[0] == pytest.approx(5.0)
    assert result["duration_s"] == 3.0


def test_failed_goal_does_not_wait_for_success_diagnostics(delayed_diagnostics):
    """Failure cannot consume the diagnostic grace period or become success."""
    node, _, clock, response = delayed_diagnostics
    response.status = 6
    response.result.error_code.val = -1
    result = Benchmark.run(node, object(), 10.0, expected_cycles=3)
    assert result["status"] == 6
    assert result["cycle_diagnostics_complete"] is False
    assert clock[0] == 3.0


def test_reset_without_expected_cycles_needs_no_cycle_labels(delayed_diagnostics):
    """A Simulator reset has no bracket cycles to await."""
    node, _, clock, _ = delayed_diagnostics
    result = Benchmark.run(node, object(), 10.0)
    assert result["cycle_diagnostics_complete"] is True
    assert clock[0] == 3.0


def test_original_profile_cycle_labels_are_collected(delayed_diagnostics):
    """The profile comparator can await the original application's labels."""
    node, queued, _, _ = delayed_diagnostics
    queued.extend(
        [
            {"message": "StopwatchEnd Info: Time elapsed: 1.0s"},
            {"message": "StopwatchEnd Info: CYCLE: original bracket cycle"},
        ]
    )
    result = Benchmark.run(node, object(), 10.0, expected_cycles=1)
    assert result["cycle_diagnostics_complete"] is True
    assert len(node.messages) == 2
