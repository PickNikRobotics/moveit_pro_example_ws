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
"""Exercise recorder finalization without a live ROS graph or simulator."""
import importlib.util
import io
import json
import sys
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest


@pytest.fixture
def recorder_module():
    script = Path(__file__).resolve().parents[1] / "scripts/record_stack_evidence.py"
    spec = importlib.util.spec_from_file_location("record_stack_evidence", script)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture
def recorder(tmp_path, monkeypatch, recorder_module):
    module = recorder_module
    monkeypatch.setattr(module, "args", SimpleNamespace(output=tmp_path), raising=False)
    node = object.__new__(module.Recorder)
    node.truth_file = (tmp_path / "qpos.jsonl").open("w")
    node.video = None
    node.grasp_proxy = True
    node.last_frame = None
    node.frame_stamps = []
    cubes = {
        "red": {"position": [0, 0, 0.025], "quaternion_wxyz": [1, 0, 0, 0]},
        "blue": {"position": [0, 0, 0.075], "quaternion_wxyz": [1, 0, 0, 0]},
    }
    node.samples = [
        {"stamp": stamp, "sim_time": stamp, "cubes": cubes} for stamp in (0.0, 2.0)
    ]
    yield node
    node.truth_file.close()
    if node.video is not None:
        node.video.stdin.close()


def supply_frames(node, count, video):
    node.frame_stamps = [index / 30 for index in range(count)]
    if count:
        node.last_frame = np.zeros((8, 8, 3), dtype=np.uint8)
    if video:
        node.video = SimpleNamespace(stdin=io.BytesIO(), wait=Mock(return_value=0))


def assert_closed(node):
    assert node.truth_file.closed
    if node.video is not None:
        assert node.video.stdin.closed
        node.video.wait.assert_called_once_with()


@pytest.mark.parametrize("frame_count,video", [(0, False), (1, False), (1, True)])
def test_insufficient_frames_report_clear_error_and_close_resources(
    recorder, tmp_path, frame_count, video
):
    supply_frames(recorder, frame_count, video)
    with pytest.raises(RuntimeError, match="No live image or simulator pose evidence"):
        recorder.finish()
    assert_closed(recorder)
    assert not (tmp_path / "result.json").exists()
    assert not (tmp_path / "after.png").exists()


@pytest.mark.parametrize("video", [False, True])
@pytest.mark.parametrize("stamps", [[1.0, 1.0], [1.0, 0.0]])
def test_nonpositive_frame_duration_reports_clear_error_and_closes_resources(
    recorder, tmp_path, stamps, video
):
    supply_frames(recorder, 2, video)
    recorder.frame_stamps = stamps
    with pytest.raises(RuntimeError, match="Nonpositive elapsed camera frame time"):
        recorder.finish()
    assert_closed(recorder)
    assert not (tmp_path / "result.json").exists()
    assert not (tmp_path / "after.png").exists()


@pytest.mark.parametrize("video", [False, True])
def test_two_frames_write_timing_and_stack_report(recorder, tmp_path, video):
    supply_frames(recorder, 2, video)
    recorder.finish()
    assert_closed(recorder)
    report = json.loads((tmp_path / "result.json").read_text())
    assert report["passed"] is True
    assert report["camera"]["frame_count"] == 2
    assert report["camera"]["source_fps"] == pytest.approx(30)
    assert report["camera"]["max_source_gap_s"] == pytest.approx(1 / 30)
    assert report["tail_duration_s"] == pytest.approx(2)
    assert (tmp_path / "after.png").is_file()


@pytest.mark.parametrize("finish_fails", [False, True])
def test_main_cleans_up_ros_after_finalization(
    recorder_module, tmp_path, monkeypatch, finish_fails
):
    node = Mock()
    if finish_fails:
        node.finish.side_effect = RuntimeError(
            "No live image or simulator pose evidence received"
        )
    monkeypatch.setattr(recorder_module, "Recorder", Mock(return_value=node))
    ros = Mock()
    ros.ok.return_value = True
    monkeypatch.setattr(recorder_module, "rclpy", ros)
    monkeypatch.setattr(
        sys, "argv", ["record_stack_evidence", str(tmp_path / "run"), "--duration", "0"]
    )
    if finish_fails:
        with pytest.raises(RuntimeError, match="No live image"):
            recorder_module.main()
    else:
        recorder_module.main()
    node.finish.assert_called_once_with()
    node.destroy_node.assert_called_once_with()
    ros.shutdown.assert_called_once_with()


@pytest.mark.parametrize(
    "failures",
    [
        ("spin",),
        ("spin", "finish"),
        ("spin", "finish", "destroy", "shutdown"),
        ("finish", "destroy", "shutdown"),
        ("destroy", "shutdown"),
        ("shutdown",),
    ],
)
def test_main_preserves_first_failure_and_attempts_all_cleanup(
    recorder_module, tmp_path, monkeypatch, failures
):
    node, ros = Mock(), Mock()
    errors = {stage: RuntimeError(stage + " failed") for stage in failures}
    actions = {
        "spin": ros.spin_once,
        "finish": node.finish,
        "destroy": node.destroy_node,
        "shutdown": ros.shutdown,
    }
    for stage, error in errors.items():
        actions[stage].side_effect = error
    # Run one iteration when spin succeeds, then exit normally.
    ros.ok.side_effect = [True, False, True] if "spin" not in failures else [True, True]
    monkeypatch.setattr(recorder_module, "Recorder", Mock(return_value=node))
    monkeypatch.setattr(recorder_module, "rclpy", ros)
    monkeypatch.setattr(sys, "argv", ["record_stack_evidence", str(tmp_path / "run")])
    with pytest.raises(RuntimeError) as caught:
        recorder_module.main()
    assert caught.value is errors[failures[0]]
    for stage in failures[1:]:
        assert any(str(errors[stage]) in note for note in caught.value.__notes__)
    node.finish.assert_called_once_with()
    node.destroy_node.assert_called_once_with()
    ros.shutdown.assert_called_once_with()


@pytest.mark.parametrize("interruption", [KeyboardInterrupt, "external"])
def test_interrupted_run_still_reports_missing_evidence(
    recorder_module, tmp_path, monkeypatch, interruption
):
    node, ros = Mock(), Mock()
    ros.ok.return_value = True
    ros.spin_once.side_effect = (
        recorder_module.ExternalShutdownException()
        if interruption == "external"
        else interruption()
    )
    node.finish.side_effect = RuntimeError("No live image evidence")
    monkeypatch.setattr(recorder_module, "Recorder", Mock(return_value=node))
    monkeypatch.setattr(recorder_module, "rclpy", ros)
    monkeypatch.setattr(sys, "argv", ["record_stack_evidence", str(tmp_path / "run")])
    with pytest.raises(RuntimeError, match="No live image"):
        recorder_module.main()
    node.destroy_node.assert_called_once_with()
    ros.shutdown.assert_called_once_with()


def test_failed_node_construction_shuts_down_ros_without_masking_error(
    recorder_module, tmp_path, monkeypatch
):
    error = RuntimeError("model load failed")
    ros = Mock()
    ros.shutdown.side_effect = RuntimeError("shutdown failed")
    monkeypatch.setattr(recorder_module, "Recorder", Mock(side_effect=error))
    monkeypatch.setattr(recorder_module, "rclpy", ros)
    monkeypatch.setattr(sys, "argv", ["record_stack_evidence", str(tmp_path / "run")])
    with pytest.raises(RuntimeError) as caught:
        recorder_module.main()
    assert caught.value is error
    assert "shutdown failed" in caught.value.__notes__[0]
    ros.shutdown.assert_called_once_with()


@pytest.mark.parametrize("failing_close", ["truth", "stdin"])
def test_finish_reaps_encoder_even_when_stream_close_fails(recorder, failing_close):
    supply_frames(recorder, 2, True)
    stream = recorder.truth_file if failing_close == "truth" else recorder.video.stdin
    error = OSError(failing_close + " close failed")

    def close_and_fail():
        stream.close()
        raise error

    proxy = Mock(close=Mock(side_effect=close_and_fail))
    if failing_close == "truth":
        recorder.truth_file = proxy
    else:
        recorder.video.stdin = proxy
    recorder.video.wait.return_value = 1
    with pytest.raises(OSError) as caught:
        recorder.finish()
    assert caught.value is error
    assert "Video encoder failed" in caught.value.__notes__[0]
    recorder.video.wait.assert_called_once_with()
    # Restore the closed real stream for fixture teardown.
    if failing_close == "truth":
        recorder.truth_file = stream
    else:
        recorder.video.stdin = stream
