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
"""Check argument defaults/overrides and the offline (no-pip) failure path.

Never invokes lerobot or a real port: subprocess.run and input are patched out.
"""

from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).parents[1] / "script"))
import calibrate_so101  # noqa: E402


def test_parse_args_defaults_to_the_udev_symlink_and_arm_id():
    args = calibrate_so101.parse_args(["follower"])
    assert args.arm == "follower"
    assert args.port is None
    assert args.calibration_id is None


def test_parse_args_accepts_an_override_port_and_id():
    args = calibrate_so101.parse_args(
        ["leader", "--port", "/dev/ttyACM3", "--id", "so101_leader_2"]
    )
    assert args.port == "/dev/ttyACM3"
    assert args.calibration_id == "so101_leader_2"


def test_parse_args_rejects_an_unknown_arm():
    with pytest.raises(SystemExit):
        calibrate_so101.parse_args(["gripper"])


def test_run_calibration_uses_defaults_and_execs_lerobot(monkeypatch, tmp_path):
    subdir = tmp_path / "robots" / "so_follower"
    monkeypatch.setitem(calibrate_so101.ARMS["follower"], "calibration_subdir", subdir)

    commands = []

    def fake_run(cmd, check):
        assert check
        subdir.mkdir(parents=True)
        (subdir / "so101_follower.json").write_text("{}")
        commands.append(cmd)

    monkeypatch.setattr(calibrate_so101.subprocess, "run", fake_run)
    monkeypatch.setattr("builtins.input", lambda *_: "")

    calibrate_so101.run_calibration("follower", "/dev/so101_follower", "so101_follower")

    assert commands[0] == [
        calibrate_so101.sys.executable,
        "-m",
        "lerobot.scripts.lerobot_calibrate",
        "--robot.type=so101_follower",
        "--robot.port=/dev/so101_follower",
        "--robot.id=so101_follower",
    ]


def test_run_calibration_fails_loudly_if_no_json_appears(monkeypatch, tmp_path):
    monkeypatch.setitem(
        calibrate_so101.ARMS["leader"],
        "calibration_subdir",
        tmp_path / "teleoperators" / "so_leader",
    )
    monkeypatch.setattr(calibrate_so101.subprocess, "run", lambda cmd, check: None)
    monkeypatch.setattr("builtins.input", lambda *_: "")

    with pytest.raises(SystemExit):
        calibrate_so101.run_calibration("leader", "/dev/so101_leader", "so101_leader")


def test_run_calibration_reports_a_nonzero_exit_without_a_traceback(
    monkeypatch, tmp_path
):
    monkeypatch.setitem(
        calibrate_so101.ARMS["leader"],
        "calibration_subdir",
        tmp_path / "teleoperators" / "so_leader",
    )

    def fake_run(cmd, check):
        raise calibrate_so101.subprocess.CalledProcessError(1, cmd)

    monkeypatch.setattr(calibrate_so101.subprocess, "run", fake_run)
    monkeypatch.setattr("builtins.input", lambda *_: "")

    with pytest.raises(SystemExit) as exit_info:
        calibrate_so101.run_calibration("leader", "/dev/so101_leader", "so101_leader")
    assert "status 1" in str(exit_info.value)
    assert "5 V" in str(exit_info.value)


def test_run_calibration_says_so_when_existing_json_is_kept(
    monkeypatch, tmp_path, capsys
):
    subdir = tmp_path / "robots" / "so_follower"
    subdir.mkdir(parents=True)
    existing = subdir / "so101_follower.json"
    existing.write_text("{}")
    monkeypatch.setitem(calibrate_so101.ARMS["follower"], "calibration_subdir", subdir)
    monkeypatch.setattr(calibrate_so101.subprocess, "run", lambda cmd, check: None)
    monkeypatch.setattr("builtins.input", lambda *_: "")

    calibrate_so101.run_calibration("follower", "/dev/so101_follower", "so101_follower")

    out = capsys.readouterr().out
    assert "Kept existing" in out
    assert "Wrote" not in out


def test_main_refuses_to_run_when_lerobot_is_not_importable(monkeypatch):
    monkeypatch.setattr(calibrate_so101, "check_lerobot_available", lambda: False)
    with pytest.raises(SystemExit):
        calibrate_so101.main(["follower"])
