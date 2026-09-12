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
"""Check so101_drivers.launch.py's config/so101_drivers.yaml default loading."""

import importlib.util
from pathlib import Path

import pytest

_LAUNCH_FILE = Path(__file__).parents[1] / "launch" / "so101_drivers.launch.py"
_spec = importlib.util.spec_from_file_location("so101_drivers_launch", _LAUNCH_FILE)
_module = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_module)
load_driver_defaults = _module.load_driver_defaults

# Every launch argument generate_launch_description() declares from the YAML.
EXPECTED_KEYS = {
    "real_slew_rate_rad_s",
    "leader_wrist_roll_offset_rad",
    "leader_timeout_s",
    "leader_port",
    "wrist_camera_device",
    "scene_camera_device",
    "wrist_camera_brightness",
    "scene_camera_brightness",
}


def test_load_driver_defaults_has_every_declared_launch_argument():
    assert EXPECTED_KEYS <= load_driver_defaults().keys()


def test_load_driver_defaults_matches_the_checked_in_values():
    """A DeclareLaunchArgument default has to str()-format sensibly - this
    also pins the checked-in tuning values against an accidental edit."""
    defaults = load_driver_defaults()
    assert defaults["real_slew_rate_rad_s"] == pytest.approx(4.0)
    assert defaults["leader_wrist_roll_offset_rad"] == pytest.approx(1.68)
    assert defaults["leader_timeout_s"] == pytest.approx(0.5)
    assert defaults["leader_port"] == "/dev/so101_leader"
    assert defaults["wrist_camera_brightness"] == 0
    assert defaults["scene_camera_brightness"] == 128
