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

"""Tests for the public map-mode boolean conversion used by ROS launch conditions."""

import importlib.util
from pathlib import Path

import pytest

LAUNCH_FILE = (
    Path(__file__).resolve().parent.parent
    / "launch"
    / "sim"
    / "robot_drivers_to_persist_sim.launch.py"
)
SPEC = importlib.util.spec_from_file_location("lunar_sim_robot_drivers", LAUNCH_FILE)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


@pytest.mark.parametrize(("value", "expected"), [(False, "false"), (True, "true")])
def test_launch_bool_returns_lowercase_ros_boolean(value: bool, expected: str) -> None:
    """YAML booleans become lowercase values that ROS launch conditions interpret correctly."""
    assert MODULE._launch_bool(value) == expected


@pytest.mark.parametrize("value", [0, 1, "false", "true", None])
def test_launch_bool_rejects_non_boolean_config_values(value: object) -> None:
    """Malformed config values fail before they can select an unintended map mode."""
    with pytest.raises(TypeError, match="Expected a boolean config.yaml value"):
        MODULE._launch_bool(value)
