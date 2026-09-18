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
"""Check so101_arm_bridge.launch.py's source resolution and config-error handling."""

import importlib.util
import os
from pathlib import Path
from unittest.mock import patch

import pytest

_LAUNCH_FILE = Path(__file__).parents[1] / "launch" / "so101_arm_bridge.launch.py"
_spec = importlib.util.spec_from_file_location("so101_arm_bridge_launch", _LAUNCH_FILE)
_module = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_module)
select_source = _module.select_source
read_hardware_interface = _module.read_hardware_interface


def test_auto_defers_to_hardware_interface():
    assert select_source("auto", "real") == "real"
    assert select_source("auto", "mock") == "fake"
    assert select_source("auto", None) == "fake"
    assert select_source("auto", "anything-else") == "fake"


def test_fake_and_real_force_the_source_regardless_of_config():
    assert select_source("fake", "real") == "fake"
    assert select_source("real", "mock") == "real"
    assert select_source("real", None) == "real"


def test_read_hardware_interface_returns_none_without_user_ws():
    """Launched standalone, outside a full instance, is safe to read as fake."""
    with patch.dict(os.environ, {}, clear=True):
        assert read_hardware_interface() is None


def test_read_hardware_interface_raises_on_a_broken_config():
    """Inside a full instance, a config load failure must fail the launch, not
    silently fall back to fake."""
    with patch.dict(os.environ, {"USER_WS": "/tmp"}, clear=True):
        with patch.object(
            _module, "get_config_package", side_effect=RuntimeError("boom")
        ):
            with pytest.raises(RuntimeError):
                read_hardware_interface()
