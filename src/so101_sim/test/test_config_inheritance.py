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
"""Fail loudly if so101_sim stops inheriting from so101_base_config.

Runs the overlay through `load_system_config`, the same loader `moveit_pro run`
uses, and asserts on the merged result the Agent actually gets: the base
package's robot description with only `hardware_interface` overridden, and
everything else - including the runtime launch file and the Objective
library, so101_sim adds neither - inherited unchanged.
"""

from pathlib import Path

import pytest
from moveit_studio_utils_py.system_config import load_system_config


@pytest.fixture(scope="module")
def loaded(tmp_path_factory):
    # user_ws only needs to exist; a src/ tree is optional.
    return load_system_config("so101_sim", tmp_path_factory.mktemp("user_ws"))


@pytest.fixture(scope="module")
def config(loaded):
    return loaded[0]


def test_inheritance_chain_is_base_then_overlay(loaded):
    assert loaded[2] == ["so101_base_config", "so101_sim"]


def test_robot_description_comes_from_the_base_package(config):
    robot_description = config.hardware.robot_description
    assert robot_description.urdf.package == "so101_base_config"
    assert robot_description.srdf.package == "so101_base_config"


def test_overlay_forces_mock_and_keeps_the_rest_of_urdf_params(config):
    params = {
        key: value
        for entry in config.hardware.robot_description.urdf_params
        for key, value in entry.items()
    }
    assert params["hardware_interface"] == "mock"
    assert {"usb_port", "calibration_file"} <= params.keys()
    assert params["calibration_file"].package == "so101_base_config"


def test_runtime_launch_file_comes_from_the_base_package(config):
    assert config.runtime_launch_file.package == "so101_base_config"


def test_objective_library_is_inherited_unchanged(config):
    libraries = config.objectives.objective_library_paths
    assert list(libraries)[-1] == "so101_objectives"
    assert "so101_sim_objectives" not in libraries
    assert libraries["so101_objectives"].package_name == "so101_base_config"
    for library in libraries.values():
        assert library.share_path.is_dir()


def test_waypoints_come_from_the_base_package(config):
    assert config.objectives.waypoints_file.package_name == "so101_base_config"
    assert Path(config.waypoints_file_path).is_file()
