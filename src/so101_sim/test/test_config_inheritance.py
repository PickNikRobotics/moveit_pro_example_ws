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

This does not re-implement MoveIt Pro's `SystemConfigParser` merge; it checks
the two things that make the overlay an overlay: `based_on_package` still
names `so101_base_config`, that package is actually installed alongside this
one (the `<depend>` in package.xml is real, not just a string in a comment),
and the override this overlay exists for - forcing mock hardware - is still
present.
"""

from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory


def load_yaml(package_name, relative_path):
    path = Path(get_package_share_directory(package_name)) / relative_path
    return yaml.safe_load(path.read_text())


def urdf_param(urdf_params, key):
    for entry in urdf_params:
        if key in entry:
            return entry[key]
    raise KeyError(key)


def test_overlay_is_based_on_so101_base_config():
    overlay = load_yaml("so101_sim", "config/config.yaml")
    assert overlay["based_on_package"] == "so101_base_config"


def test_so101_base_config_is_installed_and_resolvable():
    # Raises PackageNotFoundError if so101_sim's <depend> on so101_base_config
    # ever gets dropped from package.xml, or so101_base_config fails to build.
    base_share = Path(get_package_share_directory("so101_base_config"))
    assert (base_share / "config" / "config.yaml").exists()


def test_overlay_forces_mock_hardware():
    overlay = load_yaml("so101_sim", "config/config.yaml")
    urdf_params = overlay["hardware"]["robot_description"]["urdf_params"]
    assert urdf_param(urdf_params, "hardware_interface") == "mock"


def test_base_config_defaults_are_still_there_for_the_overlay_to_inherit():
    """The overlay only overrides `hardware_interface`; everything else it
    relies on - waypoints, Objectives, the udev-backed calibration file - must
    still come from the base package's own config.yaml."""
    base = load_yaml("so101_base_config", "config/config.yaml")
    assert "based_on_package" not in base
    urdf_params = base["hardware"]["robot_description"]["urdf_params"]
    assert urdf_param(urdf_params, "usb_port") == "/dev/so101_follower"
    waypoints_file = base["objectives"]["waypoints_file"]
    assert waypoints_file["package_name"] == "so101_base_config"
    base_share = Path(get_package_share_directory("so101_base_config"))
    assert (base_share / waypoints_file["relative_path"]).exists()
