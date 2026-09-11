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
"""Check that each hardware_interface branch names a plugin ros2_control can load."""

from pathlib import Path
import re

import pytest
import xacro
from ament_index_python.packages import get_package_share_directory
from ament_index_python.resources import get_resources

URDF = Path(__file__).parents[1] / "description" / "so101.urdf.xacro"


def hardware_plugin(hardware_interface):
    """The <plugin> the URDF's <ros2_control> block names for this branch."""
    doc = xacro.process_file(
        str(URDF), mappings={"hardware_interface": hardware_interface}
    )
    plugins = [node.firstChild.data for node in doc.getElementsByTagName("plugin")]
    assert len(plugins) == 1, plugins
    return plugins[0]


def registered_hardware_plugins():
    """Class names every hardware_interface plugin description on the path registers."""
    names = []
    for package, prefix in get_resources(
        "hardware_interface__pluginlib__plugin"
    ).items():
        index = Path(prefix) / "share/ament_index/resource_index"
        content = (
            index / "hardware_interface__pluginlib__plugin" / package
        ).read_text()
        for relative_path in content.splitlines():
            names += re.findall(
                r'name="([^"]+)"', (Path(prefix) / relative_path).read_text()
            )
    return names


@pytest.mark.parametrize(
    "hardware_interface, plugin",
    [
        ("mock", "mock_components/GenericSystem"),
        ("real", "feetech_ros2_driver/FeetechHardwareInterface"),
    ],
)
def test_branch_names_a_registered_plugin(hardware_interface, plugin):
    assert hardware_plugin(hardware_interface) == plugin
    assert plugin in registered_hardware_plugins()


def test_real_branch_resolves_to_the_vendored_driver():
    # The workspace overlay must shadow any apt ros-jazzy-feetech-ros2-driver:
    # the torque lifecycle this config relies on exists only in the vendored copy.
    share = Path(get_package_share_directory("feetech_ros2_driver"))
    assert not share.is_relative_to("/opt/ros"), share
