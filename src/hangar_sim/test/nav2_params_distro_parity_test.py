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

"""hangar_sim ships one nav2 parameter file per ROS distro.

`nav2_params.yaml` serves Humble and `nav2_params_jazzy.yaml` serves Jazzy,
because nav2's pluginlib lookup names differ between the two and the forms are
mutually exclusive. Everything else in the two files must stay identical -- any
tuning applied to one and forgotten on the other is a silent, distro-specific
config bug that only shows up on a running robot.

This test pins the complete set of allowed differences.
"""

import re
from pathlib import Path

import pytest
import yaml

PARAMS_DIR = Path(__file__).resolve().parent.parent / "params"
HUMBLE_PARAMS = PARAMS_DIR / "nav2_params.yaml"
JAZZY_PARAMS = PARAMS_DIR / "nav2_params_jazzy.yaml"

# Jazzy exports these plugins with no `name` attribute, so the pluginlib lookup
# name is the `pkg::Class` type. Humble's exports declare `name="pkg/Class"`.
EXPECTED_PLUGIN_RENAMES = {
    "planner_server.ros__parameters.GridBased.plugin": (
        "nav2_navfn_planner/NavfnPlanner",
        "nav2_navfn_planner::NavfnPlanner",
    ),
    "behavior_server.ros__parameters.spin.plugin": (
        "nav2_behaviors/Spin",
        "nav2_behaviors::Spin",
    ),
    "behavior_server.ros__parameters.backup.plugin": (
        "nav2_behaviors/BackUp",
        "nav2_behaviors::BackUp",
    ),
    "behavior_server.ros__parameters.drive_on_heading.plugin": (
        "nav2_behaviors/DriveOnHeading",
        "nav2_behaviors::DriveOnHeading",
    ),
    "behavior_server.ros__parameters.wait.plugin": (
        "nav2_behaviors/Wait",
        "nav2_behaviors::Wait",
    ),
    "behavior_server.ros__parameters.assisted_teleop.plugin": (
        "nav2_behaviors/AssistedTeleop",
        "nav2_behaviors::AssistedTeleop",
    ),
}

# Jazzy's bt_navigator always loads the default nav2 BT plugin libraries and the
# parameter is additive, so re-listing them double-registers every node.
#
# The rest are parameters Jazzy's nav2 renamed. Their Humble spellings are
# undeclared on Jazzy, so leaving them would be silently ignored rather than
# rejected -- the values happen to match Jazzy's defaults today, which is exactly
# what would make a future divergence invisible.
HUMBLE_ONLY_KEYS = {
    "bt_navigator.ros__parameters.plugin_lib_names",
    "controller_server.ros__parameters.progress_checker_plugin",
    "behavior_server.ros__parameters.costmap_topic",
    "behavior_server.ros__parameters.footprint_topic",
}

JAZZY_ONLY_KEYS = {
    "controller_server.ros__parameters.progress_checker_plugins",
    "controller_server.ros__parameters.FollowPath.open_loop",
    "behavior_server.ros__parameters.local_costmap_topic",
    "behavior_server.ros__parameters.local_footprint_topic",
}


def _flatten(node, prefix=""):
    if isinstance(node, dict) and node:
        for key, value in node.items():
            yield from _flatten(value, f"{prefix}.{key}" if prefix else str(key))
    else:
        yield prefix, node


@pytest.fixture(scope="module")
def flattened():
    humble = dict(_flatten(yaml.safe_load(HUMBLE_PARAMS.read_text())))
    jazzy = dict(_flatten(yaml.safe_load(JAZZY_PARAMS.read_text())))
    return humble, jazzy


def test_only_expected_keys_are_distro_specific(flattened):
    humble, jazzy = flattened
    assert set(humble) - set(jazzy) == HUMBLE_ONLY_KEYS
    assert set(jazzy) - set(humble) == JAZZY_ONLY_KEYS


def test_shared_keys_hold_identical_values(flattened):
    humble, jazzy = flattened
    mismatches = {
        key: (humble[key], jazzy[key])
        for key in set(humble) & set(jazzy)
        if humble[key] != jazzy[key]
    }
    assert mismatches == EXPECTED_PLUGIN_RENAMES


def test_jazzy_enables_mppi_open_loop(flattened):
    """The Jazzy backport only restores MPPI acceleration when explicitly enabled."""
    humble, jazzy = flattened
    key = "controller_server.ros__parameters.FollowPath.open_loop"
    assert key not in humble
    assert jazzy[key] is True


def test_jazzy_uses_no_slash_form_plugin_names(flattened):
    """Reject the pre-Jazzy `pkg/Class` form generally, not just today's six --
    a plugin added later to both files would otherwise slip through."""
    _, jazzy = flattened
    slash_form = {
        key: value
        for key, value in jazzy.items()
        if key.endswith(".plugin")
        and isinstance(value, str)
        and re.fullmatch(r"\w+/\w+", value)
    }
    assert slash_form == {}


def test_launch_file_references_both_params_files():
    """A typo in either filename would only surface as a bringup failure, and the
    integration suite does not observe nav2 bringup -- so pin it here."""
    launch_source = (
        PARAMS_DIR.parent / "launch" / "sim" / "robot_drivers_to_persist_sim.launch.py"
    ).read_text()
    for params in (HUMBLE_PARAMS, JAZZY_PARAMS):
        assert params.name in launch_source
        assert params.is_file()
