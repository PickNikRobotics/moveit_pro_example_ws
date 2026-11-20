# Copyright 2026 PickNik Inc.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
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

"""Exercise idle mapping availability through ROS launch conditions."""

import importlib.util
from pathlib import Path

from launch import LaunchContext
from launch.actions import GroupAction, IncludeLaunchDescription
from launch_ros.actions import Node
import pytest

PACKAGE = Path(__file__).resolve().parents[1]


@pytest.fixture
def driver_launch(monkeypatch):
    """Load the driver launch interface without requiring an installed workspace."""
    spec = importlib.util.spec_from_file_location(
        "hangar_driver_launch",
        PACKAGE / "launch/sim/robot_drivers_to_persist_sim.launch.py",
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module, "get_package_share_directory", lambda _: str(PACKAGE))
    monkeypatch.setattr(module, "_slam_from_config", lambda: "False")
    return module.generate_launch_description()


LAUNCH_BOOLEANS = [
    ("false", False),
    ("False", False),
    ("0", False),
    ("true", True),
    ("True", True),
    ("1", True),
]


@pytest.mark.parametrize("slam, slam_enabled", LAUNCH_BOOLEANS)
@pytest.mark.parametrize("localization, localization_enabled", LAUNCH_BOOLEANS)
@pytest.mark.parametrize("use_fuse", ["false", "true"])
def test_idle_mapping_requires_localization(
    driver_launch, slam, slam_enabled, localization, localization_enabled, use_fuse
):
    """The truth-only fallback cannot start Build Map; localized operation can."""
    context = LaunchContext()
    context.launch_configurations.update(
        slam=slam, localization=localization, use_fuse=use_fuse
    )
    actions = [
        child
        for group in driver_launch.entities
        if isinstance(group, GroupAction)
        for child in group.get_sub_entities()
    ]
    (mapping_launch,) = [
        action
        for action in actions
        if isinstance(action, IncludeLaunchDescription)
        and dict(action.launch_arguments).get("use_lifecycle_manager") == "true"
    ]
    (mapping_manager,) = [
        action
        for action in actions
        if isinstance(action, Node) and action.node_executable == "lifecycle_manager"
    ]
    expected = not slam_enabled and localization_enabled
    assert mapping_launch.condition.evaluate(context) is expected
    assert mapping_manager.condition.evaluate(context) is expected
