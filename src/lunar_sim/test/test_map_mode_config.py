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


@pytest.fixture
def map_mode_launch(monkeypatch):
    """Construct real launch actions with package/config discovery supplied by the fixture."""
    from launch.actions import IncludeLaunchDescription
    from launch.launch_description_sources import PythonLaunchDescriptionSource
    from launch_ros.actions import Node

    nodes = {}
    includes = {}
    sources = {}

    def source(path):
        action = PythonLaunchDescriptionSource(path)
        sources[id(action)] = Path(path).name
        return action

    def node(**kwargs):
        action = Node(**kwargs)
        nodes[kwargs["name"]] = action
        return action

    def include(source, **kwargs):
        action = IncludeLaunchDescription(source, **kwargs)
        includes[sources[id(source)]] = action
        return action

    monkeypatch.setattr(
        MODULE, "get_package_share_directory",
        lambda _: str(LAUNCH_FILE.parent.parent.parent),
    )
    monkeypatch.setattr(
        MODULE, "_urdf_param",
        lambda name: False if name in ("slam", "localization") else "default",
    )
    monkeypatch.setattr(MODULE, "_spawn_pose", lambda *_: (0.0, 0.0, 0.0))
    monkeypatch.setattr(MODULE, "NodeWithAnsiLogging", node)
    monkeypatch.setattr(MODULE, "IncludeLaunchDescription", include)
    monkeypatch.setattr(MODULE, "PythonLaunchDescriptionSource", source)
    return MODULE.generate_launch_description(), nodes, includes


BOOLEAN_VALUES = ["true", "True", " TRUE ", "1", "false", "False", " FALSE ", "0"]


@pytest.mark.parametrize("slam", BOOLEAN_VALUES)
@pytest.mark.parametrize("localization", BOOLEAN_VALUES)
def test_map_mode_has_exactly_one_tf_owner(map_mode_launch, slam, localization):
    """CLI boolean spellings select exclusive publishers and matching localization support."""
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument, OpaqueFunction

    description, nodes, includes = map_mode_launch
    context = LaunchContext()
    context.launch_configurations.update(slam=slam, localization=localization)
    for action in description.entities:
        if isinstance(action, (DeclareLaunchArgument, OpaqueFunction)):
            for child in action.execute(context) or []:
                child.execute(context)

    mapping = slam.strip().lower() in ("true", "1")
    localizing = not mapping and localization.strip().lower() in ("true", "1")
    owners = [
        includes["slam_launch.py"].condition.evaluate(context),
        includes["localization_launch.py"].condition.evaluate(context),
        nodes["static_tf_map_to_odom"].condition.evaluate(context),
    ]
    assert owners == [mapping, localizing, not (mapping or localizing)]
    assert sum(owners) == 1
    for name in ("localization_container", "scan_front_relay", "scan_rear_relay"):
        assert nodes[name].condition.evaluate(context) == localizing
    assert nodes["state_estimator"].condition is None


@pytest.mark.parametrize("argument", ["slam", "localization"])
@pytest.mark.parametrize("invalid", ["", "yes", "2", "'true'", "true or false"])
def test_invalid_map_switch_fails_before_nodes_start(map_mode_launch, argument, invalid):
    """Invalid overrides are rejected even when SLAM would take precedence."""
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument, OpaqueFunction
    from launch.conditions import InvalidConditionExpressionError

    description, _, _ = map_mode_launch
    context = LaunchContext()
    context.launch_configurations.update(slam="true", localization="false")
    context.launch_configurations[argument] = invalid
    with pytest.raises(InvalidConditionExpressionError):
        for action in description.entities:
            if isinstance(action, (DeclareLaunchArgument, OpaqueFunction)):
                for child in action.execute(context) or []:
                    child.execute(context)
            else:
                pytest.fail("A node or include was reached before switch validation")
