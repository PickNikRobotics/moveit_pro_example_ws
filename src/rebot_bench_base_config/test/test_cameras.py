# Copyright 2026 PickNik Inc.
# SPDX-License-Identifier: BSD-3-Clause
"""Optional camera startup never requires both devices or a particular bench."""
import importlib.util
from pathlib import Path
from unittest.mock import Mock

from launch import LaunchContext
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from launch_ros.actions import Node
import pytest
import yaml

PACKAGE = Path(__file__).resolve().parents[1]


@pytest.fixture
def cameras():
    spec = importlib.util.spec_from_file_location(
        "rebot_camera_launch", PACKAGE / "launch/cameras.launch.py"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    module.logger = Mock()
    return module


def context(**overrides):
    settings = yaml.safe_load((PACKAGE / "config/cameras.yaml").read_text())
    settings.update(overrides)
    result = LaunchContext()
    result.launch_configurations.update(
        {
            key: str(value).lower() if isinstance(value, bool) else str(value)
            for key, value in settings.items()
        }
    )
    return result


def executables(actions, ctx):
    return [
        perform_substitutions(
            ctx, normalize_to_list_of_substitutions(node.node_executable)
        )
        for node in actions
        if isinstance(node, Node)
    ]


def test_defaults_enable_optional_cameras_without_pinning_a_unit(cameras):
    ctx = context()
    assert ctx.launch_configurations["enable_cameras"] == "true"
    assert ctx.launch_configurations["enable_wrist_camera"] == "true"
    assert ctx.launch_configurations["enable_scene_camera"] == "true"
    assert ctx.launch_configurations["wrist_serial_no"] == ""
    assert ctx.launch_configurations["scene_serial_no"] == ""
    assert cameras.generate_launch_description().entities


@pytest.mark.parametrize(
    "devices,expected,warnings",
    [
        (set(), [], 2),
        ({"wrist"}, ["realsense2_camera_node"], 1),
        ({"scene"}, ["camera_node", "scale_depth.py", "point_cloud_xyz_node"], 1),
        (
            {"wrist", "scene"},
            [
                "realsense2_camera_node",
                "camera_node",
                "scale_depth.py",
                "point_cloud_xyz_node",
            ],
            0,
        ),
    ],
)
def test_missing_camera_skips_only_its_processes(
    cameras, monkeypatch, devices, expected, warnings
):
    monkeypatch.setattr(cameras, "detected_cameras", lambda: devices)
    ctx = context()
    actions = cameras.launch_cameras(ctx)
    assert executables(actions, ctx) == expected
    assert cameras.logger.warning.call_count == warnings


@pytest.mark.parametrize(
    "wrist,scene,expected",
    [
        (False, True, ["camera_node", "scale_depth.py", "point_cloud_xyz_node"]),
        (True, False, ["realsense2_camera_node"]),
        (False, False, []),
    ],
)
def test_camera_switches_are_independent(cameras, monkeypatch, wrist, scene, expected):
    monkeypatch.setattr(cameras, "detected_cameras", lambda: {"wrist", "scene"})
    ctx = context(enable_wrist_camera=wrist, enable_scene_camera=scene)
    actions = cameras.launch_cameras(ctx)
    assert executables(actions, ctx) == expected
    cameras.logger.warning.assert_not_called()


def test_usb_probe_recognizes_models_and_ignores_unknown_or_incomplete_devices(
    cameras, tmp_path
):
    for name, vendor, product in [
        ("wrist", "8086", "0b07"),
        ("scene_unbooted", "03e7", "2485"),
        ("scene_booted", "03e7", "f63b"),
        ("unknown", "1234", "abcd"),
    ]:
        device = tmp_path / name
        device.mkdir()
        (device / "idVendor").write_text(vendor + "\n")
        (device / "idProduct").write_text(product + "\n")
    (tmp_path / "incomplete").mkdir()
    assert cameras.detected_cameras(tmp_path) == {"wrist", "scene"}
    assert cameras.detected_cameras(tmp_path / "missing") == set()


def test_unknown_camera_does_not_start_a_supported_driver(
    cameras, monkeypatch, tmp_path
):
    device = tmp_path / "other_camera"
    device.mkdir()
    (device / "idVendor").write_text("1234")
    (device / "idProduct").write_text("abcd")
    probe = cameras.detected_cameras
    monkeypatch.setattr(cameras, "detected_cameras", lambda: probe(tmp_path))
    ctx = context()
    actions = cameras.launch_cameras(ctx)
    assert executables(actions, ctx) == []
    assert cameras.logger.warning.call_count == 2
    assert all(
        "Backend and leader remain available" in call.args[0]
        for call in cameras.logger.warning.call_args_list
    )
