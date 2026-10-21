# Copyright 2026 PickNik Inc.
# SPDX-License-Identifier: BSD-3-Clause
"""Optional camera startup never requires both devices or a particular bench."""
import importlib.util
from pathlib import Path
import sys
import time
from unittest.mock import Mock

from launch import LaunchContext, LaunchDescription, LaunchService
from launch.actions import ExecuteProcess
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
        ({"scene"}, ["camera_node", "point_cloud_xyz_node"], 1),
        (
            {"wrist", "scene"},
            ["realsense2_camera_node", "camera_node", "point_cloud_xyz_node"],
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
        (False, True, ["camera_node", "point_cloud_xyz_node"]),
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


def test_depth_is_unscaled_unless_a_measured_factor_is_configured(cameras, monkeypatch):
    monkeypatch.setattr(cameras, "detected_cameras", lambda: {"scene"})
    ctx = context()
    assert "scale_depth.py" not in executables(cameras.launch_cameras(ctx), ctx)
    ctx = context(scene_depth_scale=0.96)
    assert "scale_depth.py" in executables(cameras.launch_cameras(ctx), ctx)


def test_vanished_wrist_camera_driver_is_stopped_instead_of_flooding(cameras):
    # librealsense after an unplug: the same error forever, ignoring SIGINT/SIGTERM.
    flood = (
        "import signal, time\n"
        "signal.signal(signal.SIGINT, signal.SIG_IGN)\n"
        "signal.signal(signal.SIGTERM, signal.SIG_IGN)\n"
        "end = time.monotonic() + 20\n"
        "while time.monotonic() < end:\n"
        "    print('xioctl(VIDIOC_QBUF) failed when requesting new frame, "
        "error number: 19, No such device', flush=True)\n"
        "    time.sleep(0.01)\n"
    )
    driver = ExecuteProcess(
        cmd=[sys.executable, "-u", "-c", flood],
        output="screen",
        sigterm_timeout="0.2",
        sigkill_timeout="0.2",
    )
    service = LaunchService()
    service.include_launch_description(
        LaunchDescription([cameras.stop_on_device_loss(driver), driver])
    )
    started = time.monotonic()
    service.run()
    assert time.monotonic() - started < 10
    cameras.logger.warning.assert_called_once()
    assert "restart the backend" in cameras.logger.warning.call_args.args[0]


def test_attached_wrist_camera_driver_stops_on_its_own_device_loss(
    cameras, monkeypatch
):
    from launch.actions import RegisterEventHandler
    from launch.events.process import ProcessStderr

    monkeypatch.setattr(cameras, "detected_cameras", lambda: {"wrist"})
    ctx = context()
    actions = cameras.launch_cameras(ctx)
    (handler,) = [
        a.event_handler for a in actions if isinstance(a, RegisterEventHandler)
    ]
    (driver,) = [a for a in actions if isinstance(a, Node)]

    def output(action, text):
        return ProcessStderr(
            text=text, action=action, name="d", cmd=[], cwd=None, env=None, pid=1
        )

    assert handler.matches(output(driver, b"error number: 19, No such device\n"))
    assert not handler.matches(output(driver, b"RealSense Node Is Up!\n"))
    other = ExecuteProcess(cmd=["true"])
    assert not handler.matches(output(other, b"No such device\n"))
