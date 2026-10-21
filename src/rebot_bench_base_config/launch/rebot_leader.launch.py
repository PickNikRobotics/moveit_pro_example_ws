# Copyright 2026 PickNik Inc.
# SPDX-License-Identifier: BSD-3-Clause
"""Backend-owned leader and thermal recovery from the selected merged config."""
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node
from moveit_studio_utils_py.system_config import get_config_package, load_system_config


def startup_settings(config):
    """Return plain startup values from merged YAML without starting processes."""
    params = config.ros_global_params or {}
    enabled = params.get("rebot_leader_enabled", False)
    if not isinstance(enabled, bool):
        raise ValueError("rebot_leader_enabled must be a YAML boolean")
    urdf_params = {
        key: value
        for entry in config.hardware.robot_description.urdf_params
        for key, value in entry.items()
    }
    mode = urdf_params.get("hardware_interface", "mock")
    if mode not in ("mock", "real", "mujoco"):
        raise ValueError(f"Unsupported reBot hardware_interface: {mode}")
    settings = {
        "hardware_interface": mode,
        "thermal_guard": mode == "real",
        "bridge": None,
    }
    if enabled:
        calibration = Path(params["rebot_leader_calibration_file"])
        if not calibration.is_absolute():
            calibration = (
                Path(get_package_share_directory("rebot_bench_base_config"))
                / calibration
            )
        settings["bridge"] = {
            "publish_rate": float(params.get("rebot_leader_publish_rate", 100.0)),
            "arguments": [
                "--port",
                str(params["rebot_leader_port"]),
                "--calibration-file",
                str(calibration),
            ],
        }
    return settings


def configured_nodes(config):
    """Create the backend-owned processes selected by the startup contract."""
    settings = startup_settings(config)
    nodes = []
    if settings["thermal_guard"]:
        nodes.append(
            Node(
                package="rebot_bench_base_config",
                executable="rebot_thermal_guard.py",
                name="rebot_thermal_guard",
                output="screen",
            )
        )
    if settings["bridge"] is not None:
        bridge = settings["bridge"]
        nodes.append(
            Node(
                package="rebot_bench_base_config",
                executable="rebot_leader_bridge.py",
                name="rebot_leader_bridge",
                output="screen",
                # Leave enough time for the existing native HOME recovery.
                sigterm_timeout="130",
                parameters=[{"publish_rate": bridge["publish_rate"]}],
                arguments=bridge["arguments"],
            )
        )
    return nodes


def launch_configured_nodes(context):
    del context
    config, _, _ = load_system_config(get_config_package(), Path(os.environ["USER_WS"]))
    return configured_nodes(config)


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=launch_configured_nodes)])
