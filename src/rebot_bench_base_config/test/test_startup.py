# Copyright 2026 PickNik Inc.
# SPDX-License-Identifier: BSD-3-Clause
"""The installed backend resolves the same YAML leader choice in every mode."""
import importlib.metadata
import importlib.util
from pathlib import Path
from types import SimpleNamespace

from ament_index_python.packages import get_package_share_directory
from moveit_studio_utils_py.system_config import load_system_config
import pytest


@pytest.fixture
def startup():
    path = (
        Path(get_package_share_directory("rebot_bench_base_config"))
        / "launch/rebot_leader.launch.py"
    )
    spec = importlib.util.spec_from_file_location("rebot_startup", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize("mode", ["mock", "real", "mujoco"])
@pytest.mark.parametrize("enabled", [False, True])
def test_merged_yaml_selects_one_reader_and_real_only_thermal_owner(
    startup, mode, enabled, tmp_path
):
    config = load_system_config("rebot_bench_base_config", tmp_path)[0]
    config.ros_global_params["rebot_leader_enabled"] = enabled
    for entry in config.hardware.robot_description.urdf_params:
        if "hardware_interface" in entry:
            entry["hardware_interface"] = mode
    settings = startup.startup_settings(config)
    assert settings["hardware_interface"] == mode
    assert settings["thermal_guard"] == (mode == "real")
    assert (settings["bridge"] is not None) == enabled
    # Smoke-load the actual launch actions without executing hardware processes.
    assert len(startup.configured_nodes(config)) == int(enabled) + int(mode == "real")
    assert startup.generate_launch_description().entities
    if enabled:
        bridge = settings["bridge"]
        params = config.ros_global_params
        assert bridge["publish_rate"] == params["rebot_leader_publish_rate"]
        assert bridge["arguments"] == [
            "--port",
            params["rebot_leader_port"],
            "--id",
            params["rebot_leader_id"],
            "--calibration-file",
            str(
                Path(get_package_share_directory("rebot_bench_base_config"))
                / params["rebot_leader_calibration_file"]
            ),
        ]


@pytest.mark.parametrize("value", ["true", 1, None])
def test_invalid_yaml_enable_fails_without_starting_anything(startup, value):
    config = SimpleNamespace(ros_global_params={"rebot_leader_enabled": value})
    with pytest.raises(ValueError, match="YAML boolean"):
        startup.startup_settings(config)


@pytest.mark.parametrize("rate", [0, 19, 101, float("nan"), float("inf")])
def test_invalid_rate_fails_before_starting(startup, rate, tmp_path):
    config = load_system_config("rebot_bench_base_config", tmp_path)[0]
    config.ros_global_params.update(
        rebot_leader_enabled=True, rebot_leader_publish_rate=rate
    )
    with pytest.raises(ValueError, match="between 20 and 100"):
        startup.startup_settings(config)


def test_absolute_calibration_and_custom_reader_settings(startup, tmp_path):
    config = load_system_config("rebot_bench_base_config", tmp_path)[0]
    calibration = str(tmp_path / "calibration.json")
    config.ros_global_params.update(
        rebot_leader_enabled=True,
        rebot_leader_port="/dev/leader",
        rebot_leader_id="custom",
        rebot_leader_calibration_file=calibration,
        rebot_leader_publish_rate=20.0,
    )
    assert startup.startup_settings(config)["bridge"] == {
        "publish_rate": 20.0,
        "arguments": [
            "--port",
            "/dev/leader",
            "--id",
            "custom",
            "--calibration-file",
            calibration,
        ],
    }


def test_installed_sdk_imports_in_ros_python_without_training_dependencies():
    import sys
    import numpy
    import motorbridge_smart_servo
    from cv_bridge import CvBridge

    dist = importlib.metadata.distribution("motorbridge-smart-servo")
    assert dist.version == "0.0.4"
    assert dist.metadata["License-Expression"] == "MIT"
    assert dist.requires is None
    assert callable(motorbridge_smart_servo.FashionStarServo)
    cv = CvBridge()
    bgr = numpy.full((4, 4, 3), [0, 1, 2], dtype=numpy.uint8)
    image = cv.cv2_to_imgmsg(bgr, "bgr8")
    assert numpy.array_equal(cv.imgmsg_to_cv2(image, "rgb8"), bgr[:, :, ::-1])
    assert not any(name.startswith(("lerobot", "torch")) for name in sys.modules)
