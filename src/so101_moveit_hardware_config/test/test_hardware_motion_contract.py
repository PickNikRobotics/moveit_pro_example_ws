"""Tests for fail-closed physical SO-101 motion defaults."""

from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


PACKAGE_DIR = Path(__file__).parents[1]
CONFIG_PATH = PACKAGE_DIR / "config" / "config.yaml"
LIMITS_PATH = PACKAGE_DIR / "config" / "moveit" / "joint_limits.yaml"
OBJECTIVE_PATH = PACKAGE_DIR / "objectives" / "move_live_so101_to_waypoint.xml"
WAYPOINTS_PATH = PACKAGE_DIR / "waypoints" / "so101_hardware_waypoints.yaml"
ARM_JOINTS = {
    "Rotation_R",
    "Pitch_R",
    "Elbow_R",
    "Wrist_Pitch_R",
    "Wrist_Roll_R",
}


def test_hardware_config_uses_hardware_specific_limits_and_waypoints() -> None:
    config = yaml.safe_load(CONFIG_PATH.read_text())

    assert config["moveit_params"]["joint_limits"] == {
        "package": "so101_moveit_hardware_config",
        "path": "config/moveit/joint_limits.yaml",
    }
    assert config["objectives"]["waypoints_file"] == {
        "package_name": "so101_moveit_hardware_config",
        "relative_path": "waypoints/so101_hardware_waypoints.yaml",
    }


def test_hardware_waypoints_contain_only_commissioned_poses() -> None:
    waypoints = yaml.safe_load(WAYPOINTS_PATH.read_text())

    assert [waypoint["name"] for waypoint in waypoints] == ["Home"]


def test_live_waypoint_objective_defaults_to_slow_home_motion() -> None:
    root = ET.parse(OBJECTIVE_PATH).getroot()
    model = root.find("./TreeNodesModel/SubTree[@ID='Move Live SO101 to Waypoint']")
    assert model is not None
    defaults = {
        port.attrib["name"]: port.attrib["default"]
        for port in model
        if port.tag.endswith("_port") and "default" in port.attrib
    }

    assert defaults["waypoint_name"] == "Home"
    assert defaults["velocity_scale_factor"] == "0.25"
    assert defaults["acceleration_scale_factor"] == "0.25"


def test_physical_arm_limits_are_conservative() -> None:
    limits = yaml.safe_load(LIMITS_PATH.read_text())["joint_limits"]

    assert ARM_JOINTS <= limits.keys()
    for joint_name in ARM_JOINTS:
        joint_limits = limits[joint_name]
        assert joint_limits["has_velocity_limits"] is True
        assert joint_limits["max_velocity"] <= 0.50
        assert joint_limits["has_acceleration_limits"] is True
        assert joint_limits["max_acceleration"] <= 0.50
