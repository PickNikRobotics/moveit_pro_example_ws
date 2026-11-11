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

"""Guard the Base tab's dedicated controller: no simulator, no ROS."""

import math
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
import yaml

PACKAGE = Path(__file__).resolve().parents[1]
CONTROL = yaml.safe_load(
    (PACKAGE / "config/control/picknik_ur.ros2_control.yaml").read_text()
)
OTHER_BASE_CONTROLLERS = (
    "platform_velocity_controller",
    "platform_velocity_controller_nav2",
)


def read_yaml(relative_path: str) -> dict:
    """Read a package configuration file."""
    return yaml.safe_load((PACKAGE / relative_path).read_text())


@pytest.fixture
def base_jog() -> dict:
    """Resolve BaseJog through the config entry the Runtime loads."""
    location = read_yaml("config/config.yaml")["moveit_params"]["base_jog"]
    assert location["package"] == "hangar_sim"
    return read_yaml(location["path"])


@pytest.fixture
def controller(base_jog: dict) -> dict:
    """The parameters of the controller the Base tab activates."""
    return CONTROL[base_jog["controller"]]["ros__parameters"]


def test_base_jog_controller_is_dedicated_and_loaded_inactive(base_jog: dict) -> None:
    """The Base tab owns its own controller, which starts inactive."""
    name = base_jog["controller"]
    assert name not in OTHER_BASE_CONTROLLERS
    ros2_control = read_yaml("config/config.yaml")["ros2_control"]
    assert name in ros2_control["controllers_inactive_at_startup"]
    assert name not in ros2_control["controllers_active_at_startup"]
    assert CONTROL["controller_manager"]["ros__parameters"][name]["type"] == (
        "clearpath_mecanum_drive_controller/MecanumDriveController"
    )


@pytest.mark.parametrize("other", OTHER_BASE_CONTROLLERS)
def test_base_jog_controller_claims_the_same_wheels(
    controller: dict, other: str
) -> None:
    """Shared wheels make activating one base controller deactivate the others."""
    assert controller["command_joint_names"] == (
        CONTROL[other]["ros__parameters"]["command_joint_names"]
    )


def test_base_jog_controller_reads_body_frame_stamped_commands(
    controller: dict,
) -> None:
    """BaseJog publishes frameless TwistStamped commands in the base's body frame."""
    assert controller["use_stamped_vel"] is True
    assert controller["body_frame_control"] is True
    # 0 would hold the last wheel command for the whole command_timeout after the pad goes quiet.
    assert 0 < controller["reference_timeout"] <= 0.5


@pytest.mark.parametrize("other", OTHER_BASE_CONTROLLERS)
def test_other_base_controllers_stay_unstamped(other: str) -> None:
    """Nav2 and whole-body motion keep the unstamped input they use on main."""
    assert CONTROL[other]["ros__parameters"]["use_stamped_vel"] is False


@pytest.mark.parametrize("axis", ["linear.x", "linear.y", "angular.z"])
def test_base_jog_controller_ramps_symmetrically(controller: dict, axis: str) -> None:
    """BaseJog sends steps, so the controller must ramp them the same way in both directions."""
    assert controller[f"{axis}.has_acceleration_limits"] is True
    maximum = controller[f"{axis}.max_acceleration"]
    assert maximum > 0
    assert controller[f"{axis}.min_acceleration"] == -maximum


@pytest.mark.parametrize(
    ("limit", "axis"),
    [("max_linear_velocity", "linear.x"), ("max_angular_velocity", "angular.z")],
)
def test_base_jog_speed_caps_fit_controller(
    base_jog: dict, controller: dict, limit: str, axis: str
) -> None:
    """Both command directions stay within the controller's limits."""
    speed = base_jog[limit]
    assert math.isfinite(speed)
    assert speed > 0
    assert controller[f"{axis}.has_velocity_limits"] is True
    assert speed <= controller[f"{axis}.max_velocity"]
    assert -speed >= controller[f"{axis}.min_velocity"]


def test_base_jog_group_contains_only_base_joints(base_jog: dict) -> None:
    """The Base tab selects an existing group without arm joints."""
    srdf = ET.parse(PACKAGE / "config/moveit/picknik_ur.srdf")
    group = srdf.find(f"group[@name='{base_jog['planning_group']}']")
    assert group is not None
    assert {joint.attrib["name"] for joint in group.findall("joint")} == {
        "linear_x_joint",
        "linear_y_joint",
        "rotational_yaw_joint",
    }
    assert not group.findall("chain")


def test_base_mode_switches_controller_before_jogging(base_jog: dict) -> None:
    """Mode 8 activates the selected controller, then publishes to its own input."""
    tree = ET.parse(PACKAGE / "objectives/request_teleoperation.xml")
    branch = tree.find(".//Control[@_while='teleop_mode == 8']")
    assert branch is not None
    switch, jog = list(branch)
    assert switch.attrib["ID"] == "SwitchController"
    assert switch.attrib["activate_controllers"] == "{controllers}"
    assert jog.attrib["ID"] == "BaseJog"
    assert jog.attrib["cmd_vel_topic"] == f"/{base_jog['controller']}/cmd_vel"


def test_state_estimator_fuses_base_jog_odometry(base_jog: dict) -> None:
    """Localization and mapping keep wheel odometry while the Base tab drives."""
    fuse = read_yaml("config/fuse/fuse.yaml")["state_estimator"]["ros__parameters"]
    topic = f"/{base_jog['controller']}/odom"
    sensors = [
        name
        for name, sensor in fuse.items()
        if isinstance(sensor, dict) and sensor.get("topic") == topic
    ]
    assert len(sensors) == 1
    assert fuse["sensor_models"][sensors[0]]["type"] == "fuse_models::Odometry3D"
