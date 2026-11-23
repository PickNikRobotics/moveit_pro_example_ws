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

"""FR3 joint 6 command limits and simulated endpoint tracking."""

import math
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

from ament_index_python.packages import get_package_prefix, get_package_share_directory
import mujoco
import numpy as np
import pytest
import yaml

PACKAGE = Path(__file__).resolve().parents[1]
JOINT = "fr3_joint6"


@pytest.fixture(scope="module")
def urdf():
    """Expand the kitchen description with its installed FR3 joint limits."""
    result = subprocess.run(
        [
            "xacro",
            str(PACKAGE / "description/franka.urdf.xacro"),
            "mujoco_viewer:=false",
            "mujoco_keyframe:=default",
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    return ET.fromstring(result.stdout)


@pytest.fixture(scope="module")
def urdf_limits(urdf):
    limit = urdf.find(f"joint[@name='{JOINT}']/limit")
    return np.array([float(limit.get("lower")), float(limit.get("upper"))])


@pytest.fixture(scope="module")
def model():
    """Compile the kitchen robot without scene objects or external contacts."""
    plugin = Path(get_package_prefix("mujoco_package_uri")) / "bin/mujoco_plugin"
    mujoco.mj_loadPluginLibrary(
        str(plugin / "libmujoco_package_uri_resource_provider.so")
    )
    return mujoco.MjModel.from_xml_path(str(PACKAGE / "description/mujoco/fr3.xml"))


@pytest.fixture(scope="module")
def command_limits(urdf):
    interface = urdf.find(
        f"ros2_control/joint[@name='{JOINT}']/command_interface[@name='position']"
    )
    return np.array(
        [
            float(interface.find(f"param[@name='{name}']").text)
            for name in ("min", "max")
        ]
    )


@pytest.fixture(scope="module")
def planning_limits(urdf_limits):
    """Resolve the position limits selected by the kitchen configuration."""
    config = yaml.safe_load((PACKAGE / "config/config.yaml").read_text())
    selected = config["moveit_params"]["joint_limits"]
    package = (
        PACKAGE
        if selected["package"] == "kitchen_sim"
        else Path(get_package_share_directory(selected["package"]))
    )
    overrides = yaml.safe_load((package / selected["path"]).read_text())
    joint = overrides.get("joint_limits", {}).get(JOINT, {})
    # The robot model loader reads an absent has_position_limits as true and applies whichever bound the YAML sets. With
    # has_position_limits: false it returns before reading min_position and max_position, so the URDF range applies.
    if not joint.get("has_position_limits", True):
        return urdf_limits
    return np.array(
        [
            joint.get("min_position", urdf_limits[0]),
            joint.get("max_position", urdf_limits[1]),
        ]
    )


def test_joint6_command_limits_match_urdf(command_limits, urdf_limits):
    np.testing.assert_allclose(command_limits, urdf_limits, rtol=0, atol=1e-8)


def test_joint6_simulation_limits_match_urdf(model, urdf_limits):
    joint = model.joint(JOINT)
    actuator = model.actuator(JOINT)
    assert model.jnt_limited[joint.id]
    assert model.actuator_ctrllimited[actuator.id]
    assert model.actuator_trnid[actuator.id, 0] == joint.id
    np.testing.assert_allclose(joint.range, urdf_limits, rtol=0, atol=1e-8)
    np.testing.assert_allclose(actuator.ctrlrange, urdf_limits, rtol=0, atol=1e-8)


@pytest.mark.parametrize("layer", ["urdf", "command", "joint", "actuator"])
def test_joint6_configured_planning_limits_are_executable(
    model, urdf_limits, command_limits, planning_limits, layer
):
    executable = {
        "urdf": urdf_limits,
        "command": command_limits,
        "joint": model.joint(JOINT).range,
        "actuator": model.actuator(JOINT).ctrlrange,
    }[layer]
    assert np.all(np.isfinite(planning_limits))
    assert planning_limits[0] < planning_limits[1]
    assert executable[0] <= planning_limits[0] <= planning_limits[1] <= executable[1], (
        f"{JOINT} configured planning range {planning_limits} exceeds "
        f"{layer} range {executable}"
    )


@pytest.fixture(scope="module")
def goal_time_tolerance():
    """Settling time the arm's trajectory controller allows after a trajectory ends, in seconds."""
    config = yaml.safe_load((PACKAGE / "config/config.yaml").read_text())
    selected = config["ros2_control"]["config"]
    package = (
        PACKAGE
        if selected["package"] == "kitchen_sim"
        else Path(get_package_share_directory(selected["package"]))
    )
    controllers = yaml.safe_load((package / selected["path"]).read_text())
    return float(
        controllers["joint_trajectory_admittance_controller"]["ros__parameters"][
            "default_goal_time_tolerance"
        ]
    )


@pytest.mark.parametrize("endpoint", [0, 1], ids=["lower", "upper"])
def test_joint6_reaches_urdf_endpoint(
    model, urdf, urdf_limits, goal_time_tolerance, endpoint
):
    """Reach either valid endpoint within the controller's ordinary settling window."""
    data = mujoco.MjData(model)
    for actuator_id in range(model.nu):
        name = model.actuator(actuator_id).name
        initial = urdf.find(
            f"ros2_control/joint[@name='{name}']/"
            "state_interface[@name='position']/param[@name='initial_value']"
        )
        value = float(initial.text)
        data.qpos[model.joint(name).qposadr[0]] = value
        data.ctrl[actuator_id] = value
    joint = model.joint(JOINT)
    target = urdf_limits[endpoint]
    # Start inside the requested endpoint so the test also exercises convergence.
    data.qpos[joint.qposadr[0]] = target + (0.05 if endpoint == 0 else -0.05)
    data.ctrl[model.actuator(JOINT).id] = target
    mujoco.mj_forward(model, data)
    # Check the last simulation sample at or before the controller's settling deadline.
    for _ in range(math.floor(goal_time_tolerance / model.opt.timestep)):
        mujoco.mj_step(model, data)
        assert data.ncon == 0
    error = abs(target - data.qpos[joint.qposadr[0]])
    # Require convergence from the 0.05 rad initial displacement.
    assert error <= 0.01, f"{JOINT} endpoint error: {error:.6f} rad"
