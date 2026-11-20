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

from ament_index_python.packages import get_package_prefix
import mujoco
import numpy as np
import pytest

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
def limits(urdf):
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


def test_joint6_command_limits_match_planning(urdf, limits):
    interface = urdf.find(
        f"ros2_control/joint[@name='{JOINT}']/command_interface[@name='position']"
    )
    command_limits = [
        float(interface.find(f"param[@name='{name}']").text) for name in ("min", "max")
    ]
    np.testing.assert_allclose(command_limits, limits, rtol=0, atol=1e-8)


def test_joint6_simulation_limits_match_planning(model, limits):
    joint = model.joint(JOINT)
    actuator = model.actuator(JOINT)
    assert model.jnt_limited[joint.id]
    assert model.actuator_ctrllimited[actuator.id]
    assert model.actuator_trnid[actuator.id, 0] == joint.id
    np.testing.assert_allclose(joint.range, limits, rtol=0, atol=1e-8)
    np.testing.assert_allclose(actuator.ctrlrange, limits, rtol=0, atol=1e-8)


@pytest.mark.parametrize("endpoint", [0, 1], ids=["lower", "upper"])
def test_joint6_reaches_planning_endpoint(model, urdf, limits, endpoint):
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
    target = limits[endpoint]
    # Start inside the requested endpoint so the test also exercises convergence.
    data.qpos[joint.qposadr[0]] = target + (0.05 if endpoint == 0 else -0.05)
    data.ctrl[model.actuator(JOINT).id] = target
    mujoco.mj_forward(model, data)
    for _ in range(math.ceil(0.5 / model.opt.timestep)):
        mujoco.mj_step(model, data)
        assert data.ncon == 0
    error = abs(target - data.qpos[joint.qposadr[0]])
    # Require convergence from the 0.05 rad initial displacement.
    assert error <= 0.01, f"{JOINT} endpoint error: {error:.6f} rad"
