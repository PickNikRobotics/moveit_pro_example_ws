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

"""Exercise the installed scene's real actuators and contacts without ROS motion."""

import json
from pathlib import Path

from ament_index_python.packages import get_package_prefix, get_package_share_directory
import mujoco
import numpy as np
import pytest


@pytest.mark.parametrize(
    "side,block_name,arm_positions",
    [
        (
            "right",
            "block_red_3",
            [
                0.22728149,
                0.73151791,
                -1.00153045,
                -1.84700153,
                -0.48244381,
                2.57497201,
                2.74926899,
            ],
        ),
        (
            "left",
            "block_green_2",
            [
                0.08046400,
                1.42594702,
                0.02776247,
                -1.08445314,
                1.02392209,
                2.03964838,
                1.17891876,
            ],
        ),
    ],
)
def test_regression_block_stays_in_hand_through_lift_and_carry(
    side: str, block_name: str, arm_positions: list[float]
) -> None:
    """Recorded pregrasp arm poses must retain a block during a 100 mm lift and 225 mm carry."""
    mujoco.mj_loadAllPluginLibraries(
        str(Path(get_package_prefix("mujoco_package_uri")) / "bin" / "mujoco_plugin")
    )
    scene = (
        Path(get_package_share_directory("dual_arm_sim"))
        / "description/mujoco/scene.xml"
    )
    model = mujoco.MjModel.from_xml_path(str(scene))
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, model.key("default").id)
    for number, position in enumerate(arm_positions, start=1):
        name = f"{side}_fr3v2_joint{number}"
        data.qpos[model.jnt_qposadr[model.joint(name).id]] = position
        data.ctrl[model.actuator(name).id] = position
    for number in (1, 2):
        name = f"{side}_fr3v2_finger_joint{number}"
        data.qpos[model.jnt_qposadr[model.joint(name).id]] = 0.035
    gripper = model.actuator(f"{side}_fr3v2_finger_joint1").id
    spine = model.actuator("franka_spine_vertical_joint").id
    initial_spine = data.ctrl[spine]
    carry = model.actuator("planar_y").id
    tcp = model.body(f"{side}_fr3v2_hand_tcp").id
    block = model.body(block_name).id
    table = model.geom("table_front").id
    mujoco.mj_forward(model, data)
    table_top = data.geom_xpos[table, 2] + model.geom_size[table, 2]
    block_half_width = model.geom_size[model.body_geomadr[block], 2]
    initial_relative = None
    max_slip = 0.0
    for step in range(round(5.5 / model.opt.timestep)):
        time = step * model.opt.timestep
        # Let the blocks settle, close, then lift with the spine and carry with the base.
        data.ctrl[gripper] = 0.035 if time < 0.5 else 0.0
        lift = np.clip(time - 1.5, 0.0, 1.0)
        data.ctrl[spine] = initial_spine + 0.1 * (3 * lift**2 - 2 * lift**3)
        phase = np.clip(time - 3.0, 0.0, 1.5) / 1.5
        direction = 1 if side == "left" else -1
        data.ctrl[carry] = direction * 0.3 * np.sin(np.pi * phase) ** 2
        mujoco.mj_step(model, data)
        assert np.isfinite(data.qpos).all(), (side, time, "nonfinite position")
        assert np.isfinite(data.qvel).all(), (side, time, "nonfinite velocity")
        relative = data.xmat[tcp].reshape(3, 3).T @ (data.xpos[block] - data.xpos[tcp])
        if time >= 1.5:
            if initial_relative is None:
                initial_relative = relative.copy()
            max_slip = max(max_slip, np.linalg.norm(relative - initial_relative))
    # A pad is 18.5 mm tall: 10 mm drift is already over half its height, not a stable grasp.
    assert max_slip < 0.010, (side, "block slipped relative to hand", max_slip)
    assert data.xpos[block, 2] - block_half_width > table_top + 0.075, (
        side,
        "block did not stay above the table through carry",
        data.xpos[block].copy(),
    )


@pytest.mark.parametrize(
    "filename,block_name",
    [
        ("right_arm_carry.json", "block_red_1"),
        ("right_arm_far_block_carry.json", "block_red_3"),
    ],
)
def test_regression_full_arm_carry_keeps_block_in_hand(
    filename: str, block_name: str
) -> None:
    """Recorded arm carries must retain both near and far blocks at the Objective's speed."""
    mujoco.mj_loadAllPluginLibraries(
        str(Path(get_package_prefix("mujoco_package_uri")) / "bin" / "mujoco_plugin")
    )
    scene = (
        Path(get_package_share_directory("dual_arm_sim"))
        / "description/mujoco/scene.xml"
    )
    model = mujoco.MjModel.from_xml_path(str(scene))
    data = mujoco.MjData(model)
    trajectory = json.loads((Path(__file__).parent / "data" / filename).read_text())
    for name, positions in trajectory["initial_joint_positions"].items():
        address = model.jnt_qposadr[model.joint(name).id]
        data.qpos[address : address + len(positions)] = positions
    # test_sort_blocks.cpp checks this scale at both planners' input ports.
    times = np.asarray(trajectory["time"]) / 0.25
    commands = np.asarray(trajectory["commands"])
    actuators = [model.actuator(name).id for name in trajectory["actuator_names"]]
    gripper = model.actuator("right_fr3v2_finger_joint1").id
    data.ctrl[model.actuator("left_fr3v2_finger_joint1").id] = 0.035
    block = model.body(block_name).id
    tcp = model.body("right_fr3v2_hand_tcp").id
    for step in range(round((times[-1] + 0.5) / model.opt.timestep)):
        time = step * model.opt.timestep
        # Only actuators drive the articulated robot after its initial state is set.
        for column, actuator in enumerate(actuators):
            data.ctrl[actuator] = np.interp(time, times, commands[:, column])
        data.ctrl[gripper] = 0.035 if time < 0.45 else 0.0
        mujoco.mj_step(model, data)
        assert np.isfinite(data.qpos).all(), (time, "nonfinite position")
        assert np.isfinite(data.qvel).all(), (time, "nonfinite velocity")
        if time > 6.0:
            relative = data.xmat[tcp].reshape(3, 3).T @ (
                data.xpos[block] - data.xpos[tcp]
            )
            assert abs(relative[2]) < 0.010, (
                time,
                "block slipped down out of pads",
                relative,
            )
            # A half-block-width lateral shift puts its center outside the initial grasp.
            assert np.linalg.norm(relative[:2]) < 0.025, (
                time,
                "block left the grasp",
                relative,
            )
    finger_contacts = set()
    for contact in data.contact[: data.ncon]:
        bodies = {model.body(int(model.geom_bodyid[g])).name for g in contact.geom}
        if block_name in bodies:
            finger_contacts.update(name for name in bodies if "finger" in name)
    assert finger_contacts, "block lost both finger contacts during the final hold"
    assert (
        data.xpos[block, 2] > 0.85
    ), "block fell instead of being carried above the table"
