# Copyright 2026 PickNik Inc.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
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

"""Plan the shipped transfer tree and check complete carried-block clearance offline."""

import copy
import json
import os
from pathlib import Path
import subprocess

from ament_index_python.packages import get_package_prefix, get_package_share_directory
import mujoco
import numpy as np
import pytest
import yaml

from moveit_studio_utils_py.create_urdf import create_xml_from_xacro
from moveit_studio_utils_py.system_config import load_system_config

PACKAGE = Path(get_package_share_directory("dual_arm_sim"))
STATES = json.loads(
    (Path(__file__).parent / "data/block_transfer_states.json").read_text()
)
# Bounds all cube rotations plus the observed within-grip displacement, not just the TCP.
GRIP_DRIFT_ALLOWANCE_M = 0.025


@pytest.fixture(scope="module")
def transfer_plans(tmp_path_factory, shipped_transfers):
    """Use current config/model sources and real planners without any live ROS service."""
    directory = tmp_path_factory.mktemp("block_transfer")
    mujoco.mj_loadAllPluginLibraries(
        str(Path(get_package_prefix("mujoco_package_uri")) / "bin/mujoco_plugin")
    )
    model = mujoco.MjModel.from_xml_path(str(PACKAGE / "description/mujoco/scene.xml"))
    waypoints = {
        waypoint["name"]: waypoint
        for waypoint in yaml.safe_load(
            (PACKAGE / "waypoints/waypoints.yaml").read_text()
        )
    }
    cases = copy.deepcopy(STATES)
    for case in cases:
        assert len(case["qpos"]) == model.nq
        case["joints"] = {
            model.joint(joint).name: case["qpos"][model.jnt_qposadr[joint]]
            for joint in range(model.njnt)
            if int(model.jnt_type[joint])
            in (int(mujoco.mjtJoint.mjJNT_HINGE), int(mujoco.mjtJoint.mjJNT_SLIDE))
        }
        transfer = shipped_transfers[case["arm"]]
        waypoint = waypoints[transfer["place_waypoint"]]
        case["planning_group"] = transfer["planning_group"]
        case["hand_frame"] = transfer["hand_frame"]
        case["place_waypoint"] = transfer["place_waypoint"]
        case["corner_y"] = float(transfer["corner_y"])
        case["waypoint_groups"] = waypoint["joint_group_names"]
        state = waypoint["joint_state"]
        case["waypoint"] = dict(zip(state["name"], state["position"]))
    unreachable = copy.deepcopy(cases[0])
    unreachable["name"] = "unreachable"
    unreachable["approach"]["xyz"][0] = 2.5
    cases.append(unreachable)
    config, _, _ = load_system_config(
        "dual_arm_sim", tmp_path_factory.mktemp("user_ws")
    )
    description = config.hardware.robot_description
    params = {
        "robot_description": create_xml_from_xacro(
            description.urdf, description.urdf_params
        ),
        "robot_description_semantic": create_xml_from_xacro(
            description.srdf, description.urdf_params
        ),
        **config.moveit["robot_description_kinematics"],
        **config.moveit["robot_description_planning"],
    }
    params_path = directory / "params.yaml"
    params_path.write_text(yaml.safe_dump({"/**": {"ros__parameters": params}}))
    cases_path = directory / "cases.json"
    cases_path.write_text(json.dumps(cases))
    result_path = directory / "results.json"
    process = subprocess.run(
        [
            os.environ["BLOCK_TRANSFER_PROBE"],
            str(cases_path),
            str(PACKAGE / "objectives/plan_block_transfer.xml"),
            str(result_path),
            "--ros-args",
            "--params-file",
            str(params_path),
            "--log-level",
            "error",
        ],
        capture_output=True,
        text=True,
        timeout=180,
        check=False,
    )
    assert process.returncode == 0, process.stdout + process.stderr
    return model, {row["name"]: row for row in json.loads(result_path.read_text())}


def test_unreachable_transfer_returns_no_executable_solution(transfer_plans) -> None:
    """Planning failure cannot leave a pickup or carry solution for execution."""
    _, plans = transfer_plans
    assert plans["unreachable"]["status"] == "FAILURE"
    assert "pickup" not in plans["unreachable"]
    assert "carry" not in plans["unreachable"]
    assert plans["unreachable"]["error"]


@pytest.mark.parametrize("case", STATES, ids=lambda case: case["name"])
def test_regression_complete_transfer_clears_blocks_and_splits_at_grasp(
    transfer_plans, shipped_transfers, case
) -> None:
    """Recorded near/far/cross-table pickups must plan, close at the block, and carry clear."""
    model, plans = transfer_plans
    plan = plans[case["name"]]
    assert plan["status"] == "SUCCESS", plan.get("error", plan)
    data = mujoco.MjData(model)
    data.qpos[:] = case["qpos"]
    mujoco.mj_forward(model, data)
    tcp = model.body(shipped_transfers[case["arm"]]["hand_frame"]).id
    block = model.body(case["block"]).id
    geom = int(model.body_geomadr[block])
    initial_center = data.geom_xpos[geom].copy()
    radius = np.linalg.norm(model.geom_size[geom]) + GRIP_DRIFT_ALLOWANCE_M
    obstacles = []
    for index in range(model.ngeom):
        name = model.body(int(model.geom_bodyid[index])).name
        if name == case["block"] or not name.startswith(("table_", "block_")):
            continue
        assert int(model.geom_type[index]) == int(mujoco.mjtGeom.mjGEOM_BOX)
        obstacles.append(
            (
                name,
                data.geom_xpos[index].copy(),
                data.geom_xmat[index].reshape(3, 3).copy(),
                model.geom_size[index].copy(),
            )
        )

    def at_point(stage, point):
        for name, value in zip(stage["joints"], point["q"]):
            data.qpos[model.joint(name).qposadr[0]] = value
        mujoco.mj_kinematics(model, data)

    pickup = [stage for stage in plan["pickup"] if stage["points"]]
    carry = [stage for stage in plan["carry"] if stage["points"]]
    assert len(pickup) == 2, "Pickup must end after descent, before lift."
    assert len(carry) == 3, "Carry must lift, clear the source, then reach release."
    assert pickup[-1]["joints"] == carry[0]["joints"]
    np.testing.assert_allclose(
        pickup[-1]["points"][-1]["q"], carry[0]["points"][0]["q"], atol=1e-6
    )
    at_point(pickup[-1], pickup[-1]["points"][-1])
    expected_grasp = np.array(case["approach"]["xyz"]) - [0, 0, 0.1]
    np.testing.assert_allclose(data.xpos[tcp], expected_grasp, atol=0.001)
    relative = data.xmat[tcp].reshape(3, 3).T @ (initial_center - data.xpos[tcp])
    at_point(carry[0], carry[0]["points"][-1])
    np.testing.assert_allclose(data.xpos[tcp], case["approach"]["xyz"], atol=0.001)
    minimum_clearance = float("inf")
    # Initial lift starts in intentional support contact; the full envelope applies after lift.
    for stage in carry[1:]:
        for point in stage["points"]:
            at_point(stage, point)
            center = data.xpos[tcp] + data.xmat[tcp].reshape(3, 3) @ relative
            for name, position, rotation, half_size in obstacles:
                outside = np.maximum(
                    np.abs(rotation.T @ (center - position)) - half_size, 0
                )
                clearance = float(np.linalg.norm(outside) - radius)
                minimum_clearance = min(minimum_clearance, clearance)
                assert (
                    clearance > 0
                ), f"{case['name']} carried envelope intersects {name}: {clearance:.6f}m"
    assert np.isfinite(minimum_clearance)
