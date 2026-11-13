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

"""Exercise Sort Blocks initialization and real waypoint/FK release targets offline."""

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
SPINE = "franka_spine_vertical_joint"


@pytest.fixture(scope="module")
def run_probe(tmp_path_factory):
    """Return a runner for the targets-only probe on the real robot model and shipped Objectives."""
    directory = tmp_path_factory.mktemp("placement_initialization")
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

    def run(name, cases):
        cases_path = directory / (name + "_cases.json")
        cases_path.write_text(json.dumps(cases))
        results_path = directory / (name + "_results.json")
        process = subprocess.run(
            [
                os.environ["BLOCK_TRANSFER_PROBE"],
                str(cases_path),
                str(PACKAGE / "objectives/plan_block_transfer.xml"),
                str(results_path),
                "--targets-only",
                str(PACKAGE / "objectives/sort_blocks.xml"),
                "--ros-args",
                "--params-file",
                str(params_path),
                "--log-level",
                "error",
            ],
            capture_output=True,
            text=True,
            timeout=25,
            check=False,
        )
        return process, results_path

    return run


@pytest.fixture(scope="module")
def release_targets(run_probe, shipped_transfers):
    """Run installed RetrieveWaypoint/FK and the shipped initialization, substituting motion only."""
    mujoco.mj_loadAllPluginLibraries(
        str(Path(get_package_prefix("mujoco_package_uri")) / "bin/mujoco_plugin")
    )
    model = mujoco.MjModel.from_xml_path(str(PACKAGE / "description/mujoco/scene.xml"))
    waypoints = {
        entry["name"]: entry
        for entry in yaml.safe_load((PACKAGE / "waypoints/waypoints.yaml").read_text())
    }

    def joints(entry):
        state = entry["joint_state"]
        return dict(zip(state["name"], state["position"]))

    def key_joints(name):
        key = model.key(name)
        return {
            model.joint(joint).name: float(key.qpos[model.jnt_qposadr[joint]])
            for joint in range(model.njnt)
            if int(model.jnt_type[joint])
            in (int(mujoco.mjtJoint.mjJNT_HINGE), int(mujoco.mjtJoint.mjJNT_SLIDE))
        }

    fresh = key_joints("default")
    # Also the state after a nav_demo reset and "Move Base to Ready", which stows the arms first.
    stowed = {
        **key_joints("nav_demo"),
        **joints(waypoints["Base Ready"]),
        **joints(waypoints["Stow"]),
    }
    home = joints(waypoints["Home"])
    cases = []
    for arm in ("right", "left"):
        transfer = shipped_transfers[arm]
        waypoint = waypoints[transfer["place_waypoint"]]
        for name, state, initialize in (
            ("fresh_uninitialized", fresh, False),
            ("stow_uninitialized", stowed, False),
            ("later_home", {**stowed, **home}, False),
            ("fresh_initialized", fresh, True),
            ("stow_initialized", stowed, True),
        ):
            cases.append(
                {
                    "name": arm + "_" + name,
                    "arm": arm,
                    "planning_group": transfer["planning_group"],
                    "hand_frame": transfer["hand_frame"],
                    "place_waypoint": transfer["place_waypoint"],
                    "corner_y": float(transfer["corner_y"]),
                    "joints": state,
                    "waypoint": joints(waypoint),
                    "waypoint_groups": waypoint["joint_group_names"],
                    "home": home,
                    "home_groups": waypoints["Home"]["joint_group_names"],
                    "initialize_sort": initialize,
                    "approach": {"xyz": [0.7, 0.0, 0.88], "wxyz": [1, 0, 0, 0]},
                }
            )
        counterfactual = copy.deepcopy(
            next(case for case in cases if case["name"] == arm + "_stow_uninitialized")
        )
        counterfactual["name"] = arm + "_spine_only_counterfactual"
        counterfactual["waypoint"].pop(SPINE)
        counterfactual["joints"][SPINE] = home[SPINE]
        cases.append(counterfactual)
    process, results_path = run_probe("release_targets", cases)
    assert process.returncode == 0, process.stdout + process.stderr
    results = {
        result["name"]: result for result in json.loads(results_path.read_text())
    }
    for result in results.values():
        assert result["status"] == "SUCCESS", result
    return {
        "results": results,
        "home": home[SPINE],
        "stow": stowed[SPINE],
        "template": next(case for case in cases if case["name"] == "right_later_home"),
    }


@pytest.mark.parametrize("arm", ["right", "left"])
def test_regression_first_release_after_stow_matches_later_placement(
    release_targets, arm
):
    """Fresh state masks the bug; initialization must remove the measured post-Stow height error."""
    results = release_targets["results"]
    home = release_targets["home"]
    stow = release_targets["stow"]
    first = results[arm + "_stow_initialized"]
    later = results[arm + "_later_home"]
    broken = results[arm + "_stow_uninitialized"]
    fresh = results[arm + "_fresh_uninitialized"]
    counterfactual = results[arm + "_spine_only_counterfactual"]
    assert SPINE not in broken["retrieved_joints"]
    assert stow > home
    np.testing.assert_allclose(
        np.array(broken["release_xyz"]) - later["release_xyz"],
        [0, 0, stow - home],
        atol=1e-8,
    )
    assert fresh["release_xyz"][2] == pytest.approx(later["release_xyz"][2], abs=1e-8)
    np.testing.assert_allclose(first["release_xyz"], later["release_xyz"], atol=1e-8)
    np.testing.assert_allclose(
        counterfactual["release_xyz"], later["release_xyz"], atol=1e-8
    )
    assert first["pre_transfer_spine"] == pytest.approx(home)
    assert results[arm + "_fresh_initialized"]["pre_transfer_spine"] == pytest.approx(
        home
    )


@pytest.mark.parametrize("arm", ["right", "left"])
def test_place_waypoint_groups_have_all_their_active_joints(release_targets, arm):
    """The real SRDF group model must not offer a Place waypoint for missing joints."""
    missing = release_targets["results"][arm + "_fresh_uninitialized"][
        "missing_group_joints"
    ]
    assert missing
    assert all(not joints for joints in missing.values()), missing


@pytest.mark.parametrize(
    "name, changes, message",
    [
        (
            "unknown_planning_group",
            {"planning_group": "no_such_group"},
            "Unknown group: no_such_group",
        ),
        (
            "unknown_waypoint_group",
            {"waypoint_groups": ["no_such_group"]},
            "Unknown group: no_such_group",
        ),
        (
            "unknown_hand_frame",
            {"hand_frame": "no_such_frame"},
            "Unknown hand frame: no_such_frame",
        ),
        (
            "unmoved_hand_frame",
            {"hand_frame": "left_fr3v2_hand_tcp"},
            "Planning group does not move hand frame: left_fr3v2_hand_tcp",
        ),
        ("missing_corner_y", {"corner_y": None}, "key 'corner_y' not found"),
        (
            "short_approach",
            {"approach": {"xyz": [0.7, 0.0], "wxyz": [1, 0, 0, 0]}},
            "approach needs 3 xyz and 4 wxyz numbers",
        ),
        (
            "text_approach",
            {"approach": {"xyz": [0.7, 0.0, "high"], "wxyz": [1, 0, 0, 0]}},
            "approach needs 3 xyz and 4 wxyz numbers",
        ),
        (
            "missing_home",
            {"initialize_sort": True, "home": None},
            "key 'home' not found",
        ),
        (
            "text_waypoint",
            {"waypoint": {SPINE: "high"}},
            "waypoint must map joint names to numbers",
        ),
    ],
)
def test_probe_rejects_malformed_cases(
    release_targets, run_probe, name, changes, message
):
    """A case naming an unknown group or frame, or with a missing or mistyped field, must fail cleanly."""
    case = copy.deepcopy(release_targets["template"])
    case["name"] = "malformed_" + name
    for field, value in changes.items():
        if value is None:
            del case[field]
        else:
            case[field] = value
    process, _ = run_probe(case["name"], [case])
    assert process.returncode == 3, process.stdout + process.stderr
    assert message in process.stderr
