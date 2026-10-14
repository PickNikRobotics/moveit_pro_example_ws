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
"""Fail loudly if the RL task spec drifts from what the Runtime runs.

The trainer reproduces the ExecutePolicy call in batched simulation from
config/rl_post_training.yaml, so that file must name the Runtime's MJCF and
agree with the Objective's ExecutePolicy ports; a mismatch trains a policy on
a task the robot never executes.
"""

import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import yaml
from ament_index_python.packages import get_package_share_directory
from moveit_studio_utils_py.system_config import load_system_config

SHARE = Path(get_package_share_directory("vla_rl_sim"))
OBJECTIVE = SHARE / "objectives" / "stack_red_on_green_with_the_rl_tuned_policy.xml"


@pytest.fixture(scope="module")
def loaded(tmp_path_factory):
    return load_system_config("vla_rl_sim", tmp_path_factory.mktemp("user_ws"))


@pytest.fixture(scope="module")
def config(loaded):
    return loaded[0]


@pytest.fixture(scope="module")
def spec():
    return yaml.safe_load((SHARE / "config" / "rl_post_training.yaml").read_text())


@pytest.fixture(scope="module")
def execute_policy():
    nodes = ET.parse(OBJECTIVE).getroot().findall(".//Action[@ID='ExecutePolicy']")
    assert len(nodes) == 1
    return nodes[0].attrib


def test_inheritance_chain_is_vla_sim_then_overlay(loaded):
    assert loaded[2] == ["vla_sim", "vla_rl_sim"]


def test_objective_libraries_keep_vla_sim_and_end_with_this_package(config):
    libraries = config.objectives.objective_library_paths
    assert list(libraries)[-1] == "rl_objectives"
    assert libraries["rl_objectives"].package_name == "vla_rl_sim"
    assert libraries["sim_objectives"].package_name == "vla_sim"


def test_spec_scene_is_the_runtime_mujoco_model(config, spec):
    params = {
        key: value
        for entry in config.hardware.robot_description.urdf_params
        for key, value in entry.items()
    }
    assert config.hardware.robot_description.urdf.package == spec["scene"]["package"]
    assert params["mujoco_model"] == spec["scene"]["path"]


def test_spec_matches_the_execute_policy_ports(spec, execute_policy):
    interface = spec["policy_interface"]
    assert execute_policy["prompt"] == interface["prompt"]
    assert float(execute_policy["dt"]) == interface["dt"]
    assert (
        int(execute_policy["committed_action_steps"])
        == interface["committed_action_steps"]
    )
    assert execute_policy["image_names"].split(";") == list(
        interface["cameras"].values()
    )
    topics = [f"/{camera}/color" for camera in interface["cameras"]]
    assert execute_policy["image_topics"].split(";") == topics
    assert execute_policy["gripper_joint_name"] == interface["gripper_joint"]
    assert int(execute_policy["total_action_steps"]) == spec["task"]["episode_steps"]


def test_serving_fps_matches_the_control_period(spec):
    serving = yaml.safe_load((SHARE / "config" / "vla_serving.yaml").read_text())
    assert serving["fps"] == pytest.approx(1.0 / spec["policy_interface"]["dt"])
