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
"""Verify the simulation overlay through MoveIt Pro's real config loader."""
from pathlib import Path

import pytest
from moveit_studio_utils_py.system_config import load_system_config


@pytest.fixture(scope="module")
def loaded(tmp_path_factory):
    # user_ws only needs to exist; a src/ tree is optional.
    return load_system_config("rebot_bench_sim", tmp_path_factory.mktemp("user_ws"))


@pytest.fixture(scope="module")
def config(loaded):
    return loaded[0]


def test_inheritance_chain_is_base_then_overlay(loaded):
    assert loaded[2] == ["rebot_bench_base_config", "rebot_bench_sim"]


def test_sim_description_wraps_the_base_one_and_keeps_its_srdf(config):
    robot_description = config.hardware.robot_description
    assert robot_description.urdf.package == "rebot_bench_sim"
    assert robot_description.urdf.path == "description/rebot_bench_sim.urdf.xacro"
    assert robot_description.srdf.package == "rebot_bench_base_config"


def test_overlay_selects_mujoco_and_keeps_the_rest_of_urdf_params(config):
    params = {
        key: value
        for entry in config.hardware.robot_description.urdf_params
        for key, value in entry.items()
    }
    assert params["hardware_interface"] == "mujoco"
    assert params["mujoco_model"] == "description/scene.xml"
    assert params["mujoco_keyframe"] == "default"
    # A list of single-key dicts merges by key, so overriding one entry leaves
    # the base config's other xacro arguments inherited rather than dropped.
    # They are inert on MuJoCo, but a merge that replaced the list wholesale
    # would silently change what the base config passes.
    assert params["can_interface"] == "can0"
    assert params["torque_enable"] == "true"


def test_runtime_launch_file_comes_from_the_base_package(config):
    assert config.runtime_launch_file.package == "rebot_bench_base_config"


def test_objective_libraries_add_mujoco_to_the_base_ones(config):
    libraries = config.objectives.objective_library_paths
    assert libraries["rebot_objectives"].package_name == "rebot_bench_base_config"
    assert libraries["mujoco_objectives"].package_name == "moveit_pro_objectives"
    # MoveIt Pro saves new Objectives into the last library, which must be a
    # package in the user workspace.
    assert list(libraries)[-1] == "rebot_bench_sim_objectives"
    assert libraries["rebot_bench_sim_objectives"].package_name == "rebot_bench_sim"
    for library in libraries.values():
        assert library.share_path.is_dir()


def test_behavior_loaders_keep_the_base_ones_and_add_mujoco(config):
    core = list(config.objectives.behavior_loader_plugins["core"])
    assert core[-1] == "moveit_pro::behaviors::MujocoBehaviorsLoader"
    assert core[:-1] == [
        "moveit_pro::behaviors::CoreBehaviorsLoader",
        "moveit_pro::behaviors::MTCCoreBehaviorsLoader",
        "moveit_pro::behaviors::VisionBehaviorsLoader",
        "moveit_pro::behaviors::ConverterBehaviorsLoader",
    ]


def test_waypoints_come_from_the_base_package(config):
    assert config.objectives.waypoints_file.package_name == "rebot_bench_base_config"
    assert Path(config.waypoints_file_path).is_file()


def test_sim_replaces_camera_launch_with_empty_launch(config):
    import importlib.util
    from ament_index_python.packages import get_package_share_directory

    hook = config.hardware.additional_driver_launch_file
    assert hook.package == "moveit_studio_agent"
    spec = importlib.util.spec_from_file_location(
        "rebot_sim_empty_launch",
        str(Path(get_package_share_directory(hook.package)) / hook.path),
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    assert not module.generate_launch_description().entities
