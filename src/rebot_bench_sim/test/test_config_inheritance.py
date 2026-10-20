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


def test_sim_shadows_the_stack_speeds_at_full_speed(config):
    import xml.etree.ElementTree as ET

    import yaml

    # The Objective library a later entry provides wins a filename collision, so
    # the sim's reBot Stack Motion replaces the base config's.
    libraries = list(config.objectives.objective_library_paths.values())
    holders = [
        library
        for library in libraries
        if (library.share_path / "rebot_stack_motion.xml").is_file()
    ]
    assert [library.package_name for library in holders] == [
        "rebot_bench_base_config",
        "rebot_bench_sim",
    ]
    winner = [library.share_path / "rebot_stack_motion.xml" for library in holders]

    def factors(xml_path):
        loads = ET.parse(xml_path).findall('.//Action[@ID="LoadMotionScales"]')
        assert {load.get("leg_kind") for load in loads} == {
            "joint",
            "cartesian",
            "gripper",
        }
        (package,) = {load.get("package_name") for load in loads}
        (file_path,) = {load.get("file_path") for load in loads}
        share = next(
            library.share_path.parent
            for library in libraries
            if library.package_name == package
        )
        return package, yaml.safe_load((share / file_path).read_text())

    package, sim = factors(winner[-1])
    assert package == "rebot_bench_sim"
    assert all(value == 1.0 for leg in sim.values() for value in leg.values())
    # The base config's own settings stay in its package.
    package, real = factors(winner[0])
    assert package == "rebot_bench_base_config"
    assert set(real) == {"joint", "cartesian", "gripper"}


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


def test_sim_launches_compressed_cameras_and_only_the_explicit_grasp_proxy(config):
    import importlib.util
    from ament_index_python.packages import get_package_share_directory

    hook = config.hardware.additional_driver_launch_file
    assert hook.package == "rebot_bench_sim"
    spec = importlib.util.spec_from_file_location(
        "rebot_sim_empty_launch",
        str(Path(get_package_share_directory(hook.package)) / hook.path),
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    import yaml

    layout = yaml.safe_load(
        (
            Path(get_package_share_directory(hook.package))
            / "description/bench_layout.yaml"
        ).read_text()
    )
    entities = module.generate_launch_description().entities
    # The compressed camera topics, then the proxy controller and its node.
    assert len(entities) == (3 if layout.get("sim_grasp_proxy", False) else 1)


def test_layout_places_the_twin_over_the_inherited_bench_calibration(config):
    import xml.etree.ElementTree as ET

    import yaml
    from ament_index_python.packages import get_package_share_directory
    from moveit_studio_utils_py.create_urdf import create_xml_from_xacro

    description = config.hardware.robot_description
    params = {k: v for entry in description.urdf_params for k, v in entry.items()}
    # The base config's real-bench placements reach the sim's xacro mappings...
    assert {"world_to_base_xyz", "scene_camera_xyz"} <= params.keys()
    layout = yaml.safe_load(
        (
            Path(get_package_share_directory("rebot_bench_sim"))
            / "description/bench_layout.yaml"
        ).read_text()
    )
    # ...but MoveIt must see the frames where MuJoCo puts them, from the layout.
    robot = ET.fromstring(
        create_xml_from_xacro(description.urdf, description.urdf_params)
    )
    for joint, entry in (
        ("world_to_base", "robot_base"),
        ("scene_camera_mount_joint", "scene_camera"),
    ):
        origin = robot.find(f"joint[@name='{joint}']/origin")
        for key in ("xyz", "rpy"):
            assert [float(v) for v in origin.get(key).split()] == pytest.approx(
                layout[entry][key]
            )
