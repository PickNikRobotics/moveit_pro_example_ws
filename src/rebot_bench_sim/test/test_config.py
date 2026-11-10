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
"""Verify the standalone simulation through MoveIt Pro's real config loader."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from catkin_pkg.package import parse_package
from launch_ros.actions import Node
from moveit_studio_utils_py.create_urdf import create_xml_from_xacro
from moveit_studio_utils_py.system_config import load_system_config
import pytest
import yaml

SHARE = Path(get_package_share_directory("rebot_bench_sim"))


@pytest.fixture(scope="module")
def loaded(tmp_path_factory):
    return load_system_config("rebot_bench_sim", tmp_path_factory.mktemp("user_ws"))


@pytest.fixture(scope="module")
def config(loaded):
    return loaded[0]


def test_configuration_has_no_parent(loaded):
    assert loaded[2] == ["rebot_bench_sim"]


def test_description_and_semantics_are_local(config):
    description = config.hardware.robot_description
    assert description.urdf.package == "rebot_bench_sim"
    assert description.urdf.path == "description/rebot_bench_sim.urdf.xacro"
    assert description.srdf.package == "rebot_bench_sim"
    assert (SHARE / description.srdf.path).is_file()
    for resource in (
        config.runtime_launch_file,
        config.moveit_params.kinematics,
        config.moveit_params.joint_limits,
        config.moveit_params.pose_jog,
        config.moveit_params.joint_jog,
    ):
        assert resource.package == "rebot_bench_sim"
        assert (SHARE / resource.path).is_file()
    selector = config.ros2_control["config"]
    assert selector["package"] == "rebot_bench_sim"
    assert (SHARE / selector["path"]).is_file()


def test_only_simulation_arguments_reach_the_description(config):
    params = {
        key: value
        for entry in config.hardware.robot_description.urdf_params
        for key, value in entry.items()
    }
    assert params == {
        "hardware_interface": "mujoco",
        "mujoco_model": "description/scene.xml",
        "mujoco_keyframe": "default",
        "mujoco_viewer": False,
    }
    robot = ET.fromstring(
        create_xml_from_xacro(
            config.hardware.robot_description.urdf,
            config.hardware.robot_description.urdf_params,
        )
    )
    assert [p.text for p in robot.findall("ros2_control/hardware/plugin")] == [
        "picknik_mujoco_ros/MujocoSystem"
    ]
    assert not robot.findall("ros2_control/gpio")


def test_objective_libraries_resolve_with_the_sim_last(config):
    libraries = config.objectives.objective_library_paths
    assert set(libraries) == {
        "core_objectives",
        "motion_objectives",
        "perception_objectives",
        "visualization_objectives",
        "mujoco_objectives",
        "rebot_bench_sim_objectives",
    }
    assert list(libraries)[-1] == "rebot_bench_sim_objectives"
    assert libraries["rebot_bench_sim_objectives"].package_name == "rebot_bench_sim"
    for library in libraries.values():
        assert library.share_path.is_dir()


def test_stack_motion_loads_local_full_speed_settings(config):
    libraries = list(config.objectives.objective_library_paths.values())
    holders = [
        library
        for library in libraries
        if (library.share_path / "rebot_stack_motion.xml").is_file()
    ]
    assert [library.package_name for library in holders] == ["rebot_bench_sim"]
    loads = ET.parse(holders[0].share_path / "rebot_stack_motion.xml").findall(
        './/Action[@ID="LoadMotionScales"]'
    )
    assert {load.get("leg_kind") for load in loads} == {
        "joint",
        "cartesian",
        "gripper",
    }
    for load in loads:
        assert load.get("package_name") == "rebot_bench_sim"
        settings = yaml.safe_load((SHARE / load.get("file_path")).read_text())
        assert settings[load.get("leg_kind")] == {
            "velocity_scale_factor": 1.0,
            "acceleration_scale_factor": 1.0,
        }


def test_behavior_loaders_include_rebot_and_mujoco(config):
    assert list(config.objectives.behavior_loader_plugins["rebot"]) == [
        "rebot_bench_behaviors::RebotBenchBehaviorsLoader"
    ]
    assert list(config.objectives.behavior_loader_plugins["core"]) == [
        "moveit_pro::behaviors::CoreBehaviorsLoader",
        "moveit_pro::behaviors::MTCCoreBehaviorsLoader",
        "moveit_pro::behaviors::VisionBehaviorsLoader",
        "moveit_pro::behaviors::ConverterBehaviorsLoader",
        "moveit_pro::behaviors::MujocoBehaviorsLoader",
    ]


def test_waypoints_are_local(config):
    assert config.objectives.waypoints_file.package_name == "rebot_bench_sim"
    assert Path(config.waypoints_file_path).is_file()


@pytest.mark.parametrize("proxy", [False, True])
def test_sim_launch_starts_only_compressed_cameras_and_optional_proxy(
    config, monkeypatch, proxy
):
    hook = config.hardware.additional_driver_launch_file
    assert hook.package == "rebot_bench_sim"
    spec = importlib.util.spec_from_file_location("sim_drivers", SHARE / hook.path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    layout = yaml.safe_load((SHARE / "description/bench_layout.yaml").read_text())
    layout["sim_grasp_proxy"] = proxy
    monkeypatch.setattr(module, "yaml", SimpleNamespace(safe_load=lambda _: layout))
    entities = module.generate_launch_description().entities
    assert all(isinstance(entity, Node) for entity in entities)
    nodes = [(entity.node_package, entity.node_executable) for entity in entities]
    expected = [("rebot_bench_sim", "sim_compressed_cameras.py")]
    if proxy:
        expected += [
            ("controller_manager", "spawner"),
            ("rebot_bench_sim", "sim_grasp_proxy.py"),
        ]
    assert nodes == expected


def test_layout_places_the_robot_description(config):
    description = config.hardware.robot_description
    layout = yaml.safe_load((SHARE / "description/bench_layout.yaml").read_text())
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


def test_controller_configuration_is_complete_without_a_leader(config):
    controllers = yaml.safe_load(
        (SHARE / config.ros2_control["config"]["path"]).read_text()
    )
    declarations = controllers["controller_manager"]["ros__parameters"]
    expected = {
        "joint_state_broadcaster",
        "joint_trajectory_admittance_controller",
        "joint_velocity_controller",
        "gripper_joint_velocity_controller",
        "velocity_force_controller",
    }
    assert set(declarations) - {"update_rate"} == expected
    assert (
        set(config.ros2_control["controllers_active_at_startup"])
        | set(config.ros2_control["controllers_inactive_at_startup"])
    ) == expected
    assert all(name in controllers for name in expected)
    assert not any(
        name.startswith("rebot_leader_") for name in config.ros_global_params
    )


def test_local_package_dependencies_are_driver_free():
    # package.xml is the dependency contract consumed by colcon and rosdep.
    forbidden = {"realsense2_camera", "depthai_ros_driver", "position_controllers"}
    for package in ("rebot_bench_sim", "rebot_bench_behaviors"):
        manifest = parse_package(get_package_share_directory(package))
        dependencies = {
            dep.name
            for field in (
                "build_depends",
                "buildtool_depends",
                "build_export_depends",
                "exec_depends",
                "test_depends",
            )
            for dep in getattr(manifest, field)
        }
        assert not dependencies & forbidden
    assert "rebot_bench_behaviors" in {
        dep.name for dep in parse_package(str(SHARE)).build_depends
    }
