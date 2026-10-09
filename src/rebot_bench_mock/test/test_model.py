# Copyright 2026 PickNik Inc.
# SPDX-License-Identifier: BSD-3-Clause
"""Exercise the installed description through MoveIt Pro's model and FCL consumers."""
from math import radians
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from moveit_pro_base import planning_scene, robot_model, robot_state
import numpy as np
import pytest
import yaml


@pytest.fixture(scope="module")
def model():
    share = Path(get_package_share_directory("rebot_bench_mock"))
    return robot_model.RobotModel(
        str(share / "description/rebot.urdf"),
        str(share / "config/moveit/rebot.srdf"),
    )


def test_measured_motor_limits(model):
    low, high = model.get_group_position_bounds("manipulator")
    assert np.allclose(low, np.radians([-145, 0, 0, -106, -101, -175]))
    assert np.allclose(high, np.radians([145, 203, 239, 86, 108, 175]))
    # The gripper group has two dependent sliders and one commanded motor coordinate.
    state = robot_state.RobotState(model)
    state.set_to_default_values()
    state.joint_positions = {"joint7": radians(310)}
    state.update()
    for joint in ["gripper_joint1", "gripper_joint2"]:
        assert state.joint_positions[joint] == pytest.approx(0.008 * radians(310))
    low, high = model.get_group_position_bounds("gripper")
    assert high[-1] == pytest.approx(radians(310))
    # The planner rejects even sub-picoradian waypoint overshoots.
    assert high[-1] >= radians(310)
    assert low[-1] == 0
    assert model.get_link_model("d435_color_optical_frame") is not None


def test_saved_waypoints_and_interpolated_moves_are_collision_free(model):
    share = Path(get_package_share_directory("rebot_bench_mock"))
    waypoints = yaml.safe_load((share / "waypoints/rebot_waypoints.yaml").read_text())
    poses = {
        w["name"]: dict(zip(w["joint_state"]["name"], w["joint_state"]["position"]))
        for w in waypoints
    }
    scene = planning_scene.PlanningScene(model)
    # Keep the real unpadded meshes checked despite their padding exclusions.
    scene.allowed_collision_matrix.set_entry("link5", "gripper_end", False)
    scene.allowed_collision_matrix.set_entry("gripper_left", "gripper_right", False)
    state = robot_state.RobotState(model)
    state.set_to_default_values()
    for start, goal in [("Zeroed Rest", "Raised"), ("Raised", "Gripper Open")]:
        for fraction in np.linspace(0, 1, 51):
            state.joint_positions = {
                joint: value + fraction * (poses[goal][joint] - value)
                for joint, value in poses[start].items()
            }
            state.update()
            assert scene.is_state_valid(state, "", False), (start, goal, fraction)
    # The named pose actually raises the TCP, rather than just changing joint values.
    state.set_to_default_values("manipulator", "zeroed_rest")
    state.update()
    rest_z = state.get_global_link_transform("gripper_end")[2, 3]
    state.set_to_default_values("manipulator", "raised")
    state.update()
    assert state.get_global_link_transform("gripper_end")[2, 3] > rest_z + 0.2


def test_wrist_mount_meshes_stay_separated_across_roll(model):
    """The extra adjacent-pair exclusion cannot hide a physical mesh collision.

    Gripper_end is fixed to link6. J6 is their only relative motion with link5;
    its Z-axis rotation preserves the axial separating plane measured here.
    """
    share = Path(get_package_share_directory("rebot_bench_mock"))
    triangle = np.dtype(
        [("normal", "<f4", 3), ("vertices", "<f4", (3, 3)), ("attribute", "<u2")]
    )
    meshes = {}
    for name in ["link5", "gripper_end"]:
        path = share / "description/assets/shared" / f"{name}.STL"
        meshes[name] = np.fromfile(path, dtype=triangle, offset=84)["vertices"].reshape(
            -1, 3
        )
    state = robot_state.RobotState(model)
    state.set_to_default_values()
    for roll in np.radians([-175, 0, 175]):
        state.joint_positions = {"joint6": roll}
        state.update()
        wrist_from_world = np.linalg.inv(state.get_global_link_transform("link6"))
        extents = {}
        for name, vertices in meshes.items():
            transform = wrist_from_world @ state.get_global_link_transform(name)
            transformed = vertices @ transform[:3, :3].T + transform[:3, 3]
            extents[name] = (transformed[:, 2].min(), transformed[:, 2].max())
        gap = extents["gripper_end"][0] - extents["link5"][1]
        assert 0.0089 < gap < 0.0091
