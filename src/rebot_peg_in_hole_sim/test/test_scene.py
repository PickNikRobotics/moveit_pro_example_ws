#!/usr/bin/env python3
# Copyright 2026 PickNik Inc. All rights reserved. BSD-3-Clause.
"""Check the installed CAD scene, native camera frames and robot FK contract."""
import hashlib
import json
import math
from pathlib import Path
import struct
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_prefix, get_package_share_directory
import mujoco
from moveit_studio_utils_py.system_config import load_system_config
import numpy as np
import pytest

SHARE = Path(get_package_share_directory("rebot_peg_in_hole_sim"))
SNAPSHOT = json.loads((SHARE / "config/scene_snapshot.json").read_text())
ROBOT = ET.parse(SHARE / "description/rebot_peg.urdf").getroot()
CENTERS = ((0.076362, -0.079826), (0.1413005, -0.082661385))


@pytest.fixture(scope="module")
def model():
    mujoco.mj_loadAllPluginLibraries(
        str(Path(get_package_prefix("mujoco_package_uri")) / "bin/mujoco_plugin")
    )
    return mujoco.MjModel.from_xml_path(str(SHARE / "description/scene.xml"))


def state(model, positions=None):
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, model.key("default").id)
    for name, value in (positions or {}).items():
        data.joint(name).qpos = value
    for jaw in ("gripper_joint1", "gripper_joint2"):
        data.joint(jaw).qpos = 0.008 * (positions or {}).get("joint7", 0)
    mujoco.mj_forward(model, data)
    return data


def test_keyframe_and_static_fixture_scope(model):
    assert model.nq == 9
    assert model.nu == 7
    bench = mujoco.MjModel.from_xml_path(
        get_package_share_directory("rebot_bench_sim") + "/description/scene.xml"
    )
    rest = bench.key("default")
    np.testing.assert_array_equal(model.key("default").qpos, rest.qpos[: model.nq])
    np.testing.assert_array_equal(model.key("default").ctrl, rest.ctrl)
    for body in ("peg_jig", "peg_1", "peg_2", "hole_bracket", "charuco_board"):
        assert model.body(body).jntnum[0] == 0
        geom = model.geom(model.body(body).geomadr[0])
        assert geom.contype[0] == geom.conaffinity[0] == 0


@pytest.mark.parametrize(
    "name,key,local",
    [
        ("jig_origin", "world_T_jig", (0, 0, 0)),
        ("bracket_origin", "world_T_bracket", (0, 0, 0)),
        ("peg_1_tip", "world_T_jig", (*CENTERS[0], 0.085)),
        ("peg_2_tip", "world_T_jig", (*CENTERS[1], 0.085)),
        ("hole_1_center", "world_T_bracket", (*CENTERS[0], 0.00762)),
        ("hole_2_center", "world_T_bracket", (*CENTERS[1], 0.00762)),
        ("charuco_board_origin", "world_T_board", (0, 0, 0)),
    ],
)
def test_simulator_own_fixture_truth(model, name, key, local):
    expected = np.asarray(SNAPSHOT[key]) @ [*local, 1]
    np.testing.assert_allclose(state(model).site(name).xpos, expected[:3], atol=1e-9)


@pytest.mark.parametrize(
    "camera,key", [("scene_camera_color", "scene"), ("d435_color", "wrist")]
)
def test_camera_pose_and_native_intrinsics(model, camera, key):
    data = state(model, SNAPSHOT["wrist_observation_joints"])
    cam = data.camera(camera)
    optical = data.site(camera + "_optical_frame")
    pose = np.asarray(SNAPSHOT[f"world_T_{key}_optical"])
    np.testing.assert_allclose(optical.xpos, pose[:3, 3], atol=1e-8)
    np.testing.assert_allclose(optical.xmat.reshape(3, 3), pose[:3, :3], atol=1e-8)
    np.testing.assert_allclose(optical.xpos, cam.xpos, atol=1e-9)
    np.testing.assert_allclose(
        optical.xmat.reshape(3, 3),
        cam.xmat.reshape(3, 3) @ np.diag([1, -1, -1]),
        atol=1e-8,
    )
    assert model.cam_mode[cam.id] == mujoco.mjtCamLight.mjCAMLIGHT_FIXED
    assert tuple(model.cam_resolution[cam.id]) == (640, 480)
    fy = 240 / math.tan(math.radians(model.cam_fovy[cam.id] / 2))
    assert fy == pytest.approx(SNAPSHOT["intrinsics"][key + "_color"][4])
    assert (model.vis.global_.offwidth, model.vis.global_.offheight) == (640, 480)
    assert ROBOT.find(f"link[@name='{camera}_optical_frame']") is None


def link_poses(positions):
    poses = {"world": np.eye(4)}
    pending = list(ROBOT.findall("joint"))
    while pending:
        progressed = False
        for joint in pending[:]:
            parent = joint.find("parent").get("link")
            if parent not in poses:
                continue
            origin = joint.find("origin")
            offset = np.eye(4)
            if origin is not None:
                offset[:3, 3] = list(map(float, origin.get("xyz", "0 0 0").split()))
                angles = np.array(list(map(float, origin.get("rpy", "0 0 0").split())))
                quat, rotation = np.zeros(4), np.zeros(9)
                mujoco.mju_euler2Quat(quat, angles, "XYZ")
                mujoco.mju_quat2Mat(rotation, quat)
                offset[:3, :3] = rotation.reshape(3, 3)
            motion = np.eye(4)
            axis = joint.find("axis")
            q = positions.get(joint.get("name"), 0)
            if axis is not None:
                vector = np.array(list(map(float, axis.get("xyz").split())))
                if joint.get("type") in ("revolute", "continuous"):
                    quat, rotation = np.zeros(4), np.zeros(9)
                    mujoco.mju_axisAngle2Quat(quat, vector, q)
                    mujoco.mju_quat2Mat(rotation, quat)
                    motion[:3, :3] = rotation.reshape(3, 3)
                elif joint.get("type") == "prismatic":
                    motion[:3, 3] = vector * q
            poses[joint.find("child").get("link")] = poses[parent] @ offset @ motion
            pending.remove(joint)
            progressed = True
        assert progressed, "URDF has a disconnected or cyclic link graph"
    return poses


@pytest.mark.parametrize(
    "positions",
    [
        {},
        SNAPSHOT["wrist_observation_joints"],
        dict(zip((f"joint{i}" for i in range(1, 8)), [0.7, 1.4, 2, -0.8, 1.1, -2, 2])),
    ],
)
def test_every_robot_body_agrees_with_urdf_at_multiple_joint_positions(
    model, positions
):
    positions = dict(positions)
    for jaw in ("gripper_joint1", "gripper_joint2"):
        positions[jaw] = 0.008 * positions.get("joint7", 0)
    data = state(model, positions)
    checked = 0
    for name, pose in link_poses(positions).items():
        if mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name) < 0:
            continue
        body = data.body(name)
        np.testing.assert_allclose(body.xpos, pose[:3, 3], atol=2e-4, err_msg=name)
        np.testing.assert_allclose(
            body.xmat.reshape(3, 3), pose[:3, :3], atol=5e-4, err_msg=name
        )
        checked += 1
    assert checked >= 15


@pytest.mark.parametrize(
    "name,faces", [("part", 21508), ("jig_base", 97376), ("jig_post", 5242)]
)
def test_frozen_mesh_bytes_and_native_bounds(name, faces):
    metadata = json.loads((SHARE / "description/assets/provenance.json").read_text())[
        name
    ]
    raw = (SHARE / f"description/assets/{name}.stl").read_bytes()
    assert hashlib.sha256(raw).hexdigest() == metadata["stl_sha256"]
    assert struct.unpack_from("<I", raw, 80)[0] == faces == metadata["faces"]
    assert len(raw) == 84 + 50 * faces
    triangles = np.ndarray(
        (faces, 12), dtype="<f4", buffer=raw, offset=84, strides=(50, 4)
    )
    vertices = triangles[:, 3:].reshape(-1, 3)
    np.testing.assert_allclose(
        [vertices.min(0), vertices.max(0)], metadata["bounds_m"], atol=1e-8
    )
    assert metadata["unit"] == "meter"


def test_loader_selects_only_mujoco_and_child_objective_save_directory(tmp_path):
    config, _, chain = load_system_config("rebot_peg_in_hole_sim", tmp_path)
    assert chain == [
        "rebot_bench_base_config",
        "rebot_bench_sim",
        "rebot_peg_in_hole_sim",
    ]
    assert config.hardware.robot_description.urdf.package == "rebot_peg_in_hole_sim"
    assert config.hardware.robot_description.srdf.package == "rebot_bench_base_config"
    libraries = config.objectives.objective_library_paths
    assert list(libraries)[-1] == "rebot_peg_in_hole_objectives"
    assert libraries[list(libraries)[-1]].package_name == "rebot_peg_in_hole_sim"
    assert [p.text for p in ROBOT.findall("ros2_control/hardware/plugin")] == [
        "picknik_mujoco_ros/MujocoSystem"
    ]
    sensor = ROBOT.find("ros2_control/sensor[@name='d435_color']")
    assert sensor.find("param[@name='depth_topic']").text == (
        "/wrist_mounted_camera/aligned_depth_to_color/image_raw"
    )


def test_grid_pitch_is_in_metres(model):
    grid = model.material("mat_grid")
    assert grid.texuniform[0]
    np.testing.assert_allclose(grid.texrepeat, 1 / 0.0127, rtol=1e-6)
