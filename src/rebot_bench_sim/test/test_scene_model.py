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
"""Check the generated MuJoCo twin against the robot description and the bench."""
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np
import pytest
import xacro
import yaml
from ament_index_python.packages import (
    get_package_prefix,
    get_package_share_directory,
)

SHARE = Path(get_package_share_directory("rebot_bench_sim"))
LAYOUT = yaml.safe_load((SHARE / "description/bench_layout.yaml").read_text())


@pytest.fixture(scope="module")
def model():
    # MoveIt Pro loads the same provider, which resolves package:// mesh URIs.
    mujoco.mj_loadAllPluginLibraries(
        str(Path(get_package_prefix("mujoco_package_uri")) / "bin/mujoco_plugin")
    )
    return mujoco.MjModel.from_xml_path(str(SHARE / "description/scene.xml"))


@pytest.fixture
def data(model):
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, model.key("default").id)
    mujoco.mj_forward(model, data)
    return data


def expand(path, **mappings):
    return ET.fromstring(xacro.process_file(str(path), mappings=mappings).toxml())


def sim_urdf(**mappings):
    return expand(SHARE / "description/rebot_bench_sim.urdf.xacro", **mappings)


def rpy_matrix(roll, pitch, yaw):
    cr, sr, cp, sp = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ]
    )


def urdf_link_poses(robot, positions):
    """World pose of every URDF link, from the joint origins, axes and positions."""
    poses = {"world": np.eye(4)}
    joints = robot.findall("joint")
    while len(poses) <= len(joints):
        for joint in joints:
            parent = joint.find("parent").get("link")
            child = joint.find("child").get("link")
            if parent not in poses or child in poses:
                continue
            origin = joint.find("origin")
            xyz = [
                float(v)
                for v in (
                    origin.get("xyz", "0 0 0") if origin is not None else "0 0 0"
                ).split()
            ]
            rpy = [
                float(v)
                for v in (
                    origin.get("rpy", "0 0 0") if origin is not None else "0 0 0"
                ).split()
            ]
            offset = np.eye(4)
            offset[:3, :3] = rpy_matrix(*rpy)
            offset[:3, 3] = xyz
            motion = np.eye(4)
            axis_element = joint.find("axis")
            axis = (
                np.array([float(v) for v in axis_element.get("xyz").split()])
                if axis_element is not None
                else None
            )
            q = positions.get(joint.get("name"), 0.0)
            if joint.get("type") == "revolute":
                motion[:3, :3] = rotation_about(axis, q)
            elif joint.get("type") == "prismatic":
                motion[:3, 3] = axis * q
            poses[child] = poses[parent] @ offset @ motion
    return poses


def rotation_about(axis, angle):
    x, y, z = axis / np.linalg.norm(axis)
    k = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + math.sin(angle) * k + (1 - math.cos(angle)) * k @ k


ARM = [f"joint{i}" for i in range(1, 7)]
CONFIGURATIONS = [
    [0.0] * 6 + [0.0],
    [0.0, 0.65, 0.85, 0.3, 0.0, 0.0, 5.41],  # Raised, gripper open
    [0.7, 1.4, 2.0, -0.8, 1.1, -2.0, 2.0],
]


@pytest.mark.parametrize("configuration", CONFIGURATIONS)
def test_every_mujoco_body_and_camera_site_sits_on_its_urdf_link(model, configuration):
    data = mujoco.MjData(model)
    for name, value in zip(ARM + ["joint7"], configuration):
        data.joint(name).qpos = value
    for jaw in ["gripper_joint1", "gripper_joint2"]:
        data.joint(jaw).qpos = 0.008 * configuration[-1]
    mujoco.mj_kinematics(model, data)
    positions = dict(zip(ARM + ["joint7"], configuration))
    positions["gripper_joint1"] = positions["gripper_joint2"] = (
        0.008 * configuration[-1]
    )

    # The simulated description, and the camera optical frames it leaves to
    # MuJoCo, taken from the base description with the same placements.
    sim = urdf_link_poses(sim_urdf(), positions)
    args = {
        "world_to_base_xyz": " ".join(map(str, LAYOUT["robot_base"]["xyz"])),
        "world_to_base_rpy": " ".join(map(str, LAYOUT["robot_base"]["rpy"])),
        "scene_camera_xyz": " ".join(map(str, LAYOUT["scene_camera"]["xyz"])),
        "scene_camera_rpy": " ".join(map(str, LAYOUT["scene_camera"]["rpy"])),
        "wrist_camera_mount_xyz": " ".join(
            map(str, LAYOUT["wrist_camera_mount"]["xyz"])
        ),
        "wrist_camera_mount_rpy": " ".join(
            map(str, LAYOUT["wrist_camera_mount"]["rpy"])
        ),
    }
    base = urdf_link_poses(
        expand(
            Path(get_package_share_directory("rebot_bench_base_config"))
            / "description/rebot.urdf.xacro",
            **args,
        ),
        positions,
    )
    checked = 0
    for link, pose in sim.items():
        if (
            link == "world"
            or mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, link) < 0
        ):
            continue
        body = data.body(link)
        np.testing.assert_allclose(body.xpos, pose[:3, 3], atol=2e-4, err_msg=link)
        np.testing.assert_allclose(
            body.xmat.reshape(3, 3), pose[:3, :3], atol=5e-4, err_msg=link
        )
        checked += 1
    assert checked == 15
    for frame in ["d435_color_optical_frame", "scene_camera_color_optical_frame"]:
        assert frame not in sim, "MuJoCo publishes this frame; the URDF must not"
        site = data.site(frame)
        np.testing.assert_allclose(
            site.xpos, base[frame][:3, 3], atol=2e-4, err_msg=frame
        )
        np.testing.assert_allclose(
            site.xmat.reshape(3, 3), base[frame][:3, :3], atol=5e-4, err_msg=frame
        )


@pytest.mark.parametrize(
    ("camera", "vertical_focal_px", "topic"),
    [
        # docs/camera-acceptance.json in the base config: D435 colour fy at 640x480.
        ("d435_color", 607.8, "/wrist_mounted_camera/color/image_raw"),
        # 97.1 deg across 640 px: fx = 282.5 px for the OAK-D Pro W at 640 wide.
        (
            "scene_camera_color",
            320 / math.tan(math.radians(97.1 / 2)),
            "/scene_camera/color/image_raw",
        ),
    ],
)
def test_cameras_match_the_real_intrinsics_frames_and_topics(
    model, data, camera, vertical_focal_px, topic
):
    cam = data.camera(camera)
    site = data.site(f"{camera}_optical_frame")
    # The ROS optical frame is the MuJoCo camera turned 180 deg about its x.
    np.testing.assert_allclose(site.xpos, cam.xpos, atol=1e-9)
    np.testing.assert_allclose(
        site.xmat.reshape(3, 3),
        cam.xmat.reshape(3, 3) @ np.diag([1, -1, -1]),
        atol=1e-6,
    )
    assert (
        model.cam_mode[model.camera(camera).id] == mujoco.mjtCamLight.mjCAMLIGHT_FIXED
    )
    # Every rendered camera shares the one offscreen buffer.
    assert tuple(model.cam_resolution[model.camera(camera).id]) == (640, 480)
    assert (model.vis.global_.offwidth, model.vis.global_.offheight) == (640, 480)
    focal = 240 / math.tan(math.radians(model.cam_fovy[model.camera(camera).id] / 2))
    assert focal == pytest.approx(vertical_focal_px, rel=2e-3)
    sensors = {s.get("name"): s for s in sim_urdf().findall("ros2_control/sensor")}
    params = {p.get("name"): p.text for p in sensors[camera].findall("param")}
    assert params["rgb_topic"] == topic


def test_cubes_are_50_mm_and_settle_on_the_mat_where_the_layout_puts_them(model, data):
    for _ in range(int(1.5 / model.opt.timestep)):
        mujoco.mj_step(model, data)
    for colour in ["red", "blue"]:
        geom = model.geom(model.body(f"{colour}_cube").geomadr[0])
        np.testing.assert_allclose(2 * geom.size, [0.050] * 3)
        start = np.array(LAYOUT["cubes"][colour]["xyz"])
        resting = data.body(f"{colour}_cube").xpos
        np.testing.assert_allclose(resting[:2], start[:2], atol=1e-3)
        # Mat top is the world z = 0 plane.
        assert resting[2] == pytest.approx(0.025, abs=1e-3)
        geom_id = model.body(f"{colour}_cube").geomadr[0]
        touching = {
            model.body(
                model.geom_bodyid[c.geom2 if c.geom1 == geom_id else c.geom1]
            ).name
            for c in data.contact[: data.ncon]
            if geom_id in (c.geom1, c.geom2)
        }
        assert touching == {"mat"}


def test_arm_holds_the_keyframe_pose_without_sagging(model, data):
    # A sag of about 0.04 rad on joint4 in the folded rest pose is enough for
    # MoveIt to report link2 and link5 in collision and refuse to plan, and a
    # joint resting a hair below its 0 rad lower limit fails trajectory
    # generation from the current state. Uncompensated mass (a visual geom
    # that is not massless) shows up here as a steady offset.
    for _ in range(int(1.5 / model.opt.timestep)):
        mujoco.mj_step(model, data)
    held = [data.joint(name).qpos[0] for name in ARM + ["joint7"]]
    np.testing.assert_allclose(
        held, LAYOUT["keyframe"]["arm"] + [LAYOUT["keyframe"]["gripper"]], atol=1e-5
    )


def test_axes_triad_marks_the_world_origin(model, data):
    triad = model.body("tf_frame")
    np.testing.assert_allclose(data.body("tf_frame").xpos, 0)
    tips = {}
    for geom_id in range(triad.geomadr[0], triad.geomadr[0] + triad.geomnum[0]):
        direction = data.geom_xmat[geom_id].reshape(3, 3)[:, 2]
        tips[tuple(np.round(model.geom_rgba[geom_id][:3]))] = tuple(
            np.abs(np.round(direction))
        )
    assert tips == {(1, 0, 0): (1, 0, 0), (0, 1, 0): (0, 1, 0), (0, 0, 1): (0, 0, 1)}


def test_keyframe_follows_the_layout(model):
    key = model.key("default")
    assert key.qpos.size == model.nq
    arm = LAYOUT["keyframe"]["arm"] + [LAYOUT["keyframe"]["gripper"]]
    np.testing.assert_allclose(key.qpos[:7], arm)
    np.testing.assert_allclose(key.ctrl, arm)


def test_wrist_camera_sits_at_the_layouts_measured_mount(data):
    def pose(body):
        pose = np.eye(4)
        pose[:3, :3] = data.body(body).xmat.reshape(3, 3)
        pose[:3, 3] = data.body(body).xpos
        return pose

    camera_in_link6 = np.linalg.inv(pose("link6")) @ pose("d435_link")
    measured = LAYOUT["wrist_camera"]
    np.testing.assert_allclose(camera_in_link6[:3, 3], measured["xyz"], atol=1e-5)
    np.testing.assert_allclose(
        camera_in_link6[:3, :3], rpy_matrix(*measured["rpy"]), atol=1e-5
    )


def test_sim_description_refuses_any_other_hardware():
    with pytest.raises(xacro.XacroException):
        sim_urdf(hardware_interface="mock")
    plugins = [p.text for p in sim_urdf().findall("ros2_control/hardware/plugin")]
    assert plugins == ["picknik_mujoco_ros/MujocoSystem"]


@pytest.mark.parametrize("aperture, touches", [(0.084, False), (0.049, True)])
def test_finger_collision_geometry_preserves_the_grasping_aperture(
    model, data, aperture, touches
):
    # Place a cube between the fingertips, away from the bench. A hull spanning
    # the carriage and the finger incorrectly touches it even with the jaws open.
    for name, value in zip(ARM, CONFIGURATIONS[1][:6]):
        data.joint(name).qpos = value
    data.joint("joint7").qpos = aperture / 0.016
    for jaw in ("gripper_joint1", "gripper_joint2"):
        data.joint(jaw).qpos = aperture / 2
    mujoco.mj_kinematics(model, data)
    tip = data.body("gripper_end")
    cube = model.body("blue_cube")
    address = model.jnt_qposadr[cube.jntadr[0]]
    data.qpos[address : address + 3] = tip.xpos + tip.xmat.reshape(3, 3) @ [-0.01, 0, 0]
    data.qpos[address + 3 : address + 7] = tip.xquat
    mujoco.mj_forward(model, data)
    touching = set()
    for contact in data.contact[: data.ncon]:
        bodies = [model.body(model.geom_bodyid[g]).name for g in contact.geom]
        if "blue_cube" in bodies:
            touching.update(set(bodies) & {"gripper_left", "gripper_right"})
    assert touching == ({"gripper_left", "gripper_right"} if touches else set())


def _top_down_grasp_ik(model, target, seed):
    """Solve the six joint model locally, without a running ROS stack."""
    state = mujoco.MjData(model)
    joints = np.array(seed, dtype=float)
    rotation = np.array([[0, 0, -1], [0, -1, 0], [-1, 0, 0]])
    jac_position = np.zeros((3, model.nv))
    jac_rotation = np.zeros((3, model.nv))
    body_id = model.body("gripper_end").id
    for _ in range(200):
        state.qpos[:6] = joints
        mujoco.mj_forward(model, state)
        tip = state.body("gripper_end")
        current = tip.xmat.reshape(3, 3)
        error = np.r_[
            target - tip.xpos,
            0.5 * sum(np.cross(current[:, i], rotation[:, i]) for i in range(3)),
        ]
        if np.linalg.norm(error) < 1e-7:
            return joints
        mujoco.mj_jacBody(model, state, jac_position, jac_rotation, body_id)
        jacobian = np.vstack([jac_position[:, :6], jac_rotation[:, :6]])
        delta = jacobian.T @ np.linalg.solve(
            jacobian @ jacobian.T + 1e-6 * np.eye(6), error
        )
        joints = np.clip(
            joints + np.clip(delta, -0.15, 0.15),
            model.jnt_range[:6, 0] + 1e-5,
            model.jnt_range[:6, 1] - 1e-5,
        )
    raise RuntimeError(f"Grasp fixture IK failed: {error}")


@pytest.mark.xfail(
    strict=True,
    raises=AssertionError,
    reason="Physical mimic-jaw contact slips; sim grasp proxy does not validate grip fidelity.",
)
def test_45_mm_grip_physically_lifts_a_cube_50_mm(model, data):
    """The real aperture command must transmit force through the mimic jaws."""
    target = np.array([0.35, 0.07, 0.015])
    grasp = _top_down_grasp_ik(model, target, [0.11, 2.23, 1.84, -1.15, 0.072, 0.082])
    lifted = _top_down_grasp_ik(model, target + [0, 0, 0.05], grasp)
    data.qpos[:6] = grasp
    data.qpos[6:9] = [5.25, 0.042, 0.042]
    data.ctrl[:] = np.r_[grasp, 5.25]
    blue_address = model.jnt_qposadr[model.body("blue_cube").jntadr[0]]
    data.qpos[blue_address : blue_address + 7] = [
        target[0],
        target[1],
        0.025,
        1,
        0,
        0,
        0,
    ]
    # Isolate the grasp from whichever red placement bench_layout currently selects.
    red_address = model.jnt_qposadr[model.body("red_cube").jntadr[0]]
    data.qpos[red_address : red_address + 3] = [5, 5, 1]
    for phase, duration in [
        ("settle", 0.5),
        ("close", 1),
        ("hold", 1),
        ("lift", 2),
        ("lift_hold", 1),
    ]:
        steps = round(duration / model.opt.timestep)
        for step in range(steps):
            fraction = (step + 1) / steps
            if phase == "close":
                data.ctrl[6] = 5.25 + fraction * (2.8125 - 5.25)
            elif phase == "lift":
                data.ctrl[:6] = (1 - fraction) * grasp + fraction * lifted
            mujoco.mj_step(model, data)
    # MuJoCo body pose, not commanded joints or an attached planning-scene box.
    np.testing.assert_allclose(
        data.body("blue_cube").xpos, [target[0], target[1], 0.075], atol=0.005
    )


def test_released_stack_does_not_drift_over_five_minutes(model, data):
    """Replay the released stack through the real scene, without a grasp weld."""
    fixture = json.loads(
        (Path(__file__).parent / "fixtures/released_stack.json").read_text()
    )
    assert len(fixture["qpos"]) == model.nq
    data.qpos[:] = fixture["qpos"]
    data.joint("blue_cube_joint").qpos[0] -= 0.003
    data.qvel[:] = 0
    data.ctrl[:] = data.qpos[:7]
    assert not any(
        data.eq_active[i]
        for i in range(model.neq)
        if model.eq_type[i] == mujoco.mjtEq.mjEQ_WELD
    )
    mujoco.mj_forward(model, data)
    start = np.array([data.body(c + "_cube").xpos.copy() for c in ("red", "blue")])
    max_drift = 0.0
    max_xy = 0.0
    for step in range(math.ceil(300 / model.opt.timestep)):
        mujoco.mj_step(model, data)
        if step % 10 == 0:
            poses = np.array([data.body(c + "_cube").xpos for c in ("red", "blue")])
            max_drift = max(
                max_drift, float(np.linalg.norm(poses - start, axis=1).max())
            )
            max_xy = max(max_xy, float(np.linalg.norm((poses[1] - poses[0])[:2])))
    assert max_drift < 0.0005, f"Five-minute cube drift: {max_drift * 1000:.3f} mm"
    assert max_xy < 0.005, f"Five-minute stack XY error: {max_xy * 1000:.3f} mm"
