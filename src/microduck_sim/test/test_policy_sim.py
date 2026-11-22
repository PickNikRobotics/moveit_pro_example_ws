"""Exercise the downloaded policy against real MuJoCo/BAM physics and description."""

import os
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
import xacro
from ament_index_python.packages import get_package_share_directory

sys.path.insert(0, str(Path(__file__).parents[1] / "script"))
from policy_sim import PolicySimulation


@pytest.fixture(scope="module")
def simulation():
    sim = PolicySimulation(
        os.environ["MICRODUCK_RL_ROOT"],
        os.environ["MICRODUCK_POLICY"],
        str(Path(get_package_share_directory("hangar_sim")) / "description/hangar.xml"),
    )
    return sim


def settle(sim):
    sim.reset()
    for _ in range(100):
        sim.step()


@pytest.mark.parametrize("velocity", [(0.3, 0, 0), (0, 0, 1.5), (0, 0, -1.5)])
def test_command_moves_the_free_body_and_remains_upright(simulation, velocity) -> None:
    """Commands must move the physical free body while preserving balance."""
    sim = simulation
    settle(sim)
    start = sim.position
    yaws = []
    for _ in range(250):
        sim.step(velocity)
        rotation = Rotation.from_quat(np.roll(sim.data.qpos[3:7], -1))
        yaws.append(rotation.as_euler("xyz")[2])
        assert sim.position[2] > 0.08
        assert sim.data.xmat[sim.policy.trunk_base_id].reshape(3, 3)[2, 2] > 0.8
    if velocity[0]:
        assert sim.position[0] - start[0] > 0.06
    else:
        yaw_change = np.unwrap(yaws)[-1] - yaws[0]
        assert yaw_change * np.sign(velocity[2]) > 1.0
    stop = sim.position
    for _ in range(100):
        sim.step()
    assert np.linalg.norm(sim.position[:2] - stop[:2]) < 0.035


@pytest.mark.parametrize("yaw", [-0.5, 0.5])
def test_head_command_is_measured_and_reset_restores_pose(simulation, yaw) -> None:
    """Head commands act through the policy; reset clears its action history."""
    sim = simulation
    settle(sim)
    target = sim.policy.default_pose[5:9].copy()
    target[2] = yaw
    for _ in range(150):
        sim.step(head_target=target)
    measured = sim.data.qpos[sim.model.jnt_qposadr[sim.model.joint("head_yaw").id]]
    assert measured == pytest.approx(yaw, abs=0.1)
    sim.reset()
    np.testing.assert_allclose(sim.position, [0, 0, 0.125])
    np.testing.assert_allclose(sim.policy.last_action, 0)
    np.testing.assert_allclose(sim.data.qvel, 0)


def test_hangar_has_one_contact_floor_and_camera_can_see_robot(simulation) -> None:
    """Prevent doubled floor contacts and scene-scaled near clipping of Microduck."""
    sim = simulation
    floor = sim.model.geom("collision_SM_Floor_376 geom")
    assert floor.contype == 0 and floor.conaffinity == 0
    assert sim.model.vis.map.znear * sim.model.stat.extent < 0.05
    settle(sim)
    contact_names = {
        sim.model.geom(int(g)).name
        for contact in sim.data.contact
        for g in contact.geom
    }
    assert "microduck_ground" in contact_names
    assert "collision_SM_Floor_376 geom" not in contact_names


def test_head_camera_follows_measured_head_rotation(simulation) -> None:
    """The onboard view must rotate with the head rather than track the trunk."""
    sim = simulation
    settle(sim)
    camera = sim.model.camera("head_camera").id
    head_body = sim.model.jnt_bodyid[sim.model.joint("head_roll").id]
    assert sim.model.cam_bodyid[camera] == head_body
    assert sim.model.cam_mode[camera] == mujoco.mjtCamLight.mjCAMLIGHT_FIXED
    initial_rotation = sim.data.cam_xmat[camera].reshape(3, 3)
    assert -initial_rotation[0, 2] > 0.8
    assert initial_rotation[2, 1] > 0.8
    directions = []
    for yaw in (-0.5, 0.5):
        target = sim.policy.default_pose[5:9].copy()
        target[2] = yaw
        for _ in range(150):
            sim.step(head_target=target)
        camera_rotation = sim.data.cam_xmat[camera].reshape(3, 3)
        trunk_rotation = sim.data.xmat[sim.policy.trunk_base_id].reshape(3, 3)
        # MuJoCo looks along -Z; express that direction in the moving trunk frame.
        directions.append(trunk_rotation.T @ -camera_rotation[:, 2])
        local_position = sim.data.xmat[head_body].reshape(3, 3).T @ (
            sim.data.cam_xpos[camera] - sim.data.xpos[head_body]
        )
        np.testing.assert_allclose(local_position, sim.model.cam_pos[camera], atol=1e-8)
    assert np.dot(*directions) < np.cos(0.6)
    assert directions[1][1] > directions[0][1]


@pytest.mark.parametrize("command", [(np.nan, 0, 0), (0, np.inf, 0), (0, 0)])
def test_invalid_commands_leave_physics_untouched(simulation, command) -> None:
    """Malformed input fails before advancing physics or changing its state."""
    sim = simulation
    before = sim.data.qpos.copy()
    with pytest.raises(ValueError):
        sim.step(command)
    np.testing.assert_array_equal(sim.data.qpos, before)


def test_description_joint_frames_match_physics(simulation) -> None:
    """Independent URDF forward kinematics must agree with measured MuJoCo bodies."""
    sim = simulation
    settle(sim)
    # Refresh transforms for the current qpos without adding a physics step.
    mujoco.mj_forward(sim.model, sim.data)
    xml = xacro.process_file(
        str(Path(__file__).parents[1] / "description/microduck.urdf.xacro")
    ).toxml()
    robot = ET.fromstring(xml)
    root = np.eye(4)
    root[:3, :3] = sim.data.xmat[sim.policy.trunk_base_id].reshape(3, 3)
    root[:3, 3] = sim.position
    transforms = {"trunk_base": root}
    bodies = {"trunk_base": sim.policy.trunk_base_id}
    pending = list(robot.findall("joint"))
    while pending:
        progress = False
        for joint in pending[:]:
            parent = joint.find("parent").get("link")
            if parent not in transforms:
                continue
            child = joint.find("child").get("link")
            origin = joint.find("origin")
            frame = np.eye(4)
            if origin is not None:
                frame[:3, 3] = np.fromstring(origin.get("xyz", "0 0 0"), sep=" ")
                frame[:3, :3] = Rotation.from_euler(
                    "xyz", np.fromstring(origin.get("rpy", "0 0 0"), sep=" ")
                ).as_matrix()
            if joint.get("type") == "revolute" and joint.get("name") != "jaw":
                model_joint = sim.model.joint(joint.get("name"))
                bodies[child] = sim.model.jnt_bodyid[model_joint.id]
                angle = sim.data.qpos[model_joint.qposadr[0]]
                axis = np.fromstring(joint.find("axis").get("xyz"), sep=" ")
                frame[:3, :3] = (
                    frame[:3, :3] @ Rotation.from_rotvec(axis * angle).as_matrix()
                )
            transforms[child] = transforms[parent] @ frame
            pending.remove(joint)
            progress = True
        if not progress:
            break
    checked = 0
    for name, frame in transforms.items():
        body = bodies.get(name, -1)
        if body >= 0:
            np.testing.assert_allclose(
                frame[:3, 3], sim.data.xpos[body], atol=2e-5, err_msg=name
            )
            np.testing.assert_allclose(
                frame[:3, :3],
                sim.data.xmat[body].reshape(3, 3),
                atol=2e-5,
                err_msg=name,
            )
            checked += 1
    assert checked >= 14
