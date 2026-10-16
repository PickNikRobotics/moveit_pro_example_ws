# Copyright 2026 PickNik Inc.
# SPDX-License-Identifier: BSD-3-Clause
"""Exercise the Objective's path with the installed admittance controller and MuJoCo."""
import ctypes
import os
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import pytest
import yaml
from ament_index_python.packages import get_package_share_directory


class AdmittanceReplay:
    """Keep the production admittance integrator; supply kinematics from MuJoCo."""

    def __init__(self, parameters):
        self.library = ctypes.CDLL(os.environ["ADMITTANCE_TEST_LIBRARY"])
        array = np.ctypeslib.ndpointer(dtype=np.float64, flags="C_CONTIGUOUS")
        self.library.replay_create.argtypes = [array] * 4 + [ctypes.c_double]
        self.library.replay_create.restype = ctypes.c_void_p
        self.library.replay_step.argtypes = (
            [ctypes.c_void_p] + [array] * 6 + [ctypes.c_double, array]
        )
        self.library.replay_destroy.argtypes = [ctypes.c_void_p]

        def values(name):
            return np.array(
                [
                    parameters[name + suffix][axis]
                    for suffix in ["_position", "_rotation"]
                    for axis in "xyz"
                ],
                float,
            )

        axes = np.array(
            [
                parameters["selected_axes"][axis]
                for axis in ["x", "y", "z", "rx", "ry", "rz"]
            ],
            float,
        )
        self.handle = self.library.replay_create(
            values("mass"),
            values("damping_ratio"),
            values("stiffness"),
            axes,
            parameters["joint_damping"]["value"],
        )

    def close(self):
        self.library.replay_destroy(self.handle)

    def step(
        self, joints, reference, wrench, reference_pose, actual_pose, jacobian, period
    ):
        output = np.zeros(7)
        assert self.library.replay_step(
            self.handle,
            joints,
            reference,
            wrench,
            reference_pose,
            actual_pose,
            jacobian,
            period,
            output,
        )
        return output


def rotation_error(target, actual):
    quaternion = np.zeros(4)
    mujoco.mju_mat2Quat(quaternion, (target @ actual.T).flatten())
    error = np.zeros(3)
    mujoco.mju_quat2Vel(error, quaternion, 1.0)
    return error


class ArmModel:
    def __init__(self, share):
        self.model = mujoco.MjModel.from_xml_path(
            str(share / "description/mujoco/rafti_fingers_scene.xml")
        )
        self.data = mujoco.MjData(self.model)
        self.kinematic_data = mujoco.MjData(self.model)
        mujoco.mj_resetDataKeyframe(self.model, self.data, self.model.key("default").id)
        self.names = ["joint_" + str(i) for i in range(1, 8)]
        self.positions = [
            self.model.jnt_qposadr[self.model.joint(n).id] for n in self.names
        ]
        self.velocities = [
            self.model.jnt_dofadr[self.model.joint(n).id] for n in self.names
        ]
        self.actuators = [self.model.actuator(n).id for n in self.names]
        self.sensor_site = self.model.site("fts").id

    def pose(self, joints):
        self.kinematic_data.qpos[self.positions] = joints
        mujoco.mj_kinematics(self.model, self.kinematic_data)
        pose = np.eye(4)
        pose[:3, :3] = self.kinematic_data.site_xmat[self.sensor_site].reshape(3, 3)
        pose[:3, 3] = self.kinematic_data.site_xpos[self.sensor_site]
        return pose

    def jacobian(self):
        mujoco.mj_comPos(self.model, self.kinematic_data)
        translation = np.zeros((3, self.model.nv))
        rotation = np.zeros((3, self.model.nv))
        mujoco.mj_jacSite(
            self.model, self.kinematic_data, translation, rotation, self.sensor_site
        )
        return np.ascontiguousarray(
            np.vstack([translation[:, self.velocities], rotation[:, self.velocities]])
        )

    def inverse_kinematics(self, target, seed):
        joints = seed.copy()
        for _ in range(100):
            actual = self.pose(joints)
            error = np.r_[
                target[:3, 3] - actual[:3, 3],
                rotation_error(target[:3, :3], actual[:3, :3]),
            ]
            if np.linalg.norm(error) < 1e-7:
                return joints
            jacobian = self.jacobian()
            delta = jacobian.T @ np.linalg.solve(
                jacobian @ jacobian.T + 1e-6 * np.eye(6), error
            )
            joints += delta * min(1, 0.1 / np.linalg.norm(delta))
        raise AssertionError(f"Path IK failed: {error}")

    def target_pose(self, node):
        target = np.eye(4)
        base = self.model.body("base_link").pos
        target[:3, 3] = base + np.array(
            [float(x) for x in node.attrib["position_xyz"].split(";")]
        )
        xyzw = [float(x) for x in node.attrib["orientation_xyzw"].split(";")]
        quaternion = np.array([xyzw[3], *xyzw[:3]])
        quaternion /= np.linalg.norm(quaternion)
        matrix = np.zeros(9)
        mujoco.mju_quat2Mat(matrix, quaternion)
        # fts_joint rotates end_effector_link by pi about Z. Its origin differs by 25 um.
        target[:3, :3] = matrix.reshape(3, 3) @ np.diag([-1, -1, 1])
        return target

    def path(self, start, targets, limits):
        """Dense Cartesian IK and synchronized smooth segments bounded by shipped limits."""
        joints = [start.copy()]
        times = [0.0]
        for target in targets:
            initial = self.pose(joints[-1])
            angle_axis = rotation_error(target[:3, :3], initial[:3, :3])
            segment = []
            seed = joints[-1]
            samples = 60
            for i in range(1, samples + 1):
                u = i / samples
                blend = u * u * (3 - 2 * u)
                pose = target.copy()
                pose[:3, 3] = initial[:3, 3] + blend * (target[:3, 3] - initial[:3, 3])
                quaternion = np.array([1.0, 0.0, 0.0, 0.0])
                mujoco.mju_quatIntegrate(quaternion, angle_axis, blend)
                matrix = np.zeros(9)
                mujoco.mju_quat2Mat(matrix, quaternion)
                pose[:3, :3] = matrix.reshape(3, 3) @ initial[:3, :3]
                seed = self.inverse_kinematics(pose, seed)
                segment.append(seed)
            values = np.vstack([joints[-1], segment])
            velocity = np.array([limits[n]["max_velocity"] * 0.2 for n in self.names])
            acceleration = np.array(
                [limits[n]["max_acceleration"] * 0.5 for n in self.names]
            )
            duration = max(
                1.0,
                np.max(abs(np.diff(values, axis=0)) * samples / velocity),
                np.sqrt(
                    np.max(
                        abs(np.diff(values, n=2, axis=0)) * samples**2 / acceleration
                    )
                ),
            )
            origin = times[-1]
            times.extend(origin + duration * i / samples for i in range(1, samples + 1))
            joints.extend(segment)
        return np.array(times), np.array(joints)


def run_admittance_path(real_time_factor, external_load):
    share = Path(get_package_share_directory("kinova_sim"))
    tree = ET.parse(share / "objectives/move_along_path_admittance.xml")
    preparation = tree.find('.//SubTree[@ID="Move to Named Waypoint"]').attrib[
        "waypoint_name"
    ]
    waypoints = yaml.safe_load((share / "waypoints/waypoints.yaml").read_text())
    start = np.array(
        next(
            w["joint_state"]["position"] for w in waypoints if w["name"] == preparation
        )
    )
    parameters = yaml.safe_load(
        (share / "objectives/jtac_parameters.yaml").read_text()
    )["SetAdmittanceParameters"]["admittance"]
    limits = yaml.safe_load((share / "config/moveit/joint_limits.yaml").read_text())[
        "joint_limits"
    ]
    arm = ArmModel(share)
    targets = [
        arm.target_pose(node)
        for node in tree.findall('.//Action[@ID="CreatePoseStamped"]')
    ]
    arm.data.qpos[arm.positions] = start
    arm.data.ctrl[arm.actuators] = start
    # Start from the prepared, stationary state, as the named-waypoint approach does.
    for _ in range(1000):
        mujoco.mj_step(arm.model, arm.data)
    command_probe = None
    for cycle in range(2):
        times, references = arm.path(
            arm.data.qpos[arm.positions].copy(), targets, limits
        )
        # A sensor-frame tare removes the initial static load, rather than tracking gravity
        # through subsequent tool rotations. The force channel drives this regression.
        tare = -arm.data.sensordata.copy()
        controller = AdmittanceReplay(parameters)
        try:
            count = int((times[-1] + 0.5) * real_time_factor / arm.model.opt.timestep)
            for step in range(count + 1):
                wall_time = step * arm.model.opt.timestep / real_time_factor
                desired = np.array(
                    [np.interp(wall_time, times, references[:, i]) for i in range(7)]
                )
                desired_pose = arm.pose(desired)
                actual = arm.data.qpos[arm.positions].copy()
                actual_pose = arm.pose(actual)
                jacobian = arm.jacobian()
                wrench = -arm.data.sensordata - tare
                # A bounded external disturbance must still produce compliance and recover.
                arm.data.xfrc_applied[arm.model.body("robotiq_85_base_link").id, :3] = (
                    [3.0, 0.0, 0.0]
                    if external_load and 1 < wall_time < 2
                    else [0.0, 0.0, 0.0]
                )
                for _ in range(3):
                    output = controller.step(
                        actual,
                        desired,
                        wrench,
                        desired_pose,
                        actual_pose,
                        jacobian,
                        arm.model.opt.timestep / real_time_factor / 3,
                    )
                arm.data.ctrl[arm.actuators] = output
                mujoco.mj_step(arm.model, arm.data)
                assert np.isfinite(arm.data.qpos).all()
                assert np.isfinite(arm.data.qvel).all()
                assert np.max(abs(desired - arm.data.qpos[arm.positions])) <= 2.0
                assert np.max(abs(wrench[:3])) < 45.0
                assert np.max(abs(wrench[3:])) < 10.0
                if cycle == 0 and command_probe is None and wall_time >= 1.8:
                    command_probe = arm.pose(output)[:3, 3] - desired_pose[:3, 3]
            error = abs(references[-1] - arm.data.qpos[arm.positions])
            assert np.max(error) <= 0.1, dict(zip(arm.names, error))
            print("endpoint", cycle, real_time_factor, external_load, error)
        finally:
            controller.close()
    return command_probe


@pytest.mark.parametrize("real_time_factor", [1.0, 0.75])
def test_admittance_path_recovers_endpoint(real_time_factor):
    unforced = run_admittance_path(real_time_factor, False)
    forced = run_admittance_path(real_time_factor, True)
    # Compare commands at the same reference time: physical servo deflection or normal
    # following error alone must not pass for an actual compliant response to the load.
    response = np.linalg.norm(forced - unforced)
    assert response > 0.005, response
