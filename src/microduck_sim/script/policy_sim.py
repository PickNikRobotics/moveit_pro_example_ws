"""Microduck physics and policy, independent of ROS and its executor."""

import importlib.util
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET

import mujoco
import numpy as np


def build_scene(rl_root, hangar_path):
    """Reuse the upstream robot and hangar assets without copying either."""
    robot_dir = Path(rl_root) / "src/mjlab_microduck/robot/microduck"
    root = ET.Element("mujoco", model="microduck_hangar")
    ET.SubElement(root, "compiler", angle="radian", autolimits="true")
    for path in (robot_dir / "robot_groundcontact.xml", Path(hangar_path)):
        source = ET.parse(path).getroot()
        compiler = source.find("compiler")
        mesh_dir = compiler.get("meshdir", "") if compiler is not None else ""
        for element in source.iter():
            if element.tag == "camera" and element.get("name") == "head_camera":
                # The lens faces head-local -Z. Image-right is -Y and image-up is +X.
                element.set("quat", "0.7071067811865476 0 0 -0.7071067811865476")
            # The plane supplies the floor contact. Keeping the coplanar hangar
            # collision mesh doubles the foot contacts and suppresses the gait.
            if (
                element.tag == "geom"
                and element.get("name") == "collision_SM_Floor_376 geom"
            ):
                element.set("contype", "0")
                element.set("conaffinity", "0")
            if "file" in element.attrib:
                folder = mesh_dir if element.tag in ("mesh", "hfield") else ""
                asset = (path.parent / folder / element.get("file")).resolve()
                if not asset.is_file():
                    raise FileNotFoundError(asset)
                element.set("file", str(asset))
        for element in source:
            if element.tag not in ("compiler", "option", "keyframe", "visual"):
                root.append(element)
    ET.SubElement(root, "option", timestep="0.005", integrator="implicitfast")
    visual = ET.SubElement(root, "visual")
    ET.SubElement(visual, "global", offwidth="640", offheight="480")
    # MuJoCo scales clipping by the whole hangar's extent, not the small robot.
    ET.SubElement(visual, "map", znear="0.0001")
    ET.SubElement(visual, "headlight", ambient="0.5 0.5 0.5", diffuse="0.7 0.7 0.7")
    world = ET.SubElement(root, "worldbody")
    ET.SubElement(
        world,
        "geom",
        name="microduck_ground",
        type="plane",
        size="50 50 0.1",
        friction="1 0.005 0.0001",
        rgba="0 0 0 0",
    )
    trunk = root.find(".//body[@name='trunk_base']")
    ET.SubElement(
        trunk,
        "camera",
        name="microduck_overview",
        mode="track",
        pos="0.55 -0.75 0.40",
        xyaxes="0.806 0.591 0 -0.213 0.290 0.933",
        fovy="45",
    )
    return ET.tostring(root, encoding="unicode")


class PolicySimulation:
    """A real 50 Hz policy loop over 200 Hz MuJoCo/BAM actuator physics."""

    def __init__(self, rl_root, policy_path, hangar_path):
        module_path = Path(rl_root) / "scripts/infer_policy.py"
        spec = importlib.util.spec_from_file_location(
            "microduck_upstream_inference", module_path
        )
        upstream = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(upstream)
        actuator_model = upstream.load_bam_model(upstream.BAM_KP_FW, 7.4, None)
        with tempfile.TemporaryDirectory(prefix="microduck-scene-") as folder:
            scene = Path(folder) / "scene.xml"
            scene.write_text(build_scene(rl_root, hangar_path))
            self.model, self.data, self.actuators, _ = upstream.load_mujoco_with_bam(
                str(scene), actuator_model, 0.005, None, upstream.BAM_VIN_MIN
            )
        self.policy = upstream.PolicyInference(
            self.model,
            self.data,
            walking_onnx_path=str(policy_path),
            bam_ctrl=self.actuators,
            use_projected_gravity=True,
            new_cmd_obs=True,
        )
        if self.policy.walking_session.get_inputs()[0].shape != [1, 61]:
            raise ValueError("Expected the Microduck 61-observation policy contract")
        if self.model.nu != 14:
            raise ValueError(f"Expected 14 policy actuators, got {self.model.nu}")
        self.joint_names = [
            self.model.joint(int(j)).name for j in self.model.actuator_trnid[:, 0]
        ]
        self.base_index = int(
            self.model.jnt_qposadr[self.model.joint("trunk_base_freejoint").id]
        )
        if self.policy.walking_session.get_outputs()[0].shape != [1, 14]:
            raise ValueError("Expected the Microduck 14-action policy contract")
        self.reset()

    def reset(self):
        mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[self.base_index : self.base_index + 7] = [
            0,
            0,
            0.125,
            1,
            0,
            0,
            0,
        ]
        self.data.qpos[self.policy.joint_qpos_indices] = self.policy.default_pose
        self.actuators.reset(self.data.qpos)
        self.policy.last_action[:] = 0
        self.policy.vel_cmd[:] = 0
        self.policy.head_offset[:] = 0
        self.policy.set_position_targets(self.policy.default_pose)
        mujoco.mj_forward(self.model, self.data)

    def step(self, velocity=(0, 0, 0), head_target=None):
        command = np.asarray(velocity, dtype=np.float32)
        if command.shape != (3,) or not np.isfinite(command).all():
            raise ValueError("Velocity must contain three finite values")
        self.policy.vel_cmd[:] = np.clip(command, [-0.3, -0.2, -1.5], [0.3, 0.2, 1.5])
        if head_target is not None:
            target = np.asarray(head_target, dtype=np.float32)
            if target.shape != (4,) or not np.isfinite(target).all():
                raise ValueError("Head target must contain four finite values")
            self.policy.head_offset[:] = target - self.policy.default_pose[5:9]
        self.policy._update_command()
        action = self.policy.infer()
        if action.shape != (14,) or not np.isfinite(action).all():
            raise RuntimeError("Policy returned invalid joint actions")
        self.policy.apply_action(action)
        for _ in range(4):
            self.actuators.update()
            mujoco.mj_step(self.model, self.data)
        if (
            not np.isfinite(self.data.qpos).all()
            or not np.isfinite(self.data.qvel).all()
        ):
            raise RuntimeError("Microduck physics diverged")

    @property
    def position(self):
        return self.data.qpos[self.base_index : self.base_index + 3].copy()
