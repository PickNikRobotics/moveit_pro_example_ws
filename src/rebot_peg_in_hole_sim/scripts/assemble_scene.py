#!/usr/bin/env python3
# Copyright 2026 PickNik Inc. All rights reserved. BSD-3-Clause.
"""Build a static factory CAD scene from an explicit camera/fixture snapshot."""
import argparse
import json
import math
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_prefix, get_package_share_directory
import mujoco
import numpy as np
from PIL import Image
import xacro
import yaml

PACKAGE = "rebot_peg_in_hole_sim"
OPTICAL = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], dtype=float)
PEG_CENTERS = ((0.076362, -0.079826), (0.1413005, -0.082661385))


def numbers(values):
    return " ".join(str(float(value)) for value in values)


def transform(value):
    matrix = np.asarray(value, dtype=float)
    if matrix.shape != (4, 4) or not np.isfinite(matrix).all():
        raise ValueError("A pose must be a finite 4x4 transform")
    rotation = matrix[:3, :3]
    if (
        not np.allclose(matrix[3], [0, 0, 0, 1])
        or not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6)
        or not np.isclose(np.linalg.det(rotation), 1, atol=1e-6)
    ):
        raise ValueError("A pose must be a proper rigid transform")
    return matrix


def pose_attributes(matrix):
    matrix = transform(matrix)
    quaternion = np.zeros(4)
    mujoco.mju_mat2Quat(quaternion, matrix[:3, :3].ravel())
    return {"pos": numbers(matrix[:3, 3]), "quat": numbers(quaternion)}


def rpy(matrix):
    rotation = matrix[:3, :3]
    return [
        math.atan2(rotation[2, 1], rotation[2, 2]),
        math.atan2(-rotation[2, 0], math.hypot(rotation[0, 0], rotation[1, 0])),
        math.atan2(rotation[1, 0], rotation[0, 0]),
    ]


def body_transform(body):
    matrix = np.eye(4)
    quaternion = np.zeros(4)
    angles = np.array(list(map(float, body.get("euler", "0 0 0").split())))
    mujoco.mju_euler2Quat(quaternion, angles, "XYZ")
    rotation = np.zeros(9)
    mujoco.mju_quat2Mat(rotation, quaternion)
    matrix[:3, :3] = rotation.reshape(3, 3)
    matrix[:3, 3] = list(map(float, body.get("pos", "0 0 0").split()))
    return matrix


def write_xml(root, path):
    ET.indent(root)
    ET.ElementTree(root).write(path, encoding="unicode", xml_declaration=True)


def site(parent, name, xyz=(0, 0, 0)):
    ET.SubElement(parent, "site", name=name, pos=numbers(xyz), size="0.001", group="5")


def visual_mesh(parent, mesh, color):
    ET.SubElement(
        parent,
        "geom",
        type="mesh",
        mesh=mesh,
        rgba=color,
        contype="0",
        conaffinity="0",
        group="1",
    )


def assemble(snapshot, output, assets):
    """Emit a self-contained description using installed parent robot resources."""
    poses = {
        key: transform(value)
        for key, value in snapshot.items()
        if key.startswith("world_T_")
    }
    parent = Path(get_package_share_directory("rebot_bench_sim")) / "description"
    output.mkdir(parents=True, exist_ok=True)
    shutil.copytree(assets, output / "assets", dirs_exist_ok=True)
    layout = yaml.safe_load((parent / "bench_layout.yaml").read_text())
    link_optical = np.eye(4)
    link_optical[:3, :3] = OPTICAL
    for key, matrix in (
        ("robot_base", poses["world_T_base"]),
        ("scene_camera", poses["world_T_scene_optical"] @ np.linalg.inv(link_optical)),
    ):
        layout[key] = {"xyz": matrix[:3, 3].tolist(), "rpy": rpy(matrix)}
    foam_y = float(snapshot["foam_front_y_m"])
    layout["wall"]["xyz"][1] = foam_y + layout["wall"]["size"][1] / 2
    mat_edge = layout["mat"]["xyz"][1] - layout["mat"]["size"][1] / 2
    layout["mat"]["xyz"][1] = (mat_edge + foam_y) / 2
    layout["mat"]["size"][1] = foam_y - mat_edge
    layout_path = output / "bench_layout.yaml"
    layout_path.write_text(yaml.safe_dump(layout, sort_keys=False))

    scene = ET.parse(parent / "scene.xml").getroot()
    scene.set("model", "rebot_peg_in_hole")
    world = scene.find("worldbody")
    for name in ("red_cube", "blue_cube"):
        world.remove(world.find(f"body[@name='{name}']"))
    for name, matrix in (
        ("base_link", poses["world_T_base"]),
        (
            "scene_camera_link",
            poses["world_T_scene_optical"] @ np.linalg.inv(link_optical),
        ),
    ):
        body = world.find(f"body[@name='{name}']")
        body.attrib.pop("euler", None)
        body.attrib.update(pose_attributes(matrix))
    world.find("body[@name='wall']").set("pos", numbers(layout["wall"]["xyz"]))
    mat = world.find("body[@name='mat']")
    mat.set("pos", numbers(layout["mat"]["xyz"]))
    mat.find("geom").set("size", numbers(np.array(layout["mat"]["size"]) / 2))
    key = scene.find("keyframe/key")
    key.set("qpos", " ".join(key.get("qpos").split()[:9]))
    arm = ET.parse(parent / "rebot_arm.xml").getroot()
    robot_assets = ET.parse(parent / "rebot_assets.xml").getroot()
    for mesh in robot_assets.findall(".//mesh"):
        filename = mesh.get("file")
        if filename and not filename.startswith("package://"):
            mesh.set("file", f"package://rebot_bench_sim/description/{filename}")
    write_xml(robot_assets, output / "rebot_assets.xml")
    write_xml(arm, output / "rebot_arm.xml")

    asset = ET.SubElement(scene, "asset")
    for name in ("part", "jig_base", "jig_post"):
        ET.SubElement(asset, "mesh", name="factory_" + name, file=f"assets/{name}.stl")
    jig = ET.SubElement(
        world, "body", name="peg_jig", **pose_attributes(poses["world_T_jig"])
    )
    visual_mesh(jig, "factory_jig_base", "0.85 0.85 0.82 1")
    site(jig, "jig_origin")
    for index, (x, y) in enumerate(PEG_CENTERS, 1):
        post = ET.SubElement(
            jig,
            "body",
            name=f"peg_{index}",
            pos=numbers([x - PEG_CENTERS[1][0], y - PEG_CENTERS[1][1], 0]),
        )
        visual_mesh(post, "factory_jig_post", "0.85 0.85 0.82 1")
        site(jig, f"peg_{index}_tip", [x, y, 0.085])
    part = ET.SubElement(
        world, "body", name="hole_bracket", **pose_attributes(poses["world_T_bracket"])
    )
    visual_mesh(part, "factory_part", "0.32 0.34 0.35 1")
    site(part, "bracket_origin")
    for index, (x, y) in enumerate(PEG_CENTERS, 1):
        site(part, f"hole_{index}_center", [x, y, 0.00762])

    grid = np.full((64, 64, 3), [23, 119, 171], dtype=np.uint8)
    grid[0, :] = grid[:, 0] = [103, 167, 191]
    Image.fromarray(grid).save(output / "mat_grid.png")
    ET.SubElement(asset, "texture", name="mat_grid", type="2d", file="mat_grid.png")
    ET.SubElement(
        asset,
        "material",
        name="mat_grid",
        texture="mat_grid",
        texuniform="true",
        texrepeat=numbers([1 / 0.0127] * 2),
    )
    mat_geom = world.find("body[@name='mat']/geom")
    mat_geom.attrib.pop("rgba", None)
    mat_geom.set("material", "mat_grid")

    ET.SubElement(
        asset, "texture", name="charuco", type="2d", file="assets/charuco_board.png"
    )
    ET.SubElement(
        asset, "material", name="charuco", texture="charuco", texuniform="false"
    )
    # A thin solid mesh avoids a degenerate convex hull. Board coordinates
    # are OpenCV's: x right, y down, and the visible face points along -z.
    ET.SubElement(
        asset,
        "mesh",
        name="charuco",
        vertex="0 0 -.00015 .245 0 -.00015 .245 .175 -.00015 0 .175 -.00015 "
        "0 0 0 .245 0 0 .245 .175 0 0 .175 0",
        face="0 2 1 0 3 2 4 5 6 4 6 7 0 1 5 0 5 4 "
        "1 2 6 1 6 5 2 3 7 2 7 6 3 0 4 3 4 7",
        texcoord="0 0 1 0 1 1 0 1 0 0 1 0 1 1 0 1",
    )
    board = ET.SubElement(
        world,
        "body",
        name="charuco_board",
        **pose_attributes(poses["world_T_board"]),
    )
    ET.SubElement(
        board,
        "geom",
        type="mesh",
        mesh="charuco",
        material="charuco",
        contype="0",
        conaffinity="0",
        group="1",
    )
    site(board, "charuco_board_origin")
    write_xml(scene, output / "scene.xml")

    mujoco.mj_loadAllPluginLibraries(
        str(Path(get_package_prefix("mujoco_package_uri")) / "bin/mujoco_plugin")
    )
    model = mujoco.MjModel.from_xml_path(str(output / "scene.xml"))
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, 0)
    observed = snapshot["wrist_observation_joints"]
    for name, value in observed.items():
        data.qpos[model.joint(name).qposadr[0]] = value
    data.qpos[model.joint("gripper_joint1").qposadr[0]] = 0.008 * observed["joint7"]
    data.qpos[model.joint("gripper_joint2").qposadr[0]] = 0.008 * observed["joint7"]
    mujoco.mj_forward(model, data)
    gripper = model.body("gripper_end").id
    world_gripper = np.eye(4)
    world_gripper[:3, :3] = data.xmat[gripper].reshape(3, 3)
    world_gripper[:3, 3] = data.xpos[gripper]
    mount = arm.find(".//body[@name='d435_mount_link']")
    link_optical[:3, 3] = [0, 0.015, 0]
    tail = (
        body_transform(mount.find("body"))
        @ body_transform(mount.find("body/body"))
        @ link_optical
    )
    gripper_mount = (
        np.linalg.inv(world_gripper)
        @ poses["world_T_wrist_optical"]
        @ np.linalg.inv(tail)
    )
    mount.attrib.pop("euler", None)
    mount.attrib.update(pose_attributes(gripper_mount))
    for camera in (
        scene.find(".//camera[@name='scene_camera_color']"),
        arm.find(".//camera[@name='d435_color']"),
    ):
        key = "scene_color" if camera.get("name").startswith("scene") else "wrist_color"
        focal_y = snapshot["intrinsics"][key][4]
        camera.set("fovy", str(math.degrees(2 * math.atan(480 / (2 * focal_y)))))
        camera.set("resolution", "640 480")
    write_xml(arm, output / "rebot_arm.xml")
    write_xml(scene, output / "scene.xml")

    robot = ET.fromstring(
        xacro.process_file(
            str(parent / "rebot_bench_sim.urdf.xacro"),
            mappings={
                "layout": str(layout_path),
                "mujoco_model": "description/scene.xml",
            },
        ).toxml()
    )
    origin = robot.find("joint[@name='d435_mount_joint']/origin")
    origin.set("xyz", numbers(gripper_mount[:3, 3]))
    origin.set("rpy", numbers(rpy(gripper_mount)))
    robot.find(".//hardware/param[@name='mujoco_model_package']").text = PACKAGE
    robot.find(".//sensor[@name='d435_color']/param[@name='depth_topic']").text = (
        "/wrist_mounted_camera/aligned_depth_to_color/image_raw"
    )
    write_xml(robot, output / "rebot_peg.urdf")
    # Load the final artifact, including corrected mount and intrinsics, before installing.
    mujoco.MjModel.from_xml_path(str(output / "scene.xml"))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("snapshot", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("assets", type=Path)
    args = parser.parse_args()
    assemble(json.loads(args.snapshot.read_text()), args.output, args.assets)


if __name__ == "__main__":
    main()
