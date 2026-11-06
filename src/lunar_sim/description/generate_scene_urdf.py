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


"""Writes lunar_scene_objects.urdf, the moon base as scene objects for MoveIt and the 3D view.

The moon base is MuJoCo geometry (moon_base_geoms.xml), which only the simulator reads. This writes
the same bodies as a URDF of their own, separate from the robot model, which the Load Lunar Scene
Objects Objective adds to the planning scene with AddURDF. The URDF's root is MuJoCo's world origin,
so the Objective places it at the mj_world frame. Run it after changing moon_base_geoms.xml or
moon_base_assets.xml; test_husky_mujoco_geometry.py fails while the committed file is stale.

Usage: python3 generate_scene_urdf.py
"""
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
OUTPUT = HERE / "lunar_scene_objects.urdf"
ZERO_VECTOR = "0 0 0"  # MuJoCo's default pos and euler
# What this conversion models; anything else in moon_base_geoms.xml is refused, not dropped.
BODY_ATTRIBUTES = {"name", "pos", "euler"}
GEOM_ATTRIBUTES = {
    "name",
    "type",
    "size",
    "pos",
    "euler",
    "fromto",
    "material",
    "group",
    "contype",
    "conaffinity",
}


def _floats(text):
    return np.array([float(v) for v in text.split()])


def _fmt(values):
    return " ".join(f"{v:.8g}" if abs(v) > 1e-12 else "0" for v in values)


def _rot_x(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def _rot_y(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def _rot_z(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def mujoco_euler(text):
    """MuJoCo's default eulerseq: intrinsic x, then y, then z, in radians."""
    a, b, c = _floats(text or ZERO_VECTOR)
    return _rot_x(a) @ _rot_y(b) @ _rot_z(c)


def urdf_rpy(rotation):
    """URDF origin rpy (fixed-axis roll, pitch, yaw) of a rotation matrix."""
    pitch = np.arcsin(-np.clip(rotation[2, 0], -1.0, 1.0))
    if abs(np.cos(pitch)) < 1e-9:
        return np.array([np.arctan2(-rotation[1, 2], rotation[1, 1]), pitch, 0.0])
    return np.array(
        [
            np.arctan2(rotation[2, 1], rotation[2, 2]),
            pitch,
            np.arctan2(rotation[1, 0], rotation[0, 0]),
        ]
    )


def z_to(direction):
    """A rotation taking +z onto `direction`."""
    d = direction / np.linalg.norm(direction)
    axis = np.cross([0.0, 0.0, 1.0], d)
    s, c = np.linalg.norm(axis), d[2]
    if s < 1e-12:
        return np.eye(3) if c > 0 else _rot_x(np.pi)
    k = axis / s
    kx = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + s * kx + (1 - c) * kx @ kx


def check_supported(element, allowed):
    """Refuse an attribute this conversion would silently drop."""
    unknown = set(element.attrib) - allowed
    if unknown:
        raise SystemExit(
            f"{element.get('name')}: {sorted(unknown)} not handled by generate_scene_urdf.py"
        )
    if element.get("fromto") and element.get("euler"):
        raise SystemExit(
            f"{element.get('name')}: euler together with fromto is not handled"
        )


def shapes(geom):
    """URDF shapes for one MuJoCo geom, each as (xyz, rotation, geometry tag, attributes)."""
    check_supported(geom, GEOM_ATTRIBUTES)
    kind = geom.get("type", "sphere")
    size = _floats(geom.get("size", "0"))
    if geom.get("fromto"):
        if kind not in ("cylinder", "capsule"):
            raise SystemExit(f"{geom.get('name')}: fromto on a {kind} is not handled")
        ends = _floats(geom.get("fromto")).reshape(2, 3)
        axis = ends[1] - ends[0]
        centre, length, rotation = (
            ends.mean(axis=0),
            float(np.linalg.norm(axis)),
            z_to(axis),
        )
    else:
        centre = _floats(geom.get("pos", ZERO_VECTOR))
        rotation = mujoco_euler(geom.get("euler"))
        length = 2 * float(size[1]) if len(size) > 1 else 0.0
    if kind == "box":
        return [(centre, rotation, "box", {"size": _fmt(2 * size)})]
    if kind == "cylinder":
        return [
            (
                centre,
                rotation,
                "cylinder",
                {"radius": _fmt(size[:1]), "length": _fmt([length])},
            )
        ]
    if kind == "capsule":
        # URDF has no capsule: a cylinder with a sphere on each end.
        radius = _fmt(size[:1])
        tip = rotation @ np.array([0.0, 0.0, length / 2])
        return [
            (
                centre,
                rotation,
                "cylinder",
                {"radius": radius, "length": _fmt([length])},
            ),
            (centre - tip, np.eye(3), "sphere", {"radius": radius}),
            (centre + tip, np.eye(3), "sphere", {"radius": radius}),
        ]
    if kind == "sphere":
        return [(centre, rotation, "sphere", {"radius": _fmt(size[:1])})]
    raise SystemExit(
        f"{geom.get('name')}: geom type {kind} has no URDF equivalent here"
    )


def materials():
    root = ET.parse(HERE / "moon_base_assets.xml").getroot()
    return {m.get("name"): m.get("rgba") for m in root.iter("material")}


def check_compiler():
    """moon_base_geoms.xml's eulers are read as radians in MuJoCo's default xyz order, which every
    <compiler> in husky_scene.xml and the files it includes must therefore declare."""
    scene = ET.parse(HERE / "husky_scene.xml").getroot()
    files = [HERE / "husky_scene.xml"] + [
        HERE / i.get("file") for i in scene.iter("include")
    ]
    for path in files:
        for compiler in ET.parse(path).getroot().iter("compiler"):
            if (
                compiler.get("angle") != "radian"
                or compiler.get("eulerseq", "xyz") != "xyz"
            ):
                raise SystemExit(
                    f'{path.name}\'s <compiler> must use angle="radian" and the default eulerseq'
                )


def add_geom(link, geom, colours):
    """Write one MuJoCo geom into `link` as matching visual and collision elements."""
    material = geom.get("material")
    if material not in colours:
        raise SystemExit(
            f"{geom.get('name')}: material {material} is not in moon_base_assets.xml"
        )
    parts = shapes(geom)
    for index, (xyz, rotation, tag, attributes) in enumerate(parts):
        part_name = (
            geom.get("name") if len(parts) == 1 else f"{geom.get('name')}_{index}"
        )
        for element in ("visual", "collision"):
            part = ET.SubElement(link, element, name=part_name)
            ET.SubElement(part, "origin", xyz=_fmt(xyz), rpy=_fmt(urdf_rpy(rotation)))
            ET.SubElement(ET.SubElement(part, "geometry"), tag, **attributes)
            if element == "visual":
                ET.SubElement(part, "material", name=material)
    return material


def add_body(robot, body, colours):
    """Write one MuJoCo body as a link fixed to the scene root; returns the materials it uses."""
    name = body.get("name")
    check_supported(body, BODY_ATTRIBUTES)
    if any(child.tag != "geom" for child in body):
        raise SystemExit(f"{name}: only <geom> children are handled")
    link = ET.SubElement(robot, "link", name=name)
    used = {add_geom(link, geom, colours) for geom in body}
    joint = ET.SubElement(robot, "joint", name=f"{name}_joint", type="fixed")
    ET.SubElement(joint, "parent", link="lunar_scene_root")
    ET.SubElement(joint, "child", link=name)
    ET.SubElement(
        joint,
        "origin",
        xyz=_fmt(_floats(body.get("pos", ZERO_VECTOR))),
        rpy=_fmt(urdf_rpy(mujoco_euler(body.get("euler")))),
    )
    return used


def scene_urdf():
    """The URDF text for every body in moon_base_geoms.xml."""
    check_compiler()
    colours = materials()
    robot = ET.Element("robot", name="lunar_scene_objects")
    ET.SubElement(robot, "link", name="lunar_scene_root")
    root = ET.parse(HERE / "moon_base_geoms.xml").getroot()
    if any(child.tag != "body" for child in root):
        raise SystemExit(
            "moon_base_geoms.xml may hold only <body> elements at its top level"
        )
    used = set()
    for body in root:
        used |= add_body(robot, body, colours)
    for material in sorted(used):
        ET.SubElement(
            ET.SubElement(robot, "material", name=material),
            "color",
            rgba=colours[material],
        )
    ET.indent(robot, space="  ")
    header = (
        '<?xml version="1.0"?>\n'
        "<!-- Generated by generate_scene_urdf.py from moon_base_geoms.xml; do not edit by hand. -->\n"
    )
    return header + ET.tostring(robot, encoding="unicode") + "\n"


def main():
    OUTPUT.write_text(scene_urdf())
    print(f"wrote {OUTPUT.name}")


if __name__ == "__main__":
    main()
