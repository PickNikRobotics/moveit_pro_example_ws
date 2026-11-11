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
"""The hangar stays out of the robot description and loads into the planning scene.

Every node that loads `robot_description` parses its meshes, so the hangar lives in
`description/hangar.urdf` and the "Add Hangar to Planning Scene" Objective adds it to the
planning scene with AddURDF. These checks read the source files directly. No simulator,
no ROS.
"""

import re
import xml.etree.ElementTree as ET
from pathlib import Path

PACKAGE = Path(__file__).resolve().parent.parent
HANGAR_URDF = PACKAGE / "description" / "hangar.urdf"
ROBOT_XACRO = PACKAGE / "description" / "ur5e_ridgeback.xacro"
SRDF = PACKAGE / "config" / "moveit" / "picknik_ur.srdf"
ADD_HANGAR_OBJECTIVE = PACKAGE / "objectives" / "add_hangar_to_planning_scene.xml"
PACKAGE_URI = "package://hangar_sim/"


def _hangar() -> ET.Element:
    return ET.parse(HANGAR_URDF).getroot()


def _hangar_links() -> set[str]:
    return {link.get("name") for link in _hangar().iter("link")}


def test_hangar_urdf_is_one_tree() -> None:
    """AddURDF builds a robot model from the file, so it needs exactly one root link."""
    root = _hangar()
    links = _hangar_links()
    children = [joint.find("child").get("link") for joint in root.iter("joint")]
    parents = {joint.find("parent").get("link") for joint in root.iter("joint")}
    assert len(children) == len(set(children)), "a link has more than one parent joint"
    assert (
        parents | set(children) <= links
    ), "a joint names a link the file does not define"
    assert links - set(children) == {"hangar_base"}


def test_hangar_urdf_has_collision_geometry() -> None:
    """A hangar with no collision geometry would leave planning blind to the airplane."""
    collision_links = {
        link.get("name")
        for link in _hangar().iter("link")
        if link.find("collision") is not None
    }
    assert {
        "collision_Plane",
        "collision_Plane_009",
        "collision_SM_Pillar2_001",
    } <= collision_links


def test_hangar_meshes_exist() -> None:
    """AddURDF fails on a mesh it cannot resolve; catch a renamed asset here instead."""
    meshes = [mesh.get("filename") for mesh in _hangar().iter("mesh")]
    assert meshes
    for filename in meshes:
        assert filename.startswith(PACKAGE_URI), filename
        assert (PACKAGE / filename.removeprefix(PACKAGE_URI)).is_file(), filename


def test_robot_description_does_not_include_the_hangar() -> None:
    """The robot xacro must not define or include any hangar link."""
    robot = ROBOT_XACRO.read_text()
    assert "hangar.urdf" not in robot
    defined = set(re.findall(r'<link\s+name="([^"]+)"', robot))
    assert not defined & _hangar_links()


def test_srdf_names_no_hangar_link() -> None:
    """The hangar is a world object now; SRDF entries naming its links point at nothing."""
    srdf = ET.parse(SRDF).getroot()
    named = {
        link
        for pair in srdf.iter("disable_collisions")
        for link in (pair.get("link1"), pair.get("link2"))
    }
    assert not named & _hangar_links()


def test_objective_adds_this_file() -> None:
    """The Objective must load hangar.urdf from this package."""
    add_urdf = ET.parse(ADD_HANGAR_OBJECTIVE).find(".//Action[@ID='AddURDF']")
    assert add_urdf is not None
    assert add_urdf.get("package_name") == "hangar_sim"
    assert PACKAGE / add_urdf.get("urdf_file_path") == HANGAR_URDF
