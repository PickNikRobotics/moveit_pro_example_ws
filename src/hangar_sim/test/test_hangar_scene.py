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
ADD_HANGAR_ID = "Add Hangar to Planning Scene"


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
    """Neither the robot xacro nor a hangar_sim xacro it includes defines a hangar link."""
    robot = ROBOT_XACRO.read_text()
    assert "hangar.urdf" not in robot
    included = re.findall(
        r'filename="\$\(find hangar_sim\)/(description/[^"]+\.xacro)"', robot
    )
    assert included, "found no hangar_sim includes; the include pattern is stale"
    for source in [ROBOT_XACRO] + [PACKAGE / path for path in included]:
        defined = set(re.findall(r'<link\s+name="([^"]+)"', source.read_text()))
        assert not defined & _hangar_links(), source.name


def test_srdf_names_no_hangar_link() -> None:
    """The hangar is a world object now; SRDF entries naming its links point at nothing."""
    srdf = ET.parse(SRDF).getroot()
    named = {
        link
        for pair in srdf.iter("disable_collisions")
        for link in (pair.get("link1"), pair.get("link2"))
    }
    assert not named & _hangar_links()


def test_objective_adds_this_file_once() -> None:
    """The Objective loads hangar.urdf, and checks for the same name it adds.

    AddURDF fails when the name is already in the planning scene, so a mismatch
    would make every second run fail.
    """
    objective = ET.parse(ADD_HANGAR_OBJECTIVE)
    add_urdf = objective.find(".//Action[@ID='AddURDF']")
    is_in_scene = objective.find(".//Action[@ID='IsCollisionObjectInPlanningScene']")
    assert add_urdf is not None and is_in_scene is not None
    assert add_urdf.get("package_name") == "hangar_sim"
    assert PACKAGE / add_urdf.get("urdf_file_path") == HANGAR_URDF
    assert add_urdf.get("urdf_name") == is_in_scene.get("collision_object_name")


# Nodes that plan or command motion checked against the planning scene. Nav2 navigation is
# left out: it plans on its costmaps, not the planning scene.
MOTION_NODE_IDS = {
    "DoTeleoperateAction",
    "ExecuteTrajectory",
    "PlanCartesianPath",
    "PlanMTCTask",
    "PlanToJointGoal",
}
MOTION_SUBTREE_PREFIXES = (
    "Move to ",
    "Execute MTC Solution",
    "Interpolate to Joint State",
)


def _objectives() -> dict[str, ET.Element]:
    trees = {}
    for path in (PACKAGE / "objectives").glob("*.xml"):
        root = ET.parse(path).getroot()
        trees[root.get("main_tree_to_execute")] = root
    return trees


def _is_runnable(root: ET.Element) -> bool:
    return root.find(".//Metadata[@runnable='true']") is not None


def _calls(root: ET.Element) -> set[str]:
    tree = root.find("BehaviorTree")
    return {node.get("ID") for node in tree.iter() if node.tag != "BehaviorTree"}


def test_motion_objectives_add_the_hangar() -> None:
    """Each runnable Objective that moves the robot adds the hangar, itself or through a subtree.

    Without it, an Objective run before any other plans through the airplane.
    """
    trees = _objectives()

    def adds_hangar(name: str, seen: frozenset[str] = frozenset()) -> bool:
        calls = _calls(trees[name])
        return ADD_HANGAR_ID in calls or any(
            adds_hangar(called, seen | {name})
            for called in calls
            if called in trees and called not in seen
        )

    def moves(name: str, seen: frozenset[str] = frozenset()) -> bool:
        calls = _calls(trees[name])
        if calls & MOTION_NODE_IDS or any(
            call.startswith(MOTION_SUBTREE_PREFIXES) for call in calls
        ):
            return True
        return any(
            moves(called, seen | {name})
            for called in calls
            if called in trees and called not in seen
        )

    checked = [
        name
        for name, root in trees.items()
        if (_is_runnable(root) or name == "Request Teleoperation") and moves(name)
    ]
    assert "Request Teleoperation" in checked
    assert [name for name in checked if not adds_hangar(name)] == []
