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

"""Exercise the real MoveIt/FCL consumer, including mobile-base and padded self checks."""

import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import yaml
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from moveit_pro_base import (
    collision_detection,
    planning_scene,
    robot_model,
    robot_state,
)

PACKAGE = Path(__file__).resolve().parents[1]


@pytest.fixture(scope="module")
def expanded_urdf(tmp_path_factory):
    urdf = tmp_path_factory.mktemp("livox_moveit") / "duo.urdf"
    urdf.write_text(
        subprocess.check_output(
            ["xacro", str(PACKAGE / "description/mobile_fr3_duo.urdf.xacro")],
            text=True,
        )
    )
    return urdf


@pytest.fixture(scope="module")
def model(expanded_urdf):
    return robot_model.RobotModel(
        str(expanded_urdf), str(PACKAGE / "config/moveit/mobile_fr3_duo.srdf")
    )


def state_at(model, pose):
    state = robot_state.RobotState(model)
    state.set_to_default_values("manipulator", "ready")
    if pose != "ready":
        waypoint = next(
            w
            for w in yaml.safe_load((PACKAGE / "waypoints/waypoints.yaml").read_text())
            if w["name"] == "Stow"
        )
        state.joint_positions = dict(
            zip(waypoint["joint_state"]["name"], waypoint["joint_state"]["position"])
        )
    if pose == "raised_stow":
        state.joint_positions = {"franka_spine_vertical_joint": 0.85}
    state.update()
    return state


def collision(scene, state, group="", padded=False):
    request = collision_detection.CollisionRequest()
    request.joint_model_group_name = group
    request.pad_self_collisions = padded
    request.contacts = True
    request.max_contacts = 100
    result = collision_detection.CollisionResult()
    scene.check_self_collision(request, result, state)
    return result


@pytest.mark.parametrize("pose", ["ready", "stow", "raised_stow"])
@pytest.mark.parametrize("group", ["", "mobile_base"])
@pytest.mark.parametrize("padding", [0.0, 0.01])
def test_fixed_sensor_does_not_invalidate_whole_robot_or_base(
    model, pose, group, padding
):
    scene = planning_scene.PlanningScene(model)
    scene.set_link_padding(padding)
    result = collision(scene, state_at(model, pose), group, padding > 0)
    assert not result.collision, result.contacts


def test_sensor_acm_excludes_only_the_two_fixed_attachment_pairs(model, expanded_urdf):
    scene = planning_scene.PlanningScene(model)
    acm = scene.allowed_collision_matrix
    allowed = {
        frozenset(("franka_spine", "livox_mid360_mount")),
        frozenset(("livox_mid360_mount", "livox_frame")),
    }
    links = [link.get("name") for link in ET.parse(expanded_urdf).findall("link")]
    for sensor in ("livox_mid360_mount", "livox_frame"):
        for other in links:
            if sensor == other:
                continue
            entry = acm.get_entry(sensor, other)
            # The first returned bool means the entry exists, not that it is allowed.
            if frozenset((sensor, other)) in allowed:
                assert entry == (True, "ALWAYS"), (sensor, other, entry)
            else:
                assert entry[1] != "ALWAYS", (sensor, other, entry)


@pytest.mark.parametrize("padding", [0.0, 0.01])
@pytest.mark.parametrize(
    "attachment",
    [
        ("franka_spine", "livox_mid360_mount"),
        ("livox_mid360_mount", "livox_frame"),
    ],
)
def test_fixed_attachment_exclusions_reach_fcl(
    expanded_urdf, tmp_path, attachment, padding
):
    source_srdf = PACKAGE / "config/moveit/mobile_fr3_duo.srdf"
    contact_urdf = expanded_urdf
    if "franka_spine" in attachment:
        # The interim post does not intersect the triangulated spine cap. Plant a
        # test-only 5 mm contact witness on that cap to verify ACM propagation,
        # rather than claim that nonexistent production geometry is in contact.
        # Both controls below use this identical URDF; only one SRDF entry changes.
        robot = ET.parse(expanded_urdf)
        contact = ET.SubElement(
            robot.getroot().find("link[@name='livox_mid360_mount']"),
            "collision",
            name="test_only_fixed_contact",
        )
        ET.SubElement(contact, "origin", xyz="0.070 0 -0.240")
        ET.SubElement(ET.SubElement(contact, "geometry"), "sphere", radius="0.005")
        contact_urdf = tmp_path / "with_contact_witness.urdf"
        robot.write(contact_urdf)

    attached = robot_model.RobotModel(str(contact_urdf), str(source_srdf))
    scene = planning_scene.PlanningScene(attached)
    scene.set_link_padding(padding)
    assert not collision(
        scene, state_at(attached, "stow"), padded=padding > 0
    ).collision

    # Exercise the installed semantic loader, not just SRDF text or an ACM edit.
    semantic = ET.parse(source_srdf)
    root = semantic.getroot()
    entries = [
        entry
        for entry in root.findall("disable_collisions")
        if {entry.get("link1"), entry.get("link2")} == set(attachment)
    ]
    assert len(entries) == 1
    assert entries[0].get("reason") == "Adjacent"
    root.remove(entries[0])
    srdf = tmp_path / "without_attachment.srdf"
    semantic.write(srdf)
    without_attachment = robot_model.RobotModel(str(contact_urdf), str(srdf))
    scene = planning_scene.PlanningScene(without_attachment)
    scene.set_link_padding(padding)
    result = collision(scene, state_at(without_attachment, "stow"), padded=padding > 0)
    assert result.collision
    assert any(
        set(pair) == set(attachment) for pair in result.contacts
    ), result.contacts


def test_sensor_collision_with_world_is_not_masked(model):
    scene = planning_scene.PlanningScene(model)
    state = state_at(model, "stow")
    assert scene.is_state_valid(state, "mobile_base", False)
    obstacle = CollisionObject()
    obstacle.id = "livox_contact_test"
    obstacle.header.frame_id = "base_link"
    obstacle.operation = CollisionObject.ADD
    shape = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[0.05, 0.05, 0.05])
    pose = Pose()
    pose.orientation.w = 1.0
    transform = state.get_global_link_transform("livox_frame")
    pose.position.x, pose.position.y, pose.position.z = transform[:3, 3].tolist()
    obstacle.primitives = [shape]
    obstacle.primitive_poses = [pose]
    scene.apply_collision_object(obstacle)
    assert not scene.is_state_valid(state, "mobile_base", False)
    request = collision_detection.CollisionRequest()
    request.joint_model_group_name = "mobile_base"
    request.contacts = True
    request.max_contacts = 100
    result = collision_detection.CollisionResult()
    scene.check_collision(request, result, state)
    assert any(
        set(pair) == {"livox_frame", obstacle.id} for pair in result.contacts
    ), result.contacts
