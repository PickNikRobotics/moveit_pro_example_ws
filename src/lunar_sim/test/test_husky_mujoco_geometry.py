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

"""The MJCF wheel collision radius/width must track the vendored URDF's, and the wheel
velocity actuators must stay inside the armature/kv-vs-timestep bound documented in
AGENTS.md (an armature/kv time constant at or above the timestep silently pins the
wheels - see hangar_sim's mecanum base incident). Nothing else in the build checks
either; both failure modes are quiet (no crash, the base just doesn't move, or the
model loads with wheels floating above/through the ground).

Wheel *placement* (x/y/z body offsets) is deliberately not cross-checked here: deriving
it from the vendored xacro requires evaluating a per-side conditional expression
(`suspension_beam.urdf.xacro`'s `spacer_offset`), which this test can't do without
duplicating that logic (and getting it subtly wrong is worse than not checking it) -
see husky_a300.xml's own derivation comment for that FK chain instead.

No simulator, no ROS - the MJCF fragments and the vendored xacro are parsed as XML and
asserted on by element/attribute, mirroring hangar_sim/test/test_base_geometry.py.
"""

import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest
from PIL import Image
from ament_index_python.packages import get_package_share_directory

DESCRIPTION = Path(__file__).resolve().parent.parent / "description"
HUSKY_A300 = DESCRIPTION / "husky_a300.xml"
HUSKY_SCENE = DESCRIPTION / "husky_scene.xml"

CLEARPATH_A300_URDF = (
    Path(get_package_share_directory("clearpath_platform_description"))
    / "urdf"
    / "a300"
)

TOLERANCE_M = 1e-4


def _xacro_property(source: Path, name: str) -> float:
    """Vendored xacro files declare these as literal numbers; a non-literal value means
    the property moved behind an expression and this cross-check needs revisiting."""
    for prop in ET.parse(source).getroot().iter():
        # Namespace-agnostic: the xacro namespace URI is spelled both with and
        # without the "www." prefix across upstream files.
        if prop.tag.rpartition("}")[2] != "property":
            continue
        if prop.get("name") == name:
            return float(prop.get("value"))
    raise AssertionError(f"{source.name} has no numeric xacro:property named {name!r}")


def _outdoor_wheel_radius_width() -> tuple[float, float]:
    outdoor = CLEARPATH_A300_URDF / "drivetrain" / "wheels" / "outdoor.urdf.xacro"
    return (
        _xacro_property(outdoor, "a300_outdoor_wheel_radius"),
        _xacro_property(outdoor, "a300_outdoor_wheel_width"),
    )


def _wheel_link_roots() -> dict[str, ET.Element]:
    roots = {
        f.name: ET.parse(f).getroot() for f in DESCRIPTION.glob("*_wheel_link.xml")
    }
    assert roots, f"no *_wheel_link.xml files under {DESCRIPTION}"
    return roots


def test_wheel_collision_cylinder_matches_vendored_radius_and_width():
    radius, width = _outdoor_wheel_radius_width()
    for name, root in _wheel_link_roots().items():
        cylinders = [g for g in root.iter("geom") if g.get("type") == "cylinder"]
        assert len(cylinders) == 1, f"{name} should have exactly one cylinder geom"
        got_radius, got_half_width = (
            float(v) for v in cylinders[0].get("size").split()
        )
        assert got_radius == pytest.approx(radius, abs=TOLERANCE_M), name
        assert got_half_width == pytest.approx(width / 2, abs=TOLERANCE_M), name


def test_velocity_actuators_stay_below_the_timestep():
    """See AGENTS.md: "Velocity actuators: armature/kv time-constant must stay below the
    timestep." A violation pins the wheels without any error - this is the only check
    for it."""
    option = ET.parse(HUSKY_SCENE).getroot().find("option")
    assert option is not None and option.get(
        "timestep"
    ), f"{HUSKY_SCENE.name} has no <option timestep=...>"
    timestep = float(option.get("timestep"))

    armatures = {}
    for name, root in _wheel_link_roots().items():
        joints = [j for j in root.iter("joint") if j.get("armature")]
        assert (
            len(joints) == 1
        ), f"{name} should have exactly one joint with an armature"
        armatures[joints[0].get("name")] = float(joints[0].get("armature"))

    kvs = {
        v.get("joint"): float(v.get("kv"))
        for v in ET.parse(HUSKY_A300).getroot().iter("velocity")
        if v.get("kv")
    }
    assert armatures.keys() == kvs.keys(), (armatures, kvs)
    for joint, armature in armatures.items():
        tau = armature / kvs[joint]
        assert tau < timestep, (
            f"{joint}: armature/kv={tau}s is not below timestep={timestep}s - "
            "this wheel will not respond to velocity commands (AGENTS.md)"
        )


# freejoint is 7 values (x y z qw qx qy qz), ball 4, hinge/slide 1 - AGENTS.md.
_JOINT_QPOS = {"free": 7, "ball": 4, "hinge": 1, "slide": 1}


def _model_nq(source: Path) -> int:
    """Total qpos width of the compiled model, summed over every joint reachable from
    husky_scene.xml through its <include> chain (the compiler splices includes in
    place, so only the joint set matters here, not the nesting)."""
    root = ET.parse(source).getroot()
    # A file whose root is <mujoco> declares joints under <worldbody>; an included
    # fragment's root is itself the spliced-in <body>.
    bodies = root.findall("worldbody") if root.tag == "mujoco" else [root]

    nq = 0
    for body in bodies:
        for joint in body.iter():
            if joint.tag == "freejoint":
                nq += _JOINT_QPOS["free"]
            elif joint.tag == "joint":
                jtype = joint.get("type", "hinge")
                assert (
                    jtype in _JOINT_QPOS
                ), f"{source.name}: unknown joint type {jtype!r}"
                nq += _JOINT_QPOS[jtype]
    for include in root.iter("include"):
        nq += _model_nq(source.parent / include.get("file"))
    return nq


def test_keyframe_qpos_matches_model_dof_count():
    """See AGENTS.md: a keyframe qpos whose length differs from the model's nq crashes
    ros2_control_node at startup ("keyframe 0: invalid qpos size"), with nothing else in
    the build catching it."""
    nq = _model_nq(HUSKY_SCENE)
    for key in ET.parse(HUSKY_SCENE).getroot().iter("key"):
        qpos = key.get("qpos", "").split()
        assert len(qpos) == nq, (
            f"keyframe {key.get('name')!r} has {len(qpos)} qpos values but the model's "
            f"joints total nq={nq} - this crashes ros2_control_node at startup "
            "(AGENTS.md)"
        )


def _hfield_elevation(x_m: float, y_m: float) -> float:
    """World z of the committed heightfield at (x, y), read back through the same
    conventions generate_terrain.py writes it with: rows reversed on load (image row 0
    is maximum y), grayscale scaled over the <hfield>'s elevation_z, offset by the
    ground geom's pos.z."""
    root = ET.parse(HUSKY_SCENE).getroot()
    hfield = next(h for h in root.iter("hfield") if h.get("name") == "lunar_hfield")
    half_x, half_y, elevation_z, _base = (float(v) for v in hfield.get("size").split())
    ground = next(g for g in root.iter("geom") if g.get("hfield") == hfield.get("name"))
    ground_z = float(ground.get("pos").split()[2])

    png = np.asarray(Image.open(DESCRIPTION / "assets" / hfield.get("file")))
    heights = png[::-1, :].T / 255.0 * elevation_z + ground_z
    nx, ny = heights.shape
    col = round((x_m + half_x) / (2 * half_x) * nx)
    row = round((y_m + half_y) / (2 * half_y) * ny)
    return float(heights[col, row])


def _chassis_to_wheel_bottom_m() -> float:
    """How far the wheel contact surface sits below chassis_link's origin: the wheel
    bodies' own z offset under chassis_link, minus the collision cylinder radius."""
    root = ET.parse(HUSKY_A300).getroot()
    chassis = next(b for b in root.iter("body") if b.get("name") == "chassis_link")
    wheel_zs = {
        float(b.get("pos").split()[2])
        for b in chassis.iter("body")
        if b.get("name", "").endswith("_wheel_link")
    }
    assert len(wheel_zs) == 1, f"wheels are not coplanar under chassis_link: {wheel_zs}"

    radii = {
        float(g.get("size").split()[0])
        for root_ in _wheel_link_roots().values()
        for g in root_.iter("geom")
        if g.get("type") == "cylinder"
    }
    assert len(radii) == 1, f"wheels have differing collision radii: {radii}"
    return radii.pop() - wheel_zs.pop()


def test_keyframe_spawn_rests_on_the_heightfield():
    """The keyframe's z must be referenced to the terrain under it, not to the flat-plane
    clearance chassis_link's own pos.z carries from z=0. Getting this wrong spawns the
    wheels inside the heightfield on every hardware init and every sim reset, which
    nothing reports - the solver just shoves the base out right before an open-loop
    dead-reckon run starts."""
    keys = list(ET.parse(HUSKY_SCENE).getroot().iter("key"))
    assert keys, f"{HUSKY_SCENE.name} has no <key> to check"
    drop = _chassis_to_wheel_bottom_m()

    for key in keys:
        x, y, z = (float(v) for v in key.get("qpos").split()[:3])
        clearance = (z - drop) - _hfield_elevation(x, y)
        # A cell is 2 cm across and the field is rough at that scale, so this is not
        # exact contact - but anything past half a cell means the z was derived from
        # the wrong datum rather than from the terrain.
        assert abs(clearance) < 0.01, (
            f"keyframe {key.get('name')!r} spawns the wheel bottoms {clearance:+.4f} m "
            f"from the terrain at ({x}, {y}) - set z to the local heightfield elevation "
            f"there plus {drop:.5f}"
        )
