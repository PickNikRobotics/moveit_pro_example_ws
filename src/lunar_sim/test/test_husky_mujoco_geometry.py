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
CLAUDE.md (an armature/kv time constant at or above the timestep silently pins the
wheels - see hangar_sim's mecanum base incident). Nothing else in the build checks
either; both failure modes are quiet (no crash, the base just doesn't move, or the
model loads with wheels floating above/through the ground).

Wheel *placement* (x/y/z body offsets) is deliberately not cross-checked here: deriving
it from the vendored xacro requires evaluating a per-side conditional expression
(`suspension_beam.urdf.xacro`'s `spacer_offset`), which a regex can't do without
duplicating that logic (and getting it subtly wrong is worse than not checking it) -
see husky_a300.xml's own derivation comment for that FK chain instead.

No simulator, no ROS - pure XML/text parsing of the description sources, mirroring
hangar_sim/test/test_base_geometry.py.
"""

import re
from pathlib import Path

import pytest
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
    match = re.search(
        rf'<xacro:property\s+name="{name}"\s+value="([-0-9.eE]+)"', source.read_text()
    )
    assert match, f"{source.name} has no numeric xacro:property named {name!r}"
    return float(match.group(1))


def _outdoor_wheel_radius_width() -> tuple[float, float]:
    outdoor = CLEARPATH_A300_URDF / "drivetrain" / "wheels" / "outdoor.urdf.xacro"
    return (
        _xacro_property(outdoor, "a300_outdoor_wheel_radius"),
        _xacro_property(outdoor, "a300_outdoor_wheel_width"),
    )


def test_wheel_collision_cylinder_matches_vendored_radius_and_width():
    radius, width = _outdoor_wheel_radius_width()
    for wheel_file in DESCRIPTION.glob("*_wheel_link.xml"):
        match = re.search(
            r'type="cylinder"\s+size="([-0-9.eE]+) ([-0-9.eE]+)"',
            wheel_file.read_text(),
        )
        assert match, f"{wheel_file.name} has no cylinder collision geom"
        got_radius, got_half_width = float(match.group(1)), float(match.group(2))
        assert got_radius == pytest.approx(radius, abs=TOLERANCE_M), wheel_file.name
        assert got_half_width == pytest.approx(
            width / 2, abs=TOLERANCE_M
        ), wheel_file.name


def test_velocity_actuators_stay_below_the_timestep():
    """See CLAUDE.md: "Velocity actuators: armature/kv time-constant must stay below the
    timestep." A violation pins the wheels without any error - this is the only check
    for it."""
    scene_text = HUSKY_SCENE.read_text()
    timestep_match = re.search(r'<option[^>]*\btimestep="([-0-9.eE]+)"', scene_text)
    assert timestep_match, f"{HUSKY_SCENE.name} has no <option timestep=...>"
    timestep = float(timestep_match.group(1))

    a300_text = HUSKY_A300.read_text()
    armatures = {}
    for wheel_file in DESCRIPTION.glob("*_wheel_link.xml"):
        joint_text = wheel_file.read_text()
        match = re.search(
            r'<joint\s+name="(\w+_wheel_joint)"[^/]*armature="([-0-9.eE]+)"',
            joint_text,
            re.DOTALL,
        )
        assert match, f"{wheel_file.name} has no wheel joint with an armature"
        armatures[match.group(1)] = float(match.group(2))

    kvs = {
        m.group(1): float(m.group(2))
        for m in re.finditer(
            r'joint="(\w+_wheel_joint)"[^/]*kv="([-0-9.eE]+)"', a300_text, re.DOTALL
        )
    }
    assert armatures and armatures.keys() == kvs.keys(), (armatures, kvs)
    for joint, armature in armatures.items():
        tau = armature / kvs[joint]
        assert tau < timestep, (
            f"{joint}: armature/kv={tau}s is not below timestep={timestep}s - "
            "this wheel will not respond to velocity commands (CLAUDE.md)"
        )
