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

"""The kinematic model and the physics model must place the wheels identically.

They are authored in separate files — the URDF chain in `ur5e_ridgeback.xacro`
plus the vendored `ridgeback_description`, and the MJCF bodies in
`ur5e_ridgeback.xml` — with nothing tying the two together. When they drift, TF,
the nav footprint and collision checking describe a robot the simulator is not
running, and nothing fails loudly: only the wheels carry collision geometry
against the ground plane, so MuJoCo stays happy while the rendered robot sinks.

The checks read the description sources directly rather than expanding the
xacro, which would need the launch's argument set (`visual_parameters_file` and
friends) and would tie these assertions to launch configuration they do not care
about. No simulator, no ROS.
"""

import math
import re
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
from ament_index_python.packages import get_package_share_directory

DESCRIPTION = Path(__file__).resolve().parent.parent / "description"
MJCF = DESCRIPTION / "ur5e_ridgeback.xml"
XACRO = DESCRIPTION / "ur5e_ridgeback.xacro"
RIDGEBACK_XACRO = (
    Path(get_package_share_directory("ridgeback_description"))
    / "urdf"
    / "ridgeback.urdf.xacro"
)

# Slack for decimal rounding between the two file formats, not a physical tolerance.
# It has to stay well under the roller ring's 0.811 mm ripple, or the check cannot tell a
# correct anchor from one set to the peak radius.
TOLERANCE_M = 1e-5

# Where HAM_Assem.stl begins along its mounting axis, measured off the mesh. The mount
# rotation maps the mesh's local +y to world +z, so the plate lands this far above the
# joint origin.
MESH_BASE_OFFSET_M = 0.00635

WHEELS = (
    "front_left_wheel_link",
    "front_right_wheel_link",
    "rear_left_wheel_link",
    "rear_right_wheel_link",
)

# One MJCF include per wheel, each holding that wheel's ring of roller spheres.
WHEEL_INCLUDES = tuple(DESCRIPTION / f"{wheel}.xml" for wheel in WHEELS)


def _xacro_property(source: Path, name: str) -> float:
    match = re.search(
        rf'<xacro:property\s+name="{name}"\s+value="([-0-9.eE]+)"', source.read_text()
    )
    assert match, f"{source.name} has no numeric xacro:property named {name!r}"
    return float(match.group(1))


def _base_anchor_z() -> float:
    """z of the joint that hangs the whole robot off the world frame."""
    match = re.search(
        r'<joint name="linear_x_joint"[^>]*>(?:(?!</joint>).)*?'
        r'<origin xyz="[-0-9.eE]+ [-0-9.eE]+ ([-0-9.eE]+)"',
        XACRO.read_text(),
        re.DOTALL,
    )
    assert match, "linear_x_joint has no parsable origin in ur5e_ridgeback.xacro"
    return float(match.group(1))


def _urdf_wheel_height() -> float:
    """World z of a wheel axle, walked down the URDF chain.

    `linear_x_joint` carries the whole robot's height; the joints between it and
    `chassis_link` are planar, which `test_base_chain_is_otherwise_planar` holds to.
    Below that, chassis -> axle is `axle_offset` and the rocker and wheel joints add
    whatever z the vendored description gives them.
    """
    rocker_z, wheel_z = _rocker_and_wheel_z()
    return (
        _base_anchor_z()
        + _xacro_property(RIDGEBACK_XACRO, "axle_offset")
        + rocker_z
        + wheel_z
    )


def _rocker_and_wheel_z() -> tuple[float, float]:
    text = RIDGEBACK_XACRO.read_text()
    rocker = re.search(
        r'<joint name="\$\{prefix\}_rocker"[^>]*>(?:(?!</joint>).)*?'
        r'<origin xyz="[^"]*?\s([-0-9.eE]+)"',
        text,
        re.DOTALL,
    )
    wheel = re.search(
        r'<xacro:wheel prefix="\$\{prefix\}" side="left">\s*'
        r'<origin xyz="[^"]*?\s([-0-9.eE]+)"',
        text,
        re.DOTALL,
    )
    assert rocker and wheel, "could not read the rocker/wheel origins from ridgeback"
    return float(rocker.group(1)), float(wheel.group(1))


def _roller_ring() -> tuple[float, float, int]:
    """Ring radius, sphere radius and sphere count of one mecanum roller ring.

    The ring is spheres, not a cylinder, so it has no single radius. Callers pick
    the one their question needs — see the comment in `hangar_scene.xml`.
    """
    rings = set()
    for source in WHEEL_INCLUDES:
        text = source.read_text()
        radii = {float(m) for m in re.findall(r'<geom\s+size="([-0-9.eE]+)"', text)}
        assert len(radii) == 1, f"{source.name}: sphere radii differ, {sorted(radii)}"
        centres = {
            round(math.hypot(float(x), float(z)), 9)
            for x, _, z in re.findall(
                r'pos="([-0-9.eE]+)\s+([-0-9.eE]+)\s+([-0-9.eE]+)"', text
            )
        }
        count = len(re.findall(r"<geom\s+size=", text))
        assert max(centres) > 0 and count > 2, f"{source.name}: no roller ring found"
        rings.add((round(max(centres), 9), radii.pop(), count))
    # One wheel standing in for four is only sound while they are the same wheel.
    assert len(rings) == 1, f"the four wheels carry different rollers: {sorted(rings)}"
    return rings.pop()


def _static_ride_height() -> float:
    """Distance from the axle to the floor once the roller ring has settled.

    Between spheres the envelope drops to `a*cos(pi/N) + r`; the peak `a + r` is
    reached only N times per revolution. The base has no z DOF, so anchoring to the
    peak leaves the wheel clear of the floor at every other angle.
    """
    ring, radius, count = _roller_ring()
    return ring * math.cos(math.pi / count) + radius


def _mjcf_wheel_heights() -> dict[str, float]:
    """World z of each wheel body, accumulated down the MJCF body nesting."""
    heights: dict[str, float] = {}

    def walk(element: ET.Element, z: float) -> None:
        for body in element.findall("body"):
            body_z = z + float(body.get("pos", "0 0 0").split()[2])
            if body.get("name", "") in WHEELS:
                heights[body.get("name", "")] = body_z
            walk(body, body_z)

    worldbody = ET.parse(MJCF).getroot().find("worldbody")
    assert worldbody is not None, f"{MJCF.name} has no <worldbody>"
    walk(worldbody, 0.0)
    return heights


def test_base_chain_is_otherwise_planar() -> None:
    """`_urdf_wheel_height` reads one joint's z; the rest of the chain must add none."""
    text = XACRO.read_text() + RIDGEBACK_XACRO.read_text()
    for joint in ("linear_y_joint", "rotational_yaw_joint", "base_link_joint"):
        match = re.search(
            rf'<joint name="{joint}"[^>]*>(?:(?!</joint>).)*?'
            rf'<origin xyz="[-0-9.eE]+ [-0-9.eE]+ ([-0-9.eE]+)"',
            text,
            re.DOTALL,
        )
        if match is None:
            continue  # no origin means the identity, which is what we want
        assert float(match.group(1)) == pytest.approx(0.0, abs=TOLERANCE_M), (
            f"{joint} now adds {float(match.group(1)):+.6f} m of height, which the "
            f"wheel-height walk does not account for."
        )


def _rotation_of(body: ET.Element) -> str | None:
    """The body's orientation, or None when it is the identity however it is written."""
    identities = {
        "euler": (0.0, 0.0, 0.0),
        "quat": (1.0, 0.0, 0.0, 0.0),
        "zaxis": (0.0, 0.0, 1.0),
        "xyaxes": (1.0, 0.0, 0.0, 0.0, 1.0, 0.0),
    }
    for attr, identity in identities.items():
        raw = body.get(attr)
        if raw is None:
            continue
        values = tuple(float(v) for v in raw.split())
        if len(values) != len(identity) or any(
            abs(v - i) > 1e-9 for v, i in zip(values, identity)
        ):
            return f'{attr}="{raw}"'
    return None


def test_no_rotation_above_the_wheels() -> None:
    """The MJCF height walk sums `pos` only, so a rotated parent would invalidate it."""

    def walk(element: ET.Element) -> None:
        for body in element.findall("body"):
            name = body.get("name", "")
            if name in WHEELS:
                continue
            rotation = _rotation_of(body)
            if rotation is not None and any(
                child.get("name", "") in WHEELS for child in body.iter("body")
            ):
                raise AssertionError(
                    f"{name} sits above a wheel and carries {rotation}; summing pos z "
                    f"down this chain no longer gives a world height."
                )
            walk(body)

    worldbody = ET.parse(MJCF).getroot().find("worldbody")
    assert worldbody is not None
    walk(worldbody)


def test_mjcf_describes_every_wheel() -> None:
    """A wheel missing from the MJCF would make the comparisons below vacuous."""
    assert sorted(_mjcf_wheel_heights()) == sorted(WHEELS)


@pytest.mark.parametrize("wheel", WHEELS)
def test_urdf_and_mjcf_agree_on_wheel_height(wheel: str) -> None:
    """The two models must not drift apart, whatever height they agree on."""
    urdf_z = _urdf_wheel_height()
    mjcf_z = _mjcf_wheel_heights()[wheel]
    assert urdf_z == pytest.approx(mjcf_z, abs=TOLERANCE_M), (
        f"{wheel}: the URDF chain puts the axle at {urdf_z:.4f} m and the MJCF at "
        f"{mjcf_z:.4f} m. TF and collision checking follow the URDF while the "
        f"physics follows the MJCF."
    )


def test_urdf_wheel_radius_matches_the_rollers() -> None:
    """The URDF cylinder stands in for the MJCF spheres; it must match their peak."""
    ring, radius, _ = _roller_ring()
    declared = _xacro_property(RIDGEBACK_XACRO, "wheel_radius")
    assert ring + radius == pytest.approx(declared, abs=TOLERANCE_M), (
        f"the roller ring reaches {ring + radius:.7f} m but the URDF collision cylinder is "
        f"{declared:.7f} m. Re-sizing the rollers without updating the URDF leaves "
        f"the two models describing different wheels."
    )


def test_wheels_rest_on_the_floor() -> None:
    """Taken from the MJCF rollers, so a change to them alone still fails."""
    contact = _urdf_wheel_height() - _static_ride_height()
    assert contact == pytest.approx(0.0, abs=TOLERANCE_M), (
        f"the wheels contact z = {contact:+.4f} m rather than the floor. Negative "
        f"means the robot is modelled sunk into the ground plane; positive means it "
        f"floats — and with no z DOF on the base, nothing settles it."
    )


def test_arm_mount_agrees_between_the_models() -> None:
    """The 5 mm that matters: an offset here lands at the gripper."""
    urdf = re.search(
        r'<joint name="ham_assem_joint"[^>]*>(?:(?!</joint>).)*?'
        r'<origin xyz="[-0-9.eE]+ [-0-9.eE]+ ([-0-9.eE]+)"',
        XACRO.read_text(),
        re.DOTALL,
    )
    mjcf = re.search(
        r'<body name="ham_assem" pos="[-0-9.eE]+ [-0-9.eE]+ ([-0-9.eE]+)"',
        MJCF.read_text(),
    )
    assert urdf and mjcf, "could not read the ham_assem mount from both models"
    urdf_z = float(urdf.group(1))
    assert urdf_z == pytest.approx(float(mjcf.group(1)), abs=TOLERANCE_M), (
        f"the arm mounts at {urdf_z:.6f} m in the URDF and "
        f"{float(mjcf.group(1)):.6f} m in the MJCF, so every arm frame is offset "
        f"between TF and the physics."
    )

    # Agreeing on a wrong height would still pass the check above, so pin the mount to
    # the thing it exists to sit on.
    deck = _xacro_property(RIDGEBACK_XACRO, "deck_height")
    assert urdf_z + MESH_BASE_OFFSET_M == pytest.approx(deck, abs=TOLERANCE_M), (
        f"the arm plate lands at {urdf_z + MESH_BASE_OFFSET_M:.6f} m rather than on "
        f"the {deck:.6f} m deck."
    )
