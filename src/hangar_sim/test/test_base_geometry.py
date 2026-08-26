#!/usr/bin/env python3

# Copyright 2026 PickNik Inc.
# All rights reserved.
#
# Unauthorized copying of this code base via any medium is strictly prohibited.
# Proprietary and confidential.

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
WHEEL_INCLUDE = DESCRIPTION / "front_left_wheel_link.xml"
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

WHEELS = (
    "front_left_wheel_link",
    "front_right_wheel_link",
    "rear_left_wheel_link",
    "rear_right_wheel_link",
)


def _xacro_property(source: Path, name: str) -> float:
    match = re.search(
        rf'<xacro:property\s+name="{name}"\s+value="([-0-9.eE]+)"', source.read_text()
    )
    assert match, f"{source.name} has no numeric xacro:property named {name!r}"
    return float(match.group(1))


def _base_anchor_z() -> float:
    """z of the joint that hangs the whole robot off the world frame."""
    match = re.search(
        r'<joint name="linear_x_joint".*?<origin xyz="[-0-9.eE]+ [-0-9.eE]+ '
        r'([-0-9.eE]+)"',
        XACRO.read_text(),
        re.DOTALL,
    )
    assert match, "linear_x_joint has no parsable origin in ur5e_ridgeback.xacro"
    return float(match.group(1))


def _urdf_wheel_height() -> float:
    """World z of a wheel axle, walked down the URDF chain.

    world -> ... -> chassis_link is the base anchor; chassis_link -> axle_link is
    `axle_offset`; the rocker and wheel joints below it are planar in the vendored
    description, so any z they gain is a change this test should catch.
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
        r'<joint name="\$\{prefix\}_rocker".*?<origin xyz="[^"]*?\s([-0-9.eE]+)"',
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
    text = WHEEL_INCLUDE.read_text()
    radii = {float(m) for m in re.findall(r'<geom\s+size="([-0-9.eE]+)"', text)}
    assert len(radii) == 1, f"expected one sphere radius, found {sorted(radii)}"
    centres = {
        round(math.hypot(float(x), float(z)), 9)
        for x, _, z in re.findall(
            r'pos="([-0-9.eE]+)\s+([-0-9.eE]+)\s+([-0-9.eE]+)"', text
        )
    }
    ring = max(centres)
    count = len(re.findall(r"<geom\s+size=", text))
    assert ring > 0 and count > 2, "could not read the roller ring from the wheel"
    return ring, radii.pop(), count


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
    assert ring + radius == pytest.approx(declared, abs=1e-5), (
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
        r'<joint name="ham_assem_joint".*?<origin xyz="[-0-9.eE]+ [-0-9.eE]+ '
        r'([-0-9.eE]+)"',
        XACRO.read_text(),
        re.DOTALL,
    )
    mjcf = re.search(
        r'<body name="ham_assem" pos="[-0-9.eE]+ [-0-9.eE]+ ([-0-9.eE]+)"',
        MJCF.read_text(),
    )
    assert urdf and mjcf, "could not read the ham_assem mount from both models"
    assert float(urdf.group(1)) == pytest.approx(
        float(mjcf.group(1)), abs=TOLERANCE_M
    ), (
        f"the arm mounts at {float(urdf.group(1)):.6f} m in the URDF and "
        f"{float(mjcf.group(1)):.6f} m in the MJCF, so every arm frame is offset "
        f"between TF and the physics."
    )
