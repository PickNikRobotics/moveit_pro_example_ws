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

import ast
import functools
import math
import struct
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest
import yaml
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


def _mjcf_elements(source: Path, tag: str) -> dict[str, ET.Element]:
    """Every named <tag> in an MJCF fragment, keyed by name."""
    return {
        el.get("name"): el
        for el in ET.parse(source).getroot().iter(tag)
        if el.get("name")
    }


def _vec(element: ET.Element, attr: str) -> np.ndarray:
    value = element.get(attr)
    assert value is not None, f"{element.get('name')!r} has no {attr!r} attribute"
    return np.array([float(v) for v in value.split()])


def _urdf_joint_origin_xyz(source: Path, joint_suffix: str) -> np.ndarray:
    """The xyz of a fixed joint's <origin> in a vendored URDF/xacro file.

    Matched by name suffix, not equality: these files are unexpanded xacro macros, so the
    joint is literally named "${name}_front_camera_mount_joint" and only becomes
    "amp_sensor_arch_..." once the macro is invoked. Parsing the raw file is deliberate -
    it keeps this cross-check free of xacro, ROS and the invoking config."""
    matches = [
        joint
        for joint in ET.parse(source).getroot().iter("joint")
        if (joint.get("name") or "").endswith(joint_suffix)
    ]
    assert matches, f"{source.name} has no joint whose name ends with {joint_suffix!r}"
    assert len(matches) == 1, (
        f"{source.name} has {len(matches)} joints ending in {joint_suffix!r}: "
        f"{[j.get('name') for j in matches]} - the suffix no longer identifies one mount"
    )
    joint = matches[0]
    origin = joint.find("origin")
    assert origin is not None, f"{joint.get('name')} has no <origin>"
    rpy = origin.get("rpy", "0 0 0")
    assert [float(v) for v in rpy.split()] == [0.0, 0.0, 0.0], (
        f"{joint.get('name')} is no longer axis-aligned (rpy={rpy!r}); the OAK-D cameras in "
        "husky_a300.xml assume an unrotated mount and would need their xyaxes reworked"
    )
    return np.array([float(v) for v in origin.get("xyz").split()])


# The OAK-D Pro's published stereo baseline, and the profile hangar_sim's forward stereo
# pair uses. Shared between the two configs so captures stay comparable.
OAKD_PRO_BASELINE_M = 0.075
OAKD_PRO_FOVY_DEG = 50.53401584672457


def test_oakd_pose_matches_the_vendored_front_camera_mount():
    """The OAK-D sits on the real A300 Observer arch's own front camera mount, so its pose is
    the composition of the arch geom's offset from chassis_link with upstream's
    amp_sensor_arch_front_camera_mount joint origin. If Clearpath moves that mount, this fails
    rather than leaving the config quietly describing a camera the real robot does not have
    there."""
    arch = _mjcf_elements(HUSKY_A300, "geom")["sensor_arch"]
    mount_offset = _urdf_joint_origin_xyz(
        CLEARPATH_A300_URDF / "attachments" / "amp_sensor_arch.urdf.xacro",
        "_front_camera_mount_joint",
    )
    expected = _vec(arch, "pos") + mount_offset

    cameras = _mjcf_elements(HUSKY_A300, "camera")
    assert np.allclose(
        _vec(cameras["oakd_color"], "pos"), expected, atol=TOLERANCE_M
    ), (
        f"oakd_color is at {_vec(cameras['oakd_color'], 'pos')}, but the vendored arch puts its "
        f"front camera mount at {expected}"
    )


def test_oakd_stereo_pair_straddles_the_colour_camera_at_the_pro_baseline():
    """The mono pair is what visual odometry consumes, so its separation has to be the real
    device's baseline and its optical centres have to stay coplanar with the colour camera -
    a pair that is merely near the right place yields a plausible-looking but wrong depth
    scale, which nothing downstream can detect."""
    cameras = _mjcf_elements(HUSKY_A300, "camera")
    left = _vec(cameras["oakd_left"], "pos")
    right = _vec(cameras["oakd_right"], "pos")
    colour = _vec(cameras["oakd_color"], "pos")

    assert np.isclose(
        np.linalg.norm(left - right), OAKD_PRO_BASELINE_M, atol=TOLERANCE_M
    ), f"stereo baseline is {np.linalg.norm(left - right)}, expected {OAKD_PRO_BASELINE_M}"
    # Left is the +Y side: base axes are +X forward, +Y left.
    assert left[1] > right[1], "oakd_left must sit on the +Y (left) side of oakd_right"
    for name in ("oakd_left", "oakd_right"):
        pos = _vec(cameras[name], "pos")
        assert np.allclose(
            pos[[0, 2]], colour[[0, 2]], atol=TOLERANCE_M
        ), f"{name} is not coplanar with oakd_color: x/z {pos[[0, 2]]} vs {colour[[0, 2]]}"
        assert np.isclose(
            float(cameras[name].get("fovy")), OAKD_PRO_FOVY_DEG
        ), f"{name} fovy must match the OAK-D Pro profile shared with hangar_sim"
        assert cameras[name].get("user") == "1", f"{name} must be RGB-only"


def test_oakd_geoms_stay_behind_the_optical_plane():
    """Group 2 is what MuJoCo's offscreen pass renders, so any part of the device's own housing
    placed in front of the optical centres would occlude all three camera images. The cameras
    look along base +X, so every oakd_* geom must stay at or behind their x."""
    optical_x = _vec(_mjcf_elements(HUSKY_A300, "camera")["oakd_color"], "pos")[0]

    checked = 0
    for name, geom in _mjcf_elements(HUSKY_A300, "geom").items():
        if not name.startswith("oakd_"):
            continue
        checked += 1
        if geom.get("type") == "box":
            front_x = _vec(geom, "pos")[0] + _vec(geom, "size")[0]
        else:
            fromto = _vec(geom, "fromto")
            front_x = max(fromto[0], fromto[3])
        assert front_x <= optical_x + TOLERANCE_M, (
            f"{name} reaches x={front_x}, in front of the optical plane at x={optical_x}; "
            "it would occlude the OAK-D images"
        )
    assert checked, "no oakd_* geoms found - the mount geometry went missing"


def test_every_fixed_camera_has_an_optical_frame_site():
    """picknik_mujoco_ros/MujocoSystem requires a <camera>_optical_frame site for every
    fixed-mode camera in the model, and publishes all of them once render_publish_rate is
    non-zero. A missing site is a runtime failure, not a load-time one."""
    sites = set(_mjcf_elements(HUSKY_SCENE, "site")) | set(
        _mjcf_elements(HUSKY_A300, "site")
    )
    for source in (HUSKY_A300, HUSKY_SCENE):
        for name, camera in _mjcf_elements(source, "camera").items():
            # targetbody cameras are render-only and exempt; see husky_scene.xml's chase_camera.
            if camera.get("mode", "fixed") != "fixed":
                continue
            assert (
                f"{name}_optical_frame" in sites
            ), f"fixed camera {name!r} in {source.name} has no {name}_optical_frame site"


def test_far_horizon_stays_below_the_driven_ground():
    """The far-field horizon hfield collides, so the rover can leave by the access road, and passes underneath the drivable terrain (the centre and the band around it), so inside that square footprint it must stay below its floor.
    If it rises above, it pokes up through the ground the robot drives on - which reads as terrain,
    not as a bug, so nothing else would catch it.
    """
    hfields = _mjcf_elements(HUSKY_SCENE, "hfield")
    geoms = _mjcf_elements(HUSKY_SCENE, "geom")

    _, _, near_elevation_z, _ = _vec(hfields["lunar_hfield"], "size")
    near_floor_z = _vec(geoms["ground_plane"], "pos")[2]
    # The far horizon hides under the whole drivable square, the cratered centre plus the band; the
    # access road is covered by test_far_horizon_stays_below_the_drawn_band.
    near_half_m = max(
        max(
            abs(_vec(g, "pos")[0]) + _vec(hfields[g.get("hfield")], "size")[0],
            abs(_vec(g, "pos")[1]) + _vec(hfields[g.get("hfield")], "size")[1],
        )
        for g in _ring_geoms().values()
    )

    far_half_m, _, far_elevation_z, _ = _vec(hfields["lunar_far_hfield"], "size")
    far_base_z = _vec(geoms["far_horizon"], "pos")[2]

    assert geoms["far_horizon"].get("contype", "1") != "0", "far horizon must collide"
    assert (
        geoms["far_horizon"].get("conaffinity", "1") != "0"
    ), "far horizon must collide"

    png = (
        DESCRIPTION / "assets" / hfields["lunar_far_hfield"].get("file").split("/")[-1]
    )
    with Image.open(png) as image:
        # MuJoCo reads row 0 at the top; generate_far_terrain.py writes the grid flipped to
        # match, so undo that here to get world-axis-aligned rows.
        # 16-bit full scale, flipped on write. Divide by 65535, not the numpy dtype max: PIL
        # returns a 16-bit PNG as mode "I" in an int32 container, which would flatten every
        # height to the base.
        values01 = np.flipud(np.asarray(image).astype(np.float64)) / 65535.0
    world_z = far_base_z + values01 * far_elevation_z

    axis = np.linspace(-far_half_m, far_half_m, world_z.shape[0])
    chebyshev = np.maximum(np.abs(axis[None, :]), np.abs(axis[:, None]))
    cell_m = 2.0 * far_half_m / world_z.shape[0]
    inside = chebyshev < near_half_m - cell_m

    assert inside.any(), "far field does not overlap the driven terrain's footprint"
    assert world_z[inside].max() < near_floor_z, (
        f"far horizon rises to {world_z[inside].max():.3f} m inside the driven terrain's "
        f"footprint, at or above its {near_floor_z:.3f} m floor"
    )
    # And it has to actually reach the rim, or a trench rings the driven terrain.
    assert (
        world_z.max() > near_floor_z + near_elevation_z
    ), "far horizon never rises above the driven terrain, so it contributes no skyline"


def _far_z(gx: np.ndarray, gy: np.ndarray) -> np.ndarray:
    """The far horizon's world z at (gx, gy), bilinear between the samples MuJoCo loads."""
    hfields = _mjcf_elements(HUSKY_SCENE, "hfield")
    geoms = _mjcf_elements(HUSKY_SCENE, "geom")
    far_half_m, _, far_elevation_z, _ = _vec(hfields["lunar_far_hfield"], "size")
    far_base_z = _vec(geoms["far_horizon"], "pos")[2]
    with Image.open(
        DESCRIPTION / "assets" / hfields["lunar_far_hfield"].get("file").split("/")[-1]
    ) as image:
        # MuJoCo keeps 8 bits per hfield sample, so compare against what it loads.
        raw = np.flipud(np.asarray(image).astype(np.float64))
    full = 65535.0 if raw.max() > 255 else 255.0
    values01 = np.round(raw / full * 255.0) / 255.0
    far = far_base_z + values01 * far_elevation_z
    far_axis = np.linspace(-far_half_m, far_half_m, far.shape[0])
    # MuJoCo splits each 2.35 m cell into triangles instead, which differs by up to ~2 cm mid-cell.
    fx = np.interp(gx, far_axis, np.arange(far_axis.size))
    fy = np.interp(gy, far_axis, np.arange(far_axis.size))
    j, i = np.floor(fx).astype(int), np.floor(fy).astype(int)
    tx, ty = fx - j, fy - i
    return (
        far[i, j] * (1 - tx) * (1 - ty)
        + far[i, j + 1] * tx * (1 - ty)
        + far[i + 1, j] * (1 - tx) * ty
        + far[i + 1, j + 1] * tx * ty
    )


def test_far_horizon_stays_below_the_drawn_band():
    """Where the band is drawn, the far horizon must stay under it: the far field rises to meet the
    band's edge berm, and rising a little early shows it through the drawn ground."""
    for name in (n for n in _band_geoms() if n.endswith("_visual")):
        drawn, (x0, x1, y0, y1) = _hfield_world_z(name)
        gx, gy = np.meshgrid(
            np.linspace(x0, x1, drawn.shape[1]), np.linspace(y0, y1, drawn.shape[0])
        )
        poke = (_far_z(gx, gy) - drawn).max()
        assert poke <= 0.0, f"the far horizon rises {poke * 100:.1f} cm through {name}"


ROAD_PREFIX = "lunar_band_road"


def _band_generator_constant(name: str) -> float:
    """A numeric constant from generate_band_terrain.py, or the constant it names, read without importing it (it needs scipy)."""
    tree = ast.parse((DESCRIPTION / "generate_band_terrain.py").read_text())
    for node in tree.body:
        if isinstance(node, ast.Assign) and any(
            getattr(t, "id", None) == name for t in node.targets
        ):
            if isinstance(node.value, ast.Name):
                return _band_generator_constant(node.value.id)
            return float(ast.literal_eval(node.value))
    raise AssertionError(f"generate_band_terrain.py has no {name}")


ROAD_WALL_M = _band_generator_constant(
    "RIM_BERM_M"
)  # each wall's width, foot to footprint edge
ROAD_END_FACE_M = _band_generator_constant("ROAD_END_FACE_M")  # each wall's end face


def _band_geoms() -> dict[str, ET.Element]:
    """The band's hfield geoms, from generate_band_terrain.py's include."""
    geoms = _mjcf_elements(DESCRIPTION / "band_geoms_generated.xml", "geom")
    assert geoms, "band_geoms_generated.xml has no geoms"
    return geoms


def _ring_geoms() -> dict[str, ET.Element]:
    """The band strips around the centre, without the access road through the north berm."""
    return {n: g for n, g in _band_geoms().items() if not n.startswith(ROAD_PREFIX)}


def test_band_collision_strips_are_hidden_and_visual_strips_do_not_collide():
    """The band follows the centre's split: 2 cm collision strips in group 3, drawn by coarser
    strips that never collide. A collision strip that renders brings back the cost the centre's
    split removed; a visual strip that collides puts a second, slightly different ground under the
    wheels."""
    hfields = _mjcf_elements(HUSKY_SCENE, "hfield")
    collision = {n: g for n, g in _band_geoms().items() if not n.endswith("_visual")}
    visual = {n: g for n, g in _band_geoms().items() if n.endswith("_visual")}
    assert len(collision) == 5, sorted(collision)  # four sides and the access road
    assert len(visual) == 9, sorted(
        visual
    )  # four sides, four corners and the access road
    for name, geom in collision.items():
        assert (
            geom.get("group") == "3"
        ), f"{name} must stay in group 3 so it is not rendered"
        assert geom.get("contype") == "1", name
        assert geom.get("conaffinity") == "1", name
        assert name in hfields, f"{name} has no <hfield> in husky_scene.xml"
    for name, geom in visual.items():
        assert geom.get("contype") == "0", name
        assert geom.get("conaffinity") == "0", name
        assert geom.get("group") == "2", name


def test_band_strips_tile_the_ring_around_the_centre():
    """Each set of strips covers the 10-15 m ring exactly once, so there are neither holes the
    rover falls through nor overlaps where two grounds fight."""
    hfields = _mjcf_elements(HUSKY_SCENE, "hfield")
    inner = _vec(hfields["lunar_hfield"], "size")[0]
    for drawn in (False, True):
        boxes = []
        for name, geom in _ring_geoms().items():
            if name.endswith("_visual") != drawn:
                continue
            hx, hy = _vec(hfields[geom.get("hfield")], "size")[:2]
            x, y = _vec(geom, "pos")[:2]
            boxes.append((x - hx, x + hx, y - hy, y + hy))
        outer = max(max(-x0, x1, -y0, y1) for x0, x1, y0, y1 in boxes)
        # Cell centres sit off every strip edge, so each is strictly inside or outside a strip.
        axis = np.arange(-outer + 0.125, outer, 0.25)
        gx, gy = np.meshgrid(axis, axis)
        hits = sum(
            ((gx > x0) & (gx < x1) & (gy > y0) & (gy < y1)).astype(int)
            for x0, x1, y0, y1 in boxes
        )
        ring = np.maximum(np.abs(gx), np.abs(gy)) > inner
        kind = "visual" if drawn else "collision"
        assert (hits[ring] == 1).all(), (
            f"{kind} strips leave {(hits[ring] == 0).sum()} ring cells uncovered and "
            f"cover {(hits[ring] > 1).sum()} twice"
        )
        assert (hits[~ring] == 0).all(), f"a {kind} strip overlaps the centre"


def test_access_road_strips_continue_the_north_band():
    """The access road's strips start on the north band's outer edge, inside its span, so the road
    continues the band's ground with neither a gap the rover falls through nor an overlap.
    """
    hfields = _mjcf_elements(HUSKY_SCENE, "hfield")
    for suffix in ("", "_visual"):
        north = _band_geoms()[f"lunar_band_n{suffix}"]
        road = _band_geoms()[f"{ROAD_PREFIX}{suffix}"]
        nx, ny = _vec(north, "pos")[:2]
        nhx, nhy = _vec(hfields[north.get("hfield")], "size")[:2]
        rx, ry = _vec(road, "pos")[:2]
        rhx, rhy = _vec(hfields[road.get("hfield")], "size")[:2]
        assert ry - rhy == pytest.approx(
            ny + nhy, abs=1e-3
        ), f"{ROAD_PREFIX}{suffix} does not start on the band's edge"
        assert (
            nx - nhx <= rx - rhx
        ), f"{ROAD_PREFIX}{suffix} reaches past the north band"
        assert (
            rx + rhx <= nx + nhx
        ), f"{ROAD_PREFIX}{suffix} reaches past the north band"


def test_access_road_strips_meet_the_north_band_without_a_step():
    """Each road strip's first row is the north strip's last over the road's span, so a wheel driving
    out through the opening meets no step and the drawn ground shows no crack."""
    hfields = _mjcf_elements(HUSKY_SCENE, "hfield")
    for suffix in ("", "_visual"):
        north, (nx0, nx1, _, _) = _hfield_world_z(f"lunar_band_n{suffix}")
        road, (rx0, rx1, _, _) = _hfield_world_z(f"{ROAD_PREFIX}{suffix}")
        along = np.linspace(rx0, rx1, road.shape[1])
        expected = np.interp(along, np.linspace(nx0, nx1, north.shape[1]), north[-1])
        tolerance = (
            _vec(hfields[f"lunar_band_n{suffix}"], "size")[2]
            + _vec(hfields[f"{ROAD_PREFIX}{suffix}"], "size")[2]
        ) / 2 / 255 + 1e-4
        worst = np.abs(road[0] - expected).max()
        assert (
            worst < tolerance
        ), f"{ROAD_PREFIX}{suffix} steps {worst * 100:.1f} cm from the north band"


@functools.lru_cache(maxsize=None)
def _hfield_world_z(name: str) -> tuple[np.ndarray, tuple[float, float, float, float]]:
    """World z [row=y from -half, col=x from -half] of a scene hfield, and its geom's
    (x0, x1, y0, y1) extent."""
    hfields = _mjcf_elements(HUSKY_SCENE, "hfield")
    geoms = {**_mjcf_elements(HUSKY_SCENE, "geom"), **_band_geoms()}
    geom = next(g for g in geoms.values() if g.get("hfield") == name)
    hx, hy, elevation_z, _ = _vec(hfields[name], "size")
    gx, gy, gz = _vec(geom, "pos")
    with Image.open(DESCRIPTION / "assets" / hfields[name].get("file")) as image:
        raw = np.asarray(image).astype(np.float64)
    full = 65535.0 if raw.max() > 255 else 255.0
    # MuJoCo keeps 8 bits per hfield sample, so compare against what it loads.
    z = gz + np.flipud(np.round(raw / full * 255.0) / 255.0) * elevation_z
    z.setflags(write=False)
    return z, (
        gx - hx,
        gx + hx,
        gy - hy,
        gy + hy,
    )


def test_band_collision_strips_meet_the_centre_rim():
    """Each collision strip's row on the 10 m edge carries the centre's own rim heights, so a wheel
    crossing the edge meets no step. A strip that takes the wrong edge's profile leaves a one-cell
    ridge or trench along the whole seam, which still tiles and still renders correctly.
    """
    centre, (x0, x1, _, _) = _hfield_world_z("lunar_hfield")
    inner = x1
    axis = np.linspace(x0, x1, centre.shape[1])
    hfields = _mjcf_elements(HUSKY_SCENE, "hfield")
    for side, (rim, fixed) in {
        "n": (centre[-1, :], "y"),
        "s": (centre[0, :], "y"),
        "e": (centre[:, -1], "x"),
        "w": (centre[:, 0], "x"),
    }.items():
        band, (bx0, bx1, by0, by1) = _hfield_world_z(f"lunar_band_{side}")
        # Both heightmaps are 8-bit: half of each one's step is how far apart they can be and agree.
        tolerance = (
            _vec(hfields["lunar_hfield"], "size")[2]
            + _vec(hfields[f"lunar_band_{side}"], "size")[2]
        ) / 2 / 255 + 1e-4
        if fixed == "y":
            row = band[0, :] if side == "n" else band[-1, :]
            along = np.linspace(bx0, bx1, band.shape[1])
        else:
            row = band[:, 0] if side == "e" else band[:, -1]
            along = np.linspace(by0, by1, band.shape[0])
        inside = np.abs(along) <= inner + 1e-6
        expected = np.interp(along[inside], axis, rim)
        worst = np.abs(row[inside] - expected).max()
        assert worst < tolerance, (
            f"lunar_band_{side}'s edge row departs from the centre's rim by {worst * 100:.1f} cm - "
            "re-run generate_band_terrain.py after regenerating the centre"
        )


def _stl_vertices(path: Path) -> np.ndarray:
    data = path.read_bytes()
    count = struct.unpack("<I", data[80:84])[0]
    return (
        np.frombuffer(
            data[84:],
            dtype=np.dtype([("n", "<3f4"), ("v", "<3f4", (3,)), ("a", "<u2")]),
            count=count,
        )["v"]
        .reshape(-1, 3)
        .astype(np.float64)
    )


def _landmarks() -> list[tuple[str, np.ndarray, np.ndarray, np.ndarray]]:
    """(name, pos, scale, mesh vertices) for each landmark rock, from the generated includes."""
    meshes = _mjcf_elements(DESCRIPTION / "landmarks_assets_generated.xml", "mesh")
    out = []
    for name, geom in _mjcf_elements(
        DESCRIPTION / "landmarks_geoms_generated.xml", "geom"
    ).items():
        mesh = meshes[geom.get("mesh")]
        verts = _stl_vertices(DESCRIPTION / "assets" / mesh.get("file"))
        out.append((name, _vec(geom, "pos"), _vec(mesh, "scale"), verts))
    assert out, "landmarks_geoms_generated.xml has no geoms"
    return out


def _ground_z(x: float, y: float) -> float:
    """Collision ground under (x, y), from whichever committed heightmap covers it."""
    for name in (
        "lunar_hfield",
        "lunar_band_n",
        "lunar_band_s",
        "lunar_band_e",
        "lunar_band_w",
        ROAD_PREFIX,
    ):
        z, (x0, x1, y0, y1) = _hfield_world_z(name)
        if x0 <= x <= x1 and y0 <= y <= y1:
            col = round((x - x0) / (x1 - x0) * (z.shape[1] - 1))
            row = round((y - y0) / (y1 - y0) * (z.shape[0] - 1))
            return float(z[row, col])
    raise AssertionError(f"({x:.2f}, {y:.2f}) is outside the drivable ground")


def _scan_planes_m() -> tuple[float, float]:
    """Front and rear lidar heights above the ground: chassis, lidar link and lidar camera z."""
    root = ET.parse(DESCRIPTION / "husky_a300.xml").getroot()

    def z(element: ET.Element) -> float:
        return float((element.get("pos") or "0 0 0").split()[2])

    chassis = z(root.find(".//body[@name='chassis_link']"))
    planes = []
    for link, camera in (
        ("lidar_front_link", "lidar_front"),
        ("lidar_rear_link", "lidar_rear"),
    ):
        body = root.find(f".//body[@name='{link}']")
        planes.append(chassis + z(body) + z(body.find(f"camera[@name='{camera}']")))
    return tuple(planes)


def test_edge_berm_is_seen_by_the_front_lidar_and_clears_the_rear():
    """The berm at the 15 m edge follows the rocks' rule: it reaches the front scan plane by 0.1 m,
    so the costmaps see where the drivable ground ends, and keeps 0.1 m under the rear one. A berm
    the front lidar cannot see lets the rover climb off the collision ground unseen."""
    front, rear = _scan_planes_m()
    margin = 0.1
    for side in "nsew":
        band, (x0, x1, y0, y1) = _hfield_world_z(f"lunar_band_{side}")
        if side in "ns":
            # Profiles run across the edge, one per column away from the corners and the opening.
            coord = np.linspace(y0, y1, band.shape[0])
            along = np.linspace(x0, x1, band.shape[1])
            keep = np.abs(along) < 12.0
            if side == "n":
                keep &= np.abs(along - _road_extent()[0]) > _road_extent()[1]
            profiles = band.T[keep]
        else:
            coord = np.linspace(x0, x1, band.shape[1])
            along = np.linspace(y0, y1, band.shape[0])
            profiles = band[np.abs(along) < 12.0]
        # Measured from the nominal 13 m foot: the generator moves the foot only outward over band
        # ground, and checks the exact height above the moved foot itself.
        foot = np.abs(np.abs(coord) - 13.0).argmin()
        berm = np.abs(coord) > 13.0
        tops = profiles[:, berm].max(axis=1) - profiles[:, foot]
        assert tops.min() >= front + margin, (
            f"lunar_band_{side}'s berm stands only {tops.min():.3f} m above its foot, under the "
            f"front scan plane's {front + margin:.3f} m"
        )
        assert (
            tops.max() <= rear - margin
        ), f"lunar_band_{side}'s berm reaches {tops.max():.3f} m, within {margin} m of the rear scan plane"


def _nav2_circumscribed_radius_m() -> float:
    """The rover's turning radius for Nav2: its local costmap footprint's farthest corner, padded."""
    params = yaml.safe_load(
        (DESCRIPTION.parent / "params" / "nav2_params.yaml").read_text()
    )
    costmap = params["local_costmap"]["local_costmap"]["ros__parameters"]
    corners = yaml.safe_load(costmap["footprint"])
    return max(math.hypot(x, y) for x, y in corners) + costmap["footprint_padding"]


def _road_extent() -> tuple[float, float, float]:
    """The access road's centre x, half-width including its walls, and end y, from its strip."""
    _, (x0, x1, _, y1) = _hfield_world_z(ROAD_PREFIX)
    return (x0 + x1) / 2, (x1 - x0) / 2, y1


def test_berm_opening_leads_to_a_walled_road():
    """A man-made berm leaves a way out: the lane through the north berm is drivable from inside the band to the road's open end, where it meets the far horizon's ground, and its side walls follow the berm's rule up to their end faces."""
    front, rear = _scan_planes_m()
    margin = 0.1
    centre_x, half, end = _road_extent()
    lane_half = half - ROAD_WALL_M
    turn = 2 * _nav2_circumscribed_radius_m()
    assert (
        2 * lane_half >= turn
    ), f"the {2 * lane_half:.2f} m lane is too narrow to turn the rover in ({turn:.2f} m)"
    xs = np.linspace(centre_x - lane_half + 0.3, centre_x + lane_half - 0.3, 5)
    for x in xs:
        ys = np.arange(12.0, end, 0.1)
        lane = np.array([_ground_z(x, y) for y in ys])
        level = f"the lane through the berm opening is not level at x={x:.2f}: {lane.min():.3f}..{lane.max():.3f} m"
        assert np.abs(np.diff(lane)).max() < 0.03, level
        assert lane.max() - lane.min() < 0.15, level
    # At its end the road meets the far horizon's ground, which carries the rover on.
    road, (rx0, rx1, _, ry1) = _hfield_world_z(ROAD_PREFIX)
    edge_x = np.linspace(rx0, rx1, road.shape[1])
    step = road[-1] - _far_z(edge_x, np.full_like(edge_x, ry1))
    stands = f"the road's end stands {step.min():.3f}..{step.max():.3f} m above the far ground"
    assert 0.0 <= step.min(), stands
    assert step.max() < 0.03, stands
    across = np.arange(0.0, 1.0, 0.02)

    def wall_top(sign, y):
        foot = centre_x + sign * lane_half
        return max(_ground_z(foot + sign * d, y) for d in across) - _ground_z(foot, y)

    # Each wall ends in a face that falls steadily to the road; 1 cm covers 8-bit steps.
    for sign in (-1, 1):
        sink = [wall_top(sign, y) for y in np.arange(end - ROAD_END_FACE_M, end, 0.02)]
        falls = f"a road wall does not end in a falling face: {np.round(sink, 3)}"
        assert np.diff(sink).max() < 0.01, falls
        assert sink[-1] < 0.05, falls
    tops = []
    # From where the berm's and the wall's 0.4 m faces have both reached the crest, to each wall's
    # end face.
    for y in np.arange(13.5, end - ROAD_END_FACE_M, 0.1):
        for sign in (-1, 1):
            tops.append(wall_top(sign, y))
    tops = np.array(tops)
    assert (
        tops.min() >= front + margin
    ), f"a road wall stands only {tops.min():.3f} m above its foot"
    assert (
        tops.max() <= rear - margin
    ), f"a road wall reaches {tops.max():.3f} m, within {margin} m of the rear scan plane"


def test_landmark_rocks_roll_under_the_rover_or_clear_both_lidar_planes():
    """Every landmark rock is either low enough for the A300 to roll over, or reaches a lidar plane
    and keeps its top at least 0.1 m from both, so pitch does not make it flicker in and out of a
    scan. A rock in between stops the rover while staying invisible to the costmaps. Measured on the
    exposed top - what stands above the ground under the rock - since the mesh is partly buried.
    """
    front, rear = _scan_planes_m()
    margin = 0.1
    roll_over = 0.10  # under the 0.136 m belly and the 0.165 m wheel radius
    angles = np.linspace(0, 2 * np.pi, 16, endpoint=False)
    for name, pos, scale, verts in _landmarks():
        top_z = pos[2] + (verts[:, 2] * scale[2]).max()
        reach = np.hypot(verts[:, 0] * scale[0], verts[:, 1] * scale[1]).max()
        ring = [
            _ground_z(pos[0] + reach * math.cos(a), pos[1] + reach * math.sin(a))
            for a in angles
        ]
        # Seen from the centre and from the highest and lowest ground at the rock's edge.
        for ground in (_ground_z(pos[0], pos[1]), max(ring), min(ring)):
            top = top_z - ground
            if top <= roll_over:
                continue
            assert (
                top >= front + margin
            ), f"{name} stands {top:.3f} m on some side: too tall to roll over, too low for either lidar to see"
            assert (
                abs(top - rear) >= margin
            ), f"{name}'s top stands {top:.3f} m on some side, within {margin} m of the rear lidar plane"


def test_small_rocks_roll_under_the_rover():
    """The scattered small rocks sit below both lidar planes, so none may stand more than 0.10 m above
    the lowest ground beside it: a taller one stops the rover, or high-centres it, unseen.
    """
    roll_over = 0.10  # under the 0.136 m belly and the 0.165 m wheel radius
    meshes = _mjcf_elements(DESCRIPTION / "rocks_assets_generated.xml", "mesh")
    geoms = _mjcf_elements(DESCRIPTION / "rocks_geoms_generated.xml", "geom")
    assert geoms, "rocks_geoms_generated.xml has no geoms"
    for name, geom in geoms.items():
        mesh = meshes[geom.get("mesh")]
        verts = _stl_vertices(DESCRIPTION / "assets" / mesh.get("file")) * _vec(
            mesh, "scale"
        )
        x, y, z = _vec(geom, "pos")
        reach = np.hypot(verts[:, 0], verts[:, 1]).max()
        lowest = min(
            _ground_z(x + dx, y + dy)
            for dx in (-reach, 0.0, reach)
            for dy in (-reach, 0.0, reach)
        )
        top = z + verts[:, 2].max() - lowest
        assert top <= roll_over, (
            f"{name} stands {top:.3f} m above the ground beside it, too tall to roll over and "
            "below both lidar planes"
        )


def test_landmark_rocks_sit_in_the_ground_under_them():
    """Each rock's base is buried in the committed ground all the way round, not just at its centre:
    on a slope a rock seated by its centre sample leaves its downhill edge in the air, and regenerating
    the terrain without re-running generate_landmarks.py leaves rocks at the old ground's height.
    """
    geoms = _mjcf_elements(DESCRIPTION / "landmarks_geoms_generated.xml", "geom")
    for name, pos, scale, verts in _landmarks():
        w, _, _, z = _vec(geoms[name], "quat")
        yaw = 2 * math.atan2(z, w)
        world = verts * scale
        c, s = math.cos(yaw), math.sin(yaw)
        x = pos[0] + c * world[:, 0] - s * world[:, 1]
        y = pos[1] + s * world[:, 0] + c * world[:, 1]
        height = world[:, 2].max() - world[:, 2].min()
        low = world[:, 2] <= world[:, 2].min() + 0.1 * height
        for vx, vy, vz in zip(x[low], y[low], pos[2] + world[low, 2]):
            ground = _ground_z(vx, vy)
            assert (
                vz <= ground + 0.005
            ), f"{name}'s base floats {(vz - ground) * 100:.1f} cm above the ground at ({vx:.2f}, {vy:.2f})"
        depth = _ground_z(pos[0], pos[1]) - (pos[2] + world[:, 2].min())
        assert (
            depth <= 0.35 * height + 0.15
        ), f"{name} is buried {depth:.2f} m deep - stale placement?"


def test_driven_ground_is_collision_only_and_hidden_from_rendering():
    """The driven heightfield carries contact and must not also be rendered.

    MuJoCo uses one hfield mesh for both, and this field's full resolution is what the Dead Reckon
    Square's closure error is calibrated against, so it cannot be reduced for render cost. Instead
    it sits in geom group 3, which MuJoCo's default visualization options exclude from rendering,
    and ground_visual draws a downsampled twin. If this geom loses group 3 both surfaces render and
    the render cost silently returns (measured 5.3 ms -> 134 ms per camera frame); if it loses its
    contype/conaffinity the robot drives on nothing.
    """
    geoms = _mjcf_elements(HUSKY_SCENE, "geom")
    driven = geoms["ground_plane"]
    assert driven.get("group") == "3", (
        "the driven ground must stay in geom group 3 so it is not rendered; "
        "rendering it costs ~25x the visual twin"
    )
    assert (
        driven.get("contype") == "1" and driven.get("conaffinity") == "1"
    ), "the driven ground is the only colliding ground surface"


def test_visual_ground_matches_the_driven_ground_it_stands_in_for():
    """The render-only twin must be the current driven field, resampled.

    Two ways this goes wrong silently. Declared geometry: a different hfield size or geom pos
    floats the visible ground off the one the wheels touch. Stale content: someone regenerates
    lunar_hfield.png and forgets generate_visual_terrain.py, so the robot is seen driving on
    terrain that no longer exists. Both look like scenery, not like a bug.
    """
    geoms = _mjcf_elements(HUSKY_SCENE, "geom")
    hfields = _mjcf_elements(HUSKY_SCENE, "hfield")

    visual = geoms["ground_visual"]
    assert visual.get("contype") == "0" and visual.get("conaffinity") == "0"
    assert np.allclose(
        _vec(visual, "pos"), _vec(geoms["ground_plane"], "pos"), atol=TOLERANCE_M
    )
    assert np.allclose(
        _vec(hfields["lunar_hfield_visual"], "size"),
        _vec(hfields["lunar_hfield"], "size"),
        atol=TOLERANCE_M,
    ), "visual and driven hfields must declare the same size"

    def heights(name):
        png = DESCRIPTION / "assets" / hfields[name].get("file").split("/")[-1]
        with Image.open(png) as image:
            assert (
                image.mode == "L"
            ), f"{png.name} is not 8-bit; the 255 scale below assumes it"
            full = np.asarray(image).astype(np.float64)
            # Compare on the coarse grid: upsampling the twin would test the interpolator.
            coarse = np.asarray(image.resize((64, 64), Image.BILINEAR)).astype(
                np.float64
            )
        # Normalise by the shared 8-bit full scale both fields are written at, not by each image's
        # own max: self-normalising would let a correctly-shaped but amplitude-scaled twin pass.
        return full, coarse / 255.0

    driven_full, driven = heights("lunar_hfield")
    visual_full, visual_h = heights("lunar_hfield_visual")
    assert visual_full.shape[0] < driven_full.shape[0], "the twin exists to be cheaper"
    # A resample of the same terrain agrees closely once both are reduced to a common grid;
    # an unrelated or stale field does not.
    # 0.02 of full scale. The committed pair differs by 0.0039, so this leaves ~5x headroom for
    # resampling while still catching an amplitude error the old self-normalised check could not.
    assert np.abs(driven - visual_h).max() < 0.02, (
        "the visual twin does not match the current driven heightfield - re-run "
        "generate_visual_terrain.py after regenerating lunar_hfield.png"
    )


def test_enable_vo_publishes_the_oakd_pair_as_mono8_rectified_stereo():
    """With enable_vo, picknik_mujoco_ros publishes the pair on the topics VO subscribes to."""
    root = ET.parse(DESCRIPTION / "husky_a300_mujoco.xacro").getroot()
    enabled_blocks = [
        block
        for block in root.iter("{http://www.ros.org/wiki/xacro}if")
        if block.attrib["value"] == "$(arg enable_vo)"
    ]
    sensors = {
        sensor.attrib["name"]: {
            param.attrib["name"]: param.text for param in sensor.iter("param")
        }
        for block in enabled_blocks
        for sensor in block.iter("sensor")
    }

    assert sensors["oakd_stereo"] == {
        "stereo_left_camera": "oakd_left",
        "stereo_right_camera": "oakd_right",
    }
    for side in ("left", "right"):
        assert sensors[f"oakd_{side}"] == {
            "rgb_topic": f"/oakd/{side}/image_rect",
            "camera_info_topic": f"/oakd/{side}/camera_info",
            "image_encoding": "mono8",
        }
