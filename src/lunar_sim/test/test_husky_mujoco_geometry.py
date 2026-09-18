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


def test_rear_camera_mount_matches_vendored_urdf():
    """Keep full-precision mount FK, including the camera's 34.5 degree down-pitch."""
    enclosure = ET.parse(CLEARPATH_A300_URDF / "attachments/amp_enclosure.urdf.xacro")
    arch = ET.parse(CLEARPATH_A300_URDF / "attachments/amp_sensor_arch.urdf.xacro")
    antenna = enclosure.find(".//joint[@name='${name}_antenna_mount_link']/origin")
    mount = arch.find(".//joint[@name='${name}_rear_camera_mount_joint']/origin")
    position = np.fromstring(antenna.get("xyz"), sep=" ") + np.fromstring(
        mount.get("xyz"), sep=" "
    )
    # This composition assumes the unrotated enclosure/arch chain declared by
    # husky_a300_mujoco.xacro. Fail if upstream starts rotating the antenna frame.
    np.testing.assert_allclose(np.fromstring(antenna.get("rpy"), sep=" "), 0)
    roll, pitch, yaw = np.fromstring(mount.get("rpy"), sep=" ")
    cr, cp, cy = np.cos(np.array([roll, pitch, yaw]) / 2)
    sr, sp, sy = np.sin(np.array([roll, pitch, yaw]) / 2)
    quaternion = np.array(
        [
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
        ]
    )
    name = "sensor_arch_rear_camera_mount"
    body = ET.parse(HUSKY_A300).find(
        f".//body[@name='chassis_link']/body[@name='{name}']"
    )
    assert body is not None
    np.testing.assert_allclose(
        np.fromstring(body.get("pos"), sep=" "), position, atol=1e-12
    )
    got_quaternion = np.fromstring(body.get("quat", "1 0 0 0"), sep=" ")
    np.testing.assert_allclose(got_quaternion, quaternion, atol=1e-12)
    assert body.find(f"camera[@name='{name}']") is not None
    assert body.find(f"site[@name='{name}_optical_frame']") is not None

    # picknik_mujoco_ros/cameras.cpp throws "Camera resolution mismatch" at hardware
    # init for any non-lidar camera whose resolution differs from the scene's
    # offscreen buffer, so these two numbers are not independently tunable.
    global_ = ET.parse(HUSKY_SCENE).find(".//visual/global")
    camera = body.find(f"camera[@name='{name}']")
    assert [int(value) for value in camera.get("resolution").split()] == [
        int(global_.get("offwidth")),
        int(global_.get("offheight")),
    ]


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


# The OAK-D Pro's published stereo baseline, and the profile hangar_sim pins in
# params/forward_stereo.yaml. Shared between the two configs so captures stay comparable.
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
    """The far-field horizon hfield is visual only and passes underneath the driven terrain, so
    inside that terrain's square footprint it must stay below its floor. If it rises above,
    it pokes up through the ground the robot drives on - which reads as terrain, not as a bug,
    so nothing else would catch it. Mirrors the guard in generate_far_terrain.py."""
    hfields = _mjcf_elements(HUSKY_SCENE, "hfield")
    geoms = _mjcf_elements(HUSKY_SCENE, "geom")

    near_half_m, _, near_elevation_z, _ = _vec(hfields["lunar_hfield"], "size")
    near_floor_z = _vec(geoms["ground_plane"], "pos")[2]

    far_half_m, _, far_elevation_z, _ = _vec(hfields["lunar_far_hfield"], "size")
    far_base_z = _vec(geoms["far_horizon"], "pos")[2]

    assert geoms["far_horizon"].get("contype") == "0", "far horizon must not collide"
    assert (
        geoms["far_horizon"].get("conaffinity") == "0"
    ), "far horizon must not collide"

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
