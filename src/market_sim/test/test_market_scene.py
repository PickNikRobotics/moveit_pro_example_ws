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

"""Checks on the generated supermarket and the MuJoCo scene that includes it.

Needs numpy, and mujoco with MoveIt Pro's package:// resource provider for the model checks (both
ship in the MoveIt Pro image).
"""

import filecmp
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest
import yaml

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE / "scripts"))

import generate_market  # noqa: E402

SCENE = PACKAGE / "mjcf" / "scene.xml"
URDF = PACKAGE / "description" / "mobile_fr3_duo.urdf.xacro"
# picknik_mujoco_ros renders cameras and lidars into a fixed 2000-geom scene and drops the rest.
RENDER_GEOM_LIMIT = 2000
RENDER_GEOM_MARGIN = 100
ROBOT_NQ = len(generate_market.ROBOT_QPOS)


def test_generated_files_match_the_generator(tmp_path):
    generate_market.main(tmp_path)
    generated = sorted(
        p.relative_to(tmp_path) for p in tmp_path.rglob("*") if p.is_file()
    )
    assert generated
    stale = [
        str(p)
        for p in generated
        if not filecmp.cmp(tmp_path / p, PACKAGE / p, shallow=False)
    ]
    assert not stale, f"Rerun scripts/generate_market.py; out of date: {stale}"


def test_rack_collision_boxes_enclose_the_shelf_meshes():
    """Every vertex of a gondola's two shelf units must lie inside its planning-scene box.

    The box is written from shelf_geometry.json's overall extents, the same source the
    simulator's own box collision uses. This checks that claim against the meshes themselves,
    so a mesh edit that grows the rack fails here instead of letting the arm plan into it.
    """
    shelf = generate_market.load_shelf()
    geometry = shelf[2]["overall"]
    back_depth = geometry["y"][1]
    size = np.array(
        [
            geometry["x"][1] - geometry["x"][0],
            2 * (geometry["y"][1] - geometry["y"][0]),
            geometry["z"][1] - geometry["z"][0],
        ]
    )
    centre = np.array([0.0, 0.0, size[2] / 2])
    lower, upper = centre - size / 2, centre + size / 2

    vertices = np.vstack(
        [v for v, _, _ in generate_market.gondola_parts(shelf[0]["full"], back_depth)]
    )
    assert len(vertices)
    assert np.all(vertices >= lower - 1e-6), (
        f"shelf mesh reaches below the collision box: "
        f"{vertices.min(axis=0)} vs {lower}"
    )
    assert np.all(vertices <= upper + 1e-6), (
        f"shelf mesh reaches past the collision box: "
        f"{vertices.max(axis=0)} vs {upper}"
    )


def test_every_gondola_has_a_rack_collision_box():
    """The planning-scene URDF must carry one box per gondola, and meshes only where picked."""
    layout = generate_market.Layout(generate_market.FLOOR_PLAN)
    root = ET.parse(PACKAGE / "description" / "market_racks.urdf").getroot()
    links = {link.get("name"): link for link in root.findall("link")}
    for gondola in layout.gondolas:
        link = links.get(gondola["name"])
        assert link is not None, f"no planning-scene link for {gondola['name']}"
        assert len(link.findall("collision")) == 1
    visual_aisles = {
        name.rsplit("_", 1)[0] for name, link in links.items() if link.findall("visual")
    }
    assert visual_aisles == set(generate_market.REAL_AISLES)


def test_planar_range_matches_the_urdf_travel_limit(model):
    import xml.etree.ElementTree as ET

    urdf = ET.parse(URDF).getroot()
    for joint in ("planar_x", "planar_y"):
        limit = urdf.find(f"joint[@name='{joint}']/limit")
        bounds = [float(limit.get("lower")), float(limit.get("upper"))]
        np.testing.assert_allclose(model.joint(joint).range, bounds)


@pytest.fixture(scope="module")
def model():
    mujoco = pytest.importorskip("mujoco")
    from ament_index_python.packages import get_package_prefix

    # The robot loads its franka_description meshes through package:// URIs, which plain MuJoCo
    # cannot resolve; MoveIt Pro resolves them with this resource-provider plugin.
    plugin = Path(get_package_prefix("mujoco_package_uri")) / "bin" / "mujoco_plugin"
    mujoco.mj_loadPluginLibrary(
        str(plugin / "libmujoco_package_uri_resource_provider.so")
    )
    return mujoco.MjModel.from_xml_path(str(SCENE))


def test_visible_geoms_fit_the_render_limit(model):
    visible = int(np.sum(model.geom_group <= 2)) + int(np.sum(model.site_group <= 2))
    assert visible <= RENDER_GEOM_LIMIT - RENDER_GEOM_MARGIN


def test_default_keyframe_is_the_only_key_and_starts_the_robot_at_the_origin(model):
    mujoco = pytest.importorskip("mujoco")
    assert model.nkey == 1
    default = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "default")
    assert default == 0
    assert model.nq == ROBOT_NQ
    np.testing.assert_allclose(model.key_qpos[default][:3], 0.0)
    np.testing.assert_allclose(
        model.key_qpos[default], generate_market.ROBOT_QPOS, atol=1e-4
    )


def test_only_robot_joints_are_named(model):
    # The simulator gives every NAMED joint a ros2_control state; product free joints must stay unnamed.
    mujoco = pytest.importorskip("mujoco")
    for joint in range(model.njnt):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint)
        if model.jnt_type[joint] == mujoco.mjtJoint.mjJNT_FREE:
            assert not name
        else:
            assert name
    assert model.joint("planar_x").qposadr[0] == 0


def test_every_fixed_camera_has_an_optical_frame_site(model):
    mujoco = pytest.importorskip("mujoco")
    for camera in range(model.ncam):
        if model.cam_mode[camera] == mujoco.mjtCamLight.mjCAMLIGHT_FIXED:
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_CAMERA, camera)
            assert (
                mujoco.mj_name2id(
                    model, mujoco.mjtObj.mjOBJ_SITE, f"{name}_optical_frame"
                )
                >= 0
            )


def test_map_marks_every_shelf_and_frees_the_start(model):
    mujoco = pytest.importorskip("mujoco")
    from PIL import Image

    info = yaml.safe_load((PACKAGE / "maps" / "market.yaml").read_text())
    grid = np.array(Image.open(PACKAGE / "maps" / info["image"]))[::-1]
    resolution, origin = info["resolution"], info["origin"]

    def cell(x, y):
        return grid[
            int((y - origin[1]) / resolution), int((x - origin[0]) / resolution)
        ]

    assert cell(0.0, 0.0) == 254
    # Take the gondolas by the names the generator gave them, rather than by shape: the
    # stocking carts are also joint-free bodies on the world body, and are deliberately
    # absent from the maps.
    layout = generate_market.Layout(generate_market.FLOOR_PLAN)
    names = [gondola["name"] for gondola in layout.gondolas]
    shelves = [
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name) for name in names
    ]
    assert all(b >= 0 for b in shelves), "a gondola body is missing from the model"
    assert len(shelves) == 404
    poses = [(model.body_pos[b], model.body_quat[b]) for b in shelves]
    for (x, y, _), rotation in poses:
        # A point 0.3 m into the shelf, along its +Y axis.
        yaw = 2.0 * np.arctan2(rotation[3], rotation[0])
        assert cell(x - 0.3 * np.sin(yaw), y + 0.3 * np.cos(yaw)) == 0


def test_lidar_height_matches_the_model(model):
    mujoco = pytest.importorskip("mujoco")
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, 0)
    mujoco.mj_forward(model, data)
    for lidar in ("lidar_front", "lidar_rear"):
        height = data.cam_xpos[model.camera(lidar).id][2]
        assert height == pytest.approx(generate_market.LIDAR_HEIGHT, abs=1e-3), lidar


def test_localization_map_matches_the_lidar_plane(model):
    """Ranges cast in the scene at lidar height match the localization map at several poses."""
    mujoco = pytest.importorskip("mujoco")
    from PIL import Image

    info = yaml.safe_load((PACKAGE / "maps" / "market_localization.yaml").read_text())
    grid = np.array(Image.open(PACKAGE / "maps" / info["image"]))[::-1]
    resolution, origin = info["resolution"], info["origin"]
    data = mujoco.MjData(model)
    robot = model.body_rootid[model.jnt_bodyid[model.joint("planar_x").id]]
    groups = np.array([1, 1, 1, 0, 0, 0], dtype=np.uint8)
    max_range = 20.0

    def scene_range(start, direction):
        travelled = 0.0
        for _ in range(20):
            geom = np.array([-1], dtype=np.int32)
            hit = mujoco.mj_ray(model, data, start, direction, groups, 1, -1, geom)
            if hit < 0 or travelled + hit > max_range:
                return None
            if model.body_rootid[model.geom_bodyid[geom[0]]] != robot:
                return travelled + hit
            # The ray hit the robot itself: continue from just past that point.
            start = start + direction * (hit + 1e-3)
            travelled += hit + 1e-3
        return None

    def map_range(start, direction):
        for r in np.arange(resolution / 2, max_range, resolution / 5):
            x, y = start[:2] + r * direction[:2]
            row, col = int((y - origin[1]) / resolution), int(
                (x - origin[0]) / resolution
            )
            if not (0 <= row < grid.shape[0] and 0 <= col < grid.shape[1]):
                return None
            if grid[row, col] == 0:
                return r
        return None

    layout = generate_market.Layout(generate_market.FLOOR_PLAN)
    back_depth = generate_market.load_shelf()[2]["overall"]["y"][1]
    poses = [(0.0, 0.0, 0.0)] + list(
        generate_market.aisle_goals(layout, back_depth).values()
    )
    for x, y, yaw in poses:
        mujoco.mj_resetDataKeyframe(model, data, 0)
        data.qpos[model.joint("planar_x").qposadr[0]] = x
        data.qpos[model.joint("planar_y").qposadr[0]] = y
        data.qpos[model.joint("planar_theta").qposadr[0]] = yaw
        mujoco.mj_forward(model, data)
        errors = []
        for lidar in ("lidar_front", "lidar_rear"):
            start = data.cam_xpos[model.camera(lidar).id].copy()
            for angle in np.radians(np.arange(0, 360, 4)):
                direction = np.array([np.cos(angle), np.sin(angle), 0.0])
                seen = scene_range(start, direction)
                mapped = map_range(start, direction)
                if seen is not None and mapped is not None:
                    errors.append(abs(seen - mapped))
        assert len(errors) > 100, (x, y)
        assert np.median(errors) < 2 * resolution, (x, y, np.median(errors))


def test_aisle_goals_lie_on_free_map_cells():
    import xml.etree.ElementTree as ET

    from PIL import Image

    info = yaml.safe_load((PACKAGE / "maps" / "market.yaml").read_text())
    grid = np.array(Image.open(PACKAGE / "maps" / info["image"]))[::-1]
    resolution, origin = info["resolution"], info["origin"]
    objectives = sorted((PACKAGE / "objectives").glob("navigate_to_aisle_*.xml"))
    assert len(objectives) == len(generate_market.REAL_AISLES)
    for path in objectives:
        goal = ET.parse(path).find(".//Action[@ID='CreatePoseStamped']")
        x, y, _ = (float(v) for v in goal.get("position_xyz").split(";"))
        row = int((y - origin[1]) / resolution)
        col = int((x - origin[0]) / resolution)
        # The base footprint's half-length, in cells, around the goal must be free.
        reach = int(0.4 / resolution)
        window = grid[row - reach : row + reach + 1, col - reach : col + reach + 1]
        assert np.all(window == 254), path.name
