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
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest
import yaml
from PIL import Image

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


def test_duo_actuators_cover_base_spine_arms_and_grippers(model) -> None:
    """The synchro-drive simulation actuates three planar joints, not A300 wheels."""
    expected = {"planar_x", "planar_y", "planar_theta", "franka_spine_vertical_joint"}
    expected.update(
        f"{side}_fr3v2_joint{joint}"
        for side in ("left", "right")
        for joint in range(1, 8)
    )
    expected.update(f"{side}_fr3v2_finger_joint1" for side in ("left", "right"))
    assert model.nu == len(expected)
    assert {model.actuator(i).name for i in range(model.nu)} == expected
    for name in ("planar_x", "planar_y", "planar_theta", "franka_spine_vertical_joint"):
        assert model.actuator_trnid[model.actuator(name).id, 0] == model.joint(name).id


def test_horizontal_lidar_ring_samples_the_optical_axis(model):
    """The horizontal lidar ring must land on a render pixel centre, without vertical quantization."""
    height = model.vis.global_.offheight
    assert height % 2 == 1
    assert int(height / 2) == (height - 1) / 2
    for camera in range(model.ncam):
        if model.cam_user[camera, 0] != 2:
            np.testing.assert_array_equal(
                model.cam_resolution[camera], [model.vis.global_.offwidth, height]
            )


def test_imu_reports_body_orientation_and_rotation(model):
    """Exercise actual MuJoCo sensors independently of planar encoders and ground-truth ROS."""
    mujoco = pytest.importorskip("mujoco")
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, 0)
    yaw = 0.7
    joint = model.joint("planar_theta")
    data.qpos[joint.qposadr[0]] = yaw
    data.qvel[joint.dofadr[0]] = 0.4
    mujoco.mj_forward(model, data)
    site = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "imu_site")
    readings = {}
    for i in range(model.nsensor):
        if model.sensor_objid[i] == site:
            start = model.sensor_adr[i]
            readings[model.sensor_type[i]] = data.sensordata[
                start : start + model.sensor_dim[i]
            ].copy()
    np.testing.assert_allclose(
        readings[mujoco.mjtSensor.mjSENS_FRAMEQUAT],
        [np.cos(yaw / 2), 0, 0, np.sin(yaw / 2)],
        atol=1e-10,
    )
    np.testing.assert_allclose(
        readings[mujoco.mjtSensor.mjSENS_GYRO], [0, 0, 0.4], atol=1e-10
    )
    assert np.isfinite(readings[mujoco.mjtSensor.mjSENS_ACCELEROMETER]).all()


def test_fuse_parameters_parse_with_ros() -> None:
    """ROS YAML uses different scalar rules than PyYAML, notably the Y axis."""
    subprocess.run(
        [
            sys.executable,
            "-c",
            "import rclpy; rclpy.init(); rclpy.shutdown()",
            "--ros-args",
            "--params-file",
            str(PACKAGE / "config/fuse/fuse.yaml"),
        ],
        check=True,
        capture_output=True,
        text=True,
    )


def test_resized_shelves_and_aisles_match_the_requested_dimensions(model) -> None:
    """Compiled shelf faces bound a two-metre gondola and four-metre aisle."""
    mujoco = pytest.importorskip("mujoco")
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    faces = []
    for name in ("B1_1", "B2_1"):
        body = model.body(name).id
        boxes = [
            g
            for g in range(model.ngeom)
            if model.geom_bodyid[g] == body
            and model.geom_type[g] == mujoco.mjtGeom.mjGEOM_BOX
        ]
        assert len(boxes) == 1
        geom = boxes[0]
        assert 2 * model.geom_size[geom, 1] == pytest.approx(2.0)
        faces.append(
            (
                data.geom_xpos[geom, 1] - model.geom_size[geom, 1],
                data.geom_xpos[geom, 1] + model.geom_size[geom, 1],
            )
        )
    assert faces[1][0] - faces[0][1] == pytest.approx(4.0)
    demo = ET.parse(PACKAGE / "objectives/demo_d4_b2_south_return.xml")
    waypoints = demo.findall(".//Action[@pose_stamped='{b2_waypoint}']")
    assert waypoints
    for waypoint in waypoints:
        y = float(waypoint.get("position_xyz").split(";")[1])
        assert faces[1][0] - y == pytest.approx(0.9)


def test_d4_dock_is_visible_to_both_lidars_and_in_both_maps(model):
    """A docking landmark must agree between physics, lidar surfaces and planning."""
    mujoco = pytest.importorskip("mujoco")
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, model.key("default").id)
    mujoco.mj_forward(model, data)
    dock = model.geom("demo_dock_d4").id
    assert model.geom_group[dock] == 2
    assert model.geom_contype[dock] != 0
    for side in ("front", "rear"):
        height = data.cam_xpos[model.camera("lidar_" + side).id, 2]
        assert abs(height - data.geom_xpos[dock, 2]) < model.geom_size[dock, 2]
    for name in ("market", "market_localization"):
        meta = yaml.safe_load((PACKAGE / "maps" / (name + ".yaml")).read_text())
        pixels = np.array(Image.open(PACKAGE / "maps" / meta["image"]))[::-1]
        west_face = data.geom_xpos[dock, 0] - model.geom_size[dock, 0]
        x, y = (np.array([west_face, 0.0]) - meta["origin"][:2]) / meta["resolution"]
        assert np.any(pixels[int(y) - 1 : int(y) + 2, int(x) - 1 : int(x) + 2] < 100)


def test_in_place_refinement_keeps_a_local_seed() -> None:
    """The seed AMCL receives when refining an estimate keeps a 10 cm / 0.1 rad spread."""
    name = "Refine Localization In Place Subtree"
    top = ET.parse(PACKAGE / "objectives/refine_localization_in_place.xml")
    call = top.find(f".//BehaviorTree//SubTree[@ID='{name}']")
    subtree = ET.parse(PACKAGE / "objectives/refine_localization_in_place_subtree.xml")
    ports = {
        port.get("name"): port.get("default")
        for port in subtree.findall(f"TreeNodesModel/SubTree[@ID='{name}']/input_port")
    }

    def received(value):
        """Resolve a subtree attribute through its isolated, declared input ports."""
        if not (value.startswith("{") and value.endswith("}")):
            return value
        assert value[1:-1] in ports, f"{value} is not a declared port of {name}"
        return call.get(value[1:-1], ports[value[1:-1]])

    seeds = [
        node
        for node in subtree.findall(
            f"BehaviorTree[@ID='{name}']//Action[@ID='SetInitialPose']"
        )
        if node.get("pose") == "{seed_pose}"
    ]
    assert received("{seed_pose}") == call.get("seed_pose")
    assert len(seeds) == 1
    assert float(received(seeds[0].get("xy_variance"))) == pytest.approx(0.01)
    assert float(received(seeds[0].get("yaw_variance"))) == pytest.approx(0.01)


def test_stocking_carts_are_physical_unmapped_lidar_obstacles(model) -> None:
    """Both lidars see physical carts where the static maps show free space."""
    mujoco = pytest.importorskip("mujoco")
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, model.key("default").id)
    mujoco.mj_forward(model, data)
    for position in generate_market.STOCKING_CART_POSITIONS:
        for side in ("front", "rear"):
            z = data.cam_xpos[model.camera("lidar_" + side).id, 2]
            origin = np.array([position[0] - 0.12, position[1] - 2, z])
            direction = np.array([0.0, 1.0, 0.0])
            hit = np.zeros(1, dtype=np.int32)
            distance = mujoco.mj_ray(
                model,
                data,
                origin,
                direction,
                np.array([1, 1, 1, 0, 0, 0], dtype=np.uint8),
                1,
                1,
                hit,
            )
            assert 0 < distance < 2
            assert "stocking_" in model.body(model.geom_bodyid[hit[0]]).name
            assert model.geom_contype[hit[0]] != 0
            point = (origin + distance * direction)[:2]
            for name in ("market", "market_localization"):
                meta = yaml.safe_load((PACKAGE / "maps" / (name + ".yaml")).read_text())
                pixels = np.array(Image.open(PACKAGE / "maps" / meta["image"]))[::-1]
                x, y = ((point - meta["origin"][:2]) / meta["resolution"]).astype(int)
                assert np.all(pixels[y - 1 : y + 2, x - 1 : x + 2] == 254)
