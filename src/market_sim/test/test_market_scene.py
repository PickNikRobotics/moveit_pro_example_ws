"""Checks on the generated supermarket and the MuJoCo scene that includes it. No ROS.

Needs numpy, and mujoco for the model checks (both ship in the MoveIt Pro image).
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
import generate_localization_map  # noqa: E402

SCENE = PACKAGE / "mjcf" / "scene.xml"
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


def test_a300_has_only_four_wheel_actuators(model):
    assert model.nu == 4
    assert {model.actuator(i).name for i in range(model.nu)} == {
        f"{end}_{side}_wheel_joint"
        for end in ("front", "rear")
        for side in ("left", "right")
    }


@pytest.fixture(scope="module")
def model():
    mujoco = pytest.importorskip("mujoco")
    return mujoco.MjModel.from_xml_path(str(SCENE))


def test_visible_geoms_fit_the_render_limit(model):
    visible = int(np.sum(model.geom_group <= 2)) + int(np.sum(model.site_group <= 2))
    assert visible <= RENDER_GEOM_LIMIT - RENDER_GEOM_MARGIN


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


def mocap_bodies(model):
    """Bodies a keyframe can move, in the order of the keyframes' mpos and mquat."""
    bodies = [b for b in range(model.nbody) if model.body_mocapid[b] >= 0]
    return sorted(bodies, key=lambda b: model.body_mocapid[b])


def test_default_keyframe_matches_the_scene_file(model):
    # The boot state, the scene file's home poses and a qpos mirror must all agree.
    mujoco = pytest.importorskip("mujoco")
    default = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "default")
    assert default >= 0
    np.testing.assert_allclose(
        model.key_qpos[default][ROBOT_NQ:], model.qpos0[ROBOT_NQ:], atol=1e-4
    )
    bodies = mocap_bodies(model)
    assert len(bodies) == 36
    np.testing.assert_allclose(
        model.key_mpos[default].reshape(-1, 3), model.body_pos[bodies], atol=1e-4
    )
    np.testing.assert_allclose(
        model.key_mquat[default].reshape(-1, 4), model.body_quat[bodies], atol=1e-4
    )


def test_loose_gondolas_have_empty_neighbours(model):
    mujoco = pytest.importorskip("mujoco")
    empty = model.mesh("market_pair_empty").id
    for aisle, columns in generate_market.AD_LOOSE_COLUMNS.items():
        for column in columns:
            for neighbour in (column - 1, column + 1):
                body = model.body(f"{aisle}_{neighbour}").id
                meshes = [
                    model.geom_dataid[g]
                    for g in range(model.ngeom)
                    if model.geom_bodyid[g] == body
                    and model.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH
                ]
                assert meshes == [empty], (aisle, neighbour)


def test_every_keyframe_exists_and_starts_the_robot_at_the_origin(model):
    mujoco = pytest.importorskip("mujoco")
    for name in ("default", "loose_e_i", "loose_a_d"):
        key = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, name)
        assert key >= 0, name
        np.testing.assert_allclose(model.key_qpos[key][:3], [0.0, 0.0, 0.13597])


def test_only_robot_joints_are_named(model):
    # The simulator gives every NAMED joint a ros2_control state; product free joints must stay unnamed.
    mujoco = pytest.importorskip("mujoco")
    for joint in range(model.njnt):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint)
        if model.jnt_type[joint] == mujoco.mjtJoint.mjJNT_FREE:
            assert not name
        else:
            assert name
    assert model.joint("front_left_wheel_joint").qposadr[0] == 7


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


@pytest.mark.parametrize("key", [name for name, _ in generate_market.KEYFRAMES])
def test_loose_products_settle_and_sleep(model, key):
    mujoco = pytest.importorskip("mujoco")
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, model.key(key).id)
    for _ in range(int(3.0 / model.opt.timestep)):
        mujoco.mj_step(model, data)
    start = model.key_qpos[model.key(key).id][ROBOT_NQ:].reshape(-1, 7)[:, :3]
    end = data.qpos[ROBOT_NQ:].reshape(-1, 7)[:, :3]
    assert np.max(np.linalg.norm(end - start, axis=1)) < 0.01
    # Only the robot's kinematic tree stays awake; resting products cost nothing.
    assert data.ntree_awake == 1


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
    # Shelves are joint-free bodies under the world; carts have their own map checks.
    shelves = [
        b
        for b in range(1, model.nbody)
        if model.body_parentid[b] == 0
        and model.body_jntnum[b] == 0
        and model.body_geomnum[b]
        and not model.body(b).name.startswith("stocking_")
    ]
    assert len(shelves) == 416
    poses = [(model.body_pos[b], model.body_quat[b]) for b in shelves]
    # One map serves every keyframe, so the moving shelves must be on it everywhere.
    for key in range(model.nkey):
        positions = model.key_mpos[key].reshape(-1, 3)
        quats = model.key_mquat[key].reshape(-1, 4)
        poses += list(zip(positions, quats))
    for (x, y, _), rotation in poses:
        # A point 0.3 m into the shelf, along its +Y axis.
        yaw = 2.0 * np.arctan2(rotation[3], rotation[0])
        assert cell(x - 0.3 * np.sin(yaw), y + 0.3 * np.cos(yaw)) == 0


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


def test_localization_transforms_the_ui_marker_into_map():
    """The marker may arrive in a robot frame, so localization must apply TF."""
    tree = ET.parse(PACKAGE / "objectives" / "localize_robot.xml")
    conversion = tree.find('.//Action[@input_pose="{clicked_pose}"]')
    assert conversion is not None
    assert conversion.get("ID") == "TransformPoseFrame"
    assert conversion.get("target_frame_id") == "map"
    assert conversion.get("output_pose") == "{initial_pose}"


def test_localization_surface_section_does_not_fill_obstacle_interiors():
    """A likelihood field must distinguish a shelf face from points inside its footprint."""
    vertices = generate_localization_map.BOX_VERTICES.astype(float)
    triangles = vertices[generate_localization_map.BOX_FACES]
    segments = generate_localization_map.section_segments(triangles, 0.0)
    assert segments
    for a, b in segments:
        midpoint = (a + b) / 2
        assert np.max(np.abs(midpoint)) == pytest.approx(1.0)
    assert not generate_localization_map.section_segments(triangles, 2.0)


def test_generated_localization_map_matches_visual_surfaces():
    """The committed AMCL map must reproduce the default scene's two lidar-height slices."""
    grid, meta = generate_localization_map.generate(PACKAGE)
    np.testing.assert_array_equal(
        grid[::-1], np.array(Image.open(PACKAGE / "maps/localization.png"))
    )
    assert meta == yaml.safe_load((PACKAGE / "maps/localization.yaml").read_text())


def test_imu_reports_body_orientation_and_rotation(model):
    """Exercise actual MuJoCo sensors independently of wheel encoders and ground-truth ROS."""
    mujoco = pytest.importorskip("mujoco")
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, 0)
    yaw = 0.7
    data.qpos[3:7] = [np.cos(yaw / 2), 0, 0, np.sin(yaw / 2)]
    data.qvel[3:6] = [0, 0, 0.4]
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
        readings[mujoco.mjtSensor.mjSENS_FRAMEQUAT], data.qpos[3:7], atol=1e-10
    )
    np.testing.assert_allclose(
        readings[mujoco.mjtSensor.mjSENS_GYRO], [0, 0, 0.4], atol=1e-10
    )
    assert np.isfinite(readings[mujoco.mjtSensor.mjSENS_ACCELEROMETER]).all()


def test_fuse_parameters_parse_with_ros():
    """ROS YAML uses different scalar rules than PyYAML (notably the Y axis)."""
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
    for name in ("market", "localization"):
        meta = yaml.safe_load((PACKAGE / "maps" / (name + ".yaml")).read_text())
        pixels = np.array(Image.open(PACKAGE / "maps" / meta["image"]))[::-1]
        west_face = data.geom_xpos[dock, 0] - model.geom_size[dock, 0]
        x, y = (np.array([west_face, 0.0]) - meta["origin"][:2]) / meta["resolution"]
        assert np.any(pixels[int(y) - 1 : int(y) + 2, int(x) - 1 : int(x) + 2] < 100)


def test_in_place_refinement_keeps_a_local_seed() -> None:
    """Refining an existing estimate must retain a 10 cm / 0.1 rad seed spread."""
    top = ET.parse(PACKAGE / "objectives/refine_localization_in_place.xml")
    call = top.find(".//SubTree[@ID='Refine Localization In Place Subtree']")
    assert float(call.get("xy_variance")) == pytest.approx(0.01)
    assert float(call.get("yaw_variance")) == pytest.approx(0.01)
    subtree = ET.parse(PACKAGE / "objectives/refine_localization_in_place_subtree.xml")
    seed = subtree.find(".//Action[@ID='SetInitialPose']")
    assert seed.get("xy_variance") == "{xy_variance}"
    assert seed.get("yaw_variance") == "{yaw_variance}"
    assert seed.get("pose") == "{seed_pose}"


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
            for name in ("market", "localization"):
                meta = yaml.safe_load((PACKAGE / "maps" / (name + ".yaml")).read_text())
                pixels = np.array(Image.open(PACKAGE / "maps" / meta["image"]))[::-1]
                x, y = ((point - meta["origin"][:2]) / meta["resolution"]).astype(int)
                assert np.all(pixels[y - 1 : y + 2, x - 1 : x + 2] == 254)
