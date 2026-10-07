"""Checks on the generated supermarket and the MuJoCo scene that includes it. No ROS.

Needs numpy, and mujoco for the model checks (both ship in the MoveIt Pro image).
"""

import filecmp
import re
import sys
from pathlib import Path

import numpy as np
import pytest
import yaml

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE / "scripts"))

import generate_market  # noqa: E402

SCENE = PACKAGE / "mjcf" / "scene.xml"
# This scene's copy of the robot model, which must match mobile_fr3_duo_sim's byte for byte,
# except for the planar base joints' travel range, widened for the store.
ROBOT_MODEL_FILES = ["mobile_fr3_duo.xml", "sensors.xml"]
PLANAR_RANGE = re.compile(r'(name="planar_[xy]"(?:\s+\w+="[^"]*")*?\s+range=)"[^"]*"')
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


def mobile_fr3_duo_sim_mjcf():
    try:
        from ament_index_python.packages import get_package_share_directory

        return Path(get_package_share_directory("mobile_fr3_duo_sim")) / "mjcf"
    except (ImportError, LookupError):
        vendored = "external_dependencies/moveit_pro_franka_ws/src/mobile_fr3_duo_sim"
        return PACKAGE.parent / vendored / "mjcf"


def test_robot_model_matches_mobile_fr3_duo_sim():
    # MuJoCo resolves an included file's meshes against that file's own folder, so
    # the scene cannot include the robot from the other package; it keeps a copy.
    source = mobile_fr3_duo_sim_mjcf()
    if not source.is_dir():
        pytest.skip("mobile_fr3_duo_sim is not available")
    names = ROBOT_MODEL_FILES + [
        f"assets/{p.name}" for p in (source / "assets").iterdir() if p.is_file()
    ]
    names.remove("mobile_fr3_duo.xml")
    differ = [
        n
        for n in names
        if not filecmp.cmp(source / n, PACKAGE / "mjcf" / n, shallow=False)
    ]
    assert not differ, f"Copy these from {source}: {differ}"
    upstream = (source / "mobile_fr3_duo.xml").read_text()
    local = (PACKAGE / "mjcf" / "mobile_fr3_duo.xml").read_text()
    assert len(PLANAR_RANGE.findall(local)) == 2
    assert PLANAR_RANGE.sub(r'\1""', local) == PLANAR_RANGE.sub(r'\1""', upstream)


def test_planar_range_matches_the_urdf_travel_limit(model):
    config = yaml.safe_load((PACKAGE / "config" / "config.yaml").read_text())
    params = {
        key: value
        for entry in config["hardware"]["robot_description"]["urdf_params"]
        for key, value in entry.items()
    }
    limit = float(params["base_travel_limit"])
    for joint in ("planar_x", "planar_y"):
        np.testing.assert_allclose(model.joint(joint).range, [-limit, limit])


@pytest.fixture(scope="module")
def model():
    mujoco = pytest.importorskip("mujoco")
    return mujoco.MjModel.from_xml_path(str(SCENE))


def test_visible_geoms_fit_the_render_limit(model):
    visible = int(np.sum(model.geom_group <= 2)) + int(np.sum(model.site_group <= 2))
    assert visible <= RENDER_GEOM_LIMIT - RENDER_GEOM_MARGIN


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
        np.testing.assert_allclose(model.key_qpos[key][:3], 0.0)


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
    from PIL import Image

    info = yaml.safe_load((PACKAGE / "maps" / "market.yaml").read_text())
    grid = np.array(Image.open(PACKAGE / "maps" / info["image"]))[::-1]
    resolution, origin = info["resolution"], info["origin"]

    def cell(x, y):
        return grid[
            int((y - origin[1]) / resolution), int((x - origin[0]) / resolution)
        ]

    assert cell(0.0, 0.0) == 254
    # Shelves are the only joint-free bodies hung straight off the world body.
    shelves = [
        b
        for b in range(1, model.nbody)
        if model.body_parentid[b] == 0
        and model.body_jntnum[b] == 0
        and model.body_geomnum[b]
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
