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

"""Exercise native lidar/noise, compiled mount rays, cloud shaping and fresh 10 Hz delivery."""

import importlib.util
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np
import pytest
import rclpy
from ament_index_python.packages import get_package_prefix
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE / "test"))
from test_duo_model import geom_points, robot_geoms, stowed_data  # noqa: E402

spec = importlib.util.spec_from_file_location(
    "livox_mid360_cloud", PACKAGE / "scripts/livox_mid360_cloud.py"
)
livox = importlib.util.module_from_spec(spec)
spec.loader.exec_module(livox)


@pytest.fixture(scope="module")
def model():
    plugin = Path(get_package_prefix("mujoco_package_uri")) / "bin/mujoco_plugin"
    mujoco.mj_loadPluginLibrary(
        str(plugin / "libmujoco_package_uri_resource_provider.so")
    )
    return mujoco.MjModel.from_xml_path(str(PACKAGE / "mjcf/scene.xml"))


@pytest.fixture(scope="module")
def urdf():
    return subprocess.check_output(
        ["xacro", str(PACKAGE / "description/mobile_fr3_duo.urdf.xacro")], text=True
    )


def source_cloud(*, bigendian=False, range_m=3.0):
    """One full native sweep with known directions and ranges, including negative elevations."""
    azimuth = np.radians(np.linspace(-180, 180, 333, endpoint=False) + 180 / 333)
    elevation = np.radians(np.arange(-70, 71))[:, None]
    xyz = np.empty((141, 333, 3), np.float32)
    xyz[..., 0] = range_m * np.cos(elevation) * np.sin(azimuth)
    xyz[..., 1] = -range_m * np.sin(elevation)
    xyz[..., 2] = range_m * np.cos(elevation) * np.cos(azimuth)
    msg = PointCloud2()
    msg.header.frame_id = "livox_mid360_optical_frame"
    msg.header.stamp.sec = 123
    msg.header.stamp.nanosec = 456
    msg.width, msg.height = 333, 141
    msg.point_step, msg.row_step = 12, 333 * 12
    msg.fields = [
        PointField(name=n, offset=i * 4, datatype=PointField.FLOAT32, count=1)
        for i, n in enumerate("xyz")
    ]
    msg.is_bigendian = bigendian
    msg.data = xyz.astype(">f4" if bigendian else "<f4").tobytes()
    return msg


@pytest.mark.parametrize("bigendian", [False, True])
def test_cloud_has_asymmetric_fov_density_axes_and_original_stamp(bigendian):
    msg = source_cloud(bigendian=bigendian)
    original = bytes(msg.data)
    out = livox.shape_livox_cloud(msg)
    points = np.frombuffer(out.data, dtype="<f4").reshape(60, 333, 3)
    assert out.width * out.height == 19980
    assert out.header.frame_id == "livox_frame"
    assert out.header.stamp == msg.header.stamp
    assert out.row_step == out.width * out.point_step
    assert not out.is_bigendian
    assert not out.is_dense
    assert bytes(msg.data) == original
    np.testing.assert_allclose(np.linalg.norm(points, axis=-1), 3, atol=1e-6)
    elevation = np.degrees(
        np.arctan2(points[..., 2], np.linalg.norm(points[..., :2], axis=-1))
    )
    np.testing.assert_allclose(elevation[:, 0], np.arange(-7, 53), atol=1e-5)
    # One exactly horizontal ring at row 7, and full 360-degree bearings with no duplicate seam.
    assert np.max(np.abs(points[7, :, 2])) < 1e-6
    bearing = np.unwrap(np.arctan2(points[7, :, 1], points[7, :, 0]))
    np.testing.assert_allclose(np.diff(bearing), -2 * np.pi / 333, atol=1e-6)
    assert abs(bearing[-1] - bearing[0]) == pytest.approx(
        2 * np.pi * 332 / 333, abs=1e-6
    )


def test_invalid_returns_remain_invalid_without_zero_points():
    msg = source_cloud()
    xyz = np.frombuffer(msg.data, dtype="<f4").copy().reshape(141, 333, 3)
    xyz[63, 0] = [np.nan, 1, 2]
    xyz[64, 1] = [1, np.inf, 2]
    msg.data = xyz.tobytes()
    out = livox.shape_livox_cloud(msg)
    points = np.frombuffer(out.data, dtype="<f4").reshape(60, 333, 3)
    assert np.isnan(points[0, 0]).all()
    assert np.isnan(points[1, 1]).all()
    assert np.isfinite(points[2:]).all()


@pytest.mark.parametrize(
    "change", ["frame", "dimensions", "stride", "fields", "truncated"]
)
def test_different_native_source_contract_is_rejected(change):
    msg = source_cloud()
    if change == "frame":
        msg.header.frame_id = "some_other_camera"
    elif change == "dimensions":
        msg.height = 104
    elif change == "stride":
        msg.row_step += 4
    elif change == "fields":
        msg.fields[0].datatype = PointField.UINT32
    else:
        msg.data = msg.data[:-1]
    with pytest.raises(ValueError):
        livox.shape_livox_cloud(msg)


def test_mount_and_optical_frame_agree_with_expanded_urdf(model, urdf):
    root = ET.fromstring(urdf)
    data = stowed_data(model)
    for name in ("livox_mid360_mount", "livox_frame"):
        joint = next(
            j for j in root.findall("joint") if j.find("child").get("link") == name
        )
        body = model.body(name)
        assert model.body(int(model.body_parentid[body.id])).name == joint.find(
            "parent"
        ).get("link")
        np.testing.assert_allclose(
            body.pos,
            [float(v) for v in joint.find("origin").get("xyz").split()],
            atol=1e-6,
        )
        rpy = joint.find("origin").get("rpy", "0 0 0")
        roll, pitch, yaw = [float(v) for v in rpy.split()]
        assert pitch == yaw == 0
        expected = [np.cos(roll / 2), np.sin(roll / 2), 0, 0]
        np.testing.assert_allclose(body.quat, expected, atol=1e-6)
    camera = model.camera("livox_mid360").id
    site = model.site("livox_mid360_optical_frame").id
    np.testing.assert_allclose(data.cam_xpos[camera], [0.209750, 0, 1.700], atol=1e-6)
    np.testing.assert_allclose(
        data.xmat[model.body("livox_frame").id].reshape(3, 3)[:, 2],
        [0, 0, -1],
        atol=1e-6,
    )
    # Native rows stay -7..+52; actual mount turns them into base/world -52..+7.
    cloud = livox.shape_livox_cloud(source_cloud())
    xyz = np.frombuffer(cloud.data, dtype="<f4").reshape(-1, 3)
    world = xyz @ data.xmat[model.body("livox_frame").id].reshape(3, 3).T
    elevation = np.degrees(
        np.arctan2(world[:, 2], np.linalg.norm(world[:, :2], axis=1))
    )
    assert elevation.min() == pytest.approx(-52, abs=1e-4)
    assert elevation.max() == pytest.approx(7, abs=1e-4)
    np.testing.assert_allclose(data.site_xpos[site], data.cam_xpos[camera], atol=1e-6)
    np.testing.assert_allclose(
        data.site_xmat[site].reshape(3, 3),
        data.cam_xmat[camera].reshape(3, 3) @ np.diag([1, -1, -1]),
        atol=1e-6,
    )
    assert root.find("link[@name='livox_mid360_optical_frame']") is None


@pytest.mark.parametrize("pose", ["default", "stow"])
@pytest.mark.parametrize(
    "base_pose",
    [(0, 0, 0), (-8.476, 10.557953, np.pi / 2), (34.556, 22.557953, np.pi / 2)],
)
def test_horizontal_slice_retains_shelves_and_walls_with_small_bracket_sector(
    model, pose, base_pose
):
    data = stowed_data(model) if pose == "stow" else mujoco.MjData(model)
    if pose == "default":
        mujoco.mj_resetDataKeyframe(model, data, 0)
    for name, value in zip(("planar_x", "planar_y", "planar_theta"), base_pose):
        data.qpos[model.joint(name).qposadr[0]] = value
    mujoco.mj_forward(model, data)
    start = data.cam_xpos[model.camera("livox_mid360").id].copy()
    robot = set(robot_geoms(model))
    groups = np.array([1, 1, 1, 0, 0, 0], np.uint8)
    shelf_hits, wall_hits, bracket_hits = 0, 0, 0
    bracket = model.body("livox_mid360_mount").id
    for angle in np.radians(np.arange(0, 360, 0.5)):
        geom = np.array([-1], np.int32)
        distance = mujoco.mj_ray(
            model,
            data,
            start,
            np.array([np.cos(angle), np.sin(angle), 0]),
            groups,
            1,
            -1,
            geom,
        )
        if distance < 0:
            continue  # Store doorways are genuinely open, not missing robot occlusion.
        hit = int(geom[0])
        if hit in robot:
            # C170 accepts the support post's narrow shadow, not a hidden arm/body return.
            assert model.geom_bodyid[hit] == bracket, (
                pose,
                base_pose,
                model.geom(hit).name,
            )
            bracket_hits += 1
            continue
        body_name = model.body(int(model.geom_bodyid[hit])).name
        shelf_hits += bool(
            body_name and body_name[0] in "ABCDEFGHI" and "_" in body_name
        )
        wall_hits += "wall" in (model.geom(hit).name or "")
    assert 0 < bracket_hits <= 20  # At most 10 deg of the 720-bearing horizontal slice.
    assert shelf_hits > 200
    assert wall_hits > 10


def test_centered_sensor_clears_the_measured_cap_over_its_downward_fov(model):
    data = stowed_data(model)
    base = model.body("base_link").id
    rotation = data.xmat[base].reshape(3, 3)
    spine = model.body("franka_spine").id
    points = np.vstack(
        [
            (geom_points(model, data, geom) - data.xpos[base]) @ rotation
            for geom in range(model.ngeom)
            if model.geom_bodyid[geom] == spine
            and (model.geom_contype[geom] or model.geom_conaffinity[geom])
        ]
    )
    top = points[points[:, 2] >= points[:, 2].max() - 1e-5]
    center = (top.min(axis=0) + top.max(axis=0)) / 2
    origin = (
        data.cam_xpos[model.camera("livox_mid360").id] - data.xpos[base]
    ) @ rotation
    np.testing.assert_allclose(origin[:2], center[:2], atol=1e-6)
    radial_extent = np.linalg.norm(top[:, :2] - origin[:2], axis=1).max()
    needed_height = top[:, 2].max() + radial_extent * np.tan(np.radians(52))
    assert origin[2] - needed_height > 0.03  # C170 has ~35 mm full-cap FOV clearance.


def test_added_sensor_has_no_dof_and_stays_inside_existing_navigation_footprint(model):
    data = stowed_data(model)
    assert model.nq == 31
    for name in ("livox_mid360_mount", "livox_frame"):
        body = model.body(name).id
        assert model.body_jntnum[body] == 0
        for geom in range(model.ngeom):
            if model.geom_bodyid[geom] != body:
                continue
            points = geom_points(model, data, geom)
            assert points[:, 0].min() > -0.4 and points[:, 0].max() < 0.8
            assert np.max(np.abs(points[:, 1])) < 0.36
    # Physics remains finite with the inverted sensor and provisional support.
    for _ in range(100):
        mujoco.mj_step(model, data)
        assert np.isfinite(data.qpos).all()
        assert np.isfinite(data.qvel).all()


def test_installed_native_consumer_applies_lidar_config_and_range_noise(
    model, urdf, tmp_path
):
    expanded = tmp_path / "duo.urdf"
    expanded.write_text(urdf)
    probe = (
        Path(get_package_prefix("market_sim"))
        / "lib/market_sim/market_sim_test_livox_sensor_probe"
    )
    output = subprocess.check_output(
        [str(probe), str(PACKAGE / "mjcf/scene.xml"), str(expanded)], text=True
    )
    rows = {
        line.split()[0]: [float(v) for v in line.split()[1:]]
        for line in output.splitlines()
        if line.startswith(("config ", "shell ", "dropouts "))
    }
    assert rows["config"] == pytest.approx([333, 141, 140, 360, 0.1, 40, 0.007, 0.35])
    finite, mean_error, stddev, low, high = rows["shell"]
    assert finite == 333 * 60
    assert (
        abs(mean_error) < 0.006
    )  # Single-pixel render quantization is independent of sensor noise.
    assert 0.006 < stddev < 0.012
    assert low == pytest.approx(-7, abs=0.001)
    assert high == pytest.approx(52, abs=0.001)
    assert 0.62 < rows["dropouts"][0] / (333 * 60) < 0.68


def spin_until(node, predicate, timeout=2.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline and not predicate():
        rclpy.spin_once(node, timeout_sec=0.02)
    assert predicate()


def test_node_publishes_fresh_clouds_at_10hz_and_never_replays_a_stopped_source():
    rclpy.init()
    node = livox.LivoxMid360Cloud()
    received = []
    ages = []

    def receive(msg):
        received.append((time.monotonic(), msg))
        ages.append((node.get_clock().now().nanoseconds - livox.stamp_ns(msg)) * 1e-9)

    try:
        node.create_subscription(
            PointCloud2,
            "/livox/lidar",
            receive,
            qos_profile_sensor_data,
        )
        source = node.create_publisher(
            PointCloud2, "/livox/mujoco/points", qos_profile_sensor_data
        )
        spin_until(node, lambda: source.get_subscription_count() > 0)
        msg = source_cloud()
        start = time.monotonic()
        next_source = start
        while time.monotonic() - start < 1.5:
            now = time.monotonic()
            if now >= next_source:
                msg.header.stamp = node.get_clock().now().to_msg()
                source.publish(msg)
                next_source += 0.05
            rclpy.spin_once(node, timeout_sec=0.005)
        assert 11 <= len(received) <= 17
        intervals = np.diff([t for t, _ in received])
        assert np.median(intervals) == pytest.approx(0.1, abs=0.025)
        stamps = [livox.stamp_ns(m) for _, m in received]
        assert all(b > a for a, b in zip(stamps, stamps[1:]))
        assert all(
            m.header.frame_id == "livox_frame" and m.width * m.height == 19980
            for _, m in received
        )
        assert np.median(ages) < 0.05  # Publish on arrival, not a timer phase.
        # Allow in-flight DDS delivery, then no repeats while the source is stopped.
        until = time.monotonic() + 0.3
        while time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=0.02)
        count = len(received)
        until = time.monotonic() + 0.35
        while time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=0.02)
        assert len(received) == count
        # An older acquisition, including a repeated last sweep, cannot restart output.
        source.publish(msg)
        until = time.monotonic() + 0.15
        while time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=0.02)
        assert len(received) == count
        msg.header.stamp = node.get_clock().now().to_msg()
        source.publish(msg)
        spin_until(node, lambda: len(received) > count)
    finally:
        node.destroy_node()
        rclpy.shutdown()
