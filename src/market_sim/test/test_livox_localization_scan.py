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

"""The live slice must retain the h170 survey's frame, gates, binning and map wiring."""

import hashlib
import importlib.util
import math
import sys
import time
import xml.etree.ElementTree as ET
from pathlib import Path
from unittest.mock import Mock

import mujoco
import numpy as np
import pytest
import rclpy
import yaml
from ament_index_python.packages import (
    get_package_prefix,
    get_package_share_directory,
)
from launch import LaunchContext
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2, PointField

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE / "scripts"))
from livox_localization_scan import (
    LivoxLocalizationScan,
    SENSOR_BASE_XYZ,
    SLICE_HEIGHT_M,
    slice_livox_cloud,
)  # noqa: E402


def cloud(points, *, bigendian=False):
    """Packed native XYZ in the inverted livox_frame, with other samples invalid."""
    xyz = np.full((333 * 60, 3), np.nan, np.float32)
    xyz[: len(points)] = points
    msg = PointCloud2()
    msg.header.frame_id = "livox_frame"
    msg.header.stamp.sec = 123
    msg.header.stamp.nanosec = 456
    msg.width, msg.height = 333, 60
    msg.point_step, msg.row_step = 12, 333 * 12
    msg.fields = [
        PointField(name=n, offset=i * 4, datatype=PointField.FLOAT32, count=1)
        for i, n in enumerate("xyz")
    ]
    msg.is_bigendian = bigendian
    msg.data = xyz.astype(">f4" if bigendian else "<f4").tobytes()
    return msg


def finite_ranges(scan):
    return np.asarray(scan.ranges)[np.isfinite(scan.ranges)]


@pytest.mark.parametrize("bigendian", [False, True])
def test_scan_preserves_survey_geometry_stamp_and_native_cloud(bigendian):
    msg = cloud([[2, 0, 0], [0, -3, 0]], bigendian=bigendian)
    original = bytes(msg.data)
    scan = slice_livox_cloud(msg)
    assert bytes(msg.data) == original
    assert scan.header.stamp == msg.header.stamp
    assert scan.header.frame_id == "livox_slice"
    assert scan.range_min == pytest.approx(0.1)
    assert scan.range_max == 40.0
    assert scan.scan_time == pytest.approx(0.1)
    assert scan.time_increment == 0.0
    assert len(scan.ranges) == 333
    assert scan.angle_min == pytest.approx(-math.pi + math.pi / 333)
    assert scan.angle_increment == pytest.approx(2 * math.pi / 333)
    assert scan.angle_max == pytest.approx(scan.angle_min + 332 * scan.angle_increment)
    assert scan.ranges[166] == 2.0  # X forward from the sensor's projected origin.
    assert scan.ranges[249] == 3.0  # Native Y right becomes base/scan Y left.
    assert len(finite_ranges(scan)) == 2
    assert all(value > 0 for value in scan.ranges)


def test_height_band_is_in_base_link_not_native_or_world():
    # Native Z points down: +/-0.02 m maps to base Z=1.68..1.72 m.
    scan = slice_livox_cloud(
        cloud([[2, 0, 0.02], [0, 3, -0.02], [-4, 0, 0.02001], [0, -5, -0.02001]])
    )
    np.testing.assert_allclose(sorted(finite_ranges(scan)), [2, 3])


def test_half_metre_self_gate_uses_sensor_xy_not_base_xy_or_3d_range():
    # x=0.49 is 0.70 m from base_link, but inside the sensor-centred self gate.
    # hypot(0.4999, 0.02) >0.50 must still be excluded by the planar self gate.
    # A rear return at x=-0.50 is only 0.29 m from base, but must survive.
    scan = slice_livox_cloud(
        cloud([[0.49, 0, 0], [0.4999, 0, 0.02], [-0.50, 0, 0], [0, -0.50, 0]])
    )
    np.testing.assert_allclose(finite_ranges(scan), [0.5, 0.5])


def test_source_range_limit_remains_3d_and_nearest_planar_hit_wins():
    # The second 40 m point exceeds the original 3D source-range limit, though
    # its planar range is 40.0. An invalid component can never produce a hit.
    scan = slice_livox_cloud(
        cloud([[40, 0, 0], [0, 40, 0.02], [3, 0, 0], [2, 0, 0.019], [1, 0, np.inf]])
    )
    np.testing.assert_allclose(finite_ranges(scan), [2])
    assert np.isinf(scan.ranges[83])


def test_beams_use_nearest_centres_and_wrap_at_the_seam():
    increment = 2 * math.pi / 333
    minimum = -math.pi + math.pi / 333
    # Edge-floor binning would put the first return in bin 19, not bin 20.
    angles = [minimum + 19.8 * increment, minimum + 20.2 * increment, -math.pi, math.pi]
    points = [
        [r * math.cos(a), -r * math.sin(a), 0] for r, a in zip((3, 2, 4, 5), angles)
    ]
    scan = slice_livox_cloud(cloud(points))
    assert scan.ranges[20] == pytest.approx(2.0)
    assert scan.ranges[332] == 4.0
    assert len(finite_ranges(scan)) == 2


@pytest.mark.parametrize(
    "change", ["frame", "dimensions", "stride", "fields", "truncated"]
)
def test_rejects_other_source_contracts(change):
    msg = cloud([[2, 0, 0]])
    if change == "frame":
        msg.header.frame_id = "base_link"
    elif change == "dimensions":
        msg.height = 141
    elif change == "stride":
        msg.row_step += 4
    elif change == "fields":
        msg.fields[0].datatype = PointField.UINT32
    else:
        msg.data = msg.data[:-1]
    with pytest.raises(ValueError, match="333x60 packed XYZ"):
        slice_livox_cloud(msg)


def test_stopped_duplicate_older_or_invalid_source_is_not_replayed():
    rclpy.init()
    node = LivoxLocalizationScan()
    node._publisher = Mock()
    try:
        msg = cloud([[2, 0, 0]])
        node._receive(msg)
        node._receive(msg)
        msg.header.stamp.sec -= 1
        node._receive(msg)
        assert node._publisher.publish.call_count == 1
        msg.header.stamp.sec += 2
        msg.header.frame_id = "wrong"
        node._receive(msg)
        assert node._publisher.publish.call_count == 1
        msg.header.frame_id = "livox_frame"
        node._receive(msg)  # Invalid input must not consume this fresh stamp.
        assert node._publisher.publish.call_count == 2
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_ros_delivers_one_scan_per_fresh_cloud_and_stops_with_the_source():
    rclpy.init()
    node = LivoxLocalizationScan()
    received = []
    try:
        source = node.create_publisher(
            PointCloud2, "/livox/lidar", qos_profile_sensor_data
        )
        node.create_subscription(
            LaserScan, "/livox/scan", received.append, qos_profile_sensor_data
        )
        deadline = time.monotonic() + 3
        while source.get_subscription_count() == 0 and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.02)
        assert source.get_subscription_count() > 0
        msg = cloud([[2, 0, 0]])
        stamps = []
        for _ in range(5):
            msg.header.stamp = node.get_clock().now().to_msg()
            stamps.append(msg.header.stamp)
            source.publish(msg)
            deadline = time.monotonic() + 1
            while len(received) < len(stamps) and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.01)
        assert [m.header.stamp for m in received] == stamps
        source.publish(msg)  # Duplicate must not restart delivery.
        deadline = time.monotonic() + 0.3
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.02)
        assert len(received) == len(stamps)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_projection_calibration_matches_the_unchanged_fixed_inverted_mount():
    plugin = Path(get_package_prefix("mujoco_package_uri")) / "bin/mujoco_plugin"
    mujoco.mj_loadPluginLibrary(
        str(plugin / "libmujoco_package_uri_resource_provider.so")
    )
    model = mujoco.MjModel.from_xml_path(str(PACKAGE / "mjcf/scene.xml"))
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, 0)
    mujoco.mj_forward(model, data)
    base = model.body("base_link").id
    sensor = model.body("livox_frame").id
    base_rotation = data.xmat[base].reshape(3, 3)
    sensor_origin = (data.xpos[sensor] - data.xpos[base]) @ base_rotation
    np.testing.assert_allclose(sensor_origin, SENSOR_BASE_XYZ, atol=1e-6)
    np.testing.assert_allclose(
        base_rotation.T @ data.xmat[sensor].reshape(3, 3),
        np.diag([1, -1, -1]),
        atol=1e-6,
    )
    assert SLICE_HEIGHT_M == SENSOR_BASE_XYZ[2]


def test_installed_map_is_the_verified_partial_survey_not_the_geometry_control():
    share = Path(get_package_share_directory("market_sim"))
    metadata = yaml.safe_load((share / "maps/market_livox_h170.yaml").read_text())
    assert metadata == {
        "image": "market_livox_h170.png",
        "mode": "trinary",
        "resolution": 0.05,
        "origin": [-46.132, -7.747, 0],
        "negate": 0,
        "occupied_thresh": 0.65,
        "free_thresh": 0.196,
    }
    digest = hashlib.sha256(
        (share / "maps" / metadata["image"]).read_bytes()
    ).hexdigest()
    assert digest == "4434bfc188506534226b1ee9b5893ff2b2736e98cfd6c1b43f5219ef291abb7a"


def test_launch_localizer_mapper_and_refinement_use_the_same_slice():
    spec = importlib.util.spec_from_file_location(
        "market_driver_launch",
        PACKAGE / "launch/sim/robot_drivers_to_persist_sim.launch.py",
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    description = module.generate_launch_description()
    context = LaunchContext()
    arguments = {a.name: a for a in description.entities if hasattr(a, "default_value")}
    default_map = "".join(
        s.perform(context) for s in arguments["localization_map"].default_value
    )
    assert Path(default_map).name == "market_livox_h170.yaml"
    text = (PACKAGE / "launch/sim/robot_drivers_to_persist_sim.launch.py").read_text()
    assert 'executable="livox_localization_scan.py"' in text
    assert 'static_transform("base_link", "livox_slice", x=0.210)' in text
    params = yaml.safe_load((PACKAGE / "params/nav2_params.yaml").read_text())
    assert params["amcl"]["ros__parameters"]["scan_topic"] == "/livox/scan"
    assert params["slam_toolbox"]["ros__parameters"]["scan_topic"] == "/livox/scan"
    assert params["amcl"]["ros__parameters"]["map_topic"] == "/localization_map"
    for name in ("local_costmap", "global_costmap"):
        obstacle = params[name][name]["ros__parameters"]["obstacle_layer"]
        assert [obstacle[f"scan_{side}"]["topic"] for side in ("front", "rear")] == [
            "/scan_front",
            "/scan_rear",
        ]
    tree = ET.parse(PACKAGE / "objectives/refine_localization_in_place_subtree.xml")
    assert [
        a.get("topic_name") for a in tree.findall(".//Action[@ID='GetLaserScan']")
    ] == ["/livox/scan"]
    assert len(tree.findall(".//Action[@ID='ScanMatchResidual']")) == 1
