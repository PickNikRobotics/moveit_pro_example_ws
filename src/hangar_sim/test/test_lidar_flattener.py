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


"""Pins the contract script/lidar_flattener.py has to keep with its consumers.

The flattener is the only thing standing between the MJCF's depth-camera lidars and
params/laser_filter_params.yaml, dual_laser_merger, AMCL and both costmap obstacle layers,
all of which were written against the <rangefinder> path. Its published window, beam order
and beam count therefore are the interface, and each is checked here against the MJCF it
has to agree with rather than against a copy of the numbers.
"""

import ast
import importlib.util
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pytest
from sensor_msgs.msg import PointCloud2, PointField
import yaml


PACKAGE_ROOT = Path(__file__).parents[1]
MODEL = PACKAGE_ROOT / "description" / "ur5e_ridgeback.xml"
FILTERS = PACKAGE_ROOT / "params" / "laser_filter_params.yaml"
FLATTENER = PACKAGE_ROOT / "script" / "lidar_flattener.py"
LAUNCH = PACKAGE_ROOT / "launch" / "sim" / "robot_drivers_to_persist_sim.launch.py"

# Both units run the same code path, keyed only by name for the publisher and frame
# lookup, so every behavioral check below is run against each of them rather than
# against the front one alone.
UNITS = ("lidar_front", "lidar_rear")


def load_flattener_module():
    spec = importlib.util.spec_from_file_location("lidar_flattener", FLATTENER)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def launch_static_transforms():
    """The static_transform_publisher argument lists, read out of the launch file.

    Parsed rather than imported: the launch file pulls in the whole ROS launch stack,
    and the point here is to read what it declares, not to run it.
    """
    tree = ast.parse(LAUNCH.read_text())
    transforms = {}
    for node in ast.walk(tree):
        if not isinstance(node, ast.Assign):
            continue
        call = node.value
        if not isinstance(call, ast.Call) or getattr(call.func, "id", None) != "Node":
            continue
        kwargs = {kw.arg: kw.value for kw in call.keywords}
        executable = kwargs.get("executable")
        if (
            not isinstance(executable, ast.Constant)
            or executable.value != "static_transform_publisher"
        ):
            continue
        arguments = kwargs.get("arguments")
        if not isinstance(arguments, ast.List):
            continue
        values = [
            element.value
            for element in arguments.elts
            if isinstance(element, ast.Constant)
        ]
        # x y z yaw pitch roll parent child
        if len(values) == 8:
            transforms[values[-1]] = values
    return transforms


def mjcf_mount_positions():
    """pos of each lidar_*_mount body, the ground truth the static TFs must match."""
    root = ET.parse(MODEL).getroot()
    return {
        body.attrib["name"]: [float(value) for value in body.attrib["pos"].split()]
        for body in root.iter("body")
        if body.attrib.get("name", "").startswith("lidar_")
        and body.attrib["name"].endswith("_mount")
    }


def lidar_cameras():
    root = ET.parse(MODEL).getroot()
    return {
        camera.attrib["name"]: camera
        for camera in root.iter("camera")
        if int(float(camera.attrib.get("user", "0").split()[0])) == 2
    }


@pytest.fixture(scope="module")
def flattener():
    """A LidarFlattener with its declared defaults, on an initialized rclpy context."""
    import rclpy

    module = load_flattener_module()
    rclpy.init()
    node = module.LidarFlattener()
    yield node
    node.destroy_node()
    rclpy.shutdown()


def make_cloud(width, height, ranges_by_column, elevation_row):
    """An organized cloud shaped like the plugin's, with one range per column.

    Columns not named in ranges_by_column are NaN, which is how the plugin's stitcher
    leaves a beam that hit nothing. Only `elevation_row` carries points, so a flattener
    reading the wrong row sees an all-NaN scan and cannot pass by accident.
    """
    xyz = np.full((height, width, 3), np.nan, dtype=np.float32)
    fov_x = 270.0
    for column, distance in ranges_by_column.items():
        azimuth = math.radians(column * (fov_x / (width - 1)) - fov_x / 2.0)
        xyz[elevation_row, column] = (
            distance * math.sin(azimuth),
            0.0,
            distance * math.cos(azimuth),
        )

    cloud = PointCloud2()
    cloud.height, cloud.width = height, width
    cloud.point_step = 12
    cloud.row_step = width * 12
    cloud.is_dense = False
    cloud.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    cloud.data = xyz.tobytes()
    return cloud


def test_declared_window_matches_the_mjcf(flattener):
    """The node's sweep and range defaults must be the ones the MJCF declares.

    The plugin reads FOV_X, RANGE_MIN and RANGE_MAX off the camera and uses them to
    build the cloud; the flattener redeclares the same three as parameters because a
    cloud carries none of them. Two sources of truth, so they get compared.
    """
    cameras = lidar_cameras()
    assert set(cameras) == {"lidar_front", "lidar_rear"}
    for camera in cameras.values():
        _, fov_x, range_min, range_max = (
            float(value) for value in camera.attrib["user"].split()
        )
        assert flattener.angle_min == pytest.approx(0.0)
        assert flattener.angle_max - flattener.angle_min == pytest.approx(
            math.radians(fov_x)
        )
        assert flattener.range_min == pytest.approx(range_min)
        assert flattener.range_max == pytest.approx(range_max)


def test_declared_window_matches_the_filter_bounds():
    """laser_filter_params.yaml's self-hit arcs assume angle_min=0, angle_max=270 deg."""
    filters = yaml.safe_load(FILTERS.read_text())
    for chain in ("laser_angular_filter_front", "laser_angular_filter_rear"):
        lower = filters[chain]["ros__parameters"]["filter1"]["params"]
        upper = filters[chain]["ros__parameters"]["filter2"]["params"]
        # Both arcs are anchored to the ends of the sweep, and each is deliberately
        # widened past its end so the boundary beam cannot slip through. That only
        # holds while the flattener publishes 0 to 270 deg: a window starting
        # anywhere else leaves the self-hits these two stages exist to remove.
        assert lower["lower_angle"] < 0.0
        assert upper["upper_angle"] >= math.radians(270.0)
        assert upper["upper_angle"] - math.radians(270.0) < math.radians(1.0)


def test_increment_is_the_tim571_datasheet_resolution(flattener):
    """811 beams over 270 deg is exactly 1/3 deg, the TIM571's angular resolution."""
    width = int(lidar_cameras()["lidar_front"].attrib["resolution"].split()[0])
    assert width == 811
    increment = (flattener.angle_max - flattener.angle_min) / (width - 1)
    assert math.degrees(increment) == pytest.approx(1.0 / 3.0)


@pytest.mark.parametrize("unit", UNITS)
def test_row_is_reversed_so_beam_zero_is_the_last_column(flattener, unit):
    """Cloud columns run clockwise; LaserScan runs counter-clockwise.

    This is the one step that is easy to get backwards, and getting it backwards
    mirrors the map rather than failing outright, so it is pinned with a distinct
    range at each end of the sweep.
    """
    width, height = 811, 3
    cloud = make_cloud(width, height, {0: 3.0, width - 1: 7.0}, elevation_row=1)
    scan = flatten(flattener, cloud, unit)

    assert len(scan.ranges) == width
    # Column width-1 sits at the sweep's +135 deg, which is where the scan's angle_min
    # (beam 0, X of the lidar_*_ROS frame) points.
    assert scan.ranges[0] == pytest.approx(7.0, abs=1e-3)
    assert scan.ranges[-1] == pytest.approx(3.0, abs=1e-3)
    assert scan.angle_min == pytest.approx(0.0)
    assert scan.angle_max == pytest.approx(math.radians(270.0))


@pytest.mark.parametrize("unit", UNITS)
def test_only_the_elevation_zero_row_is_read(flattener, unit):
    """Rows 0 and 2 sit at +/-fovy/2 and would put floor and ceiling in the scan."""
    width, height = 811, 3
    for row in (0, 2):
        cloud = make_cloud(width, height, {400: 4.0}, elevation_row=row)
        scan = flatten(flattener, cloud, unit)
        assert all(math.isinf(value) for value in scan.ranges)

    cloud = make_cloud(width, height, {400: 4.0}, elevation_row=1)
    scan = flatten(flattener, cloud, unit)
    assert sum(1 for value in scan.ranges if math.isfinite(value)) == 1


@pytest.mark.parametrize("unit", UNITS)
def test_out_of_range_and_missing_returns_become_inf(flattener, unit):
    """AMCL and both costmap layers only skip a reading they can compare to range_max."""
    width, height = 811, 3
    cloud = make_cloud(width, height, {10: 0.01, 20: 400.0, 30: 5.0}, elevation_row=1)
    scan = flatten(flattener, cloud, unit)
    assert not any(math.isnan(value) for value in scan.ranges)
    assert sum(1 for value in scan.ranges if math.isfinite(value)) == 1
    assert scan.range_min == pytest.approx(0.05)
    assert scan.range_max == pytest.approx(25.0)


def test_scans_are_stamped_in_the_statically_published_frames(flattener):
    """Every frame the flattener stamps must actually be broadcast by the launch file.

    A scan stamped in a frame nothing publishes is not a loud failure: tf2 simply
    cannot transform it, and AMCL and both costmap layers drop the observation while
    the sensor still looks alive on the topic.
    """
    frames = {name: frame for name, (_, frame) in flattener.publishers_by_name.items()}
    assert frames == {"lidar_front": "lidar_front_ROS", "lidar_rear": "lidar_rear_ROS"}

    transforms = launch_static_transforms()
    for frame in frames.values():
        assert frame in transforms, f"{frame} is stamped but never broadcast"
        assert transforms[frame][-2] == "ridgeback_base_link"


@pytest.mark.parametrize(
    "unit,yaw_deg",
    # Beam 0 is X of the _ROS frame. The front fan is centered on +X and spans 270 deg,
    # so it starts 135 deg clockwise of center; the rear fan is centered on -X, so it
    # starts at +45 deg. Get either wrong and the ranges stay right while the bearings
    # rotate, which reads downstream as a map that will not close.
    [("lidar_front", -135.0), ("lidar_rear", 45.0)],
)
def test_static_frames_match_the_mjcf_mounts(unit, yaw_deg):
    """The static TFs are hand-copied from the MJCF, so they get compared to it."""
    transform = launch_static_transforms()[f"{unit}_ROS"]
    x, y, z, yaw = (float(value) for value in transform[:4])
    mount = mjcf_mount_positions()[f"{unit}_mount"]

    assert (x, y, z) == pytest.approx(tuple(mount)), "TF drifted from the MJCF mount"
    assert math.degrees(yaw) == pytest.approx(yaw_deg, abs=1e-4)


def flatten(node, cloud, unit="lidar_front"):
    """Run one cloud through the node as `unit` and return the LaserScan published."""
    published = []
    publisher, frame = node.publishers_by_name[unit]
    node.publishers_by_name[unit] = (_Capture(published), frame)
    try:
        node._cloud_callback(cloud, unit)
    finally:
        node.publishers_by_name[unit] = (publisher, frame)
    assert len(published) == 1
    return published[0]


class _Capture:
    """Stands in for a Publisher so the callback can run without a live graph."""

    def __init__(self, sink):
        self._sink = sink

    def publish(self, message):
        self._sink.append(message)
