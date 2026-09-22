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


"""Pins what script/scan_localization_relay.py puts in front of AMCL.

AMCL reads one topic, so the relay is what decides which lidars it sees. The checks
below run recorded /scan_{front,rear}_filtered messages through the node and assert
against the scans themselves -- their ray count, their angular window and the bearings
they actually cover around the base -- rather than against the launch file's syntax.

test/data/recorded_filtered_scans.json holds three consecutive front/rear pairs taken
off a driving robot; a synthetic scan could be given any coverage the test wanted, so
the coverage claim is made against recorded returns.
"""

import copy
import importlib.util
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
from sensor_msgs.msg import LaserScan
import yaml


PACKAGE_ROOT = Path(__file__).parents[1]
MODEL = PACKAGE_ROOT / "description" / "ur5e_ridgeback.xml"
NAV2_PARAMS = PACKAGE_ROOT / "params" / "nav2_params.yaml"
RELAY = PACKAGE_ROOT / "script" / "scan_localization_relay.py"
RECORDED = PACKAGE_ROOT / "test" / "data" / "recorded_filtered_scans.json"

# 45 deg apiece. Coarse on purpose: a sector is called covered by any single valid
# return in it, and an empty sector has to mean a direction the pair cannot see at
# all rather than a stretch of hangar floor that happened to be out of range.
SECTORS = 8


def load_relay_module():
    spec = importlib.util.spec_from_file_location("scan_localization_relay", RELAY)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def recorded_scans():
    """The recorded messages, in the order they arrived, as LaserScans."""
    scans = []
    for record in json.loads(RECORDED.read_text())["scans"]:
        scan = LaserScan()
        scan.header.frame_id = record["frame_id"]
        scan.header.stamp.sec = int(record["stamp"])
        scan.header.stamp.nanosec = round((record["stamp"] % 1) * 1e9)
        for field in (
            "angle_min",
            "angle_max",
            "angle_increment",
            "time_increment",
            "scan_time",
            "range_min",
            "range_max",
        ):
            setattr(scan, field, record[field])
        scan.ranges = [float(value) for value in record["ranges"]]
        scans.append((record["topic"], scan))
    return scans


def lidar_cameras():
    """The two MJCF lidar cameras, keyed by name. user[0]==2 is THREE_D_LIDAR."""
    root = ET.parse(MODEL).getroot()
    return {
        camera.attrib["name"]: camera
        for camera in root.iter("camera")
        if int(float(camera.attrib.get("user", "0").split()[0])) == 2
    }


def quat_rotate(quat, vector):
    """Rotate vector by an MJCF (w, x, y, z) quaternion."""
    w, x, y, z = quat
    vx, vy, vz = vector
    tx, ty, tz = 2 * (y * vz - z * vy), 2 * (z * vx - x * vz), 2 * (x * vy - y * vx)
    return (
        vx + w * tx + y * tz - z * ty,
        vy + w * ty + z * tx - x * tz,
        vz + w * tz + x * ty - y * tx,
    )


def sensor_frame_yaws():
    """Yaw of each lidar_*_ROS frame (beam 0) in ridgeback_base_link, from the MJCF.

    A MuJoCo camera looks down its own -Z, and the fan is centered on that view
    direction, so beam 0 sits half a sweep clockwise of it. The mounts are unrotated
    children of ridgeback_base_link, so the camera quat alone sets the bearing.
    """
    root = ET.parse(MODEL).getroot()
    yaws = {}
    for body in root.iter("body"):
        for camera in body.findall("camera"):
            if camera.attrib["name"] not in lidar_cameras():
                continue
            assert not {"quat", "euler", "axisangle", "xyaxes", "zaxis"} & set(
                body.attrib
            )
            assert body in root.find(".//body[@name='ridgeback_base_link']")
            quat = [float(value) for value in camera.attrib["quat"].split()]
            view_x, view_y, _ = quat_rotate(quat, (0.0, 0.0, -1.0))
            sweep = math.radians(float(camera.attrib["user"].split()[1]))
            center = math.atan2(view_y, view_x)
            yaws[camera.attrib["name"] + "_ROS"] = center - sweep / 2
    return yaws


def covered_sectors(scan, yaw):
    """Which sectors around the base carry a valid return from this scan."""
    sectors = set()
    for beam, distance in enumerate(scan.ranges):
        if not scan.range_min <= distance <= scan.range_max:
            continue
        bearing = scan.angle_min + beam * scan.angle_increment + yaw
        sectors.add(int(bearing % (2 * math.pi) / (2 * math.pi) * SECTORS))
    return sectors


@pytest.fixture(scope="module")
def relay():
    """A ScanLocalizationRelay on an initialized rclpy context."""
    import rclpy

    module = load_relay_module()
    rclpy.init()
    node = module.ScanLocalizationRelay()
    yield node
    node.destroy_node()
    rclpy.shutdown()


def published_by_relay(node, scans):
    """Run scans through the node and return what it published, in order."""
    published = []
    publisher = node.publisher
    node.publisher = _Capture(published)
    try:
        for scan in scans:
            node._relay(scan)
    finally:
        node.publisher = publisher
    return published


def test_each_recorded_scan_is_one_full_sensor_fan():
    """Ray count and angular window, checked against the MJCF that produces them.

    AMCL now scores one lidar's own fan per update instead of a combined scan, so the
    fan is the interface: 811 beams over the MJCF's 270 deg sweep, starting at the
    lidar_*_ROS frame's X axis.
    """
    cameras = lidar_cameras()
    assert set(cameras) == {"lidar_front", "lidar_rear"}
    beams = {int(camera.attrib["resolution"].split()[0]) for camera in cameras.values()}
    sweeps = {
        round(float(camera.attrib["user"].split()[1]), 6) for camera in cameras.values()
    }
    assert beams == {811} and sweeps == {270.0}

    for _, scan in recorded_scans():
        assert len(scan.ranges) == 811
        assert scan.angle_min == pytest.approx(0.0)
        assert scan.angle_max == pytest.approx(math.radians(270.0), abs=1e-4)
        assert scan.angle_increment == pytest.approx(
            math.radians(270.0) / 810, abs=1e-6
        )


def test_relay_republishes_each_scan_untouched(relay):
    """One scan in, the same scan out: nothing is combined, resampled or restamped.

    A message carrying two lidars' returns is the thing this arrangement exists to
    avoid, so the output being message-for-message identical to the input is the
    property worth pinning.
    """
    recorded = recorded_scans()
    sent = [copy.deepcopy(scan) for _, scan in recorded]
    published = published_by_relay(relay, sent)

    assert len(published) == len(recorded)
    for (_, source), relayed in zip(recorded, published):
        assert relayed == source


def test_both_lidars_reach_the_localizer(relay):
    """Both sensors stay in use; they only stop sharing a message."""
    module = load_relay_module()
    recorded = recorded_scans()
    assert {topic for topic, _ in recorded} == set(module.SOURCE_TOPICS)

    published = published_by_relay(relay, [scan for _, scan in recorded])
    frames = [scan.header.frame_id for scan in published]
    assert frames.count("lidar_front_ROS") == frames.count("lidar_rear_ROS") == 3


def test_a_front_rear_pair_covers_every_bearing_around_the_base():
    """The two fans together see all round the base; neither one does alone.

    This is the coverage the merged 360 deg scan used to provide. It survives because
    both lidars still feed AMCL -- the pair just arrives as two messages.
    """
    yaws = sensor_frame_yaws()
    assert set(yaws) == {"lidar_front_ROS", "lidar_rear_ROS"}

    by_frame = {}
    for _, scan in recorded_scans():
        by_frame.setdefault(scan.header.frame_id, scan)
    front = covered_sectors(by_frame["lidar_front_ROS"], yaws["lidar_front_ROS"])
    rear = covered_sectors(by_frame["lidar_rear_ROS"], yaws["lidar_rear_ROS"])

    assert front | rear == set(range(SECTORS))
    assert front != set(range(SECTORS))
    assert rear != set(range(SECTORS))


def test_amcl_reads_the_relay_topic_and_the_costmaps_do_not():
    """The wiring: AMCL on the relayed topic, everything else on the raw filtered ones.

    The costmap obstacle layers and slam_toolbox take the per-lidar scans directly and
    are deliberately untouched by the relay.
    """
    module = load_relay_module()
    params = yaml.safe_load(NAV2_PARAMS.read_text())

    assert params["amcl"]["ros__parameters"]["scan_topic"] == module.LOCALIZATION_TOPIC
    assert (
        params["slam_toolbox"]["ros__parameters"]["scan_topic"]
        == "/scan_front_filtered"
    )

    observed = set()
    for costmap in ("local_costmap", "global_costmap"):
        layer = params[costmap][costmap]["ros__parameters"]["obstacle_layer"]
        for source in layer["observation_sources"].split():
            observed.add(layer[source]["topic"])
    assert observed == set(module.SOURCE_TOPICS)


class _Capture:
    """Stands in for a Publisher so the relay can run without a live graph."""

    def __init__(self, sink):
        self._sink = sink

    def publish(self, message):
        self._sink.append(message)
