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


"""Pins the scan wiring AMCL depends on: both lidars, one topic, one scan per message.

localization_launch.py relays /scan_front_filtered and /scan_rear_filtered onto the single
topic AMCL subscribes to, instead of merging them into one 360-degree scan. The relays are
read out of that launch description rather than restated here, so wiring only one of the two
fails these tests.

What this proves: every scan published on a source topic reaches the localizer's topic
unmodified, and both lidars' scans get there. What it does NOT prove: that the filter's
corrections end up drawing on both sensors. Both scans of a pair share a stamp in
simulation, so the second is gated out for lack of motion and a correction follows whichever
arrived first -- in the recorded runs, the front lidar about 90% of the time. That the
arrangement localizes well anyway is measured in the scan-sync-alternating-vs-merge report,
not here.
"""

import importlib.util
import math
from pathlib import Path
import subprocess
import time

from launch import LaunchContext
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from launch_ros.actions import LoadComposableNodes
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import LaserScan
import yaml


PACKAGE_ROOT = Path(__file__).parents[1]
LAUNCH = PACKAGE_ROOT / "launch" / "sim" / "localization_launch.py"
NAV2_PARAMS = PACKAGE_ROOT / "params" / "nav2_params.yaml"

# The fan every scan on these topics carries, fixed by the MJCF lidars and preserved by
# script/lidar_flattener.py: 811 beams over 270 deg, starting at the sensor frame's X axis.
BEAMS = 811
SWEEP = math.radians(270.0)


def declared_relays():
    """(input_topic, output_topic) for each relay localization_launch.py loads."""
    spec = importlib.util.spec_from_file_location("localization_launch", LAUNCH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    context = LaunchContext()

    def text(value):
        return perform_substitutions(context, normalize_to_list_of_substitutions(value))

    relays = []
    for action in module.generate_launch_description().entities:
        if not isinstance(action, LoadComposableNodes):
            continue
        for node in action._LoadComposableNodes__composable_node_descriptions:
            if text(node.package) != "topic_tools":
                continue
            # Only the two topics are read: the node's other parameter is a
            # launch argument, which no bare context can resolve. launch_ros
            # normalizes each value to its YAML encoding, so the scalar comes
            # back as a document rather than a bare string.
            topics = {}
            for name, value in node.parameters[0].items():
                key = text(name)
                if key in ("input_topic", "output_topic"):
                    topics[key] = yaml.safe_load(text(value))
            relays.append((topics["input_topic"], topics["output_topic"]))
    return relays


def scan(frame_id, seconds):
    """One lidar fan, shaped like the ones the filter chains publish."""
    message = LaserScan()
    message.header.frame_id = frame_id
    message.header.stamp.sec = seconds
    message.angle_min = 0.0
    message.angle_max = SWEEP
    message.angle_increment = SWEEP / (BEAMS - 1)
    message.range_min = 0.05
    message.range_max = 25.0
    # A distinct range per beam, so a relay that reordered or truncated would show up.
    message.ranges = [1.0 + (index % 97) * 0.01 for index in range(BEAMS)]
    return message


@pytest.fixture(scope="module")
def relayed():
    """Run the declared relays, publish one scan per source topic, collect the output.

    The relays are the stock topic_tools components the launch file loads, run as
    processes here because a pytest has no component container to load them into.
    """
    relays = declared_relays()
    assert relays, "localization_launch.py loads no topic_tools relay"
    output_topics = {output for _, output in relays}
    assert len(output_topics) == 1, f"relays disagree on the output topic: {relays}"
    output_topic = output_topics.pop()

    processes = [
        subprocess.Popen(
            [
                "ros2",
                "run",
                "topic_tools",
                "relay",
                "--ros-args",
                "-p",
                f"input_topic:={source}",
                "-p",
                f"output_topic:={output_topic}",
            ],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        for source, _ in relays
    ]

    rclpy.init()
    node = rclpy.create_node("scan_localization_probe")
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    received = []
    node.create_subscription(LaserScan, output_topic, received.append, 10)
    publishers = {
        source: node.create_publisher(LaserScan, source, 10) for source, _ in relays
    }

    # The relays discover each source topic's type before they subscribe, so give
    # discovery a moment rather than racing it.
    deadline = time.time() + 30.0
    while time.time() < deadline and any(
        node.count_subscribers(source) == 0 for source in publishers
    ):
        executor.spin_once(timeout_sec=0.1)

    sent = [
        (source, scan(f"lidar_{index}_ROS", 11 + index))
        for index, source in enumerate(publishers)
    ]
    for source, message in sent:
        publishers[source].publish(message)

    deadline = time.time() + 30.0
    while time.time() < deadline and len(received) < len(sent):
        executor.spin_once(timeout_sec=0.1)

    yield sent, received

    node.destroy_node()
    rclpy.shutdown()
    for process in processes:
        process.terminate()
        process.wait(timeout=10)


def test_every_source_scan_reaches_the_localizer_topic(relayed):
    """A scan handed to a relay comes back out whole: same frame, stamp, fan and ranges."""
    sent, received = relayed
    assert len(received) == len(sent)

    by_frame = {message.header.frame_id: message for message in received}
    for _, source_scan in sent:
        relayed_scan = by_frame[source_scan.header.frame_id]
        assert relayed_scan.header.stamp == source_scan.header.stamp
        assert len(relayed_scan.ranges) == BEAMS
        assert relayed_scan.angle_min == pytest.approx(0.0)
        assert relayed_scan.angle_max == pytest.approx(SWEEP)
        assert relayed_scan.angle_increment == pytest.approx(SWEEP / (BEAMS - 1))
        assert list(relayed_scan.ranges) == pytest.approx(list(source_scan.ranges))


def test_both_lidars_reach_the_localizer_topic(relayed):
    """The coverage the merged scan used to provide: both sensors, not just the front one."""
    sent, received = relayed
    assert {message.header.frame_id for message in received} == {
        source_scan.header.frame_id for _, source_scan in sent
    }
    assert len(received) == 2, "one relay per lidar, so two scans should arrive"


def test_amcl_reads_the_relayed_topic_and_the_costmaps_do_not():
    """AMCL on the relayed topic; the costmaps and slam_toolbox on the per-lidar ones."""
    relays = declared_relays()
    sources = {source for source, _ in relays}
    output_topic = {output for _, output in relays}.pop()
    params = yaml.safe_load(NAV2_PARAMS.read_text())

    assert params["amcl"]["ros__parameters"]["scan_topic"] == output_topic
    assert (
        params["slam_toolbox"]["ros__parameters"]["scan_topic"]
        == "/scan_front_filtered"
    )

    observed = set()
    for costmap in ("local_costmap", "global_costmap"):
        layer = params[costmap][costmap]["ros__parameters"]["obstacle_layer"]
        for name in layer["observation_sources"].split():
            observed.add(layer[name]["topic"])
    assert observed == sources
