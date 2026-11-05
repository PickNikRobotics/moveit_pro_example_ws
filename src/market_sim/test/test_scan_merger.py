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

"""The SLAM scan must represent both lidars at one acquisition instant."""

import sys
from pathlib import Path
from unittest.mock import Mock

import pytest
import rclpy
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import LaserScan

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from scan_merger import ScanMerger  # noqa: E402


@pytest.fixture
def merger():
    rclpy.init()
    node = ScanMerger()
    node.scan_pub = Mock()
    node.mapping_pub = Mock()
    transform = TransformStamped()
    transform.transform.rotation.w = 1.0
    node._lookup_transform = Mock(return_value=transform)
    yield node
    node.destroy_node()
    rclpy.shutdown()


def scan(stamp, angle):
    msg = LaserScan()
    msg.header.stamp = stamp
    msg.header.frame_id = "lidar"
    msg.angle_min = angle
    msg.angle_increment = 0.01
    msg.range_min = 0.1
    msg.range_max = 25.0
    msg.ranges = [2.0]
    return msg


def test_synchronized_mapping_scan_contains_both_lidars_without_phantom_rays(
    merger,
) -> None:
    """Keep both opposed returns and invalidate unobserved bins for slam_toolbox."""
    stamp = merger.get_clock().now().to_msg()
    merger._front_callback(scan(stamp, 0.0))
    merger._rear_callback(scan(stamp, 3.14159265))
    merger._merge_and_publish()
    msg = merger.mapping_pub.publish.call_args.args[0]
    assert msg.header.stamp == stamp
    assert msg.header.frame_id == "base_link"
    assert sum(value >= msg.range_min for value in msg.ranges) == 2
    assert all(value == 0.0 or value == pytest.approx(2.0) for value in msg.ranges)


def test_mapping_does_not_combine_different_acquisition_instants(merger) -> None:
    """Continue navigation output but withhold the single-instant SLAM output."""
    first = merger.get_clock().now().to_msg()
    second = merger.get_clock().now().to_msg()
    assert first != second
    merger._front_callback(scan(first, 0.0))
    merger._rear_callback(scan(second, 3.14159265))
    merger._merge_and_publish()
    merger.scan_pub.publish.assert_called_once()
    merger.mapping_pub.publish.assert_not_called()
