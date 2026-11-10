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

"""Mapping must not let SLAM's estimate steer ground-truth navigation."""

import importlib.util
import subprocess
import sys
from pathlib import Path
from unittest.mock import Mock

import pytest
import rclpy
import yaml
from geometry_msgs.msg import TransformStamped
from launch import LaunchContext
from sensor_msgs.msg import LaserScan

PACKAGE = Path(__file__).resolve().parents[1]


def load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize("slam", ["false", "true"])
def test_truth_navigation_is_restricted_to_mapping(slam) -> None:
    """A ground-truth survey cannot silently replace normal lidar-localized navigation."""
    module = load_module(
        "market_driver_launch",
        PACKAGE / "launch/sim/robot_drivers_to_persist_sim.launch.py",
    )
    context = LaunchContext()
    context.launch_configurations["slam"] = slam
    if slam == "false":
        with pytest.raises(RuntimeError, match="requires slam:=true"):
            module.require_mapping_for_truth_navigation(context)
    else:
        assert module.require_mapping_for_truth_navigation(context) == []


def test_navigation_rewrite_leaves_slam_odometry_and_frames_unchanged() -> None:
    """Nav2 uses only truth frames/map while SLAM still estimates from planar-encoder/IMU odometry."""
    module = load_module(
        "market_driver_launch",
        PACKAGE / "launch/sim/robot_drivers_to_persist_sim.launch.py",
    )
    original = yaml.safe_load((PACKAGE / "params/nav2_params.yaml").read_text())
    rewritten = module.configure_navigation_parameters(
        str(PACKAGE / "params/nav2_params.yaml"), ground_truth=True
    )
    params = yaml.safe_load(Path(rewritten.perform(LaunchContext())).read_text())
    assert params["slam_toolbox"] == original["slam_toolbox"]
    for name in ("local_costmap", "global_costmap"):
        costmap = params[name][name]["ros__parameters"]
        assert costmap["global_frame"] == "world"
        assert costmap["robot_base_frame"] == "base_link_ground_truth"
        for side in ("front", "rear"):
            assert (
                costmap["obstacle_layer"][f"scan_{side}"]["topic"]
                == f"/navigation_scan_{side}"
            )
    assert (
        params["global_costmap"]["global_costmap"]["ros__parameters"]["static_layer"][
            "map_topic"
        ]
        == "/navigation_map"
    )
    assert (
        params["bt_navigator"]["ros__parameters"]["odom_topic"] == "/ground_truth/odom"
    )
    assert params["behavior_server"]["ros__parameters"]["local_frame"] == "world"


@pytest.mark.parametrize("ground_truth", [False, True])
def test_resolved_nav2_parameters_parse_with_ros(ground_truth) -> None:
    """The deployed parameters expand YAML aliases that ROS itself cannot parse."""
    module = load_module(
        "market_driver_launch",
        PACKAGE / "launch/sim/robot_drivers_to_persist_sim.launch.py",
    )
    configured = module.configure_navigation_parameters(
        str(PACKAGE / "params/nav2_params.yaml"), ground_truth=ground_truth
    ).perform(LaunchContext())
    params = yaml.safe_load(Path(configured).read_text())
    local = params["local_costmap"]["local_costmap"]["ros__parameters"]
    global_ = params["global_costmap"]["global_costmap"]["ros__parameters"]
    assert local["footprint"] == global_["footprint"]
    if not ground_truth:
        assert local["robot_base_frame"] == "base_link"
        assert (
            params["controller_server"]["ros__parameters"]["odom_topic"]
            == "/odom_filtered"
        )
    subprocess.run(
        [
            sys.executable,
            "-c",
            "import rclpy; rclpy.init(); rclpy.shutdown()",
            "--ros-args",
            "--params-file",
            configured,
        ],
        check=True,
        capture_output=True,
        text=True,
    )


def test_navigation_scan_preserves_acquisition_and_uses_rigid_truth_mount() -> None:
    """Relabel only a cloned scan and reuse base-relative extrinsics, never world or odom."""
    module = load_module(
        "market_navigation_scan_relay", PACKAGE / "scripts/navigation_scan_relay.py"
    )
    rclpy.init()
    node = module.NavigationScanRelay()
    try:
        transform = TransformStamped()
        transform.transform.translation.x = 0.45
        transform.transform.rotation.w = 1.0
        node.buffer = Mock()
        node.buffer.lookup_transform.return_value = transform
        node.broadcaster = Mock()
        node.publishers_by_side["front"] = Mock()
        scan = LaserScan()
        scan.header.frame_id = "lidar_front_ROS"
        scan.header.stamp.sec = 123
        scan.ranges = [1.0, 2.0]
        node.relay(scan, "front")
        node.buffer.lookup_transform.assert_called_once()
        assert node.buffer.lookup_transform.call_args.args[:2] == (
            "base_link",
            "lidar_front_ROS",
        )
        sent = node.broadcaster.sendTransform.call_args.args[0]
        assert sent.header.frame_id == "base_link_ground_truth"
        assert sent.transform.translation.x == pytest.approx(0.45)
        out = node.publishers_by_side["front"].publish.call_args.args[0]
        assert out.header.frame_id == sent.child_frame_id
        assert out.header.stamp == scan.header.stamp
        assert out.ranges == scan.ranges
        assert scan.header.frame_id == "lidar_front_ROS"
    finally:
        node.destroy_node()
        rclpy.shutdown()
