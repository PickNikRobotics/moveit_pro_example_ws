"""Mapping must not let SLAM's estimate steer ground-truth navigation."""

import importlib.util
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


def test_navigation_rewrite_leaves_slam_odometry_and_frames_unchanged() -> None:
    """Nav2 uses only truth frames/map while SLAM still estimates from wheel/IMU odometry."""
    module = load_module(
        "market_driver_launch",
        PACKAGE / "launch/sim/robot_drivers_to_persist_sim.launch.py",
    )
    original = yaml.safe_load((PACKAGE / "params/nav2_params.yaml").read_text())
    rewritten = module.ground_truth_navigation_parameters(
        str(PACKAGE / "params/nav2_params.yaml")
    )
    params = yaml.safe_load(Path(rewritten.perform(LaunchContext())).read_text())
    assert params["slam_toolbox"] == original["slam_toolbox"]
    for name in ("local_costmap", "global_costmap"):
        costmap = params[name][name]["ros__parameters"]
        assert costmap["global_frame"] == "world"
        assert costmap["robot_base_frame"] == "chassis_link_ground_truth"
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


def test_navigation_scan_preserves_acquisition_and_uses_rigid_truth_mount() -> None:
    """Relabel only a cloned scan and reuse chassis-relative extrinsics, never world or odom."""
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
            "chassis_link",
            "lidar_front_ROS",
        )
        sent = node.broadcaster.sendTransform.call_args.args[0]
        assert sent.header.frame_id == "chassis_link_ground_truth"
        assert sent.transform.translation.x == pytest.approx(0.45)
        out = node.publishers_by_side["front"].publish.call_args.args[0]
        assert out.header.frame_id == sent.child_frame_id
        assert out.header.stamp == scan.header.stamp
        assert out.ranges == scan.ranges
        assert scan.header.frame_id == "lidar_front_ROS"
    finally:
        node.destroy_node()
        rclpy.shutdown()
