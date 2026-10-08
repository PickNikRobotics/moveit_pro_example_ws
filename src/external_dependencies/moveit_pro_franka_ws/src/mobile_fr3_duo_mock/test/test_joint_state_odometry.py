"""Tests for script/joint_state_odometry.py. No ROS: the node runs on a stand-in rclpy."""

import math
import sys
import types
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "script"))

import joint_state_odometry  # noqa: E402
from joint_state_odometry import planar_odometry  # noqa: E402


def test_pose_is_the_joint_positions():
    pose, _ = planar_odometry([1.0, -2.0, 0.5], [0.0, 0.0, 0.0])
    assert pose == pytest.approx((1.0, -2.0, 0.5))


def test_world_velocity_is_rotated_into_the_base_frame():
    # Heading +90 deg: moving along world +y is moving forward.
    _, twist = planar_odometry([0.0, 0.0, math.pi / 2], [0.0, 0.3, 0.2])
    assert twist == pytest.approx((0.3, 0.0, 0.2))


def test_sideways_motion_at_a_heading():
    # Heading 180 deg: world -y is the base's +y.
    _, twist = planar_odometry([0.0, 0.0, math.pi], [0.0, -0.2, 0.0])
    assert twist == pytest.approx((0.0, 0.2, 0.0))


def _vector(**fields):
    return types.SimpleNamespace(**{k: fields.get(k, 0.0) for k in "xyzw"})


def _odometry():
    ns = types.SimpleNamespace
    return ns(
        header=ns(stamp=None, frame_id=""),
        child_frame_id="",
        pose=ns(pose=ns(position=_vector(), orientation=_vector(w=1.0))),
        twist=ns(twist=ns(linear=_vector(), angular=_vector())),
    )


class _FakeNode:
    """Records what the node creates, so the test can run its callbacks."""

    def __init__(self, name):
        self.published = []
        self.subscriptions = {}
        self.timers = []

    def declare_parameter(self, name, default):
        return types.SimpleNamespace(value=default)

    def create_publisher(self, msg_type, topic, qos):
        return types.SimpleNamespace(publish=self.published.append)

    def create_subscription(self, msg_type, topic, callback, qos):
        self.subscriptions[topic] = callback

    def create_timer(self, period, callback):
        self.timers.append((period, callback))

    def destroy_node(self):
        pass


@pytest.fixture
def node(monkeypatch):
    spun = []
    rclpy = types.ModuleType("rclpy")
    rclpy.init = lambda: None
    rclpy.spin = spun.append
    rclpy.try_shutdown = lambda: None
    rclpy.node = types.ModuleType("rclpy.node")
    rclpy.node.Node = _FakeNode
    nav_msgs = types.ModuleType("nav_msgs")
    nav_msgs.msg = types.ModuleType("nav_msgs.msg")
    nav_msgs.msg.Odometry = _odometry
    sensor_msgs = types.ModuleType("sensor_msgs")
    sensor_msgs.msg = types.ModuleType("sensor_msgs.msg")
    sensor_msgs.msg.JointState = object
    for module in (
        rclpy,
        rclpy.node,
        nav_msgs,
        nav_msgs.msg,
        sensor_msgs,
        sensor_msgs.msg,
    ):
        monkeypatch.setitem(sys.modules, module.__name__, module)
    joint_state_odometry.main()
    return spun[0]


def _joint_state(stamp, planar_x, planar_y, planar_theta, velocities=(0.0, 0.0, 0.0)):
    # Other joints come first, so the node must look the planar joints up by name.
    return types.SimpleNamespace(
        header=types.SimpleNamespace(stamp=stamp),
        name=["left_fr3_joint1", "planar_theta", "planar_x", "planar_y"],
        position=[0.3, planar_theta, planar_x, planar_y],
        velocity=[0.0, velocities[2], velocities[0], velocities[1]],
    )


def _tick(node):
    ((_, callback),) = node.timers
    callback()


def test_odometry_is_published_from_one_timer_at_the_sim_odometry_rate(node):
    assert [period for period, _ in node.timers] == [pytest.approx(1.0 / 50.0)]


def test_no_odometry_before_the_first_planar_joint_state(node):
    _tick(node)
    node.subscriptions["/joint_states"](
        types.SimpleNamespace(
            header=types.SimpleNamespace(stamp="t0"),
            name=["left_fr3_joint1"],
            position=[0.3],
            velocity=[0.0],
        )
    )
    _tick(node)
    assert node.published == []


def test_joint_states_alone_publish_nothing(node):
    for i in range(20):
        node.subscriptions["/joint_states"](_joint_state(f"t{i}", 0.1 * i, 0.0, 0.0))
    assert node.published == []


def test_each_tick_publishes_the_latest_joint_state_with_its_stamp(node):
    for i in range(20):
        node.subscriptions["/joint_states"](_joint_state(f"t{i}", 0.1 * i, 0.0, 0.0))
    node.subscriptions["/joint_states"](
        _joint_state("latest", 1.0, -2.0, math.pi / 2, velocities=(0.0, 0.3, 0.2))
    )
    _tick(node)
    _tick(node)

    assert len(node.published) == 2
    for odom in node.published:
        assert odom.header.stamp == "latest"
        assert (odom.header.frame_id, odom.child_frame_id) == ("odom", "base_link")
        pose = odom.pose.pose
        assert (pose.position.x, pose.position.y) == pytest.approx((1.0, -2.0))
        assert (pose.orientation.z, pose.orientation.w) == pytest.approx(
            (math.sin(math.pi / 4), math.cos(math.pi / 4))
        )
        twist = odom.twist.twist
        assert (twist.linear.x, twist.linear.y, twist.angular.z) == pytest.approx(
            (0.3, 0.0, 0.2)
        )
