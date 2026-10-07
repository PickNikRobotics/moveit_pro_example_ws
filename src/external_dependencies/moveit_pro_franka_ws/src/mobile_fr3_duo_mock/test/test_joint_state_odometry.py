"""Tests for the pure part of script/joint_state_odometry.py. No ROS."""

import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "script"))

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
