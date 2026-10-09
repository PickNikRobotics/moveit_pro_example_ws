"""Tests for the pure part of scripts/reset_relocalizer.py. No ROS."""

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

from reset_relocalizer import is_jump  # noqa: E402


def test_driving_is_not_a_jump():
    assert not is_jump((0.0, 0.0, 0.0), (0.01, 0.0, 0.01), 0.5, 0.5)


def test_keyframe_reset_is_a_jump():
    assert is_jump((-33.0, 2.2, 1.5), (0.0, 0.0, 0.0), 0.5, 0.5)


def test_turn_in_place_reset_is_a_jump():
    assert is_jump((0.0, 0.0, 1.5), (0.0, 0.0, 0.0), 0.5, 0.5)


def test_heading_wrap_is_not_a_jump():
    assert not is_jump(
        (0.0, 0.0, math.pi - 0.01), (0.0, 0.0, -math.pi + 0.01), 0.5, 0.5
    )
