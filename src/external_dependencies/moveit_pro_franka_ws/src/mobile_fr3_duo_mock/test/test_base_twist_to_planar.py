"""Tests for the pure part of script/base_twist_to_planar.py. No ROS."""

import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "script"))

from base_twist_to_planar import planar_velocities  # noqa: E402


def test_forward_command_follows_the_heading():
    assert planar_velocities(0.3, 0.0, 0.0, math.pi / 2) == pytest.approx(
        [0.0, 0.3, 0.0]
    )


def test_sideways_command_is_rotated_and_turn_passes_through():
    assert planar_velocities(0.0, 0.2, 0.5, math.pi) == pytest.approx([0.0, -0.2, 0.5])
