#!/usr/bin/env python3

# Copyright 2026 PickNik Inc.
# All rights reserved.
#
# Unauthorized copying of this code base via any medium is strictly prohibited.
# Proprietary and confidential.

"""Tests for the commanded-joint assembly that becomes the recorded `action`.

Each case is a way the action could silently degrade into a copy of
`observation.state`, or into a constant, either of which trains a policy that
cannot move.
"""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "script"))
from joint_command_bridge import (  # noqa: E402
    DEFAULT_REFERENCE_TIMEOUT_S,
    assemble_joint_command,
    is_reference_fresh,
)

ARM = ["joint_a", "joint_b"]
GRIPPER = "gripper_joint"
ORDER = [*ARM, GRIPPER]


def test_returns_none_when_arm_joint_has_no_source() -> None:
    """Publishing a short vector would misalign every downstream joint."""
    result = assemble_joint_command(
        joint_order=ORDER,
        gripper_joint=GRIPPER,
        reference_names=None,
        reference_positions=None,
        reference_fresh=False,
        measured={"joint_a": 0.1},  # joint_b absent
        gripper_cmd=0.4,
        gripper_fallback=0.0,
    )
    assert result is None


def test_uses_controller_reference_for_arm_when_fresh() -> None:
    """The setpoint leads the measured position, and that lead is the learning signal."""
    result = assemble_joint_command(
        joint_order=ORDER,
        gripper_joint=GRIPPER,
        reference_names=["joint_a", "joint_b"],
        reference_positions=[1.0, 2.0],
        reference_fresh=True,
        measured={"joint_a": 9.0, "joint_b": 9.0},
        gripper_cmd=0.6,
        gripper_fallback=0.0,
    )
    assert result == [1.0, 2.0, 0.6]


def test_falls_back_to_measured_when_reference_stale() -> None:
    """A gripper dwell is labelled with the held pose, not the last goal's setpoint."""
    result = assemble_joint_command(
        joint_order=ORDER,
        gripper_joint=GRIPPER,
        reference_names=["joint_a", "joint_b"],
        reference_positions=[1.0, 2.0],
        reference_fresh=False,
        measured={"joint_a": 0.5, "joint_b": 0.6},
        gripper_cmd=0.6,
        gripper_fallback=0.0,
    )
    assert result == [0.5, 0.6, 0.6]


def test_gripper_uses_fallback_before_first_command() -> None:
    result = assemble_joint_command(
        joint_order=ORDER,
        gripper_joint=GRIPPER,
        reference_names=["joint_a", "joint_b"],
        reference_positions=[1.0, 2.0],
        reference_fresh=True,
        measured={},
        gripper_cmd=None,
        gripper_fallback=0.0,
    )
    assert result == [1.0, 2.0, 0.0]


def test_gripper_zero_command_is_not_treated_as_unset() -> None:
    """0.0 is the oracle's OPEN command and is falsy; the fallback differs to catch it."""
    result = assemble_joint_command(
        joint_order=ORDER,
        gripper_joint=GRIPPER,
        reference_names=["joint_a", "joint_b"],
        reference_positions=[1.0, 2.0],
        reference_fresh=True,
        measured={},
        gripper_cmd=0.0,
        gripper_fallback=0.6,
    )
    assert result == [1.0, 2.0, 0.0]


def test_mismatched_reference_lengths_are_ignored() -> None:
    """Zipping them would bind joint_a to the wrong value and drop joint_b silently."""
    result = assemble_joint_command(
        joint_order=ORDER,
        gripper_joint=GRIPPER,
        reference_names=["joint_a", "joint_b"],
        reference_positions=[1.0],  # length mismatch
        reference_fresh=True,
        measured={"joint_a": 0.5, "joint_b": 0.6},
        gripper_cmd=0.6,
        gripper_fallback=0.0,
    )
    assert result == [0.5, 0.6, 0.6]


def test_no_gripper_joint_builds_arm_only_vector() -> None:
    result = assemble_joint_command(
        joint_order=ARM,
        gripper_joint=None,
        reference_names=["joint_a", "joint_b"],
        reference_positions=[1.0, 2.0],
        reference_fresh=True,
        measured={},
        gripper_cmd=None,
        gripper_fallback=0.0,
    )
    assert result == [1.0, 2.0]


def test_order_follows_joint_order_not_controller_order() -> None:
    """A positional copy of the controller's vector would train on permuted labels."""
    result = assemble_joint_command(
        joint_order=["joint_b", "joint_a", GRIPPER],
        gripper_joint=GRIPPER,
        reference_names=["joint_a", "joint_b"],
        reference_positions=[1.0, 2.0],
        reference_fresh=True,
        measured={},
        gripper_cmd=0.6,
        gripper_fallback=0.0,
    )
    assert result == [2.0, 1.0, 0.6]


def test_gripper_never_takes_the_controller_reference() -> None:
    """Checking the reference first would flatten the commanded open/close."""
    result = assemble_joint_command(
        joint_order=ORDER,
        gripper_joint=GRIPPER,
        reference_names=["joint_a", "joint_b", GRIPPER],
        reference_positions=[1.0, 2.0, 0.123],
        reference_fresh=True,
        measured={},
        gripper_cmd=0.6,
        gripper_fallback=0.0,
    )
    assert result == [1.0, 2.0, 0.6]


def test_reference_freshness_pins_the_timeout_on_both_sides() -> None:
    """The timeout decides whether a dwell is labelled with the held pose."""
    timeout = DEFAULT_REFERENCE_TIMEOUT_S
    assert is_reference_fresh(now=timeout - 1e-6, stamp=0.0, timeout=timeout)
    assert not is_reference_fresh(now=timeout + 1e-6, stamp=0.0, timeout=timeout)
    assert not is_reference_fresh(
        now=timeout, stamp=0.0, timeout=timeout
    ), "the boundary itself is stale"


def test_empty_joint_order_builds_an_empty_vector() -> None:
    """The node never calls it this way, but the contract is one entry per name."""
    assert (
        assemble_joint_command(
            joint_order=[],
            gripper_joint=GRIPPER,
            reference_names=["joint_a"],
            reference_positions=[1.0],
            reference_fresh=True,
            measured={},
            gripper_cmd=0.6,
            gripper_fallback=0.0,
        )
        == []
    )
