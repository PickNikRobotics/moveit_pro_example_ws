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

"""Tests for the channel layout and source precedence of the joint command bridge."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "script"))

from joint_command_bridge import assemble_joint_command  # noqa: E402

GRIPPER = "gripper_joint"
ORDER = ["joint_a", "joint_b", GRIPPER]
REFERENCE = {"joint_a": 1.0, "joint_b": 2.0}


def test_reference_wins_for_arm_when_fresh() -> None:
    """The setpoint leads the measured position, and that lead is the learning signal."""
    result = assemble_joint_command(
        joint_order=ORDER,
        gripper_joint=GRIPPER,
        reference=REFERENCE,
        measured={"joint_a": 9.0, "joint_b": 9.0},
        gripper_command=0.6,
    )
    assert result == [1.0, 2.0, 0.6]


def test_falls_back_to_measured_when_reference_is_empty() -> None:
    """A gripper dwell is labelled with the held pose, not the last goal's setpoint.

    The node empties the reference once it goes stale, so an empty one is how a
    dwell reaches this function.
    """
    result = assemble_joint_command(
        joint_order=ORDER,
        gripper_joint=GRIPPER,
        reference={},
        measured={"joint_a": 0.5, "joint_b": 0.6},
        gripper_command=0.6,
    )
    assert result == [0.5, 0.6, 0.6]


def test_gripper_never_takes_the_controller_reference() -> None:
    """Checking the reference first would flatten the commanded open/close."""
    result = assemble_joint_command(
        joint_order=ORDER,
        gripper_joint=GRIPPER,
        reference={**REFERENCE, GRIPPER: 0.123},
        measured={},
        gripper_command=0.6,
    )
    assert result == [1.0, 2.0, 0.6]


def test_order_follows_joint_order_not_controller_order() -> None:
    """A positional copy of the controller's vector would train on permuted labels."""
    result = assemble_joint_command(
        joint_order=["joint_b", "joint_a", GRIPPER],
        gripper_joint=GRIPPER,
        reference=REFERENCE,
        measured={},
        gripper_command=0.6,
    )
    assert result == [2.0, 1.0, 0.6]


def test_a_joint_with_no_source_publishes_nothing() -> None:
    """Padding the vector would label a channel the robot never reported."""
    result = assemble_joint_command(
        joint_order=ORDER,
        gripper_joint=GRIPPER,
        reference={"joint_a": 1.0},
        measured={},
        gripper_command=0.6,
    )
    assert result is None


def test_a_partial_reference_is_completed_from_measured() -> None:
    """Each joint resolves by name, so a short controller vector cannot shift channels."""
    result = assemble_joint_command(
        joint_order=ORDER,
        gripper_joint=GRIPPER,
        reference={"joint_a": 1.0},
        measured={"joint_a": 9.0, "joint_b": 0.6},
        gripper_command=0.6,
    )
    assert result == [1.0, 0.6, 0.6]
