#!/usr/bin/env python3

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
"""Check the tick-error math against the feetech_ros2_driver formula, and against mock."""

from pathlib import Path
import math
import sys

import pytest
import yaml
from ament_index_python.packages import get_package_share_directory

sys.path.insert(0, str(Path(__file__).parents[1] / "script"))
from so101_arm_bridge import JOINT_NAMES  # noqa: E402
from verify_calibration import (  # noqa: E402
    VerifyCalibration,
    format_report,
    ticks_error,
)


def mock_start_positions():
    """The pose config/initial_positions.yaml puts the mock hardware in."""
    path = (
        Path(get_package_share_directory("so101_base_config"))
        / "config"
        / "initial_positions.yaml"
    )
    initial = yaml.safe_load(path.read_text())["initial_positions"]
    return [initial[name] for name in JOINT_NAMES]


def test_ticks_error_is_the_inverse_of_the_driver_formula():
    # feetech_ros2_driver: reported_rad = (tick - offset) * 2*pi/4096. Held at
    # URDF zero (actual == 0), the error in ticks is reported_rad * 4096/2pi.
    assert ticks_error(0.0) == pytest.approx(0.0)
    assert ticks_error(math.pi) == pytest.approx(2048.0)
    assert ticks_error(-math.pi / 2.0) == pytest.approx(-1024.0)


def test_ticks_error_matches_mock_initial_positions():
    """On mock, /joint_states reports initial_positions.yaml verbatim, so
    Verify Calibration must report exactly these ticks for that pose."""
    positions = mock_start_positions()
    # JOINT_NAMES order: shoulder_pan, shoulder_lift, elbow_flex, wrist_flex,
    # wrist_roll, gripper.
    expected = [0.0, 475.13, -230.54, 196.95, 218.38, 319.12]
    assert [ticks_error(p) for p in positions] == pytest.approx(expected, abs=0.01)


def test_format_report_names_every_joint_and_carries_the_recipe():
    positions = mock_start_positions()
    report = format_report(JOINT_NAMES, positions)
    for name in JOINT_NAMES:
        assert name in report
    assert "offset += error_ticks" in report


@pytest.fixture
def ros_context():
    import rclpy

    rclpy.init()
    yield
    rclpy.try_shutdown()


def test_on_verify_reports_the_measured_pose(ros_context):
    from sensor_msgs.msg import JointState
    from std_srvs.srv import Trigger

    node = VerifyCalibration()
    pose = mock_start_positions()
    message = JointState()
    message.name = list(JOINT_NAMES)
    message.position = list(pose)
    node.on_joint_states(message)

    response = node.on_verify(Trigger.Request(), Trigger.Response())
    assert response.success
    assert response.message == format_report(JOINT_NAMES, pose)
    node.destroy_node()


def test_on_verify_refuses_a_stale_pose(ros_context):
    from rclpy.duration import Duration
    from sensor_msgs.msg import JointState
    from std_srvs.srv import Trigger

    node = VerifyCalibration()
    message = JointState()
    message.name = list(JOINT_NAMES)
    message.position = mock_start_positions()
    node.on_joint_states(message)
    node.latest_positions_time -= Duration(seconds=node.joint_states_timeout_s + 1.0)

    response = node.on_verify(Trigger.Request(), Trigger.Response())
    assert not response.success
    assert "old" in response.message
    node.destroy_node()


def test_on_verify_fails_without_a_measured_pose(ros_context):
    from std_srvs.srv import Trigger

    node = VerifyCalibration()
    response = node.on_verify(Trigger.Request(), Trigger.Response())
    assert not response.success
    node.destroy_node()
