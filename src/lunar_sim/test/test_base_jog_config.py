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

"""base_jog.yaml names platform_velocity_controller and repeats its limits by hand. If the
controller is renamed, the Base tab activates a controller that does not exist; if its limits
drop below base_jog's, the Base tab's 100% asks for more than the controller allows. Nothing
else in the build notices either.

No simulator, no ROS - both YAML files are parsed and compared directly.
"""

from pathlib import Path

import yaml

CONFIG = Path(__file__).resolve().parent.parent / "config"
BASE_JOG = yaml.safe_load((CONFIG / "moveit" / "base_jog.yaml").read_text())
CONTROL = yaml.safe_load(
    (CONFIG / "control" / "husky_a300.ros2_control.yaml").read_text()
)
CONTROLLER = CONTROL[BASE_JOG["controller"]]["ros__parameters"]


def test_base_jog_controller_is_a_diff_drive_controller():
    controllers = CONTROL["controller_manager"]["ros__parameters"]
    assert (
        controllers[BASE_JOG["controller"]]["type"]
        == "diff_drive_controller/DiffDriveController"
    )


def test_base_jog_limits_within_controller_limits():
    assert BASE_JOG["max_linear_velocity"] <= CONTROLLER["linear.x.max_velocity"]
    assert BASE_JOG["max_angular_velocity"] <= CONTROLLER["angular.z.max_velocity"]


def test_base_jog_overlay_uses_controller_body_frame() -> None:
    """Frameless base commands remain visible in the controller's body frame."""
    frontend = yaml.safe_load((CONFIG / "frontend_settings.yaml").read_text())
    assert frontend["nav2CmdVelFrame"] == CONTROLLER["base_frame_id"]
