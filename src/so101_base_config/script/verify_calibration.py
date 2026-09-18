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

"""Report each joint's Feetech `offset` error, in servo ticks, at URDF zero.

Run after calibration with the arm held at its URDF-zero pose (every joint
mid-travel; gripper NEAR-CLOSED, which is URDF zero for this gripper) - see the
`Verify Calibration` Objective. feetech_ros2_driver reports position as
``(tick - offset) * 2*pi/4096`` (``feetech_driver::to_radians`` applied after
subtracting ``offset`` in ``read()``). Held at URDF zero, the reported angle
*is* the offset error in radians, so converting it back to ticks
(``* 4096/(2*pi)``) gives exactly how far each motor's ``offset`` in
``config/so101_follower_calibration.yaml`` is wrong - the same formula the
README's bench procedure uses for the manual correction.

This also runs on mock: mock hardware starts at `config/initial_positions.yaml`
and reports it verbatim, so the reported errors there are that pose's radians
converted to ticks - a fixed, checkable expectation with no arm attached.
"""

import math
import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

from so101_arm_bridge import JOINT_NAMES, order_like

# feetech_ros2_driver's kStsResolution (feetech_driver/common.hpp): ticks per
# servo revolution.
STS_TICKS_PER_REV = 4096

CORRECTION_RECIPE = (
    "Correct config/so101_follower_calibration.yaml: offset += error_ticks "
    "(round to a whole tick). Raising offset lowers the reported angle."
)


def ticks_error(position_rad):
    """Servo ticks a joint's `offset` is wrong by, held at URDF zero."""
    return position_rad * STS_TICKS_PER_REV / (2.0 * math.pi)


def format_report(names, positions_rad):
    """One line per joint (reported angle and offset error), plus the recipe."""
    lines = [
        f"{name}: reported {position:+.4f} rad, offset error {ticks_error(position):+.1f} ticks"
        for name, position in zip(names, positions_rad)
    ]
    lines.append(CORRECTION_RECIPE)
    return "\n".join(lines)


class VerifyCalibration(Node):
    """Report per-joint offset error while the arm is held at URDF zero."""

    def __init__(self):
        super().__init__("verify_calibration")
        joint_states_topic = self.declare_parameter(
            "joint_states_topic", "/joint_states"
        ).value
        # How old the last /joint_states sample may be and still be reported.
        # A stale sample means the broadcaster died or the bus went quiet, and
        # tick errors from an old pose look exactly like good ones.
        self.joint_states_timeout_s = self.declare_parameter(
            "joint_states_timeout_s", 2.0
        ).value
        self.latest_positions = None
        self.latest_positions_time = None
        self.create_subscription(
            JointState, joint_states_topic, self.on_joint_states, 10
        )
        self.create_service(Trigger, "~/verify", self.on_verify)
        self.get_logger().info(
            "verify_calibration ready; hold the arm at URDF zero (every joint "
            "mid-travel, gripper near-closed) and run the Verify Calibration "
            "Objective"
        )

    def on_joint_states(self, message):
        """Keep the latest complete measured pose, in JOINT_NAMES order."""
        try:
            self.latest_positions = order_like(
                list(message.name), list(message.position)
            )
            self.latest_positions_time = self.get_clock().now()
        except (KeyError, ValueError):
            # A broadcaster publishing a subset is not an error worth logging
            # every tick - just keep the last complete pose.
            return

    def on_verify(self, request, response):
        del request
        if self.latest_positions is None:
            response.success = False
            response.message = "no /joint_states yet; is the driver running?"
            return response
        age_s = (self.get_clock().now() - self.latest_positions_time).nanoseconds * 1e-9
        if age_s > self.joint_states_timeout_s:
            response.success = False
            response.message = (
                f"last /joint_states is {age_s:.1f} s old, older than "
                f"joint_states_timeout_s ({self.joint_states_timeout_s} s); "
                "refusing to report a stale pose - is the driver still running?"
            )
            return response
        report = format_report(JOINT_NAMES, self.latest_positions)
        self.get_logger().info("Verify Calibration:\n" + report)
        response.success = True
        response.message = report
        return response


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    rclpy.init(args=argv)
    try:
        rclpy.spin(VerifyCalibration())
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
