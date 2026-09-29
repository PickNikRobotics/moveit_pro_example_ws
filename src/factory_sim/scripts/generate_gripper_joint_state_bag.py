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

"""Write the finger joint state bag for the articulated-tool Objectives.

The bag stands in for the drivers of the two attachable grippers in
description/articulated_tools/. Each gripper publishes sensor_msgs/JointState
on /joint_states at 50 Hz, one message per gripper per tick, the way each
tool's own joint_state_broadcaster would. Both grippers publish, so the bag
works whichever one is attached.

Each 6 s cycle is the same for both grippers:

    0.0 - 2.0 s  open, held
    2.0 - 3.0 s  closing
    3.0 - 5.0 s  closed, held
    5.0 - 6.0 s  opening

The bag holds two cycles (12 s). Play it with --loop to keep the fingers
moving, or play a slice to leave them in one place, for example
--start-offset 3.5 --playback-duration 0.5 to leave them closed.

Run inside a MoveIt Pro container, from the workspace root:

    source /opt/ros/jazzy/setup.bash
    python3 src/factory_sim/scripts/generate_gripper_joint_state_bag.py

Rerunning it writes the same messages with the same timestamps. The .mcap
bytes can still differ between runs.
"""

import argparse
import shutil
from pathlib import Path

import rosbag2_py
from builtin_interfaces.msg import Time
from rclpy.serialization import serialize_message
from sensor_msgs.msg import JointState

TOPIC = "/joint_states"
RATE_HZ = 50
CYCLE_S = 6.0
CYCLES = 2
# 2026-09-29T00:00:00Z. A fixed start gives the same timestamps on every run.
START_NS = 1_790_640_000 * 1_000_000_000

# Joint names and fully closed positions, matching the gripper URDFs. Open is 0 for all of them.
GRIPPERS = {
    "parallel_gripper": {
        "parallel_gripper_left_finger_joint": 0.04,
        # Mimics the left finger. A joint_state_broadcaster publishes mimic joints too.
        "parallel_gripper_right_finger_joint": 0.04,
    },
    "angular_gripper": {
        "angular_gripper_left_finger_joint": 0.38,
        "angular_gripper_right_finger_joint": 0.38,
    },
}

DEFAULT_OUTPUT = (
    Path(__file__).resolve().parent.parent / "bags" / "gripper_joint_states"
)


def closure(t: float) -> tuple[float, float]:
    """Return how closed the fingers are (0 open, 1 closed) and its rate, at time t."""
    t = t % CYCLE_S
    if t < 2.0:
        return 0.0, 0.0
    if t < 3.0:
        # Smoothstep from open to closed over 1 s.
        s = t - 2.0
        return 3 * s**2 - 2 * s**3, 6 * s - 6 * s**2
    if t < 5.0:
        return 1.0, 0.0
    if t < 6.0:
        s = t - 5.0
        return 1.0 - (3 * s**2 - 2 * s**3), -(6 * s - 6 * s**2)
    return 0.0, 0.0


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Write the finger joint state bag for the articulated-tool Objectives."
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=DEFAULT_OUTPUT,
        help=f"bag directory to write, replaced if it exists (default: {DEFAULT_OUTPUT})",
    )
    args = parser.parse_args()

    if args.output.exists():
        shutil.rmtree(args.output)
    args.output.parent.mkdir(parents=True, exist_ok=True)

    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=str(args.output), storage_id="mcap"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    writer.create_topic(
        rosbag2_py.TopicMetadata(
            id=0,
            name=TOPIC,
            type="sensor_msgs/msg/JointState",
            serialization_format="cdr",
        )
    )

    ticks = int(CYCLES * CYCLE_S * RATE_HZ)
    for tick in range(ticks):
        t = tick / RATE_HZ
        stamp_ns = START_NS + tick * 1_000_000_000 // RATE_HZ
        fraction, rate = closure(t)
        for joints in GRIPPERS.values():
            msg = JointState()
            msg.header.stamp = Time(
                sec=stamp_ns // 1_000_000_000, nanosec=stamp_ns % 1_000_000_000
            )
            msg.name = list(joints)
            msg.position = [closed * fraction for closed in joints.values()]
            msg.velocity = [closed * rate for closed in joints.values()]
            msg.effort = [0.0] * len(joints)
            writer.write(TOPIC, serialize_message(msg), stamp_ns)

    del writer
    print(
        f"Wrote {ticks * len(GRIPPERS)} messages on {TOPIC}, "
        f"{CYCLES * CYCLE_S:.0f} s, to {args.output}"
    )


if __name__ == "__main__":
    main()
