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
"""Write the build's snapshot of bench_layout.yaml, with derived entries added.

Usage: layout_snapshot.py <bench_layout.yaml> <rebot.urdf.xacro> <output.yaml>

Adds `wrist_camera_mount`, the d435_mount_joint origin (gripper_end ->
d435_mount_link) that puts d435_link at the layout's `wrist_camera` pose in
link6, through the base description's own link6 -> gripper_end and
d435_mount_link -> d435_link joints.
"""
import math
import sys
import xml.etree.ElementTree as ET

import yaml


def transform(xyz, rpy):
    """4x4 matrix of a URDF origin (fixed-axis roll, pitch, yaw)."""
    cr, sr = math.cos(rpy[0]), math.sin(rpy[0])
    cp, sp = math.cos(rpy[1]), math.sin(rpy[1])
    cy, sy = math.cos(rpy[2]), math.sin(rpy[2])
    rotation = [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]
    return [row + [xyz[i]] for i, row in enumerate(rotation)] + [[0, 0, 0, 1]]


def multiply(a, b):
    return [
        [sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)] for i in range(4)
    ]


def invert(a):
    rotation = [[a[j][i] for j in range(3)] for i in range(3)]
    translation = [-sum(rotation[i][k] * a[k][3] for k in range(3)) for i in range(3)]
    return [row + [translation[i]] for i, row in enumerate(rotation)] + [[0, 0, 0, 1]]


def origin(a):
    """URDF xyz and rpy of a 4x4 matrix."""
    rpy = [
        math.atan2(a[2][1], a[2][2]),
        math.asin(max(-1.0, min(1.0, -a[2][0]))),
        math.atan2(a[1][0], a[0][0]),
    ]
    return [round(a[i][3], 6) for i in range(3)], [round(v, 6) for v in rpy]


def joint_transform(urdf, parent, child):
    for joint in urdf.iter("joint"):
        # ros2_control <joint> entries carry no parent/child.
        if joint.find("parent") is None:
            continue
        if (
            joint.find("parent").get("link") == parent
            and joint.find("child").get("link") == child
        ):
            o = joint.find("origin")
            return transform(
                *(list(map(float, o.get(k).split())) for k in ("xyz", "rpy"))
            )
    raise SystemExit(f"no joint {parent} -> {child} in the base description")


def main(layout_path, urdf_path, output_path):
    with open(layout_path) as f:
        layout = yaml.safe_load(f)
    urdf = ET.parse(urdf_path).getroot()
    link6_to_gripper_end = joint_transform(urdf, "link6", "gripper_end")
    mount_to_camera = multiply(
        joint_transform(urdf, "d435_mount_link", "d435_bottom_screw_frame"),
        joint_transform(urdf, "d435_bottom_screw_frame", "d435_link"),
    )
    link6_to_camera = transform(
        layout["wrist_camera"]["xyz"], layout["wrist_camera"]["rpy"]
    )
    mount = multiply(
        multiply(invert(link6_to_gripper_end), link6_to_camera), invert(mount_to_camera)
    )
    xyz, rpy = origin(mount)
    layout["wrist_camera_mount"] = {"xyz": xyz, "rpy": rpy}
    with open(output_path, "w") as f:
        f.write(
            "# Generated at build time from config/bench_layout.yaml; edit that file.\n"
        )
        yaml.safe_dump(layout, f, sort_keys=False)


if __name__ == "__main__":
    main(*sys.argv[1:])
