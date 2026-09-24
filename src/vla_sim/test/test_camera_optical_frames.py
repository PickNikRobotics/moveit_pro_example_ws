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

"""Keep rendered depth rays and their published ROS optical frames aligned."""

import math
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest


@pytest.mark.parametrize(
    ("filename", "camera_name"),
    [
        ("cube_stack_scene.xml", "scene_camera"),
        ("cube_stack_scene.xml", "overview_camera"),
        ("gen3_7dof.xml", "wrist_camera"),
    ],
)
def test_camera_optical_frame(filename: str, camera_name: str) -> None:
    """ROS optical axes keep camera X and reverse MuJoCo camera Y and Z."""
    scene = Path(__file__).resolve().parents[1] / "description" / "mujoco" / filename
    root = ET.parse(scene).getroot()
    camera = root.find(f".//camera[@name='{camera_name}']")
    site = root.find(f".//site[@name='{camera_name}_optical_frame']")
    assert camera is not None
    assert site is not None
    parents = {child: parent for parent in root.iter() for child in parent}
    assert parents[camera] is parents[site], "Camera and optical site must share a body"
    assert camera.get("mode", "fixed") == "fixed"
    assert [float(v) for v in site.get("pos", "0 0 0").split()] == pytest.approx(
        [float(v) for v in camera.get("pos", "0 0 0").split()]
    )

    # MJCF uses wxyz. Postmultiply by a local-X half turn to convert
    # X-right/Y-up/Z-back into REP-103's X-right/Y-down/Z-forward.
    w, x, y, z = (float(v) for v in camera.attrib["quat"].split())
    expected = (-x, w, z, -y)
    actual = tuple(float(v) for v in site.attrib["quat"].split())
    assert len(actual) == 4
    norm = math.sqrt(sum(v * v for v in expected) * sum(v * v for v in actual))
    # q and -q are the same rotation; MJCF also normalizes quaternion inputs.
    alignment = abs(sum(a * b for a, b in zip(expected, actual))) / norm
    assert alignment == pytest.approx(1.0, abs=1e-12), camera_name
