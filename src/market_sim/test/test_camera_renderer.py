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

"""Check color-stream ownership without changing MuJoCo depth or lidar sensors."""

import os
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

DESCRIPTION = (
    Path(__file__).resolve().parents[1] / "description/mobile_fr3_duo.urdf.xacro"
)


@pytest.mark.parametrize("renderer", ["", "opengl", "nyx", "both"])
def test_color_stream_has_one_renderer(renderer: str) -> None:
    """Keep OpenGL defaults or hand both color streams to the selected renderer."""
    result = subprocess.run(
        ["xacro", str(DESCRIPTION)],
        env={**os.environ, "MOVEIT_PRO_CAMERA_RENDERER": renderer},
        check=True,
        capture_output=True,
        text=True,
    )
    control = ET.fromstring(result.stdout).find("ros2_control")
    for camera in ("scene_camera", "camera_front"):
        sensors = control.findall(f"sensor[@name='{camera}']")
        if renderer in ("", "opengl"):
            assert not sensors
            continue
        assert len(sensors) == 1
        params = {p.attrib["name"]: p.text for p in sensors[0]}
        if renderer == "nyx":
            assert params == {"color_enabled": "false"}
        else:
            assert params == {
                "rgb_topic": f"{camera}/opengl/color",
                "camera_info_topic": f"{camera}/opengl/camera_info",
            }
    for lidar in ("lidar_front", "lidar_rear"):
        assert (
            control.find(f"sensor[@name='{lidar}']/param[@name='points_topic']").text
            == f"/{lidar}/points"
        )


def test_unknown_renderer_fails_description() -> None:
    """Reject typos before starting a configuration with missing color streams."""
    result = subprocess.run(
        ["xacro", str(DESCRIPTION)],
        env={**os.environ, "MOVEIT_PRO_CAMERA_RENDERER": "invalid"},
        capture_output=True,
        text=True,
    )
    assert result.returncode != 0
    assert "MOVEIT_PRO_CAMERA_RENDERER" in result.stderr
    assert "opengl, nyx or both" in result.stderr
