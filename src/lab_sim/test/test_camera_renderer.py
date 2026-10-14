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

"""The camera renderer switch in picknik_ur.xacro routes each RGB camera's color as documented."""

import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import xacro
from ament_index_python.packages import get_package_share_directory

DESCRIPTION = (
    Path(get_package_share_directory("lab_sim")) / "description/picknik_ur.xacro"
)
DEFAULT_CAMERAS = ("scene_camera", "wrist_camera")
# config.yaml's urdf_params, which the description requires.
URDF_PARAMS = {
    "mujoco_keyframe": "default",
    "mujoco_model": "description/scene.xml",
    "mujoco_viewer": "false",
}


def camera_params(**mappings) -> dict[str, dict[str, str]]:
    """Every ros2_control sensor's params, merged across the elements that name it."""
    robot = ET.fromstring(
        xacro.process_file(
            str(DESCRIPTION), mappings={**URDF_PARAMS, **mappings}
        ).toxml()
    )
    params: dict[str, dict[str, str]] = {}
    for sensor in robot.iter("sensor"):
        merged = params.setdefault(sensor.get("name"), {})
        merged.update({p.get("name"): p.text for p in sensor.findall("param")})
    return params


@pytest.mark.parametrize("renderer", ["opengl", ""])
def test_opengl_and_an_empty_value_leave_mujoco_color_on_the_real_topics(renderer):
    """MuJoCo keeps publishing color where consumers expect it."""
    params = camera_params(camera_renderer=renderer)
    for camera in DEFAULT_CAMERAS:
        assert params.get(camera, {}).get("color_enabled") != "false"
        assert "opengl" not in params.get(camera, {}).get("rgb_topic", "")


def test_nyx_turns_off_mujoco_color_for_each_listed_camera():
    """Nyx's node owns <camera>/color, so MuJoCo must not publish there too."""
    params = camera_params(camera_renderer="nyx")
    for camera in DEFAULT_CAMERAS:
        assert params[camera]["color_enabled"] == "false"


def test_both_moves_mujoco_color_under_an_opengl_prefix():
    """Neither renderer publishes on <camera>/color, so no consumer reads one expecting the other."""
    params = camera_params(camera_renderer="both")
    for camera in DEFAULT_CAMERAS:
        assert params[camera]["rgb_topic"] == f"{camera}/opengl/color"
        assert params[camera]["camera_info_topic"] == f"{camera}/opengl/camera_info"
        assert params[camera].get("color_enabled") != "false"


@pytest.mark.parametrize(
    ("cameras", "switched"),
    [
        ("", ()),
        ("wrist_camera", ("wrist_camera",)),
        ("scene_camera wrist_camera", DEFAULT_CAMERAS),
    ],
)
def test_only_the_listed_cameras_are_switched(cameras, switched):
    """A config reusing this description switches exactly the cameras it lists in nyx_cameras."""
    params = camera_params(camera_renderer="nyx", nyx_cameras=cameras)
    for camera in DEFAULT_CAMERAS:
        disabled = params.get(camera, {}).get("color_enabled") == "false"
        assert disabled == (camera in switched)


def test_an_unrecognized_renderer_fails_naming_the_variable_and_its_values():
    """A typo fails the description with the fix, rather than leaving the config without color."""
    with pytest.raises(
        xacro.XacroException,
        match="MOVEIT_PRO_CAMERA_RENDERER is 'nxy'.*opengl, nyx or both",
    ):
        camera_params(camera_renderer="nxy")
