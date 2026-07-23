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

import math
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


PACKAGE_ROOT = Path(__file__).parents[1]
PARAMETERS = PACKAGE_ROOT / "params" / "forward_stereo.yaml"
MODEL = PACKAGE_ROOT / "description" / "ur5e_ridgeback.xml"
SCENE = PACKAGE_ROOT / "description" / "hangar_scene.xml"


def parse_vector(element, attribute):
    return tuple(float(value) for value in element.attrib[attribute].split())


def test_forward_stereo_model_matches_calibration():
    parameters = yaml.safe_load(PARAMETERS.read_text())["forward_stereo_publisher"][
        "ros__parameters"
    ]
    root = ET.parse(MODEL).getroot()
    size = root.find("size")
    assert size is not None
    assert size.attrib["nuser_cam"] == "1"
    cameras = {camera.attrib["name"]: camera for camera in root.iter("camera")}
    sites = {
        site.attrib["name"]: site for site in root.iter("site") if "name" in site.attrib
    }

    left_camera = cameras["forward_stereo_left"]
    right_camera = cameras["forward_stereo_right"]
    left_site = sites[parameters["left_frame_id"]]
    right_site = sites[parameters["right_frame_id"]]

    expected_resolution = (parameters["width"], parameters["height"])
    assert parse_vector(left_camera, "resolution") == expected_resolution
    assert parse_vector(right_camera, "resolution") == expected_resolution
    # picknik_mujoco_ros CameraConfig::DepthType::RGB_ONLY. The stereo bridge
    # consumes color only, so depth rendering and point-cloud generation must
    # remain disabled for these cameras.
    assert left_camera.attrib["user"] == "1"
    assert right_camera.attrib["user"] == "1"
    assert float(left_camera.attrib["fovy"]) == parameters["vertical_fov_degrees"]
    assert float(right_camera.attrib["fovy"]) == parameters["vertical_fov_degrees"]

    left_position = parse_vector(left_camera, "pos")
    right_position = parse_vector(right_camera, "pos")
    assert left_position == parse_vector(left_site, "pos")
    assert right_position == parse_vector(right_site, "pos")
    assert left_position[0] == right_position[0]
    assert left_position[2] == right_position[2]
    assert math.isclose(left_position[1] - right_position[1], parameters["baseline_m"])

    camera_axes = (0.0, -1.0, 0.0, 0.0, 0.0, 1.0)
    optical_axes = (0.0, -1.0, 0.0, 0.0, 0.0, -1.0)
    assert parse_vector(left_camera, "xyaxes") == camera_axes
    assert parse_vector(right_camera, "xyaxes") == camera_axes
    assert parse_vector(left_site, "xyaxes") == optical_axes
    assert parse_vector(right_site, "xyaxes") == optical_axes

    scene_root = ET.parse(SCENE).getroot()
    global_visual = scene_root.find("./visual/global")
    assert global_visual is not None
    offscreen_resolution = (
        int(global_visual.attrib["offwidth"]),
        int(global_visual.attrib["offheight"]),
    )
    camera_resolutions = {
        tuple(int(value) for value in camera.attrib["resolution"].split())
        for document_root in (scene_root, root)
        for camera in document_root.iter("camera")
    }
    assert camera_resolutions == {offscreen_resolution}


def test_right_projection_translation_uses_ros_stereo_convention():
    parameters = yaml.safe_load(PARAMETERS.read_text())["forward_stereo_publisher"][
        "ros__parameters"
    ]
    focal_length_px = parameters["height"] / (
        2.0 * math.tan(math.radians(parameters["vertical_fov_degrees"]) / 2.0)
    )
    projection_tx = -focal_length_px * parameters["baseline_m"]

    horizontal_fov_degrees = math.degrees(
        2.0 * math.atan(parameters["width"] / (2.0 * focal_length_px))
    )

    assert math.isclose(focal_length_px, 762.7222992602944)
    assert math.isclose(horizontal_fov_degrees, 80.0)
    assert math.isclose(projection_tx, -57.20417244452208)
