# Copyright 2026 PickNik Inc.
# SPDX-License-Identifier: BSD-3-Clause
"""The robot description expands the way the Runtime expands it at startup."""
import xml.etree.ElementTree as ET

from moveit_studio_utils_py.create_urdf import create_xml_from_xacro
from moveit_studio_utils_py.system_config import load_system_config


def test_robot_description_expands(tmp_path):
    config, _, _ = load_system_config("space_satellite_sim", tmp_path)
    description = config.hardware.robot_description
    urdf = create_xml_from_xacro(description.urdf, description.urdf_params)
    assert ET.fromstring(urdf).tag == "robot"
