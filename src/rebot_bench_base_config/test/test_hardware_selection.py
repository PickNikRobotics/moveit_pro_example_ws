# Copyright 2026 PickNik Inc.
# SPDX-License-Identifier: BSD-3-Clause
"""The arm backend defaults to mock and rejects unimplemented hardware modes."""
from pathlib import Path
import xml.etree.ElementTree as ET
from ament_index_python.packages import get_package_share_directory
import pytest
import xacro


def expand(**mappings):
    path = (
        Path(get_package_share_directory("rebot_bench_base_config"))
        / "description/rebot.urdf.xacro"
    )
    return ET.fromstring(xacro.process_file(str(path), mappings=mappings).toxml())


def test_default_and_explicit_mock_have_only_generic_system():
    for params in ({}, {"hardware_interface": "mock"}):
        model = expand(**params)
        assert [p.text for p in model.findall("ros2_control/hardware/plugin")] == [
            "mock_components/GenericSystem"
        ]
        assert len(model.findall("ros2_control/joint")) == 7


def test_real_driver_is_not_implemented():
    with pytest.raises(xacro.XacroException):
        expand(hardware_interface="real")
