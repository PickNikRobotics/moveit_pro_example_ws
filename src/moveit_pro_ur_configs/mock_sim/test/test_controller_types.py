# Copyright 2026 PickNik Inc.
# SPDX-License-Identifier: BSD-3-Clause
"""Every controller mock_sim configures must be loadable by the controller manager."""
from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python import get_resource, get_resources
from ament_index_python.packages import get_package_share_directory
import yaml

PLUGIN_CATEGORY = "controller_interface__pluginlib__plugin"


def declared_controller_types():
    """Lookup names pluginlib resolves for controller_interface plugins."""
    types = set()
    for package, prefix in get_resources(PLUGIN_CATEGORY).items():
        content, _ = get_resource(PLUGIN_CATEGORY, package)
        for description in content.split():
            for plugin in ET.parse(Path(prefix) / description).getroot().iter("class"):
                types.add(plugin.get("name", plugin.get("type")))
    return types


def test_every_configured_controller_type_is_installed():
    share = Path(get_package_share_directory("mock_sim"))
    config = yaml.safe_load(
        (share / "config/control/picknik_ur.ros2_control.yaml").read_text()
    )
    configured = {
        controller["type"]
        for controller in config["controller_manager"]["ros__parameters"].values()
        if isinstance(controller, dict) and "type" in controller
    }
    assert configured - declared_controller_types() == set()
