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

"""Teleop and named gripper Objectives must command the hangar vacuum hardware."""

from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
import yaml

PACKAGE = Path(__file__).resolve().parent.parent


@pytest.mark.parametrize(
    "filename, objective_name, position",
    [
        ("open_gripper.xml", "Open Gripper", 0.0),
        ("close_gripper.xml", "Close Gripper", 1.0),
    ],
)
def test_gripper_objective_uses_active_vacuum_controller(
    filename, objective_name, position
):
    config = yaml.safe_load((PACKAGE / "config/config.yaml").read_text())
    assert "vacuum_gripper" in config["ros2_control"]["controllers_active_at_startup"]
    root = ET.parse(PACKAGE / "objectives" / filename).getroot()
    assert root.attrib["main_tree_to_execute"] == objective_name
    tree = root.find(f"BehaviorTree[@ID='{objective_name}']")
    assert tree is not None
    actions = tree.findall(".//Action[@ID='MoveGripperAction']")
    assert len(actions) == 1
    assert (
        actions[0].attrib["gripper_command_action_name"]
        == "/vacuum_gripper/gripper_cmd"
    )
    assert float(actions[0].attrib["position"]) == position
    metadata = root.find(
        f"TreeNodesModel/SubTree[@ID='{objective_name}']/MetadataFields"
    )
    assert metadata is not None
    assert metadata.find("Metadata[@runnable='true']") is not None
