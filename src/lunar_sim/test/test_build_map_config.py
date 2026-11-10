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

"""Static wiring checks for Build Map; these do not replace a live UI/SLAM run."""

from pathlib import Path
import xml.etree.ElementTree as ET

import yaml

PACKAGE = Path(__file__).resolve().parent.parent


def _root() -> ET.Element:
    """Read the candidate XML on every test, including local mutation probes."""
    return ET.parse(PACKAGE / "objectives" / "build_map.xml").getroot()


def _find(root: ET.Element, path: str) -> ET.Element:
    """Require a wiring element and report its path when it is absent."""
    element = root.find(path)
    assert element is not None, f"Missing XML element: {path}"
    return element


def test_mapping_check_precedes_user_input_and_save() -> None:
    """No operator drive or save is reached until the save service is available."""
    steps = list(_find(_root(), "BehaviorTree/Control"))
    assert [step.get("ID") for step in steps] == [
        "Fallback",
        "WaitForUserDecision",
        "Request Teleoperation",
        "CallSaveMapService",
    ]
    check, failure = list(steps[0])
    assert check.get("ID") == "WaitForServiceAvailable"
    assert (
        check.get("service_name")
        == steps[-1].get("service_name")
        == "/slam_toolbox/save_map"
    )
    assert float(check.attrib["timeout"]) > 0
    assert _find(failure, "Action[@ID='LogMessage']").get("log_level") == "ERROR"
    assert failure.find("Action[@ID='AlwaysFailure']") is not None


def test_teleoperation_uses_only_lunar_override_ports() -> None:
    """The arm-free BaseJog override accepts every supplied port; no JTC is assumed."""
    call = _find(_root(), ".//SubTree[@ID='Request Teleoperation']")
    override = ET.parse(PACKAGE / "objectives" / "request_teleoperation.xml").getroot()
    ports = {
        p.get("name") for p in override.findall("TreeNodesModel/SubTree/input_port")
    }
    assert set(call.attrib) - {"ID", "_collapsed"} <= ports
    assert call.get("enable_user_interaction") == "true"
    # DoTeleoperateAction sets the initial mode before controllers arrive in feedback.
    assert call.get("initial_teleop_mode") == "3"
    assert (
        _find(override, ".//Action[@ID='BaseJog']").get("cmd_vel_topic") == "/cmd_vel"
    )


def test_save_name_is_exposed_and_timeouts_are_bounded() -> None:
    """Operators can choose a server-side prefix without changing the save action."""
    root = _root()
    save = _find(root, ".//Action[@ID='CallSaveMapService']")
    port = _find(root, "TreeNodesModel/SubTree/input_port[@name='map_name']")
    assert save.get("map_name") == "{map_name}"
    assert port.get("default") == "lunar_map"
    assert port.get("type") == "std::string"
    for name in ("response_timeout", "wait_for_server_available_timeout"):
        assert 0 < float(save.attrib[name]) <= 30


def test_objective_is_runnable_and_identity_matches() -> None:
    """The UI discovers one consistently named runnable Navigation Objective."""
    root = _root()
    assert root.get("BTCPP_format") == "4"
    assert (
        root.get("main_tree_to_execute")
        == _find(root, "BehaviorTree").get("ID")
        == "Build Map"
    )
    model = _find(root, "TreeNodesModel/SubTree")
    assert model.get("ID") == "Build Map"
    metadata = {
        next(iter(m.attrib.items())) for m in model.findall("MetadataFields/Metadata")
    }
    assert ("runnable", "true") in metadata
    assert ("subcategory", "Navigation") in metadata


def test_operator_prompt_matches_mapping_sensor_and_base_controls() -> None:
    """The port does not promise hangar's dual scans or arm controls on lunar_sim."""
    params = yaml.safe_load((PACKAGE / "params" / "nav2_params.yaml").read_text())
    assert (
        params["slam_toolbox"]["ros__parameters"]["scan_topic"]
        == "/scan_front_filtered"
    )
    prompt = _find(_root(), ".//Action[@ID='WaitForUserDecision']").attrib["prompt"]
    assert "front lidar" in prompt
    assert "Base tab" in prompt
    assert "does not check for collisions" in prompt
    assert "keeps mapping" in prompt


def test_mapping_remains_opt_in_and_behaviors_are_loaded() -> None:
    """Adding the Objective does not enable mapping or change the default TF owner."""
    config = yaml.safe_load((PACKAGE / "config" / "config.yaml").read_text())
    params = {
        k: v
        for p in config["hardware"]["robot_description"]["urdf_params"]
        for k, v in p.items()
    }
    assert params["slam"] is False
    assert params["localization"] is False
    loaders = config["objectives"]["behavior_loader_plugins"]["core"]
    assert "moveit_pro::behaviors::CoreBehaviorsLoader" in loaders
    assert "moveit_pro::behaviors::NavBehaviorsLoader" in loaders
