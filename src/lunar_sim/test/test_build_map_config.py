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
    """Thresholds and the mapping manager are checked before any operator prompt or pause."""
    steps = list(_find(_root(), "BehaviorTree/Control"))
    assert [step.get("ID") for step in steps] == [
        "ScriptCondition",
        "Fallback",
        "WaitForUserDecision",
        "GetLatestTransform",
        "ConvertTransformStampedToPoseStamped",
        "CallManageLifecycleNodes",
        "Script",
        "ForceSuccess",
        "ScriptCondition",
        "GetLatestTransform",
        "ConvertTransformStampedToPoseStamped",
        "ReinterpretPoseFrame",
        "TransformPoseWithPose",
        "Verify Localization Recovery",
        "CallManageLifecycleNodes",
        "ScriptCondition",
    ]
    assert "inlier_distance" in steps[0].get("code")
    assert "min_inlier_fraction" in steps[0].get("code")
    check, failure = list(steps[1])
    assert check.get("ID") == "WaitForServiceAvailable"
    assert check.get("service_name") == "/lifecycle_manager_mapping/manage_nodes"
    assert float(check.attrib["timeout"]) > 0
    assert _find(failure, "Action[@ID='LogMessage']").get("log_level") == "ERROR"
    assert failure.find("Action[@ID='AlwaysFailure']") is not None


def test_lifecycle_switch_pauses_and_resumes_navigation_around_localization() -> None:
    """Navigation pauses before the switch; resumes only after restore and verification."""
    steps = list(_find(_root(), "BehaviorTree/Control"))
    pause, resume = steps[5], steps[14]
    for step in (pause, resume):
        assert step.get("service_name") == "/lifecycle_manager_navigation/manage_nodes"
    assert (pause.get("command"), resume.get("command")) == ("1", "2")
    switch = _find(steps[7], ".//Action[@ID='SwitchLifecycleManagers']")
    assert switch.get("source_manager") == "/lifecycle_manager_localization"
    assert switch.get("target_manager") == "/lifecycle_manager_mapping"
    assert switch.get("reset_source") == "true"
    # Restore must succeed, and the pose be verified, before navigation resumes.
    assert steps[8].get("code") == "localization_restored"
    verify = steps[13]
    assert verify.get("inlier_distance") == "{inlier_distance}"
    assert verify.get("min_inlier_fraction") == "{min_inlier_fraction}"
    # map_saved is checked last so an aborted run still ends failed.
    assert steps[15].get("code") == "map_saved"
    # The recovery pose carries the pre-mapping map -> odom through odometry.
    start = steps[3]
    assert (start.get("target_frame_id"), start.get("source_frame_id")) == (
        "map",
        "odom",
    )
    now = steps[9]
    assert (now.get("target_frame_id"), now.get("source_frame_id")) == (
        "odom",
        "footprint",
    )


def test_verification_fails_closed_without_lunar_thresholds() -> None:
    """Scan-match thresholds have no default, and the check uses the front scan and amcl range."""
    sub = ET.parse(
        PACKAGE / "objectives" / "verify_localization_recovery.xml"
    ).getroot()
    for name in ("inlier_distance", "min_inlier_fraction"):
        for tree in (_root(), sub):
            port = _find(tree, f"TreeNodesModel/SubTree/input_port[@name='{name}']")
            assert port.get("default") is None
    match = _find(sub, ".//Action[@ID='ScanMatchResidual']")
    assert match.get("base_frame_id") == "footprint"
    assert _find(sub, ".//Action[@ID='GetLaserScan']").get("topic_name") == (
        "/scan_front_filtered"
    )
    params = yaml.safe_load((PACKAGE / "params" / "nav2_params.yaml").read_text())
    assert (
        float(match.get("max_range"))
        == params["amcl"]["ros__parameters"]["laser_max_range"]
    )


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
    assert "does not check for collisions" in prompt
    assert "resume against the shipped map" in prompt
    assert "live map" in prompt
    assert "Continue — Save Map" in prompt
    assert "return" not in prompt


def test_localization_is_default_and_behaviors_are_loaded() -> None:
    """Build Map needs the default localization mode, with SLAM idle until the Objective runs."""
    config = yaml.safe_load((PACKAGE / "config" / "config.yaml").read_text())
    params = {
        k: v
        for p in config["hardware"]["robot_description"]["urdf_params"]
        for k, v in p.items()
    }
    assert params["slam"] is False
    assert params["localization"] is True
    assert (PACKAGE / "maps" / "lunar_map.yaml").is_file()
    map_config = yaml.safe_load((PACKAGE / "maps" / "lunar_map.yaml").read_text())
    assert (PACKAGE / "maps" / map_config["image"]).is_file()
    loaders = config["objectives"]["behavior_loader_plugins"]["core"]
    assert "moveit_pro::behaviors::CoreBehaviorsLoader" in loaders
    assert "moveit_pro::behaviors::NavBehaviorsLoader" in loaders


def test_save_confirmation_follows_success_and_names_configured_prefix() -> None:
    """Failed saves cannot reach the log, and custom prefixes are not replaced by lunar_map."""
    save_steps = list(_find(_root(), ".//Control[@name='SaveMap']"))
    assert [step.get("ID") for step in save_steps] == [
        "CallSaveMapService",
        "Script",
        "Script",
        "LogMessage",
    ]
    assert save_steps[1].attrib["code"] == "map_saved = true"
    script = save_steps[2].attrib["code"]
    assert "..map_name.." in script
    assert "Runtime" in script
    assert "working directory" in script
    assert "lunar_map" not in script
    assert save_steps[3].get("message") == "{save_message}"
