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

"""Verify planar pose selection is wired into navigation and localization."""

import ast
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest

OBJECTIVES = Path(__file__).resolve().parents[1] / "objectives"


@pytest.mark.parametrize(
    ("filename", "consumer", "input_port"),
    [
        ("navigate_to_clicked_point.xml", "ComputePathToPoseAction", "goal"),
        (
            "navigate_to_clicked_point_with_replanning.xml",
            "NavigateToPoseAction",
            "pose_stamped",
        ),
    ],
)
def test_navigation_uses_confirmed_planar_goal(
    filename: str, consumer: str, input_port: str
) -> None:
    """The shared picker supplies a map-frame pose to the existing navigation path."""
    tree = ET.parse(OBJECTIVES / filename)
    picker = tree.find(".//Action[@ID='GetPlanarPoseFromUser']")
    assert (
        picker is not None
    ), "Navigation must request an editable position and heading."
    assert picker.get("reference_frame") == "map"
    assert picker.get("view_name") == "Visualization"
    assert picker.get("user_pose") == "{goal}"
    navigation = tree.find(f".//Action[@ID='{consumer}']")
    assert navigation is not None
    assert navigation.get(input_port) == picker.get("user_pose")
    sequence = tree.find("./BehaviorTree/Control[@ID='Sequence']")
    assert sequence is not None
    children = list(sequence)
    refinement = sequence.find("./SubTree[@ID='Refine Localization In Place']")
    assert refinement is not None
    assert (
        children.index(refinement) < children.index(picker) < children.index(navigation)
    )
    assert tree.find(".//Action[@ID='GetPoseFromUser']") is None


def test_localization_uses_shared_picker_and_preserves_restore_flow() -> None:
    """A world-frame seed feeds the picker without lifting it or bypassing refinement."""
    tree = ET.parse(OBJECTIVES / "localize_robot.xml")
    picker = tree.find(".//Action[@ID='GetPlanarPoseFromUser']")
    assert (
        picker is not None
    ), "Localization must use the same planar pose control as navigation."
    assert picker.get("reference_frame") == "world"
    assert picker.get("initial_pose") == "{robot_pose}"
    assert picker.get("user_pose") == "{clicked_pose}"
    seed = tree.find(".//Action[@ID='ConvertTransformStampedToPoseStamped']")
    assert seed is not None
    assert seed.get("pose_stamped") == picker.get("initial_pose")
    reinterpret = tree.find(
        ".//Action[@ID='ReinterpretPoseFrame'][@output_pose='{initial_pose}']"
    )
    assert reinterpret is not None
    assert reinterpret.get("input_pose") == picker.get("user_pose")
    assert reinterpret.get("expected_input_frame_id") == "world"
    assert reinterpret.get("frame_id") == "map"
    refinement = tree.find(".//SubTree[@ID='Refine Localization In Place Subtree']")
    assert refinement is not None
    assert refinement.get("seed_pose") == reinterpret.get("output_pose")
    assert refinement.get("restore_pose") == "{restore_pose}"
    assert tree.find(".//Control[@ID='Fallback']") is not None
    assert tree.find(".//Action[@ID='AdjustPoseWithIMarker']") is None
    assert tree.find(".//Action[@ID='TransformPose']") is None


def test_navigation_path_approval_precedes_motion() -> None:
    """The confirmed goal is planned and approved before its path is followed."""
    tree = ET.parse(OBJECTIVES / "navigate_to_clicked_point.xml")
    sequence = tree.find("./BehaviorTree/Control[@ID='Sequence']")
    assert sequence is not None
    children = list(sequence)
    plan = sequence.find("./Action[@ID='ComputePathToPoseAction']")
    approval = sequence.find("./Action[@ID='WaitForUserPathApproval']")
    follow = sequence.find("./Action[@ID='FollowPathAction']")
    assert plan is not None
    assert approval is not None
    assert follow is not None
    assert children.index(plan) < children.index(approval) < children.index(follow)
    assert approval.get("path") == follow.get("path") == plan.get("path")


def test_queued_navigation_reuses_confirmed_collection_and_nav2_configuration() -> None:
    """The queue wrapper preserves localization and delegates approved legs to Core."""
    tree = ET.parse(OBJECTIVES / "navigate_queued_waypoints.xml")
    sequence = tree.find("./BehaviorTree/Control[@ID='Sequence']")
    assert sequence is not None
    assert [node.get("ID") for node in sequence] == [
        "Refine Localization In Place",
        "Fallback",
    ]
    behavior_tree = tree.find("./BehaviorTree")
    assert behavior_tree is not None
    assert behavior_tree.get("_favorite") == "false"
    navigation = sequence.find(
        "./Control/Control/SubTree[@ID='Navigate Planar Waypoints']"
    )
    assert navigation is not None
    assert navigation.get("reference_frame") == "map"
    assert navigation.get("waypoint_count") == "{waypoint_count}"
    assert navigation.get("planner_action_name") == "/compute_path_to_pose"
    assert navigation.get("controller_action_name") == "/follow_path"
    assert navigation.get("planner_id") == "GridBased"
    assert navigation.get("controller_id") == "FollowPath"
    assert navigation.get("goal_checker_id") == "general_goal_checker"
    assert navigation.get("max_pose_age_sec") == "1.0"
    assert navigation.get("timeout_sec") == "{timeout_sec}"
    model = tree.find("./TreeNodesModel/SubTree[@ID='Navigate Queued Waypoints']")
    assert model is not None
    count = model.find("./input_port[@name='waypoint_count']")
    assert count is not None
    assert count.get("type") == "int"
    assert count.get("default") == "2"
    timeout = model.find("./input_port[@name='timeout_sec']")
    assert timeout is not None
    assert timeout.get("type") == "double"
    assert timeout.get("default") == "-1"
    assert model.find("./MetadataFields/Metadata[@runnable='true']") is not None


def test_queued_navigation_restores_arm_controller_before_failure() -> None:
    """Rejected or failed queue execution restores control without returning success."""
    tree = ET.parse(OBJECTIVES / "navigate_queued_waypoints.xml")
    fallback = tree.find("./BehaviorTree/Control/Control[@ID='Fallback']")
    assert fallback is not None, "Queue failures must reach controller restoration."
    execution, recovery = list(fallback)
    assert execution.get("ID") == recovery.get("ID") == "Sequence"
    assert [node.get("ID") for node in execution] == [
        "SwitchController",
        "Navigate Planar Waypoints",
        "SwitchController",
    ]
    assert execution[-1].get("activate_controllers") == "joint_trajectory_controller"
    assert [node.get("ID") for node in recovery] == [
        "SwitchController",
        "AlwaysFailure",
    ]
    assert recovery[0].get("activate_controllers") == "joint_trajectory_controller"


@pytest.mark.parametrize(
    "config",
    [
        "hangar_sim",
        "april_tag_sim",
        "dual_arm_sim",
        "factory_sim",
        "grinding_sim",
        "kitchen_sim",
        "lab_sim",
        "lunar_sim",
        "market_sim",
        "rebot_bench_sim",
    ],
)
def test_headless_sweep_excludes_operator_confirmed_objectives(config: str) -> None:
    """Every inherited Core sweep lacks planar confirmation and path approval."""
    source = OBJECTIVES.parents[1] / config / "test" / "objectives_integration_test.py"
    assignments = ast.parse(source.read_text()).body
    skipped = None
    for node in assignments:
        if isinstance(node, ast.Assign):
            targets = node.targets
        elif isinstance(node, ast.AnnAssign):
            targets = [node.target]
        else:
            continue
        if any(
            isinstance(target, ast.Name) and target.id == "skip_objectives"
            for target in targets
        ):
            skipped = ast.literal_eval(node.value)
            break
    assert skipped is not None, f"{config} must declare its headless exclusions."
    assert {"Collect Planar Waypoints", "Navigate Planar Waypoints"} <= skipped
    if config == "hangar_sim":
        assert {
            "Navigate to Clicked Point",
            "Navigate to Clicked Point with Replanning",
            "Navigate Queued Waypoints",
            "Localize Robot",
        } <= skipped


def test_localization_guidance_names_pose_confirmation() -> None:
    """The operator warning identifies localization and the confirmation step."""
    tree = ET.parse(OBJECTIVES / "localize_robot_if_needed.xml")
    warning = tree.find(".//Action[@ID='LogMessage'][@log_level='warn']")
    assert warning is not None
    message = warning.get("message", "")
    assert "Robot not localized" in message
    assert "Localize Robot" in message
    assert "forward direction" in message
    assert "confirm the pose" in message


def test_localization_restore_branches_precede_refinement() -> None:
    """The estimate or placed-pose fallback supplies restore data before refinement."""
    tree = ET.parse(OBJECTIVES / "localize_robot.xml")
    sequence = tree.find("./BehaviorTree/Control[@ID='Sequence']")
    assert sequence is not None
    assert [node.get("ID") for node in sequence] == [
        "GetLatestTransform",
        "ConvertTransformStampedToPoseStamped",
        "GetPlanarPoseFromUser",
        "ReinterpretPoseFrame",
        "Fallback",
        "Refine Localization In Place Subtree",
    ]
    fallback = sequence.find("./Control[@ID='Fallback']")
    assert fallback is not None
    estimate = fallback.find(
        "./Decorator[@ID='SuppressChildErrors']/Control[@ID='Sequence']"
    )
    assert estimate is not None
    assert [node.get("ID") for node in estimate] == [
        "GetLatestTransform",
        "ConvertTransformStampedToPoseStamped",
    ]
    transform, converted = list(estimate)
    assert transform.get("target_frame_id") == "map"
    assert converted.get("transform_stamped") == transform.get("transform")
    restore = fallback.find("./Action[@ID='ReinterpretPoseFrame']")
    assert restore is not None
    assert restore.get("input_pose") == "{initial_pose}"
    assert restore.get("expected_input_frame_id") == restore.get("frame_id") == "map"
    refinement = sequence[-1]
    assert (
        converted.get("pose_stamped")
        == restore.get("output_pose")
        == refinement.get("restore_pose")
    )
