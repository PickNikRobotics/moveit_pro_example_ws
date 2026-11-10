# Copyright 2026 PickNik Inc.
# All rights reserved.
#
# Unauthorized copying of this code base via any medium is strictly prohibited.
# Proprietary and confidential.

"""Verify planar pose selection is wired into navigation and localization."""

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
