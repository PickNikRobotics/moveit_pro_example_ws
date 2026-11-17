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
import yaml

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
    assert refinement.get("restore_pose") == "{initial_pose}"
    assert refinement.get("restore_estimate") == "{restore_estimate}"
    assert refinement.get("has_restore_estimate") == "{has_restore_estimate}"
    assert tree.find(".//Control[@ID='IfThenElse']") is not None
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
        "Finally",
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
    assert (
        navigation.get("execution_controller_names")
        == "platform_velocity_controller_nav2"
    )
    assert navigation.get("holding_controller_names") == "joint_trajectory_controller"
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


def test_queued_navigation_restores_arm_controller_on_halt_and_completion() -> None:
    """The halt-safe cleanup restores control without overriding the queue result."""
    tree = ET.parse(OBJECTIVES / "navigate_queued_waypoints.xml")
    cleanup = tree.find("./BehaviorTree/Control/Control[@ID='Finally']")
    assert cleanup is not None, "Stop Objective must reach controller restoration."
    execution, restoration = list(cleanup)
    assert execution.get("ID") == "Sequence"
    assert [node.get("ID") for node in execution] == ["Navigate Planar Waypoints"]
    assert execution.find("./Action[@ID='SwitchController']") is None
    assert restoration.get("ID") == "SwitchController"
    assert restoration.get("activate_controllers") == "joint_trajectory_controller"
    assert restoration.get("timeout") == "5.0"


SOURCES = OBJECTIVES.parents[1]
CORE_LIBRARY = ("moveit_pro_objectives", "objectives/core")
NAVIGATION_LIBRARY = ("moveit_pro_objectives", "objectives/navigation")
NAV_LOADER = "moveit_pro::behaviors::NavBehaviorsLoader"


def _config_packages() -> dict[str, Path]:
    """Map each configuration package name to its directory."""
    packages = {}
    for config in SOURCES.rglob("config/config.yaml"):
        package_xml = config.parents[1] / "package.xml"
        if "external_dependencies" not in config.parts and package_xml.exists():
            name = ET.parse(package_xml).getroot().findtext("name")
            packages[name] = config.parents[1]
    return packages


PACKAGES = _config_packages()


def _registered_by_key(config: str) -> tuple[dict[str, tuple[str, str]], set[str]]:
    """Objective libraries by key and Behavior loaders, a child's keys overriding its parent's."""
    data = yaml.safe_load((PACKAGES[config] / "config" / "config.yaml").read_text())
    libraries: dict[str, tuple[str, str]] = {}
    loaders: set[str] = set()
    if parent := data.get("based_on_package"):
        libraries, loaders = _registered_by_key(parent)
    objectives = data.get("objectives") or {}
    for key, library in (objectives.get("objective_library_paths") or {}).items():
        libraries[key] = (library["package_name"], library["relative_path"])
    for plugins in (objectives.get("behavior_loader_plugins") or {}).values():
        loaders |= set(plugins)
    return libraries, loaders


def _registered(config: str) -> tuple[set[tuple[str, str]], set[str]]:
    """Objective libraries and Behavior loaders a configuration registers, inherited ones included."""
    libraries, loaders = _registered_by_key(config)
    return set(libraries.values()), loaders


@pytest.mark.parametrize("config", sorted(PACKAGES))
def test_navigation_library_is_registered_only_with_nav_behaviors(config: str) -> None:
    """A configuration that registers the navigation library also loads the Behaviors it names."""
    libraries, loaders = _registered(config)
    if NAVIGATION_LIBRARY in libraries:
        assert (
            NAV_LOADER in loaders
        ), f"{config} registers navigation Objectives without {NAV_LOADER}."


def test_hangar_registers_the_navigation_library_its_queue_calls() -> None:
    """Navigate Queued Waypoints calls the Core Navigate Planar Waypoints Subtree."""
    assert NAVIGATION_LIBRARY in _registered("hangar_sim")[0]


def test_lunar_offers_planar_waypoint_navigation() -> None:
    """lunar_sim runs the Core Navigate Planar Waypoints Subtree through its own queued-navigation Objective."""
    assert NAVIGATION_LIBRARY in _registered("lunar_sim")[0]
    tree = ET.parse(
        SOURCES / "lunar_sim" / "objectives" / "navigate_queued_waypoints.xml"
    )
    navigation = tree.find(".//SubTree[@ID='Navigate Planar Waypoints']")
    assert navigation is not None
    # The rover's velocity controller is active from startup, so the wrapper leaves controller ownership external.
    assert navigation.get("execution_controller_names") == ""
    assert navigation.get("holding_controller_names") == ""
    config = yaml.safe_load(
        (PACKAGES["lunar_sim"] / "config" / "config.yaml").read_text()
    )
    assert (
        "platform_velocity_controller"
        in config["ros2_control"]["controllers_active_at_startup"]
    ), "Empty controller ports need lunar_sim's velocity controller active from startup."
    model = tree.find(".//TreeNodesModel/SubTree/MetadataFields/Metadata[@runnable]")
    assert model is not None
    assert model.get("runnable") == "true"


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
    """Each sweep skips exactly the runnable operator-confirmed Core Objectives its configuration registers."""
    source = SOURCES / config / "test" / "objectives_integration_test.py"
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
    libraries, _ = _registered(config)
    assert ("Collect Planar Waypoints" in skipped) == (CORE_LIBRARY in libraries)
    # Navigate Planar Waypoints is a non-runnable Subtree, which the sweep fixture already filters out.
    assert "Navigate Planar Waypoints" not in skipped
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


def test_regression_localization_captures_covariance_before_seed_without_hiding_failure() -> (
    None
):
    """An existing localization must yield its complete estimate before seeding."""
    tree = ET.parse(OBJECTIVES / "localize_robot.xml")
    sequence = tree.find("./BehaviorTree/Control[@ID='Sequence']")
    assert sequence is not None
    assert [node.get("ID") for node in sequence] == [
        "GetLatestTransform",
        "ConvertTransformStampedToPoseStamped",
        "GetPlanarPoseFromUser",
        "ReinterpretPoseFrame",
        "IfThenElse",
        "Refine Localization In Place Subtree",
    ]
    selection = sequence.find("./Control[@ID='IfThenElse']")
    assert selection is not None
    condition, capture_sequence, no_estimate = list(selection)
    transform = condition.find("./Action[@ID='GetLatestTransform']")
    assert transform is not None
    assert transform.get("target_frame_id") == "map"
    assert transform.get("source_frame_id") == "ridgeback_base_link"
    assert condition.get("ID") == "SuppressChildErrors"
    assert capture_sequence.get("ID") == "Sequence"
    capture, flag = list(capture_sequence)
    assert capture.get("ID") == "GetLocalizationEstimate"
    assert capture.get("estimate_topic") == "/pose"
    assert capture.get("update_service") == "/request_nomotion_update"
    assert capture.get("global_frame_id") == "map"
    assert capture.get("timeout_sec") == "5.0"
    assert flag.get("code") == "has_restore_estimate := 'true'"
    assert no_estimate.get("code") == "has_restore_estimate := 'false'"
    refinement = sequence[-1]
    assert capture.get("estimate") == refinement.get("restore_estimate")
    assert refinement.get("has_restore_estimate") == "{has_restore_estimate}"
    assert refinement.get("restore_pose") == "{initial_pose}"
    assert capture_sequence.find(".//Decorator[@ID='SuppressChildErrors']") is None


def test_regression_refinement_preserves_full_estimate_before_applying_seed() -> None:
    """Refinement captures uncertainty before TF seed lookup and restores that estimate."""
    tree = ET.parse(OBJECTIVES / "refine_localization_in_place.xml")
    sequence = tree.find("./BehaviorTree/Control[@ID='Sequence']")
    assert sequence is not None
    assert [node.get("ID") for node in sequence] == [
        "Localize Robot If Needed",
        "GetLocalizationEstimate",
        "GetLatestTransform",
        "ConvertTransformStampedToPoseStamped",
        "Refine Localization In Place Subtree",
    ]
    capture = sequence[1]
    refinement = sequence[-1]
    assert capture.get("estimate") == refinement.get("restore_estimate")
    assert refinement.get("has_restore_estimate") == "true"


def test_regression_restore_uses_covariance_without_falling_back_after_failure() -> (
    None
):
    """A wired captured estimate is restored directly; only first localization uses pose."""
    tree = ET.parse(OBJECTIVES / "refine_localization_in_place_subtree.xml")
    selection = tree.find(".//Control[@ID='IfThenElse']")
    assert selection is not None
    condition, restore_estimate, restore_pose = list(selection)
    assert condition.get("ID") == "ScriptCondition"
    assert condition.get("code") == "has_restore_estimate == 'true'"
    assert restore_estimate.get("ID") == restore_pose.get("ID") == "SetInitialPose"
    assert restore_estimate.get("estimate") == "{restore_estimate}"
    assert "pose" not in restore_estimate.attrib
    assert restore_pose.get("pose") == "{restore_pose}"
    assert "estimate" not in restore_pose.attrib
    model = tree.find("./TreeNodesModel/SubTree")
    assert model is not None
    estimate_port = model.find("./input_port[@name='restore_estimate']")
    assert estimate_port is not None
    assert estimate_port.get("type") == "geometry_msgs::msg::PoseWithCovarianceStamped"
    flag_port = model.find("./input_port[@name='has_restore_estimate']")
    assert flag_port is not None
    assert flag_port.get("default") == "false"
    assert flag_port.get("type") == "std::string"
