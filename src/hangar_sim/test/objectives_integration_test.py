# Copyright 2025 PickNik Inc.
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

"""Integration tests for the hangar_sim objective library."""

import copy
import sys
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import rclpy
import tf2_ros
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import yaml
from rclpy.action import ActionClient
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from control_msgs.action import GripperCommand
from controller_manager_msgs.srv import ListControllers
from moveit_msgs.msg import CollisionObject, MoveItErrorCodes, PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene
from moveit_studio_sdk_msgs.action import DoObjectiveSequence
from moveit_pro_test_utils.objective_test_fixture import (
    DEFAULT_OBJECTIVE_WAIT_S,
    EndStateSpec,
    ExecuteObjectiveResource,
    JointTarget,
    MUJOCO_RESET_HOOK,
    MUJOCO_RESET_SERVICE,
    SIM_RESETTER,
    execute_objective_resource as execute_objective_resource,
    get_objective_pytest_params,
    reset_simulation_before_test as reset_simulation_before_test,
    run_objective,
    wait_for_action_servers,
    wait_for_services,
)

# hangar_sim is MuJoCo-backed (declares a mujoco_model), so it advertises
# /mujoco_system/reset_keyframe. Register it with the fixture's reset hook here
# -- the fixture documents this module as the per-sim fan-out point -- so the
# autouse reset_simulation_before_test fixture resets the keyframe between
# objectives. Registering here keeps this an example_ws-only change instead of
# editing the fixture's built-in defaults in moveit_pro.
SIM_RESETTER.register("hangar_sim", MUJOCO_RESET_HOOK)

# Loop-style objectives cancelled mid-execution rather than waited to
# completion. "Plan Path Along Surface - Loop" wraps the surface-following
# sequence in KeepRunningUntilFailure, so it never terminates on its own.
#
# NOTE (jazzy watch-item): this objective ticks SendPointCloudToUI. If the
# first dispatched CI run reproduces the jazzy `pcl::fromPCLPointCloud2: No
# data to copy` crash, move this entry to skip_objectives and cite the
# PR #19531 subscriber-gated render-skip follow-up.
cancel_objectives = {
    "Plan Path Along Surface - Loop",
}

# Objectives skipped entirely, each with the reason it is not run in CI. The
# non-runnable objectives (runnable="false" in their MetadataFields) are
# filtered out by the fixture and need not be listed here.
skip_objectives = {
    # ML segmentation / grasp pipelines: ONNX inference runs on CPU in CI
    # (the GPU runner backs MuJoCo's EGL render only), so these time out --
    # same class lab_sim skips for the same reason.
    "ML Move Boxes to Loading Zone",
    "Move Boxes Looping",  # KeepRunningUntilFailure loop over the ML pick pipeline above.
    # SAM3 diagnostic: needs the moveit_pro_sam3 model package, which the CI
    # image does not ship (GetMasks2DFromExemplar fails to resolve the encoder
    # model path). Same ML-inference class as the skips above.
    "Calibrate SAM3 Mask Areas",
    "Segment Image from Point",
    "Segment Image from Text Prompt",
    "Segment Point Cloud from Clicked Point",
    # Fails by design: ValidateTrajectory rejects the intentionally-colliding
    # demo path and the Objective ends on that failure.
    "Cartesian Path with Collision Checking",
    # User input required: no primary UI is attached in headless CI, so these
    # block on a pose-from-user prompt or an MTC-solution/path approval that
    # never arrives.
    "Navigate to Clicked Point",  # GetPoseFromUser + WaitForUserPathApproval.
    "Navigate to Clicked Point with Replanning",  # GetPoseFromUser.
    "Localize Robot",  # AdjustPoseWithIMarker needs an operator to place the marker.
    # AMCL can lose track during the earlier "Plan Path Along Surface - Loop", and the scan check then rejects.
    "Refine Localization In Place",
    "Find and Spray Plane",  # Ungated WaitForMTCSolutionApproval.
    "Solution - Find and Spray Plane",  # Ungated WaitForMTCSolutionApproval.
    "Solution - Spray Plane",  # Ungated WaitForMTCSolutionApproval.
    # IsUserAvailable returns FAILURE headless, so its top-level Sequence
    # fails before reaching WaitForMTCSolutionApproval.
    "Solution - Draw Square",
    "Cartesian Plan Simple Square",
    # Core-library objectives (moveit_pro_objectives) aggregated into every
    # config. Both need a primary UI and are skipped by lab_sim for the same
    # reasons.
    "Teleoperate",  # DoTeleoperateAction rejects the goal with no UI subscribed.
    "Marker Visualization Example",  # GetTextFromUser server unavailable headless.
<<<<<<< HEAD
    # Hand-eye calibration Objectives from moveit_pro_objectives: they generate or
    # move through calibration waypoints and detect a ChArUco board, which only
    # hand_eye_calibration_sim provides.
    "Calibrate Camera Intrinsics",
    "Calibrate Eye In Hand Camera",
    "Calibrate Eye To Hand Camera",
    "Calibrate Multiple Cameras",
    "Generate Calibration Waypoints",
    # A 121 m drive for scoring odometry against ground truth, not a smoke test;
    # it is run by hand.
    "Navigate Hangar Loop",
=======
>>>>>>> origin/v10.2
    # Intermittent Jazzy CI-runner flakes. Skipped so the suite is deterministic.
    #
    # "Solution - Draw Picknik" is the suite's longest objective and sits on
    # the 90 s execute-timeout boundary on the jazzy runner. When it overruns,
    # the mid-execution halt leaves the joint_trajectory_controller commanding
    # the interrupted draw pose; the 5 s inter-test reset cannot recover it, so
    # the next objectives plan from a self-colliding start. Skipping removes the
    # timeout flake and its downstream sim-state pollution.
    "Solution - Draw Picknik",
}

# Execute-timeout overrides for objectives that legitimately exceed the 90 s
# default on the CI runner. The surface-following objectives run the full
# wrist-snap -> raster-generation -> Cartesian-plan pipeline; on the shared CI
# runner the single-pass variant needs >90 s and the 3-pass variant passed with
# only ~20 s of headroom. (Before the nav2 TF fix these objectives failed fast
# at the point-cloud transform ~30 s in, so the default budget never bound.)
EXECUTE_TIMEOUT_OVERRIDES_S = {
    "Plan Path Along Surface": 180.0,
    "Plan Path Along Surface 3 Passes": 180.0,
}

# Action servers the hangar_sim tree types need before any objective runs.
# hangar_sim drives a vacuum gripper through ros2_control's
# GripperActionController (config/control/picknik_ur.ros2_control.yaml), which
# advertises this GripperCommand action once control comes up.
required_action_servers: list[tuple[type, str]] = [
    (GripperCommand, "/vacuum_gripper/gripper_cmd"),
]


@pytest.fixture(scope="module", autouse=True)
def wait_for_mujoco_reset_service(
    execute_objective_resource: ExecuteObjectiveResource,
) -> None:
    """Block until the MuJoCo reset service is advertised, once per module.

    hangar_sim's startup is heavy (ridgeback base + nav2 + the full controller
    set + a large scene), so ``/mujoco_system/reset_keyframe`` comes up well
    after the agent heartbeat that gates ``execute_objective_resource``. The
    autouse ``reset_simulation_before_test`` hook only allows a 5 s budget per
    test, which the first reset blows in CI. This module-scoped fixture runs
    before any per-test reset and warms the service up with a generous timeout,
    so each per-test reset then finds it immediately.
    """
    wait_for_services(
        execute_objective_resource.node, [MUJOCO_RESET_SERVICE], timeout_sec=120.0
    )


# Controllers the config spawns at startup (config.yaml ros2_control
# controllers_active_at_startup + controllers_inactive_at_startup), read from
# the config itself so the list cannot drift from it. The spawners run with
# --controller-manager-timeout 180 and can lag several minutes behind the
# first tests on a loaded CI runner, so the first controller-using objective
# (cartesian_draw_geometry_from_file) raced them and failed SwitchController
# with "Controller joint_trajectory_controller is not an existing controller"
# or "command interface platform_velocity_controller/linear_x_joint/velocity
# is not available".
_CONFIG_YAML_PATH = Path(__file__).parent.parent / "config" / "config.yaml"


def _spawned_controllers_from_config() -> frozenset[str]:
    """Read the spawned-controller names from hangar_sim's config.yaml."""
    with _CONFIG_YAML_PATH.open("r", encoding="utf-8") as config_file:
        ros2_control = yaml.safe_load(config_file)["ros2_control"]
    return frozenset(
        ros2_control["controllers_active_at_startup"]
        + ros2_control["controllers_inactive_at_startup"]
    )


REQUIRED_CONTROLLERS = _spawned_controllers_from_config()
# A controller in either of these states is loaded and configured; "inactive"
# is enough for a chained controller to export its command interfaces.
CONFIGURED_CONTROLLER_STATES = ("inactive", "active")
CONTROLLER_LOAD_TIMEOUT_S = 300.0
LIST_CONTROLLERS_CALL_TIMEOUT_S = 10.0


@pytest.fixture(scope="module", autouse=True)
def wait_for_controllers_loaded(
    execute_objective_resource: ExecuteObjectiveResource,
) -> None:
    """Block until every spawned controller is loaded and configured.

    Waits for each controller in ``REQUIRED_CONTROLLERS`` to report
    ``inactive`` or ``active`` from ``/controller_manager/list_controllers``,
    once per module. ``inactive`` is enough: a configured chained controller
    (platform_velocity_controller) already exports the command interfaces the
    whole-body joint_trajectory_controller claims on activation.

    Defined after ``wait_for_mujoco_reset_service`` on purpose: pytest runs
    module-scoped autouse fixtures in definition order, and the MuJoCo reset
    service must be warm before the first per-test reset regardless of how
    long the controller wait takes. Keep this fixture below that one.
    """
    node = execute_objective_resource.node
    client = node.create_client(ListControllers, "/controller_manager/list_controllers")
    missing = set(REQUIRED_CONTROLLERS)
    deadline = time.monotonic() + CONTROLLER_LOAD_TIMEOUT_S
    try:
        while time.monotonic() < deadline:
            if not client.wait_for_service(timeout_sec=1.0):
                continue
            future = client.call_async(ListControllers.Request())
            call_deadline = time.monotonic() + LIST_CONTROLLERS_CALL_TIMEOUT_S
            while not future.done() and time.monotonic() < call_deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
            if not future.done():
                # Abandoning an in-flight future and destroying the client
                # later trips rcl "use after free" errors — cancel it first.
                future.cancel()
                # Pace the retry instead of re-issuing the call back-to-back.
                rclpy.spin_once(node, timeout_sec=0.5)
                continue
            response = future.result()
            configured = {
                controller.name
                for controller in response.controller
                if controller.state in CONFIGURED_CONTROLLER_STATES
            }
            missing = set(REQUIRED_CONTROLLERS) - configured
            if not missing:
                return
            rclpy.spin_once(node, timeout_sec=0.5)
        pytest.fail(
            f"Controllers not loaded and configured within "
            f"{CONTROLLER_LOAD_TIMEOUT_S:.1f}s: {sorted(missing)}. "
            "Objectives that switch controllers cannot run."
        )
    finally:
        node.destroy_client(client)


# Fixed and end-effector frames the Cartesian objectives visualize and plan
# against (VisualizePose / PlanCartesianPath look up TF_FIXED_FRAME ->
# TF_TIP_FRAME).
TF_FIXED_FRAME = "world"
TF_TIP_FRAME = "grasp_link"
TF_WARMUP_TIMEOUT_S = 120.0


@pytest.fixture(scope="module", autouse=True)
def wait_for_robot_tf(
    execute_objective_resource: ExecuteObjectiveResource,
) -> None:
    """Block until ``world -> grasp_link`` is on ``/tf``, once per module.

    ``grasp_link``'s transform only appears after robot_state_publisher and the
    arm controllers start streaming joint states, which lags the agent
    heartbeat on a cold backend. The Cartesian objectives look it up
    immediately, so on the slower jazzy runner the first one
    (``cartesian_draw_geometry_from_file``) raced TF and failed with "Cannot
    find transform between frames ``world`` and ``grasp_link``". Spinning a
    short-lived listener until the transform resolves removes that startup race
    without masking the objective.
    """
    node = execute_objective_resource.node
    buffer = tf2_ros.Buffer()
    # spin_thread=False: we pump the node manually below, matching the rest of
    # the fixture's single-threaded spin model (no background TF thread).
    tf2_ros.TransformListener(buffer, node, spin_thread=False)
    deadline = time.monotonic() + TF_WARMUP_TIMEOUT_S
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if buffer.can_transform(TF_FIXED_FRAME, TF_TIP_FRAME, Time()):
            return
    pytest.fail(
        f"{TF_FIXED_FRAME} -> {TF_TIP_FRAME} transform not available within "
        f"{TF_WARMUP_TIMEOUT_S:.1f}s; cannot run Cartesian objectives."
    )


# --- End-state correctness checks ---
#
# A small, deterministic set of objectives gets an end-state assertion beyond
# SUCCESS/FAILURE. Broad population is out of scope (follow-up tied to #17769).
#
# "Move to Arm Upright" drives the manipulator group to the "Arm upright"
# waypoint via the joint_trajectory_controller. We resolve the expected target
# at runtime from /get_saved_waypoints (same source of truth the objective
# uses), then restrict the comparison to the manipulator arm joints: the saved
# waypoint's joint_state also carries the mecanum base joints (linear_x/y,
# rotational_yaw), which this objective does not command, so asserting them
# would compare against an uncommanded pose.
JOINT_CHECK_OBJECTIVE = "Move to Arm Upright"
JOINT_CHECK_WAYPOINT = "Arm upright"
MANIPULATOR_ARM_JOINTS = (
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
)


def _expected_end_state_by_id(
    objective_id: str,
    resource: ExecuteObjectiveResource,
) -> dict[str, EndStateSpec] | None:
    """Build the end-state spec for objectives we assert on; None otherwise.

    Resolves the waypoint target lazily, only for the objective under test, so
    a missing /get_saved_waypoints surfaces solely on the objective that needs
    it rather than penalizing the whole suite.
    """
    if objective_id != JOINT_CHECK_OBJECTIVE:
        return None
    waypoint_joints = resource.get_waypoint_target(JOINT_CHECK_WAYPOINT)
    arm_target = {joint: waypoint_joints[joint] for joint in MANIPULATOR_ARM_JOINTS}
    return {objective_id: EndStateSpec(joints=JointTarget(positions=arm_target))}


BASE_LINK_FRAME = "ridgeback_base_link"
# Phase 1: how long to wait for the FIRST transform into the base link (and the
# first /odom message) before declaring the stack broken. Generous on purpose:
# it only bounds startup latency on a loaded CI runner, not the assertion.
TF_FIRST_SIGHTING_TIMEOUT_S = 60.0
# Phase 2: once the frame has been seen, how long to keep sampling to catch
# additional (conflicting) parents. Competing publishers broadcast at 10-120 Hz,
# so a second parent would appear hundreds of times within this window.
TF_PARENT_SAMPLE_DURATION_S = 4.0


def test_base_link_has_single_tf_parent(
    execute_objective_resource: ExecuteObjectiveResource,
) -> None:
    """The live transform into ridgeback_base_link must have exactly one publisher.

    Regression guard for the three-parent TF conflict (MuJoCo lidar fill-in
    chain, MuJoCo odom TF, and robot_state_publisher all claiming the frame),
    which made the robot oscillate below base_link in the web UI.
    robot_state_publisher owns the edge via the virtual-rail chain; a second
    parent appearing here means a MuJoCo TF publisher was re-enabled (check
    base_link_name / odom_publish_tf in the ros2_control xacro) or fuse's
    publish_tf was turned back on.
    """
    node = execute_objective_resource.node
    parents: set[str] = set()
    odom_z_samples: list[float] = []

    # Watching /tf only is correct while rotational_yaw_joint (the edge into
    # ridgeback_base_link) is a moving joint; if it ever becomes fixed the edge
    # moves to /tf_static and this test must follow it there.
    def collect(msg: TFMessage) -> None:
        for transform in msg.transforms:
            if transform.child_frame_id == BASE_LINK_FRAME:
                parents.add(transform.header.frame_id)

    def collect_odom(msg: Odometry) -> None:
        odom_z_samples.append(msg.pose.pose.position.z)

    subscription = node.create_subscription(TFMessage, "/tf", collect, 100)
    # Piggyback an odom_planar check: odom_zero_z was a silently-ignored
    # parameter for months, so pin the real one's effect (z zeroed in /odom)
    # rather than trusting the spelling.
    odom_subscription = node.create_subscription(
        Odometry, "/odom", collect_odom, qos_profile_sensor_data
    )
    # Phase 1: wait for the first sighting so slow stack bring-up on a loaded
    # CI runner cannot masquerade as "wrong publishers" (or eat into the
    # sampling window below).
    first_sighting_deadline = time.monotonic() + TF_FIRST_SIGHTING_TIMEOUT_S
    while time.monotonic() < first_sighting_deadline and not (
        parents and odom_z_samples
    ):
        rclpy.spin_once(node, timeout_sec=0.1)

    # Distinguish "frame never seen" (a startup failure or renamed frame) from
    # "wrong publishers" so failures read correctly.
    assert parents, (
        f"No transform into {BASE_LINK_FRAME} observed on /tf within "
        f"{TF_FIRST_SIGHTING_TIMEOUT_S:.1f}s — robot_state_publisher may not "
        f"be up, or the frame was renamed."
    )

    # Phase 2: the frame is live; sample long enough that any competing
    # publisher (10-120 Hz) would land in `parents`.
    sample_deadline = time.monotonic() + TF_PARENT_SAMPLE_DURATION_S
    while time.monotonic() < sample_deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_subscription(subscription)
    node.destroy_subscription(odom_subscription)
    assert parents == {"virtual_rail_link_2"}, (
        f"Expected robot_state_publisher's virtual_rail_link_2 as the sole TF "
        f"parent of {BASE_LINK_FRAME}, saw parents: {sorted(parents)}"
    )
    assert (
        odom_z_samples
    ), "No /odom messages observed; MuJoCo odom publisher may be off."
    assert all(abs(z) < 1e-6 for z in odom_z_samples), (
        f"odom_planar should zero /odom pose z for this planar base; "
        f"saw max |z| = {max(abs(z) for z in odom_z_samples):.4f}"
    )


FUSE_INPUT_TIMEOUT_S = 60.0


def test_fuse_inputs_reach_a_reliable_subscriber(
    execute_objective_resource: ExecuteObjectiveResource,
) -> None:
    """fuse subscribes RELIABLE, which silently receives nothing from a BEST_EFFORT publisher."""
    node = execute_objective_resource.node
    topics = {
        "/imu_sensor_broadcaster/imu": Imu,
        "/platform_velocity_controller/odom": Odometry,
        "/platform_velocity_controller_nav2/odom": Odometry,
    }
    received = dict.fromkeys(topics, 0)

    def count(topic):
        return lambda _msg: received.__setitem__(topic, received[topic] + 1)

    def satisfied():
        # Only the active base controller publishes odom, so either odom topic counts.
        odom = [n for topic, n in received.items() if topic.endswith("/odom")]
        return received["/imu_sensor_broadcaster/imu"] > 0 and any(odom)

    # Default QoS is RELIABLE, as in fuse.
    subscriptions = [
        node.create_subscription(msg_type, topic, count(topic), 10)
        for topic, msg_type in topics.items()
    ]
    deadline = time.monotonic() + FUSE_INPUT_TIMEOUT_S
    while time.monotonic() < deadline and not satisfied():
        rclpy.spin_once(node, timeout_sec=0.1)
    for subscription in subscriptions:
        node.destroy_subscription(subscription)
    assert satisfied(), f"fuse input silent to a RELIABLE subscriber: {received}"


# --- Box drop footprint reservation ---
#
# "Move Boxes to Loading Zone Start from Waypoint" reserves each drop spot as a
# placed_box_N collision object before its whole-body hover move, so the base
# cannot park under it. The ML Objectives that run it are skipped above, so
# these tests run its real post-lift steps against the live planning scene,
# with only the hover move and the gripper release swapped for stand-ins.
OBJECTIVES_DIR = Path(__file__).parent.parent / "objectives"
SUCCEED = '<Action ID="AlwaysSuccess" />'
FAIL = '<Action ID="AlwaysFailure" />'
# Hover stand-in that succeeds only if the planner would see this drop's reservation.
SEES_RESERVATION = (
    '<Control ID="Sequence">'
    '<Action ID="GetCurrentPlanningScene" planning_scene_msg="{scene}" />'
    '<Action ID="IsCollisionObjectInPlanningScene" planning_scene="{scene}"'
    ' collision_object_name="{placed_box_id}" />'
    "</Control>"
)
PLACED_BOXES = ("placed_box_0", "placed_box_1")
HOVER_MARKER = "drop_hover_checked"
RELEASE_MARKER = "drop_release_checked"
TEST_OBJECTS = PLACED_BOXES + (HOVER_MARKER, RELEASE_MARKER)
# box.stl is a 0.25 m cube; one resting on the hangar floor is centered here.
RESTING_BOX_CENTER_Z = 0.125
OBJECTIVE_XML_TIMEOUT_S = 60.0


def _checked_stand_in(marker: str, outcome: str) -> str:
    """Leave a scene marker only after observing this drop's reservation."""
    sequence = ET.fromstring(SEES_RESERVATION)
    sequence.append(
        ET.Element(
            "Action",
            ID="AddCollisionMesh",
            object_id=marker,
            pose="{placed_box_pose}",
            mesh_file_path="package://hangar_sim/objectives/box.stl",
            scale="1;1;1",
            overwrite="true",
        )
    )
    sequence.append(ET.fromstring(outcome))
    return ET.tostring(sequence, encoding="unicode")


def _objective_xml(*steps: ET.Element) -> str:
    root = ET.Element("root", BTCPP_format="4", main_tree_to_execute="Box Placement")
    tree = ET.SubElement(root, "BehaviorTree", ID="Box Placement")
    ET.SubElement(tree, "Control", ID="Sequence").extend(steps)
    return ET.tostring(root, encoding="unicode")


def _place_two_boxes_xml(hover: str, release: str) -> str:
    """Place two boxes at the real loading-zone hover pose; the second uses these stand-ins."""
    place_pose = ET.parse(OBJECTIVES_DIR / "ml_move_boxes_to_loading_zone.xml").find(
        ".//Action[@ID='CreatePoseStamped'][@pose_stamped='{place_pose}']"
    )
    pick_and_place = ET.parse(
        OBJECTIVES_DIR / "move_boxes_to_loading_zone_from_waypoint.xml"
    )
    lift = pick_and_place.find(".//Action[@ID='ExecuteTrajectory']")
    steps = list(pick_and_place.find(".//Action[@ID='ExecuteTrajectory']/.."))
    after_lift = steps[steps.index(lift) + 1 :]

    def placement(hover: str, release: str) -> ET.Element:
        sequence = ET.Element("Control", ID="Sequence")
        sequence.extend(copy.deepcopy(after_lift))
        stand_ins = {"Move to Pose No Preview": hover, "MoveGripperAction": release}
        for node in list(sequence.iter()):
            for index, step in enumerate(node):
                if step.get("ID") in stand_ins:
                    node[index] = ET.fromstring(stand_ins[step.get("ID")])
        return sequence

    return _objective_xml(
        place_pose,
        ET.Element("Action", ID="Script", code="placed_count := 0"),
        placement(SEES_RESERVATION, SUCCEED),
        placement(hover, release),
    )


def _run_objective_xml(
    resource: ExecuteObjectiveResource, xml: str
) -> DoObjectiveSequence.Result:
    client = ActionClient(resource.node, DoObjectiveSequence, "/do_objective")
    try:
        assert client.wait_for_server(timeout_sec=30.0), "/do_objective unavailable"
        handle = resource.spin_until_future_complete(
            client.send_goal_async(DoObjectiveSequence.Goal(objective_xml_string=xml)),
            timeout_sec=30.0,
        )
        assert handle is not None and handle.accepted, "/do_objective rejected goal"
        result_future = handle.get_result_async()
        response = resource.spin_until_future_complete(
            result_future, timeout_sec=OBJECTIVE_XML_TIMEOUT_S
        )
        if response is None:
            handle.cancel_goal_async()
            stopped = resource.spin_until_future_complete(
                result_future, timeout_sec=30.0
            )
            assert stopped is not None, "/do_objective timed-out goal did not stop"
            raise AssertionError("/do_objective goal did not finish before timeout")
        return response.result
    finally:
        client.destroy()


def _test_scene_objects(
    resource: ExecuteObjectiveResource,
) -> dict[str, CollisionObject]:
    client = resource.node.create_client(GetPlanningScene, "/get_planning_scene")
    try:
        assert client.wait_for_service(
            timeout_sec=30.0
        ), "/get_planning_scene unavailable"
        request = GetPlanningScene.Request()
        request.components.components = (
            PlanningSceneComponents.WORLD_OBJECT_NAMES
            | PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
        )
        response = resource.spin_until_future_complete(
            client.call_async(request), timeout_sec=30.0
        )
        assert response is not None, "/get_planning_scene did not respond"
        return {
            obstacle.id: obstacle
            for obstacle in response.scene.world.collision_objects
            if obstacle.id in TEST_OBJECTS
        }
    finally:
        resource.node.destroy_client(client)


@pytest.mark.parametrize(
    "hover, release, expected_boxes, expected_markers",
    [
        (
            _checked_stand_in(HOVER_MARKER, SUCCEED),
            _checked_stand_in(RELEASE_MARKER, SUCCEED),
            PLACED_BOXES,
            (HOVER_MARKER, RELEASE_MARKER),
        ),
        (
            _checked_stand_in(HOVER_MARKER, FAIL),
            _checked_stand_in(RELEASE_MARKER, SUCCEED),
            PLACED_BOXES[:1],
            (HOVER_MARKER,),
        ),
        (
            _checked_stand_in(HOVER_MARKER, SUCCEED),
            _checked_stand_in(RELEASE_MARKER, FAIL),
            PLACED_BOXES[:1],
            (HOVER_MARKER, RELEASE_MARKER),
        ),
    ],
    ids=["placed", "hover_fails", "release_fails"],
)
def test_box_drop_footprint_is_reserved_before_hover(
    hover: str,
    release: str,
    expected_boxes: tuple[str, ...],
    expected_markers: tuple[str, ...],
    execute_objective_resource: ExecuteObjectiveResource,
) -> None:
    """Hover plans around the reserved drop; a failed drop removes only its own reservation."""
    try:
        result = _run_objective_xml(
            execute_objective_resource, _place_two_boxes_xml(hover, release)
        )
        objects = _test_scene_objects(execute_objective_resource)
    finally:
        primary_error = sys.exc_info()[1]
        try:
            cleanup_result = _run_objective_xml(
                execute_objective_resource,
                _objective_xml(
                    *(
                        ET.Element("Action", ID="RemoveCollisionObject", object_id=box)
                        for box in TEST_OBJECTS
                    )
                ),
            )
            assert (
                cleanup_result.error_code.val == MoveItErrorCodes.SUCCESS
            ), cleanup_result.error_message
        except Exception as cleanup_error:
            if primary_error is None:
                raise
            print(f"placed_box cleanup failed: {cleanup_error}")
    placed = expected_boxes == PLACED_BOXES
    assert (
        result.error_code.val == MoveItErrorCodes.SUCCESS
    ) == placed, result.error_message
    assert set(objects) == set(expected_boxes + expected_markers)
    for box_id in expected_boxes:
        box = objects[box_id]
        assert box.pose.position.z == pytest.approx(RESTING_BOX_CENTER_Z, abs=1e-6)


@pytest.mark.parametrize(
    "objective_id, should_cancel",
    get_objective_pytest_params("hangar_sim", cancel_objectives, skip_objectives),
)
def test_all_objectives(
    objective_id: str,
    should_cancel: bool,
    execute_objective_resource: ExecuteObjectiveResource,
):
    """Run (or cancel) each hangar_sim objective and assert it completes without error."""
    wait_for_action_servers(
        execute_objective_resource.node,
        [name for _, name in required_action_servers],
    )
    expected_end_state_by_id = _expected_end_state_by_id(
        objective_id, execute_objective_resource
    )
    try:
        run_objective(
            objective_id,
            should_cancel,
            execute_objective_resource,
            objective_wait_time=EXECUTE_TIMEOUT_OVERRIDES_S.get(
                objective_id, DEFAULT_OBJECTIVE_WAIT_S
            ),
            expected_end_state_by_id=expected_end_state_by_id,
        )
    except AssertionError as e:
        mode = "cancel" if should_cancel else "execute"
        pytest.fail(f"Objective '{objective_id}' failed to {mode}: {e}")
    except Exception as e:
        mode = "cancel" if should_cancel else "execute"
        pytest.fail(
            f"Objective '{objective_id}' hit an unexpected error during {mode}: "
            f"{type(e).__name__}: {e}"
        )
