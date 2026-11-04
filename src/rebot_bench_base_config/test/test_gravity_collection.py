# Copyright 2026 PickNik Inc.
# SPDX-License-Identifier: BSD-3-Clause
"""Offline collection-route checks: real collision meshes, fixed scene and native timing.

No ROS node, action, service, CAN device, simulation or hardware execution.
"""
from pathlib import Path
import xml.etree.ElementTree as ET

from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject
from moveit_pro import planning
from moveit_pro_base import collision_detection, planning_scene, robot_model, robot_state
import numpy as np
import pytest
from shape_msgs.msg import SolidPrimitive
import xacro
import yaml

PACKAGE = Path(__file__).resolve().parents[1]
OBJECTIVES = PACKAGE / "objectives"
JOINTS = [f"joint{i}" for i in range(1, 8)]
HOME = np.zeros(6)


def waypoints():
    entries = yaml.safe_load((PACKAGE / "waypoints/rebot_waypoints.yaml").read_text())
    assert len({entry["name"] for entry in entries}) == len(entries)
    return {entry["name"]: entry for entry in entries}


def collection_poses():
    entries = waypoints()
    return [np.array(entries[f"Gravity {i:02d}"]["joint_state"]["position"], dtype=float)
            for i in range(1, 49)]


def collection_route():
    entries = waypoints()
    calls = ET.parse(OBJECTIVES / "collect_rebot_gravity_data.xml").findall(
        ".//BehaviorTree/Control/SubTree[@ID='Move to Waypoint']"
    )
    return [HOME, *[np.array(entries[call.get("waypoint_name")]["joint_state"]["position"][:6], dtype=float)
                    for call in calls]]


@pytest.fixture(scope="module")
def collision_model(tmp_path_factory):
    directory = tmp_path_factory.mktemp("gravity-model")
    source = (PACKAGE / "description/rebot.urdf.xacro").read_text().replace(
        "package://rebot_bench_base_config/", PACKAGE.as_uri() + "/"
    )
    path = directory / "robot.xacro"
    path.write_text(source)
    config = yaml.safe_load((PACKAGE / "config/config.yaml").read_text())
    mappings = {key: str(value) for entry in config["hardware"]["robot_description"]["urdf_params"]
                for key, value in entry.items()}
    mappings["hardware_interface"] = "mock"
    description = xacro.process_file(str(path), mappings=mappings).toxml()
    path = directory / "robot.urdf"
    path.write_text(description)
    model = robot_model.RobotModel(str(path), str(PACKAGE / "config/moveit/rebot.srdf"))
    model.set_group_acceleration_bounds("manipulator", -np.ones(6)*1.2, np.ones(6)*1.2)
    scene = planning_scene.PlanningScene(model)
    # Standard Move to Waypoint uses the installed 1 cm planning padding.
    scene.set_link_padding(0.01)
    frames = {}
    for action in ET.parse(OBJECTIVES / "rebot_fixed_bench_scene.xml").findall(".//Action"):
        if action.get("ID") == "CreatePoseStamped":
            frames[action.get("pose_stamped")] = [float(x) for x in action.get("position_xyz").split(";")]
        elif action.get("ID") == "AddCollisionBox":
            obj = CollisionObject()
            obj.id = action.get("object_id")
            obj.header.frame_id = "world"
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = frames[action.get("pose")]
            pose.orientation.w = 1.0
            obj.primitive_poses = [pose]
            obj.primitives = [SolidPrimitive(type=SolidPrimitive.BOX,
                dimensions=[float(x) for x in action.get("dimensions").split(";")])]
            obj.operation = CollisionObject.ADD
            scene.apply_collision_object(obj)
    state = robot_state.RobotState(model)
    state.set_to_default_values()
    request = collision_detection.CollisionRequest()
    request.joint_model_group_name = "manipulator_and_gripper"
    request.contacts = True
    request.max_contacts = 100
    request.pad_self_collisions = False
    request.pad_environment_collisions = True
    return model, scene, state, request


def check(collision_model, q, *, bounds=True):
    model, scene, state, request = collision_model
    if bounds:
        low, high = model.get_group_position_bounds("manipulator")
        assert np.all(q >= low-1e-9) and np.all(q <= high+1e-9), q
    state.joint_positions = dict(zip(JOINTS, [*q, 0.0]))
    state.update()
    result = collision_detection.CollisionResult()
    scene.check_collision(request, result, state)
    return result


def test_mesh_collision_canary(collision_model):
    # This deliberately sagged state collides, proving failed resource loads
    # cannot silently turn the following thousands of checks into empty geometry.
    assert check(collision_model, [0, 0, -.08, .5, .3, 0], bounds=False).collision
    assert not check(collision_model, HOME).collision


def test_fixed_scene_is_shared_and_has_no_cubes():
    scene = ET.parse(OBJECTIVES / "rebot_fixed_bench_scene.xml")
    assert {node.get("object_id") for node in scene.findall(".//Action[@ID='AddCollisionBox']")} == {
        "rebot_bench", "rebot_wall"
    }
    for name in ("rebot_stack_scene.xml", "collect_rebot_gravity_data.xml"):
        assert ET.parse(OBJECTIVES / name).find(".//SubTree[@ID='reBot Fixed Bench Scene']") is not None


def test_all_collection_transits_and_home_paths(collision_model):
    poses = collection_poses()
    assert len(poses) == 48
    route = collection_route()
    legs = list(zip(route, route[1:]))
    for start in poses:
        yaw = start.copy()
        yaw[4] = 0.0
        legs.extend([(start, yaw), (yaw, HOME)])
    checked = 0
    for leg, (start, goal) in enumerate(legs):
        for q in planning.interpolate_joint_path([start, goal], .005):
            result = check(collision_model, q)
            assert not result.collision, (leg, q, result.contacts)
            checked += 1
    print(f"Native FCL: {len(legs)} waypoint/HOME candidate paths; {checked} states; 0.005 rad spacing")


def test_native_full_speed_timing_and_all_timed_samples(collision_model):
    model = collision_model[0]
    group = model.get_joint_model_group("manipulator")
    route = collection_route()
    duration = 0.0
    max_velocity = 0.0
    checked = 0
    for start, goal in zip(route, route[1:]):
        trajectory = planning.create_trajectory_message_from_waypoints(group, [start, goal], 1.0, 1.0, 100)
        assert len(trajectory.points) > 1
        for point in trajectory.points:
            result = check(collision_model, point.positions)
            assert not result.collision, (point.positions, result.contacts)
            assert max(abs(x) for x in point.velocities) <= .600001
            assert max(abs(x) for x in point.accelerations) <= 1.200001
            max_velocity = max(max_velocity, max(abs(x) for x in point.velocities))
            checked += 1
        end = trajectory.points[-1].time_from_start
        duration += end.sec + end.nanosec*1e-9
    print(f"Native waypoint timing cross-check: {len(route)-1} moves, {checked} timed states, "
          f"{duration:.3f} s motion, peak {max_velocity:.6f} rad/s; runtime planner may choose another path")
    assert max_velocity > .59  # full configured speed, not a reduced-scale first run


def test_waypoint_transit_envelope(collision_model):
    # Sample the coordinate box of each candidate transit as an additional
    # offline check, not a claim about every path the runtime planner can take.
    # The standard Move to Waypoint owns planning and collision validation.
    rng = np.random.default_rng(314159)
    route = collection_route()
    checks = 0
    for start, goal in zip(route, route[1:]):
        low, high = np.minimum(start, goal), np.maximum(start, goal)
        fractions = [np.array([(bits >> j) & 1 for j in range(6)]) for bits in range(64)]
        fractions.extend(rng.uniform(0, 1, size=(256, 6)))
        for fraction in fractions:
            q = low + fraction * (high-low)
            result = check(collision_model, q)
            assert not result.collision, (start, goal, q, result.contacts)
            checks += 1
    print(f"Waypoint transit envelope: {checks} sampled configurations across {len(route)-1} moves")


def test_collection_is_plain_standard_waypoints():
    root = ET.parse(OBJECTIVES / "collect_rebot_gravity_data.xml")
    sequence = root.find(".//BehaviorTree/Control[@ID='Sequence']")
    assert sequence is not None
    assert set(node.tag for node in sequence) == {"Action", "SubTree"}
    actions = {node.get("ID") for node in sequence.findall("Action")}
    assert actions == {"RecordEpisode", "SaveEpisode", "StopRecording", "WaitForDuration", "LogMessage"}
    assert {node.get("ID") for node in sequence.findall("SubTree")} == {
        "reBot Fixed Bench Scene", "Move to Waypoint"
    }
    moves = sequence.findall("SubTree[@ID='Move to Waypoint']")
    assert [node.get("waypoint_name") for node in moves] == [
        "Raised", *[f"Gravity {i:02d}" for i in range(1, 34)], "Gravity Elbow Clearance",
        *[f"Gravity {i:02d}" for i in range(34, 49)], "Gravity Retreat", "Raised", "Zeroed Rest"
    ]
    for move in moves:
        # Both installed defaults are 1.0; no robot-specific scaled wrapper.
        assert move.get("velocity_scale_factor", "1.0") == "1.0"
        assert move.get("acceleration_scale_factor", "1.0") == "1.0"
        assert move.get("joint_group_name") == "manipulator"
    assert len(sequence.findall("Action[@delay_duration='{settle_seconds}']")) == 48
    assert len(sequence.findall("Action[@delay_duration='{sample_seconds}']")) == 48
    assert [node.get("message") for node in sequence.findall("Action[@ID='LogMessage']")] == [
        f"Gravity waypoint {i:02d}: static window begins" for i in range(1, 49)
    ]
    assert not root.findall(".//ScriptCondition")
    assert not root.findall(".//input_port[@name='approval']")
    for i in range(1, 49):
        waypoint = waypoints()[f"Gravity {i:02d}"]
        assert waypoint["joint_group_names"] == ["manipulator"]
        assert waypoint["joint_state"]["name"] == JOINTS[:6]
        assert len(waypoint["joint_state"]["position"]) == 6
