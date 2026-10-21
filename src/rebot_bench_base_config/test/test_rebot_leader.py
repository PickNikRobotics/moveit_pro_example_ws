# Copyright 2026 PickNik Inc.
# SPDX-License-Identifier: BSD-3-Clause
"""Leader mapping, deadman, position-command, and collision contracts."""
import math
import os
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from rebot_leader import (
    JOINT_NAMES,
    LEADER_NAMES,
    LEADER_RANGES,
    map_positions,
)  # noqa: E402
from rebot_leader_bridge import LeaderBridge  # noqa: E402
from rebot_leader_state import LeaderState  # noqa: E402


def state_at_home(now=10.0):
    state = LeaderState()
    state.observe(JOINT_NAMES, [0.0] * 7, now)
    assert state.accept({"positions": [0.0] * 7, "sample_time": now}, now)
    return state


def test_calibrated_mapping_order_sign_scale_and_limits():
    raw = dict(zip(LEADER_NAMES, [10, 20, -30, -40, -50, 60, 70]))
    actual = map_positions(raw, lambda angle, lower, upper: (angle, 0))
    assert actual == pytest.approx(
        [math.radians(q) for q in [10, 20, 30, 40, 50, 60, 310]]
    )


@pytest.mark.parametrize("bad", [math.nan, math.inf, -math.inf])
def test_nonfinite_leader_angle_rejected(bad):
    raw = dict(zip(LEADER_NAMES, [0.0] * 7))
    raw["elbow_flex"] = bad
    with pytest.raises(ValueError):
        map_positions(raw, lambda angle, lower, upper: (angle, 0))


def test_missing_joint_rejected():
    with pytest.raises(ValueError):
        map_positions({"shoulder_pan": 0}, lambda angle, lower, upper: (angle, 0))


def test_joint_state_order_is_resolved_by_name():
    state = LeaderState()
    positions = [0.1 * i for i in range(7)]
    state.observe(list(reversed(JOINT_NAMES)), list(reversed(positions)), 1.0)
    assert state.measured == positions


@pytest.mark.parametrize(
    "sample",
    [
        {"positions": [0.0] * 6, "sample_time": 10.01},
        {"positions": [math.nan] * 7, "sample_time": 10.01},
        {"positions": [0.0] * 7, "sample_time": 9.0},
        {"positions": [0.0] * 7, "sample_time": 11.0},
        {"positions": [0.0] * 7, "sample_time": 10.0},
        {"positions": [8.0] * 7, "sample_time": 10.01},
        {"error": "USB read failed", "sample_time": 10.01},
    ],
)
def test_invalid_frame_stops_and_latches_recovery(sample):
    state = state_at_home()
    assert state.engage(10.0)
    assert not state.accept(sample, 10.01)
    assert not state.active and state.recover and state.fault
    assert not state.engage(10.01)
    assert not state.reset()


def test_no_usb_auto_reengage_after_recovery():
    state = state_at_home()
    assert state.engage(10.0)
    state.stop("USB disconnected")
    state.recover = False
    assert state.accept({"positions": [0.0] * 7, "sample_time": 10.01}, 10.01)
    assert not state.engage(10.01)
    state.observe(JOINT_NAMES, [0.2] * 7, 10.01)
    assert not state.reset()
    state.observe(JOINT_NAMES, [0.0] * 7, 10.01)
    assert state.reset() and state.engage(10.01)


def test_stale_measured_state_stops_updates():
    state = state_at_home()
    assert state.engage(10.0)
    assert not state.check_deadman(10.101)
    assert state.recover


def test_objective_heartbeat_expiry_stops_even_with_fresh_samples():
    state = state_at_home()
    assert state.engage(10.0)
    state.observe(JOINT_NAMES, [0.0] * 7, 10.49)
    assert state.accept({"positions": [0.0] * 7, "sample_time": 10.49}, 10.49)
    assert not state.check_deadman(10.501)
    assert state.recover


def test_full_range_positions_are_forwarded_without_a_speed_reference():
    state = state_at_home()
    assert state.engage(10.0)
    target = [0.6, 1.0, 1.0, 1.0, -1.0, -1.0, 5.0]
    assert state.accept({"positions": target, "sample_time": 10.01}, 10.01)
    assert state.engage(10.01) and state.check_deadman(10.01)
    assert state.target == target


def test_alignment_fault_names_each_joint_beyond_tolerance():
    from rebot_leader_state import alignment_fault

    follower = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5]
    assert alignment_fault([0.1, -0.1, 0.0, 0.0, 0.0, 0.0, 0.6], follower) == ""
    reason = alignment_fault([0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.3], follower)
    assert "shoulder lift (joint2): +14.3 deg (+0.250 rad)" in reason
    assert "gripper (joint7): -11.5 deg (-0.200 rad)" in reason
    assert "within 5.7 deg (0.1 rad)" in reason
    assert "refused to avoid a jump" in reason
    assert (
        "shoulder lift (joint2) toward a lower calibrated angle by 14.3 deg (0.250 rad)"
        in reason
    )
    assert (
        "gripper (joint7) toward a higher calibrated angle by 1.9 deg (0.033 rad)"
        in reason
    )
    assert "run Mirror reBot Leader again" in reason


def test_misaligned_leader_refuses_to_engage_until_reset():
    state = state_at_home()
    assert state.accept(
        {"positions": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.2], "sample_time": 10.01}, 10.01
    )
    assert not state.engage(10.01)
    assert (
        not state.active and "gripper (joint7): +11.5 deg (+0.200 rad)" in state.fault
    )
    assert state.accept({"positions": [0.0] * 7, "sample_time": 10.02}, 10.02)
    assert not state.engage(10.02)
    assert state.reset() and state.engage(10.02)


def test_hold_cannot_command_negative_home_side_position(monkeypatch):
    bridge = LeaderBridge.__new__(LeaderBridge)
    bridge.thermal_tripped = False
    bridge.state = state_at_home()
    bridge.state.measured = [0.0, -0.01, -0.008, 0.0, 0.0, 0.0, -0.1]
    commands = []
    bridge.publish = commands.append
    monkeypatch.setattr("rebot_leader_bridge.time.monotonic", lambda: 10.01)
    bridge.hold()
    assert commands == [[0.0] * 7]
    monkeypatch.setattr("rebot_leader_bridge.time.monotonic", lambda: 10.2)
    bridge.hold()
    assert len(commands) == 1  # No position hold from stale feedback.


def test_engage_requires_collision_scene():
    bridge = LeaderBridge.__new__(LeaderBridge)
    bridge.thermal_tripped = False
    bridge.state = state_at_home()
    bridge.start_checked = True
    bridge.verified = True
    bridge.scene_ready = False
    bridge.home_failed = False
    bridge.reader_exited = False
    response = bridge.mirror(None, SimpleNamespace())
    assert not response.success and not bridge.state.active


def test_incomplete_follower_state_does_not_refresh_timestamp():
    state = state_at_home()
    state.observe(JOINT_NAMES[:-1], [0.0] * 6, 10.09)
    assert not state.fresh(10.101)


def test_collision_path_checks_intermediate_poses():
    from rebot_leader_collision import LeaderCollision

    checker = LeaderCollision.__new__(LeaderCollision)
    checker.bounds = [(-10.0, 10.0)] * 7
    poses = []

    def check(positions):
        poses.append(positions)
        return "blocked" if 0.009 < positions[0] < 0.011 else ""

    checker.check_pose = check
    assert checker.check_path([0.0] * 7, [0.02] * 7) == "blocked"
    assert poses[0] == [0.0] * 7
    assert poses[-1] == pytest.approx([0.01] * 7)


def test_collision_path_samples_the_jaw_at_its_own_spacing():
    from rebot_leader_collision import LeaderCollision

    checker = LeaderCollision.__new__(LeaderCollision)
    checker.bounds = [(-10.0, 10.0)] * 7
    closed, opened = [0.0] * 7, [0.0] * 6 + [5.41]
    poses = []
    checker.check_pose = lambda positions: poses.append(positions) or ""
    assert checker.check_path(closed, opened) == ""
    # 0.005 rad on the jaw too would take 1083 checks for this sweep.
    assert len(poses) < 1000
    assert poses[0] == closed and poses[-1] == pytest.approx(opened)
    assert max(b[6] - a[6] for a, b in zip(poses, poses[1:])) <= 0.05
    checker.check_pose = lambda p: "jaw blocked" if 2.0 < p[6] < 2.06 else ""
    assert checker.check_path(closed, opened) == "jaw blocked"
    checker.check_pose = lambda p: "arm blocked" if 0.009 < p[0] < 0.011 else ""
    assert checker.check_path(closed, [0.02] * 6 + [5.41]) == "arm blocked"


def test_collision_target_bounds_do_not_reject_measured_overshoot():
    from rebot_leader_collision import LeaderCollision

    checker = LeaderCollision.__new__(LeaderCollision)
    checker.bounds = [(0.0, 1.0)] * 7
    poses = []
    checker.check_pose = lambda positions: poses.append(positions) or ""
    assert checker.check_path([-0.01] * 7, [0.0] * 7) == ""
    assert poses[0] == [-0.01] * 7
    assert (
        checker.check_path([0.0] * 7, [-0.001] * 7)
        == "Target exceeds robot joint bounds"
    )


@pytest.fixture(scope="module")
def collision_checker():
    from ament_index_python.packages import get_package_share_directory
    import xacro
    from rebot_leader_collision import LeaderCollision

    share = Path(get_package_share_directory("rebot_bench_base_config"))
    urdf = xacro.process_file(
        str(share / "description/rebot.urdf.xacro"),
        mappings={"hardware_interface": "mock"},
    ).toxml()
    return LeaderCollision(urdf)


def test_native_fcl_checks_monitored_world_objects(collision_checker):
    from geometry_msgs.msg import Pose
    from moveit_msgs.msg import CollisionObject, PlanningScene
    from shape_msgs.msg import SolidPrimitive

    checker = collision_checker
    assert checker.check_path([0.0] * 7, [0.0] * 7) == ""
    scene = PlanningScene()
    scene.is_diff = True
    obj = CollisionObject()
    obj.id = "obstructed_bench"
    obj.header.frame_id = "base_link"
    obj.operation = CollisionObject.ADD
    obj.primitives = [
        SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[2.0, 2.0, 2.0])
    ]
    pose = Pose()
    pose.orientation.w = 1.0
    obj.primitive_poses = [pose]
    scene.world.collision_objects = [obj]
    checker.update_world(scene)
    assert "obstructed_bench" in checker.check_path([0.0] * 7, [0.0] * 7)
    scene.is_diff = False
    scene.world.collision_objects = []
    checker.update_world(scene)
    assert checker.check_path([0.0] * 7, [0.0] * 7) == ""


@pytest.mark.parametrize("end", [0, 1])
def test_leader_range_endpoints_pass_robot_joint_bounds(collision_checker, end):
    raw = {
        name: leader_range[end]
        for name, leader_range in zip(LEADER_NAMES, LEADER_RANGES)
    }
    target = map_positions(raw, lambda angle, lower, upper: (angle, 0))
    assert LeaderState().accept({"positions": target, "sample_time": 10.0}, 10.0)
    assert (
        collision_checker.check_path(target, target)
        != "Target exceeds robot joint bounds"
    )


def test_native_full_jaw_sweep_is_checked_in_fewer_than_a_thousand_poses(
    collision_checker, monkeypatch
):
    checks = []
    check_pose = collision_checker.check_pose
    monkeypatch.setattr(
        collision_checker, "check_pose", lambda p: checks.append(p) or check_pose(p)
    )
    assert collision_checker.check_path([0.0] * 7, [0.0] * 6 + [5.41]) == ""
    assert 100 < len(checks) < 1000


def test_octomap_refuses_following(collision_checker):
    from moveit_msgs.msg import PlanningScene

    scene = PlanningScene()
    scene.world.octomap.octomap.data = [1]
    with pytest.raises(ValueError, match="Octomap"):
        collision_checker.update_world(scene)


def test_folded_link_contact_is_the_allowed_pair(collision_checker):
    checker = collision_checker
    folded = [
        0.080285143927,
        0.0,
        0.017453292520,
        -0.043633231300,
        0.057595864484,
        -0.062831851407,
        0.0,
    ]
    assert checker.check_pose(folded) == ""
    checker.scene.allowed_collision_matrix.set_entry("link2", "link5", False)
    try:
        assert checker.check_pose(folded) == "Collision: link2 / link5"
    finally:
        checker.scene.allowed_collision_matrix.set_entry("link2", "link5", True)


@pytest.fixture
def home_bridge(monkeypatch):
    bridge = LeaderBridge.__new__(LeaderBridge)
    bridge.thermal_tripped = False
    bridge.state = state_at_home()
    bridge.home_failed = False
    bridge.reader_exited = False
    bridge.real_hardware = True
    bridge.verified = bridge.scene_ready = True
    bridge.start_checked = True
    bridge.hardware_state = SimpleNamespace(
        id=list(range(1, 8)), comm_state=0, fault_bits=[0] * 7, enabled=[True] * 7
    )
    bridge.hardware_time = 10.01
    bridge.publish = lambda positions: None
    bridge.node = SimpleNamespace(
        get_logger=lambda: SimpleNamespace(
            info=lambda text: None, warn=lambda text: None, error=lambda text: None
        )
    )
    monkeypatch.setattr("rebot_leader_bridge.time.monotonic", lambda: 10.01)
    return bridge


def test_objective_preparation_is_explicit_home_only(home_bridge):
    bridge = home_bridge
    bridge.state.measured[2] = 0.2
    assert not bridge.prepare(None, SimpleNamespace()).success
    bridge.state.measured[2] = 0.0
    assert bridge.prepare(None, SimpleNamespace()).success
    assert not bridge.start_checked and not bridge.state.active
    assert bridge.warmup_heartbeat == -math.inf


def test_sagged_home_is_home_for_preparation(home_bridge):
    home_bridge.state.measured = [0.0, -0.01, -0.008, 0.0, 0.0, 0.0, 0.0]
    assert home_bridge.prepare(None, SimpleNamespace()).success


@pytest.fixture
def unprepared_ticking_bridge(home_bridge):
    read, write = os.pipe()
    os.set_blocking(read, False)
    home_bridge.stream = SimpleNamespace(fileno=lambda: read)
    home_bridge.buffer = b""
    home_bridge.collider = SimpleNamespace(check_path=lambda measured, target: "")
    home_bridge.recovery_started = None
    home_bridge.start_checked = False
    home_bridge.prepared_at = None
    home_bridge.warmup_heartbeat = -math.inf
    home_bridge.warmup_reference = None
    yield home_bridge
    os.close(read)
    os.close(write)


@pytest.mark.parametrize("position", [0.0, 0.2])
def test_mirror_without_preparation_cannot_start(unprepared_ticking_bridge, position):
    bridge = unprepared_ticking_bridge
    bridge.state.measured[2] = position
    assert not bridge.mirror(None, SimpleNamespace()).success
    assert bridge.warmup_heartbeat == -math.inf
    bridge.tick()
    assert not bridge.state.active and not bridge.start_checked
    # The timer must enforce preparation independently of the service gate.
    bridge.warmup_heartbeat = 10.01
    bridge.tick()
    assert not bridge.state.active and not bridge.start_checked


@pytest.fixture
def ticking_bridge(unprepared_ticking_bridge):
    assert unprepared_ticking_bridge.prepare(None, SimpleNamespace()).success
    return unprepared_ticking_bridge


def test_prepared_session_follows_once_mirror_starts(ticking_bridge):
    assert ticking_bridge.mirror(None, SimpleNamespace()).success
    ticking_bridge.tick()
    assert ticking_bridge.state.active and not ticking_bridge.state.recover
    assert ticking_bridge.prepared_at is None


def test_misaligned_leader_never_follows_and_reports_the_joint(ticking_bridge):
    bridge = ticking_bridge
    bridge.state.target[1] = 0.3
    assert bridge.mirror(None, SimpleNamespace()).success
    bridge.tick()
    assert not bridge.state.active
    response = bridge.mirror(None, SimpleNamespace())
    assert (
        not response.success
        and "shoulder lift (joint2): +17.2 deg (+0.300 rad)" in response.message
    )
    bridge.tick()
    assert bridge.state.recover and not bridge.state.active


@pytest.mark.parametrize("rate", [0, 19, 101, math.nan, math.inf])
def test_bridge_refuses_publish_rate_outside_20_to_100_hz(monkeypatch, rate):
    from unittest.mock import Mock
    import rebot_leader_bridge

    monkeypatch.setattr(rebot_leader_bridge, "ActionClient", Mock())
    node = Mock()
    node.declare_parameter.return_value = SimpleNamespace(value=rate)
    read, write = os.pipe()
    try:
        with pytest.raises(ValueError, match="between 20 and 100"):
            LeaderBridge(node, SimpleNamespace(fileno=lambda: read))
    finally:
        os.close(read)
        os.close(write)
    node.create_timer.assert_not_called()


def test_prepared_session_that_never_follows_returns_home(ticking_bridge, monkeypatch):
    monkeypatch.setattr("rebot_leader_bridge.time.monotonic", lambda: 12.9)
    ticking_bridge.tick()
    assert not ticking_bridge.state.recover
    monkeypatch.setattr("rebot_leader_bridge.time.monotonic", lambda: 13.02)
    ticking_bridge.tick()
    assert ticking_bridge.state.recover and ticking_bridge.state.fault
    assert ticking_bridge.prepared_at is None
    assert not ticking_bridge.mirror(None, SimpleNamespace()).success


def test_clutch_before_following_returns_home(ticking_bridge):
    ticking_bridge.clutch(None, SimpleNamespace())
    ticking_bridge.tick()
    assert ticking_bridge.state.recover and not ticking_bridge.state.active


def test_objective_preparation_refuses_disabled_or_faulted_hardware(home_bridge):
    bridge = home_bridge
    bridge.hardware_state.enabled[2] = False
    assert not bridge.prepare(None, SimpleNamespace()).success
    bridge.hardware_state.enabled[2] = True
    bridge.hardware_state.fault_bits[2] = 1
    assert not bridge.prepare(None, SimpleNamespace()).success
    bridge.hardware_state.fault_bits[2] = 0
    for enabled in ([], [True] * 6, [True] * 8):
        bridge.hardware_state.enabled = enabled
        assert not bridge.prepare(None, SimpleNamespace()).success
    bridge.hardware_state.enabled = [True] * 7
    assert bridge.prepare(None, SimpleNamespace()).success


def _stale(bridge):
    bridge.hardware_time = 9.5


def _comm_lost(bridge):
    bridge.hardware_state.comm_state = 1


def _faulted(bridge):
    bridge.hardware_state.fault_bits[3] = 1


def _disabled(bridge):
    bridge.hardware_state.enabled[4] = False


def _malformed_enabled(bridge):
    bridge.hardware_state.enabled = [True] * 6


@pytest.mark.parametrize(
    "fail", [_stale, _comm_lost, _faulted, _disabled, _malformed_enabled]
)
def test_rejected_preparation_preserves_fault_latch(home_bridge, fail):
    bridge = home_bridge
    # HOME completed after a prior session, but the stop remains latched.
    bridge.state.fault = "Clutched"
    fail(bridge)
    assert not bridge.prepare(None, SimpleNamespace()).success
    assert bridge.state.fault == "Clutched"

    bridge.hardware_state.comm_state = 0
    bridge.hardware_state.fault_bits = [0] * 7
    bridge.hardware_state.enabled = [True] * 7
    bridge.hardware_time = 10.01
    assert not bridge.mirror(None, SimpleNamespace()).success
    assert not bridge.state.active

    # Only a successful preparation may clear the latch and start warm-up.
    assert bridge.prepare(None, SimpleNamespace()).success
    assert not bridge.state.fault and not bridge.start_checked
    assert bridge.mirror(None, SimpleNamespace()).success
    assert not bridge.state.active


@pytest.fixture
def following_bridge(ticking_bridge):
    bridge = ticking_bridge
    assert bridge.mirror(None, SimpleNamespace()).success
    bridge.tick()
    assert bridge.state.active
    published = []
    bridge.publish = published.append
    target = [0.1] * 6 + [1.0]
    assert bridge.state.accept({"positions": target, "sample_time": 10.01}, 10.01)
    return bridge, published, target


def test_healthy_hardware_keeps_following(following_bridge):
    bridge, published, target = following_bridge
    bridge.tick()
    assert bridge.state.active and published == [target]


@pytest.mark.parametrize(
    "fail", [_stale, _comm_lost, _faulted, _disabled, _malformed_enabled]
)
def test_unhealthy_hardware_stops_following_and_returns_home(following_bridge, fail):
    bridge, published, target = following_bridge
    fail(bridge)
    bridge.tick()
    assert not bridge.state.active and bridge.state.recover and bridge.state.fault
    assert bridge.recovery_started is not None
    # Only the measured-position hold goes out, never the leader target.
    assert published == [[0.0] * 7]
    bridge.hardware_state.comm_state = 0
    bridge.hardware_state.fault_bits = [0] * 7
    bridge.hardware_state.enabled = [True] * 7
    bridge.hardware_time = 10.01
    assert not bridge.mirror(None, SimpleNamespace()).success
    bridge.tick()
    assert not bridge.state.active and target not in published


def test_simulated_sessions_do_not_require_hardware_status(following_bridge):
    bridge, published, target = following_bridge
    bridge.real_hardware = False
    bridge.hardware_state = None
    bridge.tick()
    assert bridge.state.active and published == [target]


def test_repeated_stop_signals_do_not_abort_the_home_return(monkeypatch):
    import signal
    import rebot_leader_bridge

    bridge = SimpleNamespace(
        state=SimpleNamespace(recover=True, stop=lambda reason: None),
        home_failed=False,
        hold=lambda: None,
    )
    spins = []

    def spin_once(node, timeout_sec):
        spins.append(timeout_sec)
        if len(spins) == 1:
            # A non-interactive launch forwards SIGINT, then escalates to SIGTERM.
            signal.raise_signal(signal.SIGINT)
            signal.raise_signal(signal.SIGTERM)
        else:
            bridge.state.recover = False

    monkeypatch.setattr(
        sys,
        "argv",
        [
            "rebot_leader_bridge.py",
            "--port",
            "/no-usb",
            "--calibration-file",
            "/existing.json",
        ],
    )
    child = SimpleNamespace(
        stdout=SimpleNamespace(close=lambda: None),
        terminate=lambda: None,
        wait=lambda **kwargs: None,
    )
    monkeypatch.setattr(
        rebot_leader_bridge.subprocess, "Popen", lambda *args, **kwargs: child
    )
    monkeypatch.setattr(sys, "stdin", SimpleNamespace(buffer=None))
    monkeypatch.setattr(
        rebot_leader_bridge, "LeaderBridge", lambda node, stream: bridge
    )
    monkeypatch.setattr(rebot_leader_bridge.rclpy, "init", lambda **kwargs: None)
    monkeypatch.setattr(
        rebot_leader_bridge.rclpy,
        "create_node",
        lambda name: SimpleNamespace(destroy_node=lambda: None),
    )
    monkeypatch.setattr(
        rebot_leader_bridge.rclpy,
        "spin",
        lambda node: signal.raise_signal(signal.SIGINT),
    )
    monkeypatch.setattr(rebot_leader_bridge.rclpy, "spin_once", spin_once)
    monkeypatch.setattr(rebot_leader_bridge.rclpy, "shutdown", lambda: None)
    handlers = {
        number: signal.getsignal(number) for number in (signal.SIGINT, signal.SIGTERM)
    }
    try:
        rebot_leader_bridge.main()
    except KeyboardInterrupt:
        pytest.fail("A repeated stop signal aborted the HOME return")
    finally:
        for number, handler in handlers.items():
            signal.signal(number, handler)
    assert len(spins) == 2 and not bridge.state.recover


def test_launch_does_not_kill_the_bridge_during_its_home_return():
    import importlib.util
    from launch import LaunchContext
    from launch.utilities import perform_substitutions

    path = Path(__file__).resolve().parents[1] / "launch" / "rebot_leader.launch.py"
    spec = importlib.util.spec_from_file_location("rebot_leader_launch", path)
    launch_file = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(launch_file)
    context = LaunchContext()
    config = SimpleNamespace(
        ros_global_params={
            "rebot_leader_enabled": True,
            "rebot_leader_port": "/no-usb",
            "rebot_leader_calibration_file": "/existing.json",
        },
        hardware=SimpleNamespace(
            robot_description=SimpleNamespace(
                urdf_params=[{"hardware_interface": "mock"}]
            )
        ),
    )
    (node,) = launch_file.configured_nodes(config)
    sigkill_after = sum(
        float(perform_substitutions(context, timeout))
        for timeout in (node.sigterm_timeout, node.sigkill_timeout)
    )
    # The bridge waits up to 120 s for HOME, then 5 s for its reader.
    assert sigkill_after > 125


def test_successful_home_requires_explicit_reengagement(home_bridge):
    bridge = home_bridge
    bridge.state.recover = True
    bridge.state.fault = "Clutched"
    bridge.recovery_started = 9.0
    bridge.recovery_future = SimpleNamespace(done=lambda: True)
    bridge.recovery_result = SimpleNamespace(
        done=lambda: True,
        result=lambda: SimpleNamespace(
            status=4, result=SimpleNamespace(error_code=SimpleNamespace(val=1))
        ),
    )
    bridge.recover_home(10.01)
    assert not bridge.state.recover
    assert not bridge.state.active
    assert bridge.state.fault == "Clutched"


def test_warmup_reference_does_not_follow_feedback_drift(monkeypatch):
    bridge = LeaderBridge.__new__(LeaderBridge)
    bridge.thermal_tripped = False
    bridge.state = state_at_home()
    bridge.warmup_reference = None
    commands = []
    bridge.publish = commands.append
    monkeypatch.setattr("rebot_leader_bridge.time.monotonic", lambda: 10.01)
    bridge.hold(fixed=True)
    bridge.state.measured = [0.03, -0.002, -0.001, 0.01, 0.02, -0.01, 0.01]
    bridge.hold(fixed=True)
    assert commands == [[0.0] * 7, [0.0] * 7]
    # Stop/clutch still snapshots the current measured position once.
    bridge.hold()
    assert commands[-1] == [0.03, 0.0, 0.0, 0.01, 0.02, -0.01, 0.01]


def test_explicit_motor_tuning_rejects_invalid_and_incomplete_settings(tmp_path):
    from rebot_leader_tune import settings

    path = tmp_path / "tuning.yaml"
    path.write_text("jaw_torque_limit_nm: 1\njoint4_speed_limit_rad_s: 33\n")
    assert settings(path) == [(7, "0x700B", 1), (4, "0x7017", 33)]
    for bad in (".nan", "0", "15", "true"):
        path.write_text(f"jaw_torque_limit_nm: {bad}\njoint4_speed_limit_rad_s: 33\n")
        with pytest.raises(ValueError):
            settings(path)
    path.write_text("jaw_kp: 12\n")
    with pytest.raises(ValueError):
        settings(path)


def test_motor_tuning_requires_fresh_healthy_home_with_all_seven_off():
    from rebot_leader_tune import safe_home

    health = SimpleNamespace(
        id=list(range(1, 8)), enabled=[False] * 7, fault_bits=[0] * 7, comm_state=0
    )
    joint = SimpleNamespace(name=JOINT_NAMES, position=[0.0] * 7)
    assert safe_home((10.0, health), (10.0, joint), 10.0)
    assert not safe_home((10.0, health), (10.0, joint), 10.2)
    for i in range(7):
        health.enabled[i] = True
        assert not safe_home((10.0, health), (10.0, joint), 10.0)
        health.enabled[i] = False
    joint.position[3] = 0.081
    assert not safe_home((10.0, health), (10.0, joint), 10.0)
    joint.position[3] = math.nan
    assert not safe_home((10.0, health), (10.0, joint), 10.0)
    joint.position[3] = 0
    health.fault_bits[2] = 1
    assert not safe_home((10.0, health), (10.0, joint), 10.0)


def test_driver_thermal_trip_yields_to_one_backend_recovery_owner(following_bridge):
    bridge, published, target = following_bridge
    bridge.observe_thermal(SimpleNamespace(data=True))
    bridge.tick()
    assert bridge.thermal_tripped and not bridge.state.active
    assert not bridge.state.recover and published == []
    assert not bridge.prepare(None, SimpleNamespace()).success
    bridge.observe_thermal(SimpleNamespace(data=False))
    assert bridge.thermal_tripped


def calibration_fixture():
    return {
        name: {
            "id": i,
            "drive_mode": 0,
            "homing_offset": 0,
            "range_min": -90,
            "range_max": 90,
        }
        for i, name in enumerate(LEADER_NAMES)
    }


@pytest.mark.parametrize(
    "field,value",
    [
        ("id", 253),
        ("homing_offset", 1),
        ("drive_mode", 1),
        ("range_min", float("nan")),
        ("range_max", -90),
    ],
)
def test_unsupported_calibration_refuses_before_opening_serial(tmp_path, field, value):
    import json
    from rebot_leader import load_calibration

    data = calibration_fixture()
    data[LEADER_NAMES[0]][field] = value
    path = tmp_path / "calibration.json"
    path.write_text(json.dumps(data))
    with pytest.raises(ValueError):
        load_calibration(path)


def test_direct_sdk_mapping_keeps_full_ranges_not_legacy_placeholder():
    from rebot_leader import read_positions

    raw = [120, 150, -180, -80, -90, 130, 30]
    bus = SimpleNamespace(
        read_angle=lambda motor_id: SimpleNamespace(raw_deg=raw[motor_id])
    )
    actual = read_positions(bus, calibration_fixture())
    assert actual == pytest.approx(
        [math.radians(q) for q in [120, 150, 180, 80, 90, 130, 180]]
    )


@pytest.mark.parametrize(
    "raw,expected", [(370, 10), (-350, 10), (730, 10), (180, 180), (-180, -180)]
)
def test_nearest_turn_unwrap_matches_seeed_window(raw, expected):
    from rebot_leader import unwrap_angle

    assert unwrap_angle(raw, -150, 150)[0] == expected


def test_reader_failure_and_shutdown_always_close_bus():
    import io
    import json
    from unittest.mock import Mock
    from rebot_leader import run_reader

    bus = SimpleNamespace(
        ping=lambda motor_id: True,
        unlock=Mock(),
        read_angle=Mock(side_effect=RuntimeError("USB missing")),
        close=Mock(),
    )
    output = io.StringIO()
    with pytest.raises(KeyboardInterrupt):
        run_reader(
            "/no-usb",
            calibration_fixture(),
            lambda *a, **kw: bus,
            output,
            sleep=Mock(side_effect=KeyboardInterrupt),
        )
    message = json.loads(output.getvalue())
    assert message["error"] == "USB missing" and "positions" not in message
    assert bus.unlock.call_count == 7
    bus.close.assert_called_once()
    bus.close.reset_mock()
    bus.ping = lambda motor_id: False
    with pytest.raises(RuntimeError, match="not found"):
        run_reader("/no-usb", calibration_fixture(), lambda *a, **kw: bus, output)
    bus.close.assert_called_once()


def test_bridge_initialization_failure_reaps_its_reader(monkeypatch):
    import signal
    from unittest.mock import Mock
    import rebot_leader_bridge

    child = SimpleNamespace(
        stdout=SimpleNamespace(close=Mock()), terminate=Mock(), wait=Mock()
    )
    node = SimpleNamespace(destroy_node=Mock())
    shutdown = Mock()
    monkeypatch.setattr(
        sys,
        "argv",
        [
            "rebot_leader_bridge.py",
            "--port",
            "/no-usb",
            "--calibration-file",
            "/existing.json",
        ],
    )
    monkeypatch.setattr(rebot_leader_bridge.rclpy, "init", lambda **kw: None)
    monkeypatch.setattr(rebot_leader_bridge.rclpy, "create_node", lambda name: node)
    monkeypatch.setattr(rebot_leader_bridge.rclpy, "shutdown", shutdown)
    monkeypatch.setattr(rebot_leader_bridge.subprocess, "Popen", lambda *a, **kw: child)
    monkeypatch.setattr(
        rebot_leader_bridge,
        "LeaderBridge",
        Mock(side_effect=RuntimeError("initialization failed")),
    )
    handlers = {
        number: signal.getsignal(number) for number in (signal.SIGINT, signal.SIGTERM)
    }
    try:
        with pytest.raises(RuntimeError, match="initialization failed"):
            rebot_leader_bridge.main()
    finally:
        for number, handler in handlers.items():
            signal.signal(number, handler)
    child.terminate.assert_called_once()
    child.wait.assert_called_once_with(timeout=5)
    child.stdout.close.assert_called_once()
    node.destroy_node.assert_called_once()
    shutdown.assert_called_once()


@pytest.mark.parametrize(
    "index,delta,direction",
    [(5, -0.12, "higher"), (5, 0.12, "lower"), (2, 0.2, "higher"), (2, -0.2, "lower")],
)
def test_alignment_instructions_account_for_leader_mapping_sign(
    index, delta, direction
):
    from rebot_leader_state import alignment_fault, joint_label

    target = [0.0] * 7
    target[index] = delta
    message = alignment_fault(target, [0.0] * 7)
    assert f"{joint_label(index)} toward a {direction} calibrated angle" in message
    assert "Star Arm's pose does not match reBot" in message
    assert "deg (" in message and "rad)" in message


def test_home_refusal_names_joint_and_gives_supported_recovery(home_bridge):
    home_bridge.state.measured[5] = -0.2
    message = home_bridge.prepare(None, SimpleNamespace()).message
    assert "wrist roll (joint6) -11.5 deg (-0.200 rad)" in message
    assert "reBot must be at HOME" in message
    assert "Run Return reBot Leader Home" in message
    assert "do not move it by hand or disable torque away from HOME" in message
    assert "run Mirror reBot Leader again" in message


def test_stale_refusal_explains_connection_and_retry(home_bridge):
    home_bridge.state.sample_time = 9.0
    response = home_bridge.prepare(None, SimpleNamespace())
    assert not response.success
    assert "fresh Star Arm or reBot joint data is missing" in response.message
    assert "Check the Star Arm USB connection" in response.message
    assert "once reBot is at HOME, restart the backend" in response.message
    assert "run Mirror reBot Leader again" in response.message


def test_reader_read_error_reaches_refusal_with_backend_restart(
    following_bridge, monkeypatch
):
    import io
    from unittest.mock import Mock
    from rebot_leader import run_reader

    bridge, published, target = following_bridge
    bus = SimpleNamespace(
        ping=lambda motor_id: True,
        unlock=Mock(),
        read_angle=Mock(side_effect=RuntimeError("servo 3 read timeout")),
        close=Mock(),
    )
    output = io.StringIO()
    with pytest.raises(KeyboardInterrupt):
        run_reader(
            "/no-usb",
            calibration_fixture(),
            lambda *a, **kw: bus,
            output,
            clock=lambda: 10.01,
            sleep=Mock(side_effect=KeyboardInterrupt),
        )
    read, write = os.pipe()
    os.write(write, output.getvalue().encode())
    bridge.stream = SimpleNamespace(fileno=lambda: read)
    try:
        bridge.tick()
    finally:
        os.close(read)
        os.close(write)
    assert not bridge.state.active and bridge.state.recover
    following = bridge.mirror(None, SimpleNamespace())
    monkeypatch.setattr("rebot_leader_bridge.time.monotonic", lambda: 10.5)
    retry = bridge.prepare(None, SimpleNamespace())
    for response in (following, retry):
        assert not response.success
        assert "Star Arm USB read failed: servo 3 read timeout" in response.message
        assert "once reBot is at HOME, restart the backend" in response.message


def test_exited_reader_refusal_names_cause_and_backend_restart(
    unprepared_ticking_bridge, monkeypatch
):
    bridge = unprepared_ticking_bridge
    errors = []
    bridge.node = SimpleNamespace(
        get_logger=lambda: SimpleNamespace(
            info=lambda text: None, warn=lambda text: None, error=errors.append
        )
    )
    read, write = os.pipe()
    os.close(write)  # The reader process exited.
    bridge.stream = SimpleNamespace(fileno=lambda: read)
    try:
        bridge.tick()
        monkeypatch.setattr("rebot_leader_bridge.time.monotonic", lambda: 10.5)
        bridge.tick()
    finally:
        os.close(read)
    assert len(errors) == 1
    for call in (bridge.prepare, bridge.mirror):
        response = call(None, SimpleNamespace())
        assert not response.success
        assert "Star Arm reader has exited" in response.message
        assert "rebot_leader_port" in response.message
        assert "Once reBot is at HOME, restart the backend" in response.message
        assert "run Mirror reBot Leader again" in response.message


def test_thermal_refusal_explains_latch_cooling_and_home_only_off(home_bridge):
    home_bridge.thermal_tripped = True
    for call in (home_bridge.prepare, home_bridge.mirror):
        response = call(None, SimpleNamespace())
        assert not response.success
        assert "temperature limit" in response.message and "latched" in response.message
        assert "turns torque off only at HOME" in response.message
        assert "torque stays on" in response.message
        assert (
            "Wait for cooling and confirmed HOME with all seven motors OFF"
            in response.message
        )
        assert "restart the backend" in response.message
        assert "only a restart clears the latch" in response.message
        assert "run Mirror reBot Leader again" in response.message


def test_three_second_timeout_message_explains_recovery_and_retry(
    ticking_bridge, monkeypatch
):
    monkeypatch.setattr("rebot_leader_bridge.time.monotonic", lambda: 13.02)
    ticking_bridge.tick()
    response = ticking_bridge.mirror(None, SimpleNamespace())
    assert not response.success
    assert "did not start within 3 seconds" in response.message
    assert "wait for recovery to finish" in response.message
    assert "Check the Star Arm USB connection" in response.message
    assert "once reBot is at HOME, restart the backend" in response.message
    assert "run Mirror reBot Leader again" in response.message
