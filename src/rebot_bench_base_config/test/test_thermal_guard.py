# Copyright 2026 PickNik Inc.
# SPDX-License-Identifier: BSD-3-Clause
"""Thermal response only cuts torque after native success and fresh quiet HOME."""
import math
from pathlib import Path
import sys
from types import SimpleNamespace as Obj
from unittest.mock import Mock
import xml.etree.ElementTree as ET

import pytest
from rclpy.time import Time

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from rebot_thermal_guard import ThermalGuard  # noqa: E402


def future(result):
    return Obj(done=lambda: True, result=lambda: result)


@pytest.fixture
def guard():
    result = ThermalGuard.__new__(ThermalGuard)
    result.node = Obj(
        get_logger=lambda: Obj(error=Mock(), info=Mock()),
        get_clock=lambda: Obj(now=lambda: Time(nanoseconds=10_000_000_000)),
    )
    result.stage, result.started = "idle", None
    result.future = result.home_handle = result.home_since = None
    result.next_clear = -float("inf")
    result.gain_commands = tuple(
        (i, gain, float(i + (10 if gain == "kp" else 0)))
        for gain in ("kd", "kp")
        for i in range(1, 7)
    )
    result.gain_index = 0
    result.gain_started = None
    result.anchor_positions = None
    result.anchor_sent_ns = -1
    result.reference_positions = None
    result.reference_stamp_ns = -1
    result.reference_time = -float("inf")
    result.reference_samples = 0
    result.anchor_handle = None
    result.anchor_result = None
    anchor_result = Obj(done=lambda: False)
    result.anchor = Obj(
        server_is_ready=lambda: True,
        send_goal_async=Mock(
            return_value=future(
                Obj(
                    accepted=True,
                    status=2,
                    get_result_async=lambda: anchor_result,
                )
            )
        ),
    )
    result.gains = Obj(
        service_is_ready=lambda: True,
        call_async=Mock(return_value=future(Obj(result=True))),
    )
    result.clear = Obj(publish=Mock())
    result.hardware = Obj(
        id=list(range(1, 8)), enabled=[True] * 7, fault_bits=[0] * 7, comm_state=0
    )
    result.hardware_time = result.measured_time = 10.0
    result.measured = [(0.0, 0.0)] * 7
    result.cancel = Obj(
        service_is_ready=lambda: True,
        call_async=Mock(return_value=future(Obj(return_code=0))),
    )
    home_result = future(Obj(status=4, result=Obj(error_code=Obj(val=1))))
    result.home = Obj(
        server_is_ready=lambda: True,
        send_goal_async=Mock(
            return_value=future(
                Obj(accepted=True, get_result_async=lambda: home_result)
            )
        ),
    )
    result.torque = Obj(
        service_is_ready=lambda: True,
        call_async=Mock(return_value=future(Obj(success=True))),
    )
    return result


def test_trip_latches_even_after_cooling_and_duplicate_notifications(
    guard, monkeypatch
):
    monkeypatch.setattr("rebot_thermal_guard.time.monotonic", lambda: 10.0)
    guard.trip(Obj(data=True))
    assert guard.stage == "anchor"
    guard.trip(Obj(data=False))
    guard.trip(Obj(data=True))
    assert guard.stage == "anchor" and guard.started == 10.0
    assert not guard.torque.call_async.called


def test_cancel_home_verify_disable_confirm_are_sequential(guard):
    guard.stage, guard.started = "cancel", 9.9
    guard.advance(10.0)  # cancel requested
    assert guard.cancel.call_async.called and not guard.home.send_goal_async.called
    guard.advance(10.0)  # cancel acknowledged
    guard.started = 9.0
    guard.advance(10.0)  # HOME requested
    assert (
        guard.home.send_goal_async.call_args.args[0].objective_name
        == "Return reBot Leader Home"
    )
    guard.advance(10.0)  # goal accepted
    guard.advance(10.0)  # HOME result succeeded
    assert guard.stage == "verify" and not guard.torque.call_async.called
    guard.advance(10.0)  # dwell begins
    guard.hardware_time = guard.measured_time = 10.6
    guard.advance(10.6)  # fresh stationary HOME for 0.5s
    assert guard.torque.call_async.call_args.args[0].data is False
    guard.advance(10.6)  # OFF accepted
    assert guard.stage == "confirm"
    guard.hardware.enabled = [False] * 7
    guard.advance(10.6)
    assert guard.stage == "cooling"
    assert not guard.clear.publish.called


@pytest.mark.parametrize(
    "bad", ["off_home", "moving", "stale_joint", "stale_motor", "fault", "malformed"]
)
def test_no_disable_on_unverified_home(guard, bad):
    guard.stage, guard.started, guard.home_since = "verify", 9.0, 8.0
    if bad == "off_home":
        guard.measured[2] = (0.081, 0.0)
    elif bad == "moving":
        guard.measured[2] = (0.0, 0.011)
    elif bad == "stale_joint":
        guard.measured_time = 9.8
    elif bad == "stale_motor":
        guard.hardware_time = 9.4
    elif bad == "fault":
        guard.hardware.fault_bits[2] = 1
    else:
        guard.hardware.enabled = []
    guard.advance(10.0)
    assert not guard.torque.call_async.called and guard.home_since is None


def test_home_failure_latches_without_torque_or_lifecycle_command(guard):
    guard.stage, guard.started = "result", 9.0
    guard.future = future(Obj(status=6, result=Obj(error_code=Obj(val=-1))))
    guard.advance(10.0)
    assert guard.stage == "failed" and not guard.torque.call_async.called
    guard.tick()
    assert guard.stage == "failed"
    guard.hardware.enabled = [False] * 7
    guard.trip(Obj(data=False))
    guard.tick()
    assert guard.stage == "failed" and not guard.clear.publish.called


def test_hot_initial_probe_with_all_motors_off_never_enables_or_moves(
    guard, monkeypatch
):
    # The trip latch arrives at hardware activation, before the agent is up.
    logger = Obj(error=Mock())
    guard.node = Obj(get_logger=lambda: logger)
    guard.cancel = Obj(service_is_ready=lambda: False, call_async=Mock())
    guard.hardware.enabled = [False] * 7
    monkeypatch.setattr("rebot_thermal_guard.time.monotonic", lambda: 10.0)
    guard.trip(Obj(data=True))
    guard.hardware_time = 13.5
    monkeypatch.setattr("rebot_thermal_guard.time.monotonic", lambda: 13.5)
    guard.tick()
    assert guard.stage == "off"
    assert not any(
        "intervention required" in call.args[0] for call in logger.error.call_args_list
    )
    assert not guard.cancel.call_async.called
    assert not guard.home.send_goal_async.called and not guard.torque.call_async.called
    assert not guard.anchor.send_goal_async.called and not guard.gains.call_async.called


def test_native_home_keeps_its_full_budget_after_slow_cancel(guard, monkeypatch):
    clock = [10.0]
    monkeypatch.setattr("rebot_thermal_guard.time.monotonic", lambda: clock[0])
    done = {"cancel": False, "home": False}
    guard.cancel.call_async = Mock(
        return_value=Obj(done=lambda: done["cancel"], result=lambda: Obj(return_code=0))
    )
    home_result = Obj(
        done=lambda: done["home"],
        result=lambda: Obj(status=4, result=Obj(error_code=Obj(val=1))),
    )
    guard.home.send_goal_async = Mock(
        return_value=future(Obj(accepted=True, get_result_async=lambda: home_result))
    )

    def tick_at(now):
        clock[0] = guard.hardware_time = guard.measured_time = now
        guard.tick()

    guard.stage, guard.started = "cancel", 10.0
    tick_at(10.0)  # cancel requested
    done["cancel"] = True
    tick_at(12.9)  # acknowledged just inside the cancel bound
    tick_at(13.5)  # HOME requested
    tick_at(13.5)  # accepted
    tick_at(13.5 + 119.0)  # a slow native HOME is still within its own budget
    assert guard.stage == "result"
    done["home"] = True
    tick_at(13.5 + 119.5)
    assert guard.stage == "verify"


@pytest.mark.parametrize(
    "stage,limit",
    [
        ("anchor", 3.0),
        ("anchor_reference", 1.0),
        ("restore", 7.0),
        ("cancel", 3.0),
        ("home", 3.0),
        ("result", 120.0),
        ("verify", 5.0),
        ("disable", 3.0),
        ("confirm", 3.0),
    ],
)
def test_every_recovery_stage_is_bounded_and_keeps_torque(guard, stage, limit):
    guard.stage, guard.started = stage, 10.0
    guard.future = Obj(done=lambda: False)
    guard.home = Obj(server_is_ready=lambda: False, send_goal_async=Mock())
    guard.anchor_positions = [q for q, _ in guard.measured]
    guard.reference_positions = list(guard.anchor_positions)
    guard.reference_stamp_ns = guard.anchor_sent_ns = 10_000_000_000
    guard.gain_started = 10.0 + limit
    guard.measured[0] = (0.2, 0.0)  # verify never sees HOME

    def advance_at(now):
        guard.hardware_time = guard.measured_time = guard.reference_time = now
        # No matching reference in anchor_reference; restore gets a current hold.
        guard.reference_samples = 2 if stage == "restore" else 0
        guard.advance(now)

    advance_at(10.0 + limit - 0.01)
    assert guard.stage == stage
    advance_at(10.0 + limit + 0.01)
    assert guard.stage == "failed"
    assert not guard.torque.call_async.called
    assert not guard.home.send_goal_async.called


@pytest.mark.parametrize(
    "bad", ["off_home", "moving", "stale_joint", "stale_motor", "fault", "one_on"]
)
def test_cooling_requires_fresh_stationary_home_and_all_seven_off(guard, bad):
    guard.stage = "cooling"
    guard.hardware.enabled = [False] * 7
    if bad == "off_home":
        guard.measured[2] = (0.081, 0.0)
    elif bad == "moving":
        guard.measured[6] = (0.0, 0.011)
    elif bad == "stale_joint":
        guard.measured_time = 9.8
    elif bad == "stale_motor":
        guard.hardware_time = 9.4
    elif bad == "fault":
        guard.hardware.comm_state = 1
    else:
        guard.hardware.enabled[6] = True
    guard.advance(10.0)
    assert not guard.clear.publish.called
    assert not guard.torque.call_async.called


@pytest.mark.parametrize("motor", range(7))
def test_cooling_ignores_history_but_requires_current_health(guard, motor):
    guard.stage = "cooling"
    guard.hardware.enabled = [False] * 7
    guard.hardware.fault_bits[motor] = 1
    guard.hardware.comm_state = 1
    guard.advance(10.0)
    assert not guard.clear.publish.called
    guard.hardware.comm_state = 0  # healthy Type-2 does not erase fault history
    guard.advance(10.0)
    assert guard.hardware.fault_bits[motor] == 1
    assert guard.clear.publish.call_args.args[0].data == 10_000_000_000
    assert guard.stage == "cooling"
    assert not guard.torque.call_async.called
    assert not guard.home.send_goal_async.called


@pytest.mark.parametrize("stage", ["cancel", "home", "verify", "confirm"])
def test_recovery_before_cooling_still_rejects_historical_faults(guard, stage):
    guard.stage, guard.started, guard.home_since = stage, 9.0, 9.0
    guard.hardware.fault_bits[6] = 1
    guard.hardware.enabled = [stage in ("home", "verify")] * 7
    assert not guard.hardware_fresh(10.0)
    guard.advance(10.0)
    assert guard.stage != "cooling" and guard.stage != "off"
    assert not guard.clear.publish.called
    assert not guard.home.send_goal_async.called
    assert not guard.torque.call_async.called


def test_cooling_requests_clear_without_enabling_and_waits_for_driver_false(guard):
    guard.stage = "cooling"
    guard.hardware.enabled = [False] * 7
    guard.advance(10.0)
    assert guard.clear.publish.call_count == 1 and guard.stage == "cooling"
    assert guard.clear.publish.call_args.args[0].data == 10_000_000_000
    guard.advance(10.01)
    assert guard.stage == "cooling"
    guard.advance(10.02)
    assert guard.clear.publish.call_count == 1
    assert not guard.torque.call_async.called
    guard.trip(Obj(data=True))
    assert guard.stage == "cooling"
    guard.trip(Obj(data=False))
    assert guard.stage == "idle"
    guard.trip(Obj(data=True))
    assert guard.stage == "anchor"  # subsequent trips re-anchor and restore again


def test_already_off_hot_start_does_not_attest_successful_home(guard):
    guard.stage, guard.started = "cancel", 10.0
    guard.hardware.enabled = [False] * 7
    guard.advance(10.0)
    guard.tick()
    assert guard.stage == "off" and not guard.clear.publish.called


@pytest.mark.parametrize("stamp_ns", [9_000_000_000, 11_000_000_000])
def test_delayed_or_future_source_state_cannot_authorize_off(guard, stamp_ns):
    from rclpy.time import Time
    from sensor_msgs.msg import JointState

    guard.node.get_clock = lambda: Obj(now=lambda: Time(nanoseconds=10_000_000_000))
    guard.measured_stamp_ns = -1
    guard.measured = None
    message = JointState()
    message.header.stamp = Time(nanoseconds=stamp_ns).to_msg()
    message.name = [f"joint{i}" for i in range(1, 8)]
    message.position, message.velocity = [0.0] * 7, [0.0] * 7
    guard.observe_joints(message)
    assert guard.measured is None
    assert not guard.at_home(10.0)


def test_replayed_source_stamp_does_not_refresh_measured_home(guard, monkeypatch):
    from rclpy.time import Time
    from sensor_msgs.msg import JointState

    guard.node.get_clock = lambda: Obj(now=lambda: Time(nanoseconds=10_000_000_000))
    guard.measured_stamp_ns = -1
    monkeypatch.setattr("rebot_thermal_guard.time.monotonic", lambda: 10.0)
    message = JointState()
    message.header.stamp = Time(nanoseconds=9_990_000_000).to_msg()
    message.name = [f"joint{i}" for i in range(1, 8)]
    message.position, message.velocity = [0.0] * 7, [0.0] * 7
    guard.observe_joints(message)
    first = guard.measured_time
    monkeypatch.setattr("rebot_thermal_guard.time.monotonic", lambda: 11.0)
    guard.observe_joints(message)
    assert guard.measured_time == first
    assert not guard.at_home(11.0)


def test_motor_state_uses_controller_steady_clock_not_ros_wall_time(guard, monkeypatch):
    from rclpy.time import Time
    from robstride_interfaces.msg import RobstrideState

    guard.node.get_clock = lambda: Obj(
        now=lambda: Time(nanoseconds=1_792_606_000_000_000_000)
    )
    guard.hardware_stamp_ns = -1
    guard.hardware = None
    monkeypatch.setattr("rebot_thermal_guard.time.monotonic_ns", lambda: 10_000_000_000)
    monkeypatch.setattr("rebot_thermal_guard.time.monotonic", lambda: 10.0)
    state = RobstrideState()
    state.header.stamp = Time(nanoseconds=9_990_000_000).to_msg()
    state.id, state.enabled, state.fault_bits = list(range(1, 8)), [True] * 7, [0] * 7
    state.comm_state = 0
    guard.observe_hardware(state)
    assert guard.hardware_fresh(10.0)
    first = guard.hardware_time
    monkeypatch.setattr("rebot_thermal_guard.time.monotonic_ns", lambda: 11_000_000_000)
    monkeypatch.setattr("rebot_thermal_guard.time.monotonic", lambda: 11.0)
    guard.observe_hardware(state)
    assert guard.hardware_time == first
    assert not guard.hardware_fresh(11.0)


def gain_description(values=None):
    values = values or [(50, 3), (150, 10), (151, 9), (49, 5), (48, 4), (47, 2)]
    root = ET.Element("robot", name="test")
    control = ET.SubElement(root, "ros2_control", name="RebotSystem", type="system")
    hardware = ET.SubElement(control, "hardware")
    ET.SubElement(hardware, "plugin").text = (
        "robstride_hardware_interface/RobstrideHardware"
    )
    for i, (kp, kd) in enumerate(values + [(12, 1)], 1):
        joint = ET.SubElement(control, "joint", name=f"joint{i}")
        ET.SubElement(joint, "param", name="id").text = str(i)
        gpio = ET.SubElement(control, "gpio", name=f"motor{i}")
        for key, value in (("ID", i), ("kp", kp), ("kd", kd)):
            ET.SubElement(gpio, "param", name=key).text = str(value)
    return root


def reference(guard, now, positions=None, *, stamp=None):
    from control_msgs.msg import JointTrajectoryControllerState

    msg = JointTrajectoryControllerState()
    msg.header.stamp = Time(
        nanoseconds=int((now if stamp is None else stamp) * 1e9)
    ).to_msg()
    msg.joint_names = [f"joint{i}" for i in range(1, 8)]
    msg.reference.positions = (
        list(guard.anchor_positions) if positions is None else positions
    )
    guard.observe_reference(msg)


def begin_anchor(guard, monkeypatch):
    gain_calls = guard.gains.call_async.call_count
    monkeypatch.setattr("rebot_thermal_guard.time.monotonic", lambda: 10.0)
    guard.trip(Obj(data=True))
    guard.advance(10.0)  # stamped stationary goal sent
    guard.advance(10.0)  # accepted, but not yet commanded
    assert guard.stage == "anchor_reference"
    assert guard.gains.call_async.call_count == gain_calls
    assert not guard.cancel.call_async.called


def confirm_anchor(guard, monkeypatch):
    for now in (10.01, 10.02):
        monkeypatch.setattr("rebot_thermal_guard.time.monotonic", lambda: now)
        guard.node.get_clock = lambda: Obj(now=lambda: Time(nanoseconds=int(now * 1e9)))
        reference(guard, now)
    guard.advance(10.02)
    assert guard.stage == "restore"


def test_every_trip_anchors_then_restores_all_configured_arm_gains_before_cancel(
    guard, monkeypatch
):
    guard.measured = [(q, 0.0) for q in (0.4, 0.7, 0.8, -0.3, 0.2, -0.1, 0.015)]
    begin_anchor(guard, monkeypatch)
    goal = guard.anchor.send_goal_async.call_args.args[0]
    assert Time.from_msg(goal.trajectory.header.stamp).nanoseconds > 0
    assert goal.trajectory.joint_names == [f"joint{i}" for i in range(1, 8)]
    assert len(goal.trajectory.points) == 2
    for point in goal.trajectory.points:
        assert list(point.positions) == [q for q, _ in guard.measured]
        assert list(point.velocities) == list(point.accelerations) == [0.0] * 7
    assert goal.trajectory.points[0].time_from_start.sec == 0
    assert goal.trajectory.points[0].time_from_start.nanosec == 0
    confirm_anchor(guard, monkeypatch)
    for index, expected in enumerate(guard.gain_commands):
        guard.advance(10.02)
        request = guard.gains.call_async.call_args.args[0]
        assert (request.id, request.item_name, request.item_data) == expected
        assert guard.gains.call_async.call_count == index + 1
        assert not guard.cancel.call_async.called
        guard.advance(10.02)
    assert guard.stage == "cancel" and guard.gain_index == 12
    assert not guard.home.send_goal_async.called and not guard.torque.call_async.called
    guard.advance(10.02)
    assert guard.cancel.call_async.call_count == 1


@pytest.mark.parametrize(
    "bad", ["old", "future", "nan", "wrong", "canceled", "one_sample"]
)
def test_ack_or_invalid_reference_never_authorizes_stiffness(guard, monkeypatch, bad):
    begin_anchor(guard, monkeypatch)
    guard.node.get_clock = lambda: Obj(now=lambda: Time(nanoseconds=10_010_000_000))
    if bad == "old":
        reference(guard, 10.01, stamp=9.9)
    elif bad == "future":
        reference(guard, 10.01, stamp=11.0)
    elif bad == "nan":
        reference(guard, 10.01, positions=[math.nan] * 7)
    elif bad == "wrong":
        reference(guard, 10.01, positions=[0.3] * 7)
    elif bad == "canceled":
        guard.anchor_handle.status = 5
        reference(guard, 10.01)
    else:
        reference(guard, 10.01)
    guard.advance(10.01)
    assert guard.stage == ("failed" if bad == "canceled" else "anchor_reference")
    assert not guard.gains.call_async.called
    assert not guard.cancel.call_async.called


@pytest.mark.parametrize(
    "bad", ["stale", "fault", "partial_off", "no_joint_state", "stale_joints"]
)
def test_invalid_feedback_cannot_anchor_or_restore(guard, monkeypatch, bad):
    if bad == "stale":
        guard.hardware_time = 9.0
    elif bad == "fault":
        guard.hardware.fault_bits[1] = 1
    elif bad == "partial_off":
        guard.hardware.enabled[1] = False
    elif bad == "no_joint_state":
        guard.measured = None
    else:
        guard.measured_time = 9.0
    monkeypatch.setattr("rebot_thermal_guard.time.monotonic", lambda: 10.0)
    guard.trip(Obj(data=True))
    guard.advance(10.0)
    assert guard.stage == "failed"
    assert not guard.anchor.send_goal_async.called
    assert not guard.gains.call_async.called and not guard.torque.call_async.called


def test_missing_configuration_waits_without_default_gain_or_anchor(guard, monkeypatch):
    guard.gain_commands = None
    monkeypatch.setattr("rebot_thermal_guard.time.monotonic", lambda: 10.0)
    guard.trip(Obj(data=True))
    guard.advance(10.0)
    assert guard.stage == "anchor"
    assert not guard.anchor.send_goal_async.called and not guard.gains.call_async.called


def test_configuration_uses_gpio_id_not_order_and_excludes_jaw(guard):
    root = gain_description()
    control = root.find("ros2_control")
    control[:] = list(reversed(control[:]))
    guard.gain_commands = None
    guard.observe_description(Obj(data=ET.tostring(root, encoding="unicode")))
    assert guard.gain_commands == (
        (1, "kd", 3.0),
        (2, "kd", 10.0),
        (3, "kd", 9.0),
        (4, "kd", 5.0),
        (5, "kd", 4.0),
        (6, "kd", 2.0),
        (1, "kp", 50.0),
        (2, "kp", 150.0),
        (3, "kp", 151.0),
        (4, "kp", 49.0),
        (5, "kp", 48.0),
        (6, "kp", 47.0),
    )
    # Initial config is immutable even if another description later arrives.
    saved = guard.gain_commands
    guard.observe_description(
        Obj(data=ET.tostring(gain_description([(0, 1)] * 6), encoding="unicode"))
    )
    assert guard.gain_commands == saved


@pytest.mark.parametrize(
    "bad", ["xml", "missing", "duplicate_id", "wrong_joint_id", "nan", "negative"]
)
def test_bad_configuration_never_supplies_guessed_gains(guard, bad):
    root = gain_description()
    control = root.find("ros2_control")
    if bad == "missing":
        control.remove(control.findall("gpio")[0])
    elif bad == "duplicate_id":
        control.findall("gpio")[1].find("param[@name='ID']").text = "1"
    elif bad == "wrong_joint_id":
        control.findall("joint")[0].find("param[@name='id']").text = "7"
    elif bad in ("nan", "negative"):
        control.findall("gpio")[0].find("param[@name='kp']").text = (
            "nan" if bad == "nan" else "-1"
        )
    guard.gain_commands = None
    guard.observe_description(
        Obj(data="invalid" if bad == "xml" else ET.tostring(root, encoding="unicode"))
    )
    assert guard.gain_commands is None


@pytest.mark.parametrize(
    "bad",
    [
        "rejected",
        "aborted",
        "changed_reference",
        "stale_reference",
        "gain_rejected",
        "gain_timeout",
    ],
)
def test_prelude_failure_stops_without_home_or_torque(guard, monkeypatch, bad):
    if bad == "rejected":
        guard.anchor.send_goal_async.return_value = future(Obj(accepted=False))
        monkeypatch.setattr("rebot_thermal_guard.time.monotonic", lambda: 10.0)
        guard.trip(Obj(data=True))
        guard.advance(10.0)
        guard.advance(10.0)
    else:
        begin_anchor(guard, monkeypatch)
        confirm_anchor(guard, monkeypatch)
        if bad == "aborted":
            guard.anchor_result = future(Obj(status=6, result=Obj(error_code=-8)))
        elif bad == "changed_reference":
            guard.reference_positions = [0.4] * 7
        elif bad == "stale_reference":
            guard.reference_time = 9.0
        elif bad == "gain_rejected":
            guard.gains.call_async.return_value = future(Obj(result=False))
        else:
            guard.gains.call_async.return_value = Obj(done=lambda: False)
        guard.advance(10.02)
        guard.tick()  # production tick does not advance failed recoveries
        if bad == "gain_timeout":
            guard.hardware_time = guard.measured_time = guard.reference_time = 10.53
            guard.advance(10.53)
    assert guard.stage == "failed"
    assert not guard.cancel.call_async.called and not guard.home.send_goal_async.called
    assert not guard.torque.call_async.called
    if bad not in ("gain_rejected", "gain_timeout"):
        assert not guard.gains.call_async.called
    else:
        assert guard.gains.call_async.call_count == 1


def test_new_latch_discards_previous_anchor_and_repeats_all_twelve_gains(
    guard, monkeypatch
):
    original_gains = guard.gain_commands
    for trial in range(2):
        guard.node.get_clock = lambda: Obj(now=lambda: Time(nanoseconds=10_000_000_000))
        guard.measured = [(trial * 0.2 + 0.1, 0.0)] * 6 + [(0.012, 0.0)]
        begin_anchor(guard, monkeypatch)
        assert guard.reference_samples == 0 and guard.gain_index == 0
        confirm_anchor(guard, monkeypatch)
        for _ in range(24):
            guard.advance(10.02)
        assert guard.stage == "cancel"
        assert guard.gains.call_async.call_count == (trial + 1) * 12
        assert guard.gain_commands == original_gains
        guard.stage = "cooling"  # existing recovery tests cover HOME/OFF to here
        guard.trip(Obj(data=False))
        assert guard.stage == "idle"
    assert guard.anchor.send_goal_async.call_count == 2
    assert list(
        guard.anchor.send_goal_async.call_args.args[0].trajectory.points[0].positions
    ) == [q for q, _ in guard.measured]


def test_pending_gain_and_duplicate_trip_never_overlap_or_restart_requests(
    guard, monkeypatch
):
    begin_anchor(guard, monkeypatch)
    confirm_anchor(guard, monkeypatch)
    pending = Obj(done=lambda: False)
    guard.gains.call_async.return_value = pending
    guard.advance(10.02)
    for _ in range(20):
        guard.trip(Obj(data=True))
        guard.advance(10.03)
    assert guard.future is pending and guard.gain_index == 0
    assert guard.gains.call_async.call_count == 1
    assert guard.anchor.send_goal_async.call_count == 1
    assert not guard.cancel.call_async.called


def test_repeated_reference_stamp_does_not_count_as_a_second_controller_cycle(
    guard, monkeypatch
):
    begin_anchor(guard, monkeypatch)
    now = 10.01
    monkeypatch.setattr("rebot_thermal_guard.time.monotonic", lambda: now)
    guard.node.get_clock = lambda: Obj(now=lambda: Time(nanoseconds=10_010_000_000))
    for _ in range(3):
        reference(guard, now)
    guard.advance(now)
    assert guard.reference_samples == 1
    assert guard.stage == "anchor_reference" and not guard.gains.call_async.called


@pytest.mark.parametrize("bad", ["fault", "stale", "off"])
def test_health_rechecked_between_damping_and_stiffness(guard, monkeypatch, bad):
    begin_anchor(guard, monkeypatch)
    confirm_anchor(guard, monkeypatch)
    for _ in range(12):
        guard.advance(10.02)
    assert guard.gain_index == 6
    if bad == "fault":
        guard.hardware.fault_bits[0] = 1
    elif bad == "stale":
        guard.hardware_time = 9.0
    else:
        guard.hardware.enabled[0] = False
    guard.advance(10.02)
    assert guard.stage == "failed"
    assert [c.args[0].item_name for c in guard.gains.call_async.call_args_list] == [
        "kd"
    ] * 6
    assert not guard.cancel.call_async.called and not guard.torque.call_async.called


def test_latched_hot_start_waits_for_first_health_then_stays_off(guard, monkeypatch):
    hardware = guard.hardware
    guard.hardware = None
    monkeypatch.setattr("rebot_thermal_guard.time.monotonic", lambda: 10.0)
    guard.trip(Obj(data=True))
    guard.advance(10.01)
    assert guard.stage == "anchor"
    hardware.enabled = [False] * 7
    guard.hardware = hardware
    guard.advance(10.02)
    assert guard.stage == "off"
    assert not guard.anchor.send_goal_async.called and not guard.gains.call_async.called
    assert not guard.cancel.call_async.called and not guard.torque.call_async.called


def test_zero_clock_cannot_send_an_unstamped_anchor(guard, monkeypatch):
    guard.node.get_clock = lambda: Obj(now=lambda: Time(nanoseconds=0))
    monkeypatch.setattr("rebot_thermal_guard.time.monotonic", lambda: 10.0)
    guard.trip(Obj(data=True))
    guard.advance(10.0)
    assert guard.stage == "failed"
    assert not guard.anchor.send_goal_async.called and not guard.gains.call_async.called


def test_stop_signals_exit_without_a_traceback(monkeypatch):
    import signal
    import rclpy
    import rebot_thermal_guard

    create_node = rclpy.create_node

    def node_that_sees_a_second_stop(name):
        node = create_node(name)
        destroy_node = node.destroy_node

        def destroy():
            # A launch forwards its own SIGINT after the terminal's.
            signal.raise_signal(signal.SIGINT)
            destroy_node()

        node.destroy_node = destroy
        return node

    monkeypatch.setattr(rclpy, "create_node", node_that_sees_a_second_stop)
    monkeypatch.setattr(rclpy, "spin", lambda node: signal.raise_signal(signal.SIGINT))
    handlers = {
        number: signal.getsignal(number) for number in (signal.SIGINT, signal.SIGTERM)
    }
    try:
        rebot_thermal_guard.main()
    except KeyboardInterrupt:
        pytest.fail("A second stop signal interrupted the guard's cleanup")
    finally:
        rclpy.try_shutdown()
        for number, handler in handlers.items():
            signal.signal(number, handler)
