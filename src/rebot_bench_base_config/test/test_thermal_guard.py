# Copyright 2026 PickNik Inc.
# SPDX-License-Identifier: BSD-3-Clause
"""Thermal response only cuts torque after native success and fresh quiet HOME."""
from pathlib import Path
import sys
from types import SimpleNamespace as Obj
from unittest.mock import Mock

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from rebot_thermal_guard import ThermalGuard  # noqa: E402


def future(result):
    return Obj(done=lambda: True, result=lambda: result)


@pytest.fixture
def guard():
    result = ThermalGuard.__new__(ThermalGuard)
    result.node = Obj(get_logger=lambda: Obj(error=Mock()))
    result.stage, result.started = "idle", None
    result.future = result.home_handle = result.home_since = None
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
    assert guard.stage == "cancel"
    guard.trip(Obj(data=False))
    guard.trip(Obj(data=True))
    assert guard.stage == "cancel" and guard.started == 10.0
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
    assert guard.stage == "off"


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
        "intervention required" in call.args[0]
        for call in logger.error.call_args_list
    )
    assert not guard.cancel.call_async.called
    assert not guard.home.send_goal_async.called and not guard.torque.call_async.called


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
