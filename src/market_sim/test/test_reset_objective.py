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

"""Execute the complete reset Objective against isolated controller and estimator peers."""

import os
import subprocess
import sys
import time
from pathlib import Path

import pytest
import rclpy
from ament_index_python.packages import get_package_prefix
from controller_manager_msgs.msg import ControllerState
from controller_manager_msgs.srv import ListControllers, SwitchController
from moveit_studio_internal_msgs.srv import ResetMujocoKeyframe
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from std_srvs.srv import Empty, Trigger

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from reset_controller_guard import ResetControllerGuard  # noqa: E402
from test_reset_relocalizer import ResetHarness  # noqa: E402

PACKAGE = Path(__file__).resolve().parents[1]


class ResetObjectiveHarness:
    """Real tree/Behavior clients and reset guard, with deterministic service dependencies."""

    def __init__(self):
        self.peer = Node("reset_objective_peer")
        self.guard = ResetControllerGuard(
            parameter_overrides=[Parameter("service_timeout_sec", value=0.5)]
        )
        self.localization = ResetHarness()
        self.localization.accept = False
        self.localization.fused_x = self.localization.amcl_x = 0.0
        self.executor = self.localization.executor
        self.executor.add_node(self.peer)
        self.executor.add_node(self.guard)
        self.states = {
            "base_jgvc": "active",
            "left_gripper_controller": "active",
            "right_gripper_controller": "active",
            "joint_trajectory_admittance_controller": "active",
            "base_jvc": "inactive",
            "joint_state_broadcaster": "active",
        }
        self.original = {
            name
            for name, state in self.states.items()
            if state == "active" and name != "joint_state_broadcaster"
        }
        self.events = []
        self.accept_switch = True
        self.reject_restore_once = False
        self.holds = []
        self.peer.create_subscription(
            Bool,
            "/market_sim/reset_controller_hold",
            lambda msg: self.holds.append(msg.data),
            10,
        )
        self.list_service = self.peer.create_service(
            ListControllers, "/controller_manager/list_controllers", self._list
        )
        self.peer.create_service(
            SwitchController, "/controller_manager/switch_controller", self._switch
        )
        self.peer.create_service(
            ResetMujocoKeyframe, "/mujoco_system/reset_keyframe", self._reset
        )
        self.peer.create_service(
            Empty, "/clear_octomap", lambda _req, response: response
        )

    def _list(self, _request, response):
        for name, state in self.states.items():
            command = name != "joint_state_broadcaster"
            response.controller.append(
                ControllerState(
                    name=name,
                    state=state,
                    required_command_interfaces=[f"{name}/velocity"] if command else [],
                    claimed_interfaces=(
                        [f"{name}/velocity"] if command and state == "active" else []
                    ),
                )
            )
        return response

    def _switch(self, request, response):
        activate = set(request.activate_controllers)
        deactivate = set(request.deactivate_controllers)
        self.events.append(("switch", activate, deactivate))
        assert (
            activate or deactivate
        ), "A reset must not send an empty controller switch"
        response.ok = self.accept_switch
        if activate and self.reject_restore_once:
            self.reject_restore_once = False
            self.states[next(iter(activate))] = "active"
            response.ok = False
        if response.ok:
            if activate:
                assert self.localization.ready[
                    -1
                ], "Controller restore must wait for actual estimator synchronization"
            for name in deactivate:
                self.states[name] = "inactive"
            for name in activate:
                self.states[name] = "active"
        return response

    def _reset(self, request, response):
        self.events.append(("keyframe", request.keyframe_name))
        assert (
            not self.active()
        ), "The complete Objective must deactivate commands before teleporting"
        response.success = True
        return response

    def active(self):
        """Observed active command controllers, excluding broadcasters."""
        return {
            name
            for name, state in self.states.items()
            if state == "active" and name != "joint_state_broadcaster"
        }

    def run(self):
        """Start a fresh installed BT engine process, matching a new Objective execution."""
        executable = (
            Path(get_package_prefix("market_sim"))
            / "lib/market_sim/market_sim_test_run_objective_tree"
        )
        path = os.environ.get(
            "MARKET_SIM_RESET_OBJECTIVE",
            str(PACKAGE / "objectives/reset_mujoco_sim.xml"),
        )
        with subprocess.Popen(
            [str(executable), path],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
        ) as process:
            deadline = time.monotonic() + 15.0
            try:
                while process.poll() is None and time.monotonic() < deadline:
                    self.executor.spin_once(timeout_sec=0.01)
                if process.poll() is None:
                    process.kill()
                    pytest.fail(
                        "Complete reset Objective exceeded its bounded test deadline"
                    )
                output, _ = process.communicate()
            finally:
                if process.poll() is None:
                    process.kill()
                    process.wait()
        assert "OBJECTIVE_RESULT=" in output, output
        return process.returncode, output

    def call(self, service):
        """Call a reset guard endpoint with the same observable peers."""
        client = self.peer.create_client(Trigger, service)
        result = client.call_async(Trigger.Request())
        self.localization.until(result.done)
        response = result.result()
        self.peer.destroy_client(client)
        return response

    def close(self):
        """Destroy only these test peers and their executor."""
        self.guard.destroy_node()
        self.peer.destroy_node()
        self.localization.close()


@pytest.fixture
def reset_objective():
    """No Runtime, robot, Simulator, or external controller manager is launched."""
    rclpy.init()
    harness = ResetObjectiveHarness()
    try:
        yield harness
    finally:
        harness.close()
        rclpy.shutdown()


def test_regression_complete_reset_failure_then_fresh_objective_retry_restores_all(
    reset_objective,
) -> None:
    """Retry after synchronization failure retains the original set, not the now-empty active list."""
    reset = reset_objective
    result, output = reset.run()
    assert result == 1 and "OBJECTIVE_RESULT=FAILURE" in output
    assert reset.active() == set()
    assert reset.states["joint_state_broadcaster"] == "active"
    assert reset.localization.requests
    assert reset.localization.ready[-1] is False
    assert reset.localization.seeds == []
    reset.localization.accept = True
    reset.events.clear()
    result, output = reset.run()
    assert result == 0, output
    assert reset.active() == reset.original
    assert reset.localization.ready[-1] is True
    assert reset.localization.seeds
    assert reset.events == [("keyframe", "default"), ("switch", reset.original, set())]


def test_regression_retry_after_partial_manual_reactivation_keeps_original_set(
    reset_objective,
) -> None:
    """Manually enabling one controller cannot replace the original restore snapshot."""
    reset = reset_objective
    assert reset.run()[0] == 1
    reset.states["base_jgvc"] = "active"
    reset.localization.accept = True
    result, output = reset.run()
    assert result == 0, output
    assert reset.active() == reset.original
    assert reset.events[-1] == ("switch", reset.original, set())


def test_reset_with_no_active_command_controllers_skips_empty_switch(
    reset_objective,
) -> None:
    """An intentionally stopped configuration can reset without activating a guessed controller set."""
    reset = reset_objective
    for name in reset.original:
        reset.states[name] = "inactive"
    reset.localization.accept = True
    result, output = reset.run()
    assert result == 0, output
    assert reset.active() == set()
    assert reset.localization.ready[-1] is True
    assert reset.events == [("keyframe", "default")]


def test_reset_switch_rejection_fails_before_keyframe_and_retry_recovers(
    reset_objective,
) -> None:
    """Rejected deactivation cannot reset a moving robot or discard the saved restore set."""
    reset = reset_objective
    reset.accept_switch = False
    result, output = reset.run()
    assert result == 1, output
    assert reset.events == [("switch", set(), reset.original)]
    reset.accept_switch = True
    reset.localization.accept = True
    result, output = reset.run()
    assert result == 0, output
    assert reset.active() == reset.original


def test_restore_cannot_release_failed_reset_without_synchronization(
    reset_objective,
) -> None:
    """Calling restore out of order cannot bypass the failed-reset stop."""
    reset = reset_objective
    assert reset.run()[0] == 1
    response = reset.call("/market_sim/restore_reset_controllers")
    assert not response.success
    assert "localization" in response.message and "synchronization" in response.message
    assert reset.active() == set()
    reset.localization.until(lambda: reset.holds and reset.holds[-1])


def test_partial_failed_restore_is_stopped_and_fresh_retry_restores_snapshot(
    reset_objective,
) -> None:
    """A rejected activation that changed some states is stopped, with its restore set retained."""
    reset = reset_objective
    reset.localization.accept = True
    reset.reject_restore_once = True
    result, output = reset.run()
    assert result == 1, output
    assert reset.active() == set()
    assert reset.holds[-1]
    result, output = reset.run()
    assert result == 0, output
    assert reset.active() == reset.original


def test_successful_restore_clears_snapshot_for_a_later_reset(reset_objective) -> None:
    """A completed reset must not retain controller choices from a previous execution."""
    reset = reset_objective
    reset.localization.accept = True
    assert reset.run()[0] == 0
    for name in reset.original - {"base_jgvc"}:
        reset.states[name] = "inactive"
    result, output = reset.run()
    assert result == 0, output
    assert reset.active() == {"base_jgvc"}
