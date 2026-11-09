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

"""Execute Undock From D4 in the installed BT engine against a live command bridge."""

import re
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import rclpy
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64MultiArray
from std_srvs.srv import Trigger

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from base_twist_to_planar import BaseTwistToPlanar  # noqa: E402

OBJECTIVE = Path(__file__).resolve().parents[1] / "objectives/undock_from_d4.xml"
LABELS = {"R": [-0.25, 0.0, 0.0], "Z": [0.0, 0.0, 0.0], "E": [0.2, 0.0, 0.0]}


class UndockHarness:
    """Real bridge with fresh readiness/encoder inputs and competing Nav2 and /cmd_vel sources."""

    def __init__(self):
        self.bridge = BaseTwistToPlanar(
            parameter_overrides=[
                Parameter("max_linear_velocity", value=0.35),
                Parameter("max_angular_velocity", value=0.5),
            ]
        )
        self.peer = Node("undock_objective_peer")
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.bridge)
        self.executor.add_node(self.peer)
        self.compete = True
        self.labels = []
        self.feedback = self.peer.create_publisher(
            JointState, "/base_state_broadcaster/joint_states", qos_profile_sensor_data
        )
        self.ready = self.peer.create_publisher(
            Bool, "/market_sim/localization_ready", 10
        )
        self.hold = self.peer.create_publisher(
            Bool,
            "/market_sim/reset_controller_hold",
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self.nav = self.peer.create_publisher(Twist, "/nav2_cmd_vel", 10)
        self.stamped = self.peer.create_publisher(TwistStamped, "/cmd_vel", 10)
        self.peer.create_subscription(
            Float64MultiArray, "/base_jgvc/commands", self._record, 10
        )
        self.peer.create_timer(0.01, self._publish)

    def _record(self, msg):
        data = list(msg.data)
        label = next(
            (k for k, v in LABELS.items() if data == pytest.approx(v)), repr(data)
        )
        self.labels.append(label)

    def _publish(self):
        now = self.peer.get_clock().now().to_msg()
        state = JointState(name=["planar_theta"], position=[0.0])
        state.header.stamp = now
        self.feedback.publish(state)
        self.ready.publish(Bool(data=True))
        self.hold.publish(Bool(data=False))
        if self.compete:
            command = Twist()
            command.linear.x = 0.2
            self.nav.publish(command)
            stamped = TwistStamped()
            stamped.header.frame_id = "base_link"
            stamped.header.stamp = now
            stamped.twist = command
            self.stamped.publish(stamped)

    def spin(self, seconds=None, until=None):
        """Spin both nodes for a duration or until a predicate holds."""
        deadline = time.monotonic() + (seconds if seconds is not None else 3.0)
        while time.monotonic() < deadline and not (until and until()):
            self.executor.spin_once(timeout_sec=0.005)
        if until is not None:
            assert until(), "Bridge did not reach its expected command"

    def settle(self, label):
        """The bridge holds one command for longer than its 0.5 s watchdog."""
        self.spin(until=lambda: self.labels[-3:] == [label] * 3)
        self.labels.clear()
        self.spin(0.6)
        assert self.labels and set(self.labels) == {label}, self.labels

    def call(self, service):
        client = self.peer.create_client(Trigger, service)
        self.spin(until=client.service_is_ready)
        future = client.call_async(Trigger.Request())
        self.spin(until=future.done)
        self.peer.destroy_client(client)
        return future.result()

    def run(self, path=OBJECTIVE, stop_after=None, during=None):
        """Run a fresh tree process; return its result and the bridge commands it produced."""
        executable = (
            Path(get_package_prefix("market_sim"))
            / "lib/market_sim/market_sim_test_run_objective_tree"
        )
        command = [str(executable), str(path)]
        if stop_after is not None:
            command.append(str(stop_after))
        self.labels.clear()
        with subprocess.Popen(
            command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True
        ) as process:
            deadline = time.monotonic() + 40.0
            try:
                while process.poll() is None and time.monotonic() < deadline:
                    self.executor.spin_once(timeout_sec=0.005)
                    if during is not None:
                        during()
                if process.poll() is None:
                    pytest.fail("Undock Objective exceeded its bounded test deadline")
                output, _ = process.communicate()
            finally:
                if process.poll() is None:
                    process.kill()
                    process.wait()
        result = re.search(r"OBJECTIVE_RESULT=(\w+)", output)
        assert result, output
        return result.group(1), "".join(
            label if len(label) == 1 else "?" for label in self.labels
        )

    def owner_released(self):
        """A fresh acquisition succeeds only when no undock still owns the command."""
        acquired = self.call("/market_sim/acquire_undock").success
        assert self.call("/market_sim/release_undock").success
        return acquired

    def close(self):
        self.executor.shutdown()
        self.bridge.destroy_node()
        self.peer.destroy_node()


@pytest.fixture
def undock():
    """No Runtime, robot, Simulator or controller manager is launched."""
    rclpy.init()
    harness = UndockHarness()
    try:
        harness.settle("E")
        yield harness
    finally:
        harness.close()
        rclpy.shutdown()


def segments(commands):
    """Collapse repeated commands into their ordered phases."""
    return re.sub(r"(.)\1+", r"\1", commands)


def test_undock_completes_owning_the_base_then_releases_at_zero(undock) -> None:
    """Competing sources never interrupt reverse or zero; new external commands resume after release."""
    result, commands = undock.run()
    assert result == "SUCCESS", commands
    assert re.fullmatch(r"E*Z*R+Z+E*", commands), segments(commands)
    reverse = commands.count("R") * 0.02
    assert 5.0 <= reverse <= 8.6, reverse
    assert re.search(r"RZ{10,}", commands), segments(commands)
    undock.settle("E")
    assert undock.owner_released()


def test_reverse_failure_still_stops_and_releases(undock, tmp_path) -> None:
    """A failure inside the reverse branch runs both zero and ownership cleanup."""
    root = ET.parse(OBJECTIVE)
    parallel = root.find(".//Control[@name='Bound the reverse command']")
    wait = parallel.find("Action[@ID='WaitForDuration']")
    fail = ET.Element("Control", ID="Sequence")
    ET.SubElement(fail, "Action", ID="WaitForDuration", delay_duration="1.0")
    ET.SubElement(fail, "Action", ID="AlwaysFailure")
    parallel.insert(list(parallel).index(wait), fail)
    parallel.remove(wait)
    failing = tmp_path / "undock_reverse_fails.xml"
    root.write(failing)
    result, commands = undock.run(failing)
    assert result == "FAILURE", commands
    assert re.fullmatch(r"E*Z*R+Z+E*", commands), segments(commands)
    assert commands.count("R") * 0.02 < 2.0
    assert re.search(r"RZ{10,}", commands), segments(commands)
    undock.settle("E")
    assert undock.owner_released()


def test_stopped_undock_stops_releases_and_resumes_only_new_commands(undock) -> None:
    """Halting mid-reverse runs cleanup; an ignored competitor does not resume once released."""
    reversing_since = []

    def silence_competitors_while_reversing():
        if not reversing_since and undock.labels[-1:] == ["R"]:
            reversing_since.append(time.monotonic())
        if reversing_since and time.monotonic() - reversing_since[0] > 1.0:
            undock.compete = False

    result, commands = undock.run(
        stop_after=3.0, during=silence_competitors_while_reversing
    )
    assert result == "STOPPED", commands
    assert not undock.compete, "The tree was stopped before the competitor was silenced"
    assert re.fullmatch(r"E*Z*R+Z+", commands), segments(commands)
    assert commands.count("R") * 0.02 < 4.0
    assert re.search(r"RZ{10,}", commands), segments(commands)
    undock.settle("Z")
    assert undock.owner_released()
    undock.compete = True
    undock.settle("E")


def test_failed_acquisition_releases_a_leaked_owner(undock) -> None:
    """An owner left by a crashed execution is released when the next acquisition fails."""
    assert undock.call("/market_sim/acquire_undock").success
    undock.settle("Z")
    result, commands = undock.run()
    assert result == "FAILURE", commands
    assert "R" not in commands, segments(commands)
    assert re.fullmatch(r"Z+E*", commands), segments(commands)
    undock.settle("E")
    assert undock.owner_released()
