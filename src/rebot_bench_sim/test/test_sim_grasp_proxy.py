#!/usr/bin/env python3

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
"""Drive the SIM-ONLY grasp proxy node through its topics: pick, then reset."""
import importlib.util
import time
from pathlib import Path

import mujoco
import pytest
import rclpy
from ament_index_python.packages import get_package_prefix
from control_msgs.msg import JointTrajectoryControllerState
from moveit_studio_internal_msgs.msg import MujocoQpos
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from std_msgs.msg import Float64MultiArray

HOLD, OPEN = 2.8125, 5.25


@pytest.fixture
def proxy():
    rclpy.init()
    script = Path(get_package_prefix("rebot_bench_sim")) / "lib/rebot_bench_sim"
    spec = importlib.util.spec_from_file_location(
        "sim_grasp_proxy", script / "sim_grasp_proxy.py"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    node = module.SimGraspProxy()
    harness = rclpy.create_node("sim_grasp_proxy_harness")
    commands = []
    harness.create_subscription(
        Float64MultiArray,
        "/sim_grasp_proxy_controller/commands",
        lambda msg: commands.append(msg.data[0]),
        10,
    )
    reference = harness.create_publisher(
        JointTrajectoryControllerState,
        "/joint_trajectory_admittance_controller/controller_state",
        qos_profile_sensor_data,
    )
    qpos = harness.create_publisher(
        MujocoQpos,
        "/mujoco/qpos_stamped",
        QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        ),
    )
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(harness)
    # The reference is best-effort and volatile: publish only once matched.
    while not (
        commands
        and reference.get_subscription_count()
        and qpos.get_subscription_count()
    ):
        executor.spin_once(timeout_sec=0.01)

    def step(joint7=None, state=None, sim_time=None):
        """Publish a gripper reference and/or a simulator state; return the command."""
        if joint7 is not None:
            msg = JointTrajectoryControllerState(joint_names=["joint7"])
            msg.reference.positions = [joint7]
            reference.publish(msg)
        if state is not None:
            qpos.publish(
                MujocoQpos(
                    schema_hash=node.schema_hash,
                    sim_time=sim_time,
                    qpos=state.tolist(),
                )
            )
        deadline = time.monotonic() + 0.5
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.01)
        return commands[-1]

    yield node.model, step
    executor.shutdown()
    node.destroy_node()
    harness.destroy_node()
    rclpy.shutdown()


def states(model):
    """The reset keyframe, and blue held between the jaws at a 49 mm aperture."""
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, model.key("default").id)
    rest = data.qpos.copy()
    for name, value in zip(
        [f"joint{i}" for i in range(1, 7)], [0, 0.65, 0.85, 0.3, 0, 0]
    ):
        data.joint(name).qpos = value
    data.joint("joint7").qpos = 0.049 / 0.016
    for jaw in ("gripper_joint1", "gripper_joint2"):
        data.joint(jaw).qpos = 0.049 / 2
    mujoco.mj_kinematics(model, data)
    tip = data.body("gripper_end")
    address = model.jnt_qposadr[model.body("blue_cube").jntadr[0]]
    data.qpos[address : address + 3] = tip.xpos + tip.xmat.reshape(3, 3) @ [-0.01, 0, 0]
    data.qpos[address + 3 : address + 7] = tip.xquat
    return rest, data.qpos.copy()


def test_proxy_welds_only_at_contact_and_releases_after_a_reset(proxy):
    model, step = proxy
    rest, held = states(model)
    # A closed command alone, with the jaws nowhere near blue, never welds.
    assert step(joint7=HOLD, state=rest, sim_time=1.0) == 0.0
    assert step(state=held, sim_time=2.0) == 1.0
    # Reset MuJoCo Sim: simulation time goes back while the stale closed
    # reference remains. The proxy must command release so MuJoCo's latch
    # cannot weld the resting cube to the gripper without contact.
    assert step(state=rest, sim_time=0.0) == 0.0
    assert step(state=rest, sim_time=0.5) == 0.0
    # Contact after the reset grasps again; opening releases.
    assert step(state=held, sim_time=1.0) == 1.0
    assert step(joint7=OPEN) == 0.0
