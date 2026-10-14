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
"""SIM-ONLY: operate a contact-gated weld from the ordinary gripper command."""
import math
from pathlib import Path

import mujoco
import rclpy
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from control_msgs.msg import JointTrajectoryControllerState
from moveit_studio_internal_msgs.msg import MujocoQpos
from picknik_nyx_renderer.qpos_mirror import apply_qpos, qpos_model_hash
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from std_msgs.msg import Float64MultiArray


class SimGraspProxy(Node):
    def __init__(self):
        super().__init__("sim_grasp_proxy")
        mujoco.mj_loadAllPluginLibraries(
            str(Path(get_package_prefix("mujoco_package_uri")) / "bin/mujoco_plugin")
        )
        share = Path(get_package_share_directory("rebot_bench_sim"))
        self.model = mujoco.MjModel.from_xml_path(str(share / "description/scene.xml"))
        self.data = mujoco.MjData(self.model)
        self.schema_hash = qpos_model_hash(mujoco, self.model)
        self.finger = self.model.body("gripper_left").id
        self.cube = self.model.body("blue_cube").id
        self.closed = False
        self.grasped = False
        self.sim_time = -math.inf
        self.publisher = self.create_publisher(
            Float64MultiArray, "/sim_grasp_proxy_controller/commands", 10
        )
        # MuJoCo's suction latch survives a keyframe reset and clears only on a
        # zero command, so start released.
        self.publish()
        self.create_subscription(
            JointTrajectoryControllerState,
            "/joint_trajectory_admittance_controller/controller_state",
            self.command,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            MujocoQpos,
            "/mujoco/qpos_stamped",
            self.state,
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        self.create_timer(0.02, self.publish)
        self.get_logger().warning(
            "SIM-ONLY GRASP PROXY enabled: contact-triggered weld; "
            "successful picks do not validate gripper contact physics."
        )

    def command(self, msg):
        if "joint7" not in msg.joint_names:
            return
        index = msg.joint_names.index("joint7")
        if index >= len(msg.reference.positions):
            return
        position = msg.reference.positions[index]
        if not math.isfinite(position):
            return
        # Arm trajectories retain the same hold command. Opening releases the
        # weld before significant jaw travel.
        self.closed = position <= 2.90
        if not self.closed:
            self.grasped = False

    def state(self, msg):
        if msg.schema_hash != self.schema_hash:
            raise RuntimeError("Live simulator model differs from the loaded model")
        # Reset MuJoCo Sim sets simulation time back to the keyframe's.
        if msg.sim_time < self.sim_time:
            self.grasped = False
        self.sim_time = msg.sim_time
        if self.closed and not self.grasped:
            # The simulator's own state; this node never moves a cube.
            apply_qpos(self.data.qpos, msg.qpos)
            mujoco.mj_fwdPosition(self.model, self.data)
            self.grasped = any(
                {self.model.geom_bodyid[g] for g in contact.geom}
                == {self.finger, self.cube}
                for contact in self.data.contact[: self.data.ncon]
            )

    def publish(self):
        # JTAC stops publishing state while idle between planned trajectories.
        # Its last motor command remains held, so the proxy must retain it too.
        self.publisher.publish(Float64MultiArray(data=[float(self.grasped)]))


def main():
    rclpy.init()
    node = SimGraspProxy()
    try:
        rclpy.spin(node)
    finally:
        node.publisher.publish(Float64MultiArray(data=[0.0]))
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
