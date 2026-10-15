#!/usr/bin/env python3
"""Publish measured Microduck simulation state and accept bounded policy commands."""

import os
from pathlib import Path
import time

import mujoco
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TransformStamped, Twist
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster

from policy_sim import PolicySimulation


COMMAND_TIMEOUT = 0.3
MOTIONS = {
    "forward": (0.30, 0, 0),
    "left": (0, 0, 1.5),
    "right": (0, 0, -1.5),
    "stand": (0, 0, 0),
}
BASE_JOINTS = ["base_x", "base_y", "base_z", "base_yaw", "base_pitch", "base_roll"]
CAMERAS = {
    "overview": ("microduck_overview", "microduck_camera_optical_frame"),
    "head": ("head_camera", "microduck_head_camera_optical_frame"),
}


class MicroduckBridge(Node):
    def __init__(self):
        super().__init__("microduck_policy_sim")
        self.policy_name = Path(os.environ["MICRODUCK_POLICY"]).stem
        self.sim = PolicySimulation(
            os.environ["MICRODUCK_RL_ROOT"],
            os.environ["MICRODUCK_POLICY"],
            get_package_share_directory("hangar_sim") + "/description/hangar.xml",
        )
        self.velocity = np.zeros(3)
        self.last_command = float("-inf")
        self.head_target = self.sim.policy.default_pose[5:9].copy()
        self.state_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.status_pub = self.create_publisher(String, "/microduck/status", 10)
        self.cameras = [
            (
                self.sim.model.camera(camera_name).id,
                frame_id,
                self.create_publisher(Image, f"/microduck/{stream}/image_raw", 2),
                self.create_publisher(
                    CameraInfo, f"/microduck/{stream}/camera_info", 2
                ),
            )
            for stream, (camera_name, frame_id) in CAMERAS.items()
        ]
        self.tf = TransformBroadcaster(self)
        self.create_subscription(String, "/microduck/command", self.command, 10)
        self.create_subscription(Twist, "/microduck/cmd_vel", self.twist, 10)
        self.create_service(Trigger, "/microduck/reset", self.reset)
        self.create_service(Trigger, "/microduck/ready", self.ready)
        self.renderer = mujoco.Renderer(self.sim.model, height=480, width=640)
        self.last_image = 0.0
        self.last_status = 0.0
        self.create_timer(0.02, self.tick)
        self.get_logger().info(
            "Microduck policy ready: 14 measured joints, 50 Hz policy, 200 Hz physics"
        )

    def command(self, message):
        if message.data in MOTIONS:
            self.velocity[:] = MOTIONS[message.data]
            self.last_command = time.monotonic()
        elif message.data in ("look_left", "look_right", "look_center"):
            self.head_target[:] = self.sim.policy.default_pose[5:9]
            self.head_target[2] = {
                "look_left": 0.5,
                "look_right": -0.5,
                "look_center": 0.0,
            }[message.data]
        else:
            self.velocity[:] = 0
            self.last_command = float("-inf")
            self.get_logger().error(
                f"Rejected unknown policy command: {message.data!r}"
            )

    def twist(self, message):
        command = [message.linear.x, message.linear.y, message.angular.z]
        if not np.isfinite(command).all():
            self.velocity[:] = 0
            self.last_command = float("-inf")
            self.get_logger().error("Rejected non-finite velocity command")
            return
        self.velocity[:] = command
        self.last_command = time.monotonic()

    def reset(self, _request, response):
        self.sim.reset()
        self.velocity[:] = 0
        self.head_target[:] = self.sim.policy.default_pose[5:9]
        self.last_command = float("-inf")
        response.success = True
        response.message = "Simulation reset to the upstream standing pose"
        return response

    def ready(self, _request, response):
        upright = self.sim.data.xmat[self.sim.policy.trunk_base_id].reshape(3, 3)[2, 2]
        response.success = bool(self.sim.position[2] > 0.07 and upright > 0.5)
        response.message = (
            "Standing policy is ready"
            if response.success
            else "Robot has fallen; run Reset Microduck"
        )
        return response

    def tick(self):
        now = time.monotonic()
        if now - self.last_command > COMMAND_TIMEOUT:
            self.velocity[:] = 0
        self.sim.step(self.velocity, self.head_target)
        stamp = self.get_clock().now().to_msg()
        qpos = self.sim.data.qpos
        offset = self.sim.base_index
        rotation = Rotation.from_quat(np.roll(qpos[offset + 3 : offset + 7], -1))
        # The supplied description places trunk_base 105 mm above base_link.
        base_position = self.sim.position - rotation.apply([0, 0, 0.105])
        roll, pitch, yaw = rotation.as_euler("xyz")
        state = JointState()
        state.header.stamp = stamp
        state.name = BASE_JOINTS + self.sim.joint_names + ["jaw"]
        # The walking model welds the jaw closed; it has no jaw actuator.
        state.position = (
            list(base_position)
            + [yaw, pitch, roll]
            + qpos[self.sim.policy.joint_qpos_indices].tolist()
            + [0.0]
        )
        self.state_pub.publish(state)
        if now - self.last_image >= 0.1:
            for camera in self.cameras:
                self.publish_camera(stamp, *camera)
            self.last_image = now
        if now - self.last_status >= 1.0:
            self.status_pub.publish(
                String(
                    data=f"policy={self.policy_name} sim_time={self.sim.data.time:.2f} position={self.sim.position.tolist()}"
                )
            )
            self.last_status = now

    def publish_camera(self, stamp, camera_id, frame_id, image_pub, info_pub):
        self.renderer.update_scene(self.sim.data, camera=camera_id)
        pixels = self.renderer.render()
        message = Image()
        message.header.stamp = stamp
        message.header.frame_id = frame_id
        message.height, message.width = pixels.shape[:2]
        message.encoding = "rgb8"
        message.step = message.width * 3
        message.data = pixels.tobytes()
        image_pub.publish(message)
        info = CameraInfo()
        info.header = message.header
        info.height, info.width = message.height, message.width
        focal = (
            0.5
            * info.height
            / np.tan(np.deg2rad(self.sim.model.cam_fovy[camera_id]) / 2)
        )
        info.distortion_model = "plumb_bob"
        info.d = [0.0] * 5
        info.k = [
            focal,
            0.0,
            info.width / 2,
            0.0,
            focal,
            info.height / 2,
            0.0,
            0.0,
            1.0,
        ]
        info.r = np.eye(3).ravel().tolist()
        info.p = [
            focal,
            0.0,
            info.width / 2,
            0.0,
            0.0,
            focal,
            info.height / 2,
            0.0,
            0.0,
            0.0,
            1.0,
            0.0,
        ]
        info_pub.publish(info)
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = "world"
        transform.child_frame_id = message.header.frame_id
        position = self.sim.data.cam_xpos[camera_id]
        (
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z,
        ) = map(float, position)
        matrix = self.sim.data.cam_xmat[camera_id].reshape(3, 3) @ np.diag([1, -1, -1])
        quaternion = Rotation.from_matrix(matrix).as_quat()
        (
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w,
        ) = map(float, quaternion)
        self.tf.sendTransform(transform)


def main():
    rclpy.init()
    node = None
    try:
        node = MicroduckBridge()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.renderer.close()
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
