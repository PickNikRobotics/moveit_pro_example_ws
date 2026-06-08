"""
pi0 DIAGNOSTIC — determine the DROID action space and test the position-target fix.

Two modes:
  --mode observe : query policy, LOG raw chunk vs current joints, publish ZERO
                   velocity (arm holds still). Confirms JOINT_POSITION vs
                   VELOCITY/DELTA without moving the arm.
  --mode track   : apply the position-target control law
                   vel = clip((target - current)/dt, ±clamp) and stream it,
                   to test whether motion becomes sensible (drives toward target,
                   not saturating).

Reuses the live-stamp + JVC command topic from the production bridge.
"""
import argparse
import time
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, JointState
from control_msgs.msg import JointJog

import websockets.sync.client
from openpi_client import msgpack_numpy

KINOVA_ARM_JOINTS = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "joint_7"]
POLICY_URI = "ws://localhost:8000"
COMMAND_TOPIC = "/joint_velocity_controller/command"
PUBLISH_HZ = 10.0


def resize_image_uint8(img, h=224, w=224):
    if img.dtype != np.uint8:
        img = img.astype(np.uint8)
    sh, sw = img.shape[:2]
    s = min(h / sh, w / sw)
    nh, nw = max(1, int(round(sh * s))), max(1, int(round(sw * s)))
    yi = (np.arange(nh) * (sh / nh)).astype(np.int64)
    xi = (np.arange(nw) * (sw / nw)).astype(np.int64)
    r = img[yi[:, None], xi[None, :]]
    out = np.zeros((h, w, 3), dtype=np.uint8)
    py, px = (h - nh) // 2, (w - nw) // 2
    out[py:py + nh, px:px + nw] = r
    return out


def ros_image_to_numpy(msg):
    arr = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    img = arr.reshape(msg.height, msg.width, 3)
    if msg.encoding == "bgr8":
        return img[:, :, ::-1].copy()
    return img


class Diag(Node):
    def __init__(self, prompt, mode, dt, clamp, scale=1.0):
        super().__init__("pi0_diag")
        self.prompt = prompt
        self.mode = mode
        self.dt = dt
        self.clamp = clamp
        self.scale = scale
        self.scene = None
        self.wrist = None
        self.joints = None
        cam_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
        jnt_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(Image, "/scene_camera/color", self._scene, cam_qos)
        self.create_subscription(Image, "/wrist_camera/color", self._wrist, cam_qos)
        self.create_subscription(JointState, "/joint_states", self._jnt, jnt_qos)
        self.cmd = self.create_publisher(JointJog, COMMAND_TOPIC, 10)
        self.ws = None
        self.packer = None

    def _scene(self, m): self.scene = ros_image_to_numpy(m)
    def _wrist(self, m): self.wrist = ros_image_to_numpy(m)

    def _jnt(self, m):
        d = dict(zip(m.name, m.position))
        if all(j in d for j in KINOVA_ARM_JOINTS):
            self.joints = np.array([d[j] for j in KINOVA_ARM_JOINTS], dtype=np.float32)

    def connect(self):
        self.get_logger().info(f"Connecting {POLICY_URI} ...")
        self.ws = websockets.sync.client.connect(POLICY_URI, compression=None, max_size=None,
                                                  ping_interval=None, close_timeout=5, open_timeout=15)
        self.ws.recv()
        self.packer = msgpack_numpy.Packer()
        self.get_logger().info("Connected. Warming...")
        self.query()
        self.get_logger().info("Warm.")

    def query(self):
        obs = {
            "observation/exterior_image_1_left": resize_image_uint8(self.scene),
            "observation/wrist_image_left": resize_image_uint8(self.wrist),
            "observation/joint_position": self.joints,
            "observation/gripper_position": np.zeros(1, dtype=np.float32),
            "prompt": self.prompt,
        }
        self.ws.send(self.packer.pack(obs))
        resp = self.ws.recv()
        if isinstance(resp, str):
            raise RuntimeError(resp)
        return np.asarray(msgpack_numpy.unpackb(resp)["actions"], dtype=np.float64)

    def publish(self, vel):
        msg = JointJog()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.joint_names = KINOVA_ARM_JOINTS
        msg.velocities = vel.tolist()
        msg.duration = 0.0
        self.cmd.publish(msg)

    def run(self, steps):
        self.get_logger().info("Waiting for sensors...")
        while self.scene is None or self.wrist is None or self.joints is None:
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info("Sensors live.")
        self.connect()
        period = 1.0 / PUBLISH_HZ
        for step in range(steps):
            t0 = time.time()
            rclpy.spin_once(self, timeout_sec=0.01)
            chunk = self.query()
            cur = self.joints.copy()
            target = chunk[0, :7]
            if step == 0:
                self.get_logger().info(f"chunk.shape={chunk.shape}  chunk[0](all 8 dims)={np.round(chunk[0],4).tolist()}")
            # ANALYSIS: compare action[0,:7] to current joints
            diff = target - cur
            old_law = np.clip(target / 0.2, -self.clamp, self.clamp)            # what the prod bridge does
            pos_law = np.clip(diff / self.dt, -self.clamp, self.clamp)          # position-target fix
            self.get_logger().info(
                f"step={step}\n"
                f"  raw target  ={np.round(target,3).tolist()}\n"
                f"  current     ={np.round(cur,3).tolist()}\n"
                f"  target-cur  ={np.round(diff,3).tolist()}  |mean|={np.abs(diff).mean():.3f}\n"
                f"  OLD vel(t/0.2)={np.round(old_law,3).tolist()}\n"
                f"  POS vel(d/dt) ={np.round(pos_law,3).tolist()}")
            direct_law = np.clip(target * self.scale, -self.clamp, self.clamp)  # action IS velocity
            if self.mode == "observe":
                self.publish(np.zeros(7))          # hold still
            elif self.mode == "track":
                self.publish(pos_law)              # position-target control (WRONG if action is velocity)
            else:  # direct
                self.publish(direct_law)           # command the action as velocity directly
            dt = time.time() - t0
            if dt < period:
                time.sleep(period - dt)
        self.publish(np.zeros(7))
        self.get_logger().info("Done.")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--prompt", required=True)
    ap.add_argument("--mode", choices=["observe", "track", "direct"], default="observe")
    ap.add_argument("--scale", type=float, default=1.0, help="scale on raw action in direct mode")
    ap.add_argument("--steps", type=int, default=20)
    ap.add_argument("--dt", type=float, default=0.5, help="horizon for (target-current)/dt in track mode")
    ap.add_argument("--clamp", type=float, default=0.3)
    a = ap.parse_args()
    rclpy.init()
    n = Diag(a.prompt, a.mode, a.dt, a.clamp, a.scale)
    try:
        n.run(a.steps)
    finally:
        try:
            n.publish(np.zeros(7))
        except Exception:
            pass
        rclpy.shutdown()


if __name__ == "__main__":
    main()
