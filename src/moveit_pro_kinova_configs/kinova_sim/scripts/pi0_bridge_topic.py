"""
pi0 streaming bridge — Phase 3 (JVC variant).

Subscribes to MoveIt Pro's MuJoCo Kinova sim sensor topics, queries the
OpenPI policy server, and STREAMS the resulting per-joint velocities to the
joint_velocity_controller (JVC) as control_msgs/JointJog messages on
/joint_velocity_controller/command.

Why JVC and not ExecuteTrajectory/JTAC: pi0 is a streaming policy (a fresh
action chunk every ~90ms). The discrete ExecuteTrajectory->JTAC path fights
that with per-goal acceptance + trajectory stitching + stamp guards (proven
dead end). JVC is a real-time streaming controller: keep publishing JointJog
velocities and the arm tracks them. The served pi05_droid checkpoint uses the
DROID JOINT_VELOCITY action space -> the policy emits joint velocities (rad/s)
DIRECTLY, so we stream them straight to JVC. No IK, no Cartesian round-trip,
and NO dt division (an earlier version wrongly treated the action as a
position-delta and divided by CONTROL_DT, inflating it ~5x into permanent clamp
saturation -- that was the "divergent wander" bug; verified 2026-06-07).

CRITICAL: JVC drops commands whose header.stamp is stale (command_timeout
0.2s). Every message MUST be stamped with the live clock, and we must publish
faster than 5Hz. (This is the OPPOSITE of the old JTAC direct-admittance path,
which wanted stamp=0.)

Run:
  1. moveit_pro run  (kinova_sim)
  2. cd ~/spike/openpi && uv run scripts/serve_policy.py --env DROID
  3. Start "Run Pi0 Policy" objective in MoveIt Pro UI  (activates JVC)
  4. python3 pi0_bridge_topic.py --prompt "pick up the red cube"
"""

import argparse
import logging
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

KINOVA_ARM_JOINTS = [
    "joint_1", "joint_2", "joint_3",
    "joint_4", "joint_5", "joint_6", "joint_7",
]

POLICY_URI  = "ws://localhost:8000"
COMMAND_TOPIC = "/joint_velocity_controller/command"
VEL_SCALE   = 1.0    # scale on the policy's velocity output (pi05_droid emits joint velocities directly, rad/s)
VEL_CLAMP   = 0.5    # rad/s, GUARDED cap (JVC max_joint_velocity ~[0.7,0.7,0.7,0.7,0.6,0.6,0.6])
PUBLISH_HZ  = 10.0   # must be > 5Hz so JVC's 0.2s command_timeout never trips between publishes


def resize_image_uint8(img: np.ndarray, h: int = 224, w: int = 224) -> np.ndarray:
    if img.dtype != np.uint8:
        img = img.astype(np.uint8)
    src_h, src_w = img.shape[:2]
    scale = min(h / src_h, w / src_w)
    nh, nw = max(1, int(round(src_h * scale))), max(1, int(round(src_w * scale)))
    yi = (np.arange(nh) * (src_h / nh)).astype(np.int64)
    xi = (np.arange(nw) * (src_w / nw)).astype(np.int64)
    resized = img[yi[:, None], xi[None, :]]
    out = np.zeros((h, w, 3), dtype=np.uint8)
    py, px = (h - nh) // 2, (w - nw) // 2
    out[py:py + nh, px:px + nw] = resized
    return out


def ros_image_to_numpy(msg: Image) -> np.ndarray:
    arr = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    img = arr.reshape(msg.height, msg.width, 3)
    if msg.encoding == "bgr8":
        return img[:, :, ::-1].copy()
    return img


class Pi0BridgeTopic(Node):

    def __init__(self, prompt: str) -> None:
        super().__init__("pi0_bridge_topic")
        self.prompt = prompt
        self.scene_rgb: Optional[np.ndarray] = None
        self.wrist_rgb: Optional[np.ndarray] = None
        self.joint_positions: Optional[np.ndarray] = None

        cam_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                             history=HistoryPolicy.KEEP_LAST, depth=1)
        jnt_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                             history=HistoryPolicy.KEEP_LAST, depth=1)

        self.create_subscription(Image, "/scene_camera/color", self._on_scene, cam_qos)
        self.create_subscription(Image, "/wrist_camera/color", self._on_wrist, cam_qos)
        self.create_subscription(JointState, "/joint_states", self._on_joints, jnt_qos)

        # Stream joint velocities straight to the JVC command topic.
        # Default-ish QoS (RELIABLE, KEEP_LAST 10) — matches the ros2_control
        # command subscription and what the smoke test used.
        self.cmd_pub = self.create_publisher(JointJog, COMMAND_TOPIC, 10)

        self.ws = None
        self.packer = None

    def _on_scene(self, msg: Image) -> None:
        self.scene_rgb = ros_image_to_numpy(msg)

    def _on_wrist(self, msg: Image) -> None:
        self.wrist_rgb = ros_image_to_numpy(msg)

    def _on_joints(self, msg: JointState) -> None:
        by_name = dict(zip(msg.name, msg.position))
        if all(j in by_name for j in KINOVA_ARM_JOINTS):
            self.joint_positions = np.array(
                [by_name[j] for j in KINOVA_ARM_JOINTS], dtype=np.float32)

    def connect(self) -> None:
        self.get_logger().info(f"Connecting to policy server at {POLICY_URI} ...")
        while True:
            try:
                self.ws = websockets.sync.client.connect(
                    POLICY_URI, compression=None, max_size=None,
                    ping_interval=None, close_timeout=5, open_timeout=15)
                self.ws.recv()  # discard metadata frame
                self.packer = msgpack_numpy.Packer()
                break
            except Exception as e:
                self.get_logger().warn(f"Policy server not ready ({e}), retrying in 10s...")
                time.sleep(10)
        self.get_logger().info("Connected. Warming policy...")
        t0 = time.time()
        self._query_policy()  # JIT compile
        self.get_logger().info(f"Warm in {time.time()-t0:.1f}s. Streaming to {COMMAND_TOPIC}.")

    def _query_policy(self) -> np.ndarray:
        scene = resize_image_uint8(self.scene_rgb)
        wrist = resize_image_uint8(self.wrist_rgb)
        obs = {
            "observation/exterior_image_1_left": scene,
            "observation/wrist_image_left":      wrist,
            "observation/joint_position":        self.joint_positions,
            "observation/gripper_position":      np.zeros(1, dtype=np.float32),
            "prompt":                            self.prompt,
        }
        self.ws.send(self.packer.pack(obs))
        resp = self.ws.recv()
        if isinstance(resp, str):
            raise RuntimeError(f"Policy server error:\n{resp}")
        result = msgpack_numpy.unpackb(resp)
        return np.asarray(result["actions"], dtype=np.float64)

    def run(self) -> None:
        # Wait for all sensors
        self.get_logger().info("Waiting for sensors...")
        while self.scene_rgb is None or self.wrist_rgb is None or self.joint_positions is None:
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info("Sensors live.")

        self.connect()

        period = 1.0 / PUBLISH_HZ
        step = 0
        while rclpy.ok():
            loop_t0 = time.time()
            rclpy.spin_once(self, timeout_sec=0.01)

            t0 = time.time()
            try:
                chunk = self._query_policy()
            except Exception as e:
                self.get_logger().error(f"Policy error: {e}")
                break
            query_ms = (time.time() - t0) * 1000

            # pi05_droid returns a (15, 8) chunk of FUTURE joint velocities
            # (7 arm joints + gripper) in rad/s. Stream the FIRST (most immediate)
            # step's joint velocities DIRECTLY -- the action IS a velocity, so no
            # dt division. GUARDED-clamp to VEL_CLAMP for safety.
            vel = np.clip(chunk[0, :7] * VEL_SCALE, -VEL_CLAMP, VEL_CLAMP)

            msg = JointJog()
            msg.header.stamp = self.get_clock().now().to_msg()   # LIVE stamp — JVC drops stale commands
            msg.header.frame_id = "base_link"
            msg.joint_names = KINOVA_ARM_JOINTS
            msg.velocities = vel.tolist()
            msg.duration = 0.0
            self.cmd_pub.publish(msg)

            self.get_logger().info(
                f"step={step} query={query_ms:.0f}ms "
                f"vel={np.round(vel, 3).tolist()}")

            step += 1
            # Hold a steady publish rate (must stay under JVC's 0.2s timeout).
            dt = time.time() - loop_t0
            if dt < period:
                time.sleep(period - dt)

    def stop_arm(self) -> None:
        """Best-effort immediate halt: publish one zero-velocity command.

        Not the primary safety mechanism — JVC's command_timeout (0.2s) already
        halts the arm as soon as we stop streaming. So if the rclpy context is
        already torn down (e.g. SIGTERM/timeout), just skip; the timeout covers it.
        """
        if not rclpy.ok():
            return
        try:
            msg = JointJog()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "base_link"
            msg.joint_names = KINOVA_ARM_JOINTS
            msg.velocities = [0.0] * len(KINOVA_ARM_JOINTS)
            msg.duration = 0.0
            self.cmd_pub.publish(msg)
        except Exception:
            pass

    def shutdown(self) -> None:
        self.stop_arm()
        if self.ws:
            try:
                self.ws.close()
            except Exception:
                pass


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[1].strip())
    parser.add_argument("--prompt", required=True)
    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO)
    rclpy.init()
    node = Pi0BridgeTopic(prompt=args.prompt)
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
