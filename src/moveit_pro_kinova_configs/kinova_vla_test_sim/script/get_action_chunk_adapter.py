#!/usr/bin/env python3
"""GetActionChunk adapter: serves ExecutePolicy from an inference process over HTTP.

Policy-agnostic: runs inside the MoveIt Pro dev container, forwards observations
to the host inference server at INFER_URL and returns chunks. Run with the
overlay sourced so moveit_pro_ml_msgs resolves.
"""

import base64
import os

import cv2
import numpy as np
import requests

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, MultiArrayDimension
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from moveit_pro_ml_msgs.srv import GetActionChunk

INFER_URL = os.environ.get("INFER_URL", "http://172.17.0.1:8973/infer")


def encode_jpeg_b64(img) -> str:
    """sensor_msgs/Image -> base64 JPEG. Accepts rgb8/bgr8/rgba8/bgra8 frames."""
    encoding = str(img.encoding).lower()
    if encoding not in ("rgb8", "bgr8", "rgba8", "bgra8"):
        raise ValueError(f"unsupported image encoding '{img.encoding}'")
    channels = 4 if encoding in ("rgba8", "bgra8") else 3
    # step is the row stride in bytes; slice off any row padding before reshaping.
    rows = np.frombuffer(bytes(img.data), dtype=np.uint8).reshape(img.height, img.step)
    arr = rows[:, : img.width * channels].reshape(img.height, img.width, channels)[
        :, :, :3
    ]
    # cv2.imencode expects BGR input, so bgr frames pass through and rgb frames flip once.
    if encoding.startswith("rgb"):
        arr = arr[:, :, ::-1]
    ok, buf = cv2.imencode(".jpg", np.ascontiguousarray(arr))
    if not ok:
        raise RuntimeError("cv2.imencode failed")
    return base64.b64encode(buf.tobytes()).decode("ascii")


class GetActionChunkAdapter(Node):
    def __init__(self) -> None:
        super().__init__("get_action_chunk_adapter")
        self.create_service(GetActionChunk, "/get_action_chunk", self._on_request)
        self._calls = 0

    def _on_request(self, request, response):
        # The request carries the policy's full trained state: the arm group's
        # joints plus the gripper joint appended last when the Objective configures one.
        state = list(request.robot_state.position)
        if len(request.images) != len(request.image_names):
            response.success = False
            response.message = (
                f"images ({len(request.images)}) and image_names "
                f"({len(request.image_names)}) length mismatch"
            )
            return response
        try:
            images = {
                request.image_names[i]: encode_jpeg_b64(img)
                for i, img in enumerate(request.images)
            }
        except (ValueError, RuntimeError) as exc:
            response.success = False
            response.message = f"image encode failed: {exc}"
            return response
        payload = {
            "state": state,
            "task": request.prompt,
            "images": images,
            "new_episode": bool(request.new_episode),
        }

        # RTC carryover: forward the previous chunk's unexecuted tail and the overlap depth.
        # A Float64MultiArray is a flat row-major buffer plus a layout, so reshape it back
        # into (steps, action width) rows; it arrives empty on the first call and when RTC is off.
        prev = request.previous_action_chunk
        if prev.data and len(prev.layout.dim) == 2:
            steps, width = prev.layout.dim[0].size, prev.layout.dim[1].size
            payload["prev_chunk_left_over"] = (
                np.asarray(prev.data, dtype=float).reshape(steps, width).tolist()
            )
            payload["inference_delay"] = int(request.frozen_prefix_steps)
        # A non-zero guidance_horizon is the Objective overriding the server's RTC window.
        if request.guidance_horizon > 0:
            payload["execution_horizon"] = int(request.guidance_horizon)

        try:
            resp = requests.post(INFER_URL, json=payload, timeout=30.0)
            resp.raise_for_status()
            data = resp.json()
        except (requests.RequestException, ValueError) as exc:
            response.success = False
            response.message = f"/infer request failed: {exc}"
            return response
        if isinstance(data, dict) and data.get("error"):
            response.success = False
            response.message = f"/infer error: {data['error']}"
            return response
        chunk = data.get("action_chunk") if isinstance(data, dict) else None
        if not chunk or "dt" not in data:
            response.success = False
            response.message = (
                "/infer response is missing a non-empty action_chunk or dt"
            )
            return response
        expected_dims = len(request.robot_state.name)
        mismatched_width = next(
            (len(step) for step in chunk if len(step) != expected_dims), None
        )
        if mismatched_width is not None:
            response.success = False
            response.message = (
                f"/infer chunk width {mismatched_width} does not match the observed "
                f"joint count {expected_dims}"
            )
            return response

        # The chunk: absolute joint positions. The action columns line up with the request's
        # state entries, so the request's joint names are also the chunk's joint names.
        traj = JointTrajectory()
        traj.joint_names = list(request.robot_state.name)
        for step in chunk:
            point = JointTrajectoryPoint()
            point.positions = [float(v) for v in step]
            traj.points.append(point)
        response.success = True
        response.chunk = traj
        response.native_control_period = float(data["dt"])

        # RTC echo: the normalized model output, one row per step, one column per action dimension.
        raw = data.get("action_chunk_raw")
        if raw:
            arr = Float64MultiArray()
            steps, width = len(raw), len(raw[0])
            arr.layout.data_offset = 0
            arr.layout.dim = [
                MultiArrayDimension(label="steps", size=steps, stride=steps * width),
                MultiArrayDimension(label="dims", size=width, stride=width),
            ]
            arr.data = np.asarray(raw, dtype=float).ravel().tolist()
            response.policy_action_chunk = arr

        self._calls += 1
        if self._calls % 10 == 0:
            self.get_logger().info(f"get_action_chunk: served {self._calls} chunks")
        return response


def main() -> None:
    rclpy.init()
    node = GetActionChunkAdapter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
