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

"""GetActionChunk adapter: bridges ExecutePolicy to the inference server.

Serves moveit_pro_ml_msgs/srv/GetActionChunk inside the MoveIt Pro agent
container and forwards each request over HTTP to the `inference_server`
container (docker/vla_inference_server.py), which owns torch and the
checkpoint. Requests carry the deployment's shared MOVEIT_FRONTEND_KEY as a
bearer token, the contract the server enforces on /infer. Policy-agnostic and
lightweight: no ML dependencies, so it always runs with the config.

A failed request answers with status ERROR, which fails the run, and the
response message is what ExecutePolicy shows the operator as the reason. The
messages therefore distinguish "server not running" from "model still
loading" from a server-reported inference error.
"""

import base64
import ipaddress
import os
from urllib.parse import urlsplit, urlunsplit

import cv2
import numpy as np
import requests

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, MultiArrayDimension
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from moveit_pro_ml_msgs.srv import GetActionChunk

DEFAULT_INFER_URL = "http://127.0.0.1:8973/infer"

# Bound on server-supplied text relayed into the UI-bound response message.
MAX_SERVER_DETAIL_CHARS = 2000


def clip_detail(text: str) -> str:
    """Clip server-supplied text so a runaway error body cannot flood the UI."""
    if len(text) <= MAX_SERVER_DETAIL_CHARS:
        return text
    return text[:MAX_SERVER_DETAIL_CHARS] + " [...]"


def resolve_http_timeout(value: float) -> float:
    """Reject a non-positive or non-finite http_timeout at startup, not per call."""
    if not np.isfinite(value) or value <= 0.0:
        raise ValueError(
            f"the http_timeout parameter must be a positive number of "
            f"seconds, got {value!r}"
        )
    return float(value)


def resolve_infer_url(value: str) -> str:
    """Require infer_url to address a server on this machine.

    Requests to it carry the deployment's shared key, and the server speaks
    plain HTTP, so a host off this machine would put that key on the wire in
    cleartext. Every supported layout is still reachable: the agent container
    runs with host networking, so loopback here reaches the product sidecar,
    another container publishing to loopback, or a bare host process.

    Only a literal loopback address is accepted. A name resolves at connect
    time, not here, so a check on `localhost` would be a check on whatever the
    resolver returns later. Only http is accepted too: the server listens
    without TLS, so an https target could not complete a handshake with it.

    @return: The address to call, rebuilt from the parts that were checked.
    """
    parts = urlsplit(value)
    if parts.scheme != "http":
        raise ValueError(
            f"the infer_url parameter must be an http URL: the inference server "
            f"listens without TLS, and the target stays on this machine, so there "
            f"is no network hop to encrypt. Got {value!r}"
        )
    # `urlsplit` and the HTTP client disagree about where the authority ends:
    # `urlsplit` reads `evil.example\@127.0.0.1` as credentials followed by a
    # loopback host, while the client stops at the backslash and connects to
    # evil.example. Credentials have no use here, since the key travels in a
    # header, so rejecting both characters leaves them nothing to disagree on.
    if "@" in parts.netloc or "\\" in parts.netloc:
        raise ValueError(
            f"the infer_url parameter must be a plain scheme://host:port URL "
            f"carrying no credentials, got {value!r}"
        )
    try:
        host = ipaddress.ip_address(parts.hostname or "")
    except ValueError:
        raise ValueError(
            f"the infer_url parameter must address a loopback IP literal such as "
            f"{DEFAULT_INFER_URL}, got {value!r}"
        ) from None
    if not host.is_loopback:
        raise ValueError(
            f"the infer_url parameter must stay on this machine: {host} is not a "
            f"loopback address. Serving the policy from another host is not "
            f"supported"
        )
    try:
        port = parts.port
    except ValueError:
        raise ValueError(
            f"the infer_url parameter has an invalid port, got {value!r}"
        ) from None
    # Rebuilt rather than returned as given: `urlsplit` drops tab and newline
    # characters before parsing, so the original string can still carry bytes
    # the client would read differently. Any fragment is dropped with them,
    # since it is never sent to the server.
    literal = f"[{host}]" if host.version == 6 else str(host)
    authority = literal if port is None else f"{literal}:{port}"
    return urlunsplit((parts.scheme, authority, parts.path, parts.query, ""))


class RequestError(Exception):
    """Operator-facing failure; the message becomes the service response message."""


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
        self.infer_url = resolve_infer_url(
            self.declare_parameter("infer_url", DEFAULT_INFER_URL).value
        )
        # Total HTTP budget, split into connect + read at the call site. Keep
        # it strictly below the caller's service timeout (ExecutePolicy's
        # policy_call_timeout, 10.0 in stack_cubes_with_the_vla_policy.xml): a hung server
        # must not wedge this single-threaded node past the point the caller
        # has already given up, or the next run's first request queues behind
        # the stale call.
        self.http_timeout = resolve_http_timeout(
            self.declare_parameter("http_timeout", 9.0).value
        )
        # The server authenticates /infer with the deployment's shared key
        # (same contract as the web backend endpoints). The agent container
        # always carries it; when it is absent the server's 401 detail flows
        # into the objective's on-screen message, so no local check is needed.
        self._frontend_key = os.environ.get("MOVEIT_FRONTEND_KEY", "").strip()
        service_name = self.declare_parameter("service_name", "/get_action_chunk").value
        self.create_service(GetActionChunk, service_name, self._on_request)
        self._calls = 0
        self.get_logger().info(f"serving '{service_name}' -> {self.infer_url}")

    def _on_request(self, request, response):
        try:
            self._fill_response(request, response)
        except RequestError as exc:
            response.status = GetActionChunk.Response.ERROR
            response.message = str(exc)
            self.get_logger().error(response.message)
        except Exception as exc:
            response.status = GetActionChunk.Response.ERROR
            response.message = f"adapter failed: {type(exc).__name__}: {exc}"
            self.get_logger().error(response.message)
        return response

    def _build_payload(self, request) -> dict:
        """Turn the service request into the server's JSON payload."""
        if len(request.images) != len(request.image_names):
            raise RequestError(
                f"images ({len(request.images)}) and image_names "
                f"({len(request.image_names)}) length mismatch"
            )
        # The payload state is built from positions while the returned chunk is
        # validated and labeled with the joint names, so the two must agree.
        if len(request.robot_state.name) != len(request.robot_state.position):
            raise RequestError(
                f"robot_state carries {len(request.robot_state.position)} positions "
                f"but {len(request.robot_state.name)} joint names"
            )
        try:
            images = {
                request.image_names[i]: encode_jpeg_b64(img)
                for i, img in enumerate(request.images)
            }
        except (ValueError, RuntimeError) as exc:
            raise RequestError(f"image encode failed: {exc}") from exc
        payload = {
            # The request carries the policy's full trained state: the arm group's
            # joints plus the gripper joint appended last when the Objective configures one.
            "state": list(request.robot_state.position),
            "task": request.prompt,
            "images": images,
            "new_episode": bool(request.new_episode),
        }

        # RTC carryover: forward the previous chunk's unexecuted tail and the overlap depth.
        # A Float64MultiArray is a flat row-major buffer plus a layout, so reshape it back
        # into (steps, action width) rows; it arrives empty on the first call and when RTC is off.
        # previous_anchor_state is intentionally not forwarded: the carryover stays in the
        # policy's own action space, where lerobot's RTC guidance operates without re-anchoring.
        prev = request.previous_action_chunk
        if prev.data:
            if len(prev.layout.dim) != 2:
                raise RequestError(
                    f"previous_action_chunk carries {len(prev.data)} values but "
                    f"its layout declares {len(prev.layout.dim)} dimensions "
                    "instead of the required 2 (steps, action width)"
                )
            steps, width = prev.layout.dim[0].size, prev.layout.dim[1].size
            if steps * width != len(prev.data):
                raise RequestError(
                    f"previous_action_chunk carries {len(prev.data)} values but "
                    f"its layout declares {steps}x{width} = {steps * width}"
                )
            payload["prev_chunk_left_over"] = (
                np.asarray(prev.data, dtype=float).reshape(steps, width).tolist()
            )
            payload["inference_delay"] = int(request.frozen_prefix_steps)
        # A non-zero guidance_horizon is the Objective overriding the server's
        # soft-guidance width; the server maps it onto lerobot's RTC horizon.
        if request.guidance_horizon > 0:
            payload["guidance_horizon"] = int(request.guidance_horizon)
        return payload

    def _post_infer(self, payload: dict) -> dict:
        """POST to the inference server; return the parsed response body."""
        # A local container either accepts immediately or is down, so connect
        # gets a small slice and the read keeps the rest. The read element is
        # a between-bytes timeout, so the split bounds the hung-server case,
        # not a slowly trickling body.
        connect_s = min(3.0, self.http_timeout / 3.0)
        headers = (
            {"Authorization": f"Bearer {self._frontend_key}"}
            if self._frontend_key
            else {}
        )
        try:
            resp = requests.post(
                self.infer_url,
                json=payload,
                headers=headers,
                timeout=(connect_s, self.http_timeout - connect_s),
            )
        except requests.ConnectionError as exc:
            raise RequestError(
                f"/infer request failed: the inference server at {self.infer_url} "
                f"is not reachable ({exc}). Was MoveIt Pro started with "
                "--with-inference-server (or the server started with "
                "--only-inference-server)? Check with "
                "'docker ps --filter name=inference_server'."
            ) from exc
        except requests.RequestException as exc:
            raise RequestError(
                f"/infer request failed: {clip_detail(str(exc))}"
            ) from exc
        # Parse the body before checking the status code: the server reports
        # problems as {"error": ...} bodies (load/inference failures with 5xx,
        # request-shape rejections with 4xx), and that detail (e.g. "still
        # loading the model") is the message the operator needs to see.
        try:
            data = resp.json()
        except ValueError:
            data = None
        if isinstance(data, dict) and data.get("error"):
            raise RequestError(f"/infer error: {clip_detail(str(data['error']))}")
        if not resp.ok or not isinstance(data, dict):
            raise RequestError(f"/infer request failed: HTTP {resp.status_code}")
        return data

    @staticmethod
    def _validate_chunk(data: dict, expected_dims: int) -> list:
        """Check the returned chunk's shape, values, and dt; return the chunk."""
        chunk = data.get("action_chunk")
        if not chunk or "dt" not in data:
            raise RequestError(
                "/infer response is missing a non-empty action_chunk or dt"
            )
        if not isinstance(chunk, list) or not all(
            isinstance(step, list) for step in chunk
        ):
            raise RequestError(
                "/infer response's action_chunk is not a steps x dims array"
            )
        dt = data["dt"]
        if not isinstance(dt, (int, float)) or not np.isfinite(dt) or dt <= 0.0:
            raise RequestError(
                f"/infer response carries an invalid dt ({dt!r}); playback "
                "pacing needs a finite value > 0"
            )
        mismatched_width = next(
            (len(step) for step in chunk if len(step) != expected_dims), None
        )
        if mismatched_width is not None:
            raise RequestError(
                f"/infer chunk width {mismatched_width} does not match the observed "
                f"joint count {expected_dims}"
            )
        try:
            chunk_arr = np.asarray(chunk, dtype=float)
        except (TypeError, ValueError) as exc:
            raise RequestError(
                f"/infer chunk carries non-numeric action values ({exc})"
            ) from exc
        if not np.isfinite(chunk_arr).all():
            raise RequestError("/infer chunk carries non-finite action values")
        return chunk

    def _fill_response(self, request, response) -> None:
        payload = self._build_payload(request)
        data = self._post_infer(payload)
        chunk = self._validate_chunk(data, len(request.robot_state.name))

        # The chunk: absolute joint positions. The action columns line up with the request's
        # state entries, so the request's joint names are also the chunk's joint names.
        traj = JointTrajectory()
        traj.joint_names = list(request.robot_state.name)
        for step in chunk:
            point = JointTrajectoryPoint()
            point.positions = [float(v) for v in step]
            traj.points.append(point)
        response.chunk = traj
        response.native_control_period = float(data["dt"])

        # RTC echo: the normalized model output, one row per step, one column per action
        # dimension; the caller returns its unexecuted tail as the next previous_action_chunk.
        # An absent or empty echo means the policy offers no RTC carryover, so
        # the next request simply arrives without one.
        raw = data.get("action_chunk_raw")
        if raw:
            if (
                not isinstance(raw, list)
                or not all(
                    isinstance(row, list) and len(row) == len(raw[0]) for row in raw
                )
                or not raw[0]
            ):
                raise RequestError(
                    "/infer response's action_chunk_raw is not a non-empty "
                    "rectangular steps x dims array"
                )
            try:
                raw_arr = np.asarray(raw, dtype=float)
            except (TypeError, ValueError) as exc:
                raise RequestError(
                    f"/infer response's action_chunk_raw carries non-numeric "
                    f"values ({exc})"
                ) from exc
            if not np.isfinite(raw_arr).all():
                raise RequestError(
                    "/infer response's action_chunk_raw carries non-finite values"
                )
            arr = Float64MultiArray()
            steps, width = raw_arr.shape
            arr.layout.data_offset = 0
            arr.layout.dim = [
                MultiArrayDimension(label="steps", size=steps, stride=steps * width),
                MultiArrayDimension(label="dims", size=width, stride=width),
            ]
            arr.data = raw_arr.ravel().tolist()
            response.policy_action_chunk = arr

        # Set last: a malformed raw echo above must fail the whole request, not
        # report a produced chunk with the echo missing.
        response.status = GetActionChunk.Response.CHUNK_PRODUCED
        self._calls += 1
        if self._calls % 10 == 0:
            self.get_logger().info(f"get_action_chunk: served {self._calls} chunks")


def main() -> None:
    rclpy.init()
    node = GetActionChunkAdapter()
    try:
        rclpy.spin(node)
    # rclpy.init() installs the signal handlers, so a stack shutdown arrives as
    # ExternalShutdownException rather than KeyboardInterrupt, and it has
    # already torn the context down: try_shutdown is the idempotent form.
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
