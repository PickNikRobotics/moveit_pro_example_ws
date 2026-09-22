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

Serves moveit_pro_ml_msgs/srv/GetActionChunk and forwards each request as one
HTTP POST, over loopback plaintext or verified TLS to an external inference
host. Each request carries MOVEIT_INFERENCE_KEY as a bearer token.
`moveit_pro run` derives that key from the deployment's frontend key, and it
has no authority over the Runtime's REST, MCP, or web-bridge endpoints.
Policy-agnostic and lightweight: no ML dependencies, so it always runs with
the config.

A failed request answers with status ERROR, which fails the run, and the
response message is what ExecutePolicy shows the operator as the reason. The
messages therefore distinguish "server not running" from "model still
loading" from a server-reported inference error.
"""

import base64
import ipaddress
import json
import os
import socket
import sys
import threading
import time
from urllib.parse import urlsplit, urlunsplit

import cv2
import numpy as np
import requests
import urllib3

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, MultiArrayDimension
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from moveit_pro_ml_msgs.srv import GetActionChunk

DEFAULT_INFER_URL = "http://127.0.0.1:8973/infer"

# Bound on server-supplied text relayed into the UI-bound response message.
MAX_SERVER_DETAIL_CHARS = 2000
# Ceiling on an /infer response body. A 50-step, 7-joint chunk with its RTC
# echo measures 14 KB and a 200-step, 32-joint one 258 KB, so this leaves room
# for a policy well past anything shipped.
MAX_RESPONSE_BYTES = 1024 * 1024
# Read size for the streamed body. The socket enforces the deadline, so this
# is sized for throughput. A full 1 MiB response reads in about 3ms.
READ_CHUNK_BYTES = 64 * 1024


class DeadlineAdapter(requests.adapters.HTTPAdapter):
    """One total budget across connecting and waiting for a response.

    `requests` applies its connect timeout per resolved address, so a name
    carrying several dead addresses can spend that timeout several times over.
    With two dead addresses ahead of a live one, a 9s budget took 12.01s.
    urllib3 subtracts the time already spent connecting from what the read is
    allowed, which holds the total however many addresses the name carries.
    """

    def send(self, request, **kwargs):
        """Re-express the connect/read pair as one total budget."""
        timeout = kwargs.get("timeout")
        if isinstance(timeout, tuple):
            connect_s, read_s = timeout
            kwargs["timeout"] = urllib3.Timeout(
                total=connect_s + read_s, connect=connect_s, read=read_s
            )
        return super().send(request, **kwargs)


def response_socket(resp):
    """Return the socket under a streaming urllib3 response (a private path), or None."""
    try:
        return resp.raw._fp.fp.raw._sock
    except AttributeError:
        return None


def shutdown_quietly(sock) -> None:
    """Shut a socket down to wake a blocked read, ignoring an already-closed one."""
    try:
        sock.shutdown(socket.SHUT_RDWR)
    except OSError:
        pass


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
    """Accept external HTTPS /infer endpoints or literal-loopback HTTP."""
    parts = urlsplit(value)
    if parts.scheme == "https":
        try:
            port = parts.port
        except ValueError:
            raise ValueError(
                f"the infer_url parameter has an invalid port, got {value!r}"
            ) from None
        if (
            not parts.hostname
            or parts.username is not None
            or "\\" in value
            or any(ord(char) <= 32 or ord(char) == 127 for char in value)
            or parts.path != "/infer"
            or "?" in value
            or "#" in value
            or port == 0
        ):
            raise ValueError(
                "infer_url must be https://<host>[:port]/infer with no "
                "credentials, query, or fragment; got host "
                f"{parts.hostname!r}, path {parts.path!r}."
            )
        host = parts.hostname
        authority = f"[{host}]" if ":" in host else host
        if port is not None:
            authority += f":{port}"
        return urlunsplit(("https", authority, "/infer", "", ""))
    if parts.scheme != "http":
        raise ValueError(
            "infer_url must use https, or http with a literal loopback address; "
            f"got scheme {parts.scheme!r}, host {parts.hostname!r}."
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
            "loopback address. Use HTTPS for an externally managed server."
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


def _finite_2d(value, field: str) -> np.ndarray:
    """Convert a response field to a finite steps x dims array, naming it on failure."""
    try:
        arr = np.asarray(value, dtype=float)
    except (TypeError, ValueError) as exc:
        raise RequestError(
            f"/infer response's {field} is not a rectangular array of numbers "
            f"({clip_detail(str(exc))})"
        ) from exc
    if arr.ndim != 2:
        raise RequestError(
            f"/infer response's {field} is {arr.ndim}-dimensional; it must be a "
            "two-dimensional steps x dims array"
        )
    if arr.shape[1] == 0:
        raise RequestError(f"/infer response's {field} has empty rows")
    if not np.isfinite(arr).all():
        raise RequestError(f"/infer response's {field} carries non-finite values")
    return arr


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
        # Total HTTP budget, split into connect + read at the call site. An
        # Objective's policy_call_timeout must be larger than this value. That
        # port defaults to 3.0, below this 9.0 default, so every Objective
        # using this adapter must set it explicitly. This node is
        # single-threaded, so a call that outlives its caller leaves the next
        # run's first request queued behind a stale one.
        self.http_timeout = resolve_http_timeout(
            self.declare_parameter("http_timeout", 9.0).value
        )
        remote = self.infer_url.startswith("https:")
        # One session for the node's lifetime, so the node reuses one
        # connection to a remote server instead of a fresh TCP and TLS
        # handshake per inference call. trust_env stays off, so observations
        # and the bearer token go to infer_url or nowhere, never through a
        # proxy or netrc entry.
        self._session = requests.Session()
        self._session.mount("https://", DeadlineAdapter())
        self._session.mount("http://", DeadlineAdapter())
        self._session.trust_env = False
        self._session.headers["Accept-Encoding"] = "identity"
        if remote:
            ca_file = os.environ.get("MOVEIT_INFERENCE_CA_FILE") or ""
            if ca_file:
                if not os.path.isfile(ca_file) or not os.access(ca_file, os.R_OK):
                    raise ValueError(
                        f"The inference CA certificate at {ca_file!r} inside "
                        "the Runtime container is not a readable file. Set "
                        "MOVEIT_INFERENCE_CA_FILE on the host to the absolute "
                        "path of a readable PEM file, which Compose bind-mounts "
                        "there, or unset it when the certificate's issuer is "
                        "publicly trusted."
                    )
                # trust_env is off, so requests uses this session value.
                self._session.verify = ca_file
            self._auth_hint = (
                "Provision the output of `moveit_pro inference-key` as "
                "MOVEIT_INFERENCE_KEY on the inference host."
            )
            self._unreachable_hint = (
                "Check that the inference host is up, that its TLS listener is "
                "reachable from this machine, and that infer_url names the host "
                "its certificate covers."
            )
        else:
            self._auth_hint = (
                "Restart the inference server with `moveit_pro run "
                "--only-inference-server` so it holds this Runtime's "
                "MOVEIT_INFERENCE_KEY, and check that a custom server requires "
                "that variable."
            )
            self._unreachable_hint = (
                "Was MoveIt Pro started with --with-inference-server (or the "
                "server started with --only-inference-server)? Check with "
                "'docker ps --filter name=inference_server'."
            )
        # An absent key is not fatal. The request goes out without a token, and
        # the server's 401 reaches the operator through the rejection message in
        # _post_infer, with a hint that names the side missing the key.
        key = os.environ.get("MOVEIT_INFERENCE_KEY", "").strip()
        if key:
            self._session.headers["Authorization"] = f"Bearer {key}"
        else:
            self._auth_hint = (
                "MOVEIT_INFERENCE_KEY is not set in the Runtime container. Start "
                "the Runtime through `moveit_pro run`, which sets it."
            )
            self.get_logger().warning(
                "MOVEIT_INFERENCE_KEY is not set, so /infer calls to "
                f"{self.infer_url} carry no bearer token."
            )
        service_name = self.declare_parameter("service_name", "/get_action_chunk").value
        self.create_service(GetActionChunk, service_name, self._on_request)
        self._calls = 0
        self.get_logger().info(
            f"serving '{service_name}' -> {self.infer_url} on a "
            f"{self.http_timeout:g}s budget. Set the Objective's "
            f"policy_call_timeout above {self.http_timeout:g}s."
        )

    def _on_request(self, request, response):
        try:
            self._fill_response(request, response)
        except RequestError as exc:
            response.status = GetActionChunk.Response.ERROR
            response.message = str(exc)
            self.get_logger().error(response.message)
        except Exception as exc:
            response.status = GetActionChunk.Response.ERROR
            response.message = (
                f"adapter failed: {type(exc).__name__}: {clip_detail(str(exc))}"
            )
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
        """POST to the inference server; return the parsed response body.

        The call runs under one wall-clock deadline and one size cap. Neither
        of `requests`' own timeouts bounds a call: connect applies per resolved
        address and read applies per `recv`, so a slow or hostile server would
        otherwise outlive the caller's service timeout and wedge this
        single-threaded node past the point ExecutePolicy has given up.
        DeadlineAdapter bounds connecting and a server that goes quiet before
        answering, and _read_bounded bounds reading the answer. A peer that
        trickles its handshake or headers is not bounded here; ExecutePolicy's
        own timeout fails the run first.
        """
        deadline = time.monotonic() + self.http_timeout
        # A reachable server accepts in well under a second on the LAN this
        # feature targets, so connect takes a small slice and the rest goes to
        # inference. Keeping the slice small also caps what a name resolving to
        # several dead addresses can spend before the total cuts in.
        connect_s = min(2.0, self.http_timeout / 4.0)
        answered = False
        try:
            resp = self._session.post(
                self.infer_url,
                json=payload,
                timeout=(connect_s, self.http_timeout - connect_s),
                allow_redirects=False,
                stream=True,
            )
            with resp:
                answered = True
                if resp.is_redirect:
                    raise RequestError(
                        f"/infer answered HTTP {resp.status_code} with a redirect "
                        "instead of a result. Point infer_url at the inference "
                        "endpoint itself."
                    )
                encoding = resp.headers.get("content-encoding", "identity")
                if encoding != "identity":
                    raise RequestError(
                        f"/infer returned a {encoding}-encoded body. Configure the "
                        "inference server to answer uncompressed JSON."
                    )
                declared = resp.headers.get("Content-Length", "")
                if declared.isdigit() and int(declared) > MAX_RESPONSE_BYTES:
                    raise RequestError(
                        f"/infer declared a {declared} byte response, over the "
                        f"{MAX_RESPONSE_BYTES} byte limit. Shorten the action "
                        "chunk the inference server returns, or raise "
                        "MAX_RESPONSE_BYTES in this adapter."
                    )
                body = self._read_bounded(resp, deadline)
        except requests.exceptions.SSLError as exc:
            raise RequestError(
                f"/infer TLS verification failed for {self.infer_url}: "
                f"{clip_detail(str(exc))}. Check that the certificate covers that "
                "hostname and that MOVEIT_INFERENCE_CA_FILE contains its issuer."
            ) from exc
        except requests.ConnectionError as exc:
            # `requests` re-raises a mid-body read timeout as ConnectionError, so
            # the server is only unreachable when it never answered at all.
            if answered:
                raise RequestError(
                    f"/infer started a response and then stopped sending it "
                    f"({clip_detail(str(exc))}). Check inference latency on the "
                    "server and the network path to it."
                ) from exc
            raise RequestError(
                f"/infer request failed: the inference server at {self.infer_url} "
                f"is not reachable ({clip_detail(str(exc))}). {self._unreachable_hint}"
            ) from exc
        except requests.Timeout as exc:
            raise RequestError(
                f"/infer timed out after {self.http_timeout:g}s "
                f"({clip_detail(str(exc))}). Check inference latency on the "
                "server and the network path to it."
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
            data = json.loads(body)
        except ValueError:
            data = None
        detail = ""
        if isinstance(data, dict) and data.get("error"):
            detail = clip_detail(str(data["error"]))
        if resp.status_code in (401, 403):
            reason = f": {detail}" if detail else ""
            raise RequestError(
                f"/infer rejected the bearer credential with HTTP "
                f"{resp.status_code}{reason}. {self._auth_hint}"
            )
        if detail:
            raise RequestError(f"/infer error: {detail}")
        if resp.status_code == 503 and self.infer_url.startswith("https:"):
            raise RequestError(
                "/infer request failed with HTTP 503 and carried no JSON "
                "detail, so the TLS proxy refused the call, most likely "
                "because another request holds its single /infer slot. Check "
                "for a second client using this inference host."
            )
        if not resp.ok:
            raise RequestError(
                f"/infer request failed with HTTP {resp.status_code} and carried "
                "no JSON detail. A proxy in front of the inference server may "
                f"have answered instead of the server itself. "
                f"{self._unreachable_hint}"
            )
        if not isinstance(data, dict):
            raise RequestError(
                f"/infer answered HTTP {resp.status_code} with a body that is not "
                "a JSON object. Check that infer_url names the inference server "
                "and not another service."
            )
        return data

    def _read_bounded(self, resp, deadline: float) -> bytes:
        """Read a streamed body, stopping at the size cap or the deadline.

        A timer shuts the socket down at the deadline, which wakes the blocked
        `recv`. The read never has to return often enough to check a clock.
        """
        socket_ = response_socket(resp)
        watchdog = None
        if socket_ is None:
            # The node's own logger, because `once` is tracked per logger
            # object and rclpy.logging.get_logger returns a new one each call.
            self.get_logger().warning(
                "Could not find the response socket under this urllib3 "
                "version, so the /infer deadline now bounds each read rather "
                "than the whole call. A stalled server can hold this node past "
                "the Objective's policy_call_timeout. Report this so the "
                "socket lookup can be updated.",
                once=True,
            )
        else:
            watchdog = threading.Timer(
                max(0.0, deadline - time.monotonic()), shutdown_quietly, (socket_,)
            )
            watchdog.daemon = True
            watchdog.start()
        body = bytearray()
        try:
            for chunk in resp.iter_content(chunk_size=READ_CHUNK_BYTES):
                body.extend(chunk)
                if len(body) > MAX_RESPONSE_BYTES:
                    raise RequestError(
                        f"/infer response exceeds the {MAX_RESPONSE_BYTES} byte "
                        "limit. Shorten the action chunk the inference server "
                        "returns, or raise MAX_RESPONSE_BYTES in this adapter."
                    )
        except requests.RequestException:
            # The shutdown arrives as a transport error rather than a timeout,
            # so compare against the deadline to pick the message.
            if time.monotonic() >= deadline:
                raise RequestError(self._read_timeout_message()) from None
            raise
        finally:
            if watchdog is not None:
                watchdog.cancel()
        if time.monotonic() >= deadline:
            raise RequestError(self._read_timeout_message())
        return bytes(body)

    def _read_timeout_message(self) -> str:
        """Return the operator-facing reason a response body took longer than its budget."""
        return (
            f"/infer timed out after {self.http_timeout:g}s while reading the "
            "response. Check inference latency on the server and the network "
            "path to it."
        )

    @staticmethod
    def _validate_chunk(data: dict, expected_dims: int) -> np.ndarray:
        """Check the returned chunk's shape, values, and dt; return the chunk."""
        chunk = data.get("action_chunk")
        if not chunk or "dt" not in data:
            raise RequestError(
                "/infer response is missing a non-empty action_chunk or dt"
            )
        dt = data["dt"]
        if not isinstance(dt, (int, float)) or not np.isfinite(dt) or dt <= 0.0:
            raise RequestError(
                f"/infer response carries an invalid dt ({clip_detail(repr(dt))}); "
                "playback "
                "pacing needs a finite value > 0"
            )
        chunk_arr = _finite_2d(chunk, "action_chunk")
        if chunk_arr.shape[1] != expected_dims:
            raise RequestError(
                f"/infer chunk width {chunk_arr.shape[1]} does not match the "
                f"observed joint count {expected_dims}"
            )
        return chunk_arr

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
            raw_arr = _finite_2d(raw, "action_chunk_raw")
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
    try:
        node = GetActionChunkAdapter()
    except ValueError as error:
        # The messages name the setting and the fix, so print them instead of
        # a traceback.
        print(f"get_action_chunk_adapter: {error}", file=sys.stderr)
        rclpy.try_shutdown()
        raise SystemExit(1) from None
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
