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

"""Tests for the GetActionChunk adapter: image encoding and request/response mapping."""

import importlib.util
import json
import os
import sys
import unittest
from pathlib import Path
from unittest.mock import MagicMock, patch

import rclpy
import requests
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Float64MultiArray, MultiArrayDimension, MultiArrayLayout

from moveit_pro_ml_msgs.srv import GetActionChunk

# get_action_chunk_adapter.py is a standalone ROS executable (install(PROGRAMS ...) in
# CMakeLists.txt), not part of an importable Python package, so it's loaded by file path.
_SCRIPT_PATH = (
    Path(__file__).resolve().parent.parent / "script" / "get_action_chunk_adapter.py"
)
_spec = importlib.util.spec_from_file_location("get_action_chunk_adapter", _SCRIPT_PATH)
get_action_chunk_adapter = importlib.util.module_from_spec(_spec)
sys.modules["get_action_chunk_adapter"] = get_action_chunk_adapter
_spec.loader.exec_module(get_action_chunk_adapter)

GetActionChunkAdapter = get_action_chunk_adapter.GetActionChunkAdapter
encode_jpeg_b64 = get_action_chunk_adapter.encode_jpeg_b64
resolve_http_timeout = get_action_chunk_adapter.resolve_http_timeout
resolve_infer_url = get_action_chunk_adapter.resolve_infer_url

# The node posts through its own session, so patching `requests.post` would
# intercept nothing and leave these tests asserting against a live call.
SESSION_POST = "get_action_chunk_adapter.requests.Session.post"


class FakeResponse:
    """A `requests.Response` stand-in for the adapter's streamed body read."""

    def __init__(
        self, payload=None, status: int = 200, headers=None, body=None
    ) -> None:
        self.status_code = status
        self.headers = headers or {}
        if body is None:
            body = json.dumps(payload if payload is not None else {}).encode()
        self._body = body

    @property
    def ok(self) -> bool:
        return self.status_code < 400

    @property
    def is_redirect(self) -> bool:
        return self.status_code in (301, 302, 303, 307, 308)

    def iter_content(self, chunk_size):
        yield self._body

    def __enter__(self):
        return self

    def __exit__(self, *exc_info) -> bool:
        return False


def stub_json(mock_post, payload, status: int = 200) -> None:
    """Answer the next POST with `payload` as an uncompressed JSON body."""
    mock_post.return_value = FakeResponse(payload, status)


def make_image(encoding: str, height: int = 2, width: int = 2) -> Image:
    """A flat-colored sensor_msgs/Image with no row padding (step == width * channels)."""
    channels = 4 if encoding in ("rgba8", "bgra8") else 3
    img = Image()
    img.encoding = encoding
    img.height = height
    img.width = width
    img.step = width * channels
    img.data = bytes([128] * (height * img.step))
    return img


def make_request(**overrides) -> GetActionChunk.Request:
    request = GetActionChunk.Request()
    request.robot_state = JointState(name=["j1", "j2"], position=[0.1, 0.2])
    request.images = [make_image("rgb8")]
    request.image_names = ["front"]
    request.prompt = "stack the blocks"
    request.new_episode = False
    for key, value in overrides.items():
        setattr(request, key, value)
    return request


class TestEncodeJpegB64(unittest.TestCase):
    """encode_jpeg_b64: sensor_msgs/Image -> base64 JPEG round trip."""

    def test_unsupported_encoding_raises(self) -> None:
        """A mono8 (or any non-{r,b}gb[a]8) image is rejected, not silently reinterpreted."""
        img = make_image("mono8")
        with self.assertRaises(ValueError):
            encode_jpeg_b64(img)

    def test_rgb8_and_bgr8_roundtrip_to_same_pixels(self) -> None:
        """rgb8 and bgr8 inputs carrying the same visual color decode to matching JPEG bytes."""
        rgb = make_image("rgb8")
        rgb.data = bytes([10, 20, 30] * (rgb.height * rgb.width))
        bgr = make_image("bgr8")
        bgr.data = bytes([30, 20, 10] * (bgr.height * bgr.width))

        rgb_b64 = encode_jpeg_b64(rgb)
        bgr_b64 = encode_jpeg_b64(bgr)

        self.assertEqual(rgb_b64, bgr_b64)

    def test_rgba8_drops_alpha_channel(self) -> None:
        """Encoding a 4-channel frame does not crash and yields a valid JPEG (3-channel)."""
        img = make_image("rgba8")
        img.data = bytes([10, 20, 30, 255] * (img.height * img.width))
        b64 = encode_jpeg_b64(img)
        self.assertTrue(len(b64) > 0)

    def test_row_padding_is_stripped(self) -> None:
        """step wider than width*channels (row padding) must not corrupt the decoded pixels."""
        img = make_image("rgb8")
        pad = 4
        img.step = img.width * 3 + pad
        img.data = bytes([10, 20, 30] * img.width + [0] * pad) * img.height
        padded_b64 = encode_jpeg_b64(img)

        unpadded = make_image("rgb8")
        unpadded.data = bytes([10, 20, 30] * (unpadded.height * unpadded.width))
        unpadded_b64 = encode_jpeg_b64(unpadded)

        self.assertEqual(padded_b64, unpadded_b64)


class TestResolveHttpTimeout(unittest.TestCase):
    """resolve_http_timeout: unusable values fail at startup, naming the parameter."""

    def test_positive_value_passes_through(self) -> None:
        """A positive timeout is returned unchanged."""
        self.assertEqual(resolve_http_timeout(9.0), 9.0)

    def test_zero_raises_naming_the_parameter(self) -> None:
        """A zero timeout fails at construction, not as a per-call generic error."""
        with self.assertRaises(ValueError) as ctx:
            resolve_http_timeout(0.0)
        self.assertIn("http_timeout", str(ctx.exception))


class TestResolveInferUrl(unittest.TestCase):
    """The Runtime's classifier owns the rule; these check the adapter's wiring."""

    def test_accepted_urls_are_rebuilt_from_the_checked_parts(self) -> None:
        """Padding and a trailing newline never reach the HTTP client."""
        default = get_action_chunk_adapter.DEFAULT_INFER_URL
        for value, expected in (
            (default, default),
            (" https://gpu.example:8443/infer ", "https://gpu.example:8443/infer"),
            ("http://[::1]:8973/infer\n", "http://[::1]:8973/infer"),
        ):
            with self.subTest(value=value):
                self.assertEqual(resolve_infer_url(value), expected)

    def test_a_refused_url_names_the_parameter_and_the_reason(self) -> None:
        """The classifier's reason reaches the log with the parameter at fault."""
        with self.assertRaises(ValueError) as ctx:
            resolve_infer_url("http://10.0.0.7:8973/infer")
        self.assertIn("infer_url", str(ctx.exception))
        self.assertIn("not a loopback address", str(ctx.exception))

    def test_an_empty_url_is_refused_naming_the_default(self) -> None:
        """A blank parameter is a configuration error, not silently the default."""
        with self.assertRaises(ValueError) as ctx:
            resolve_infer_url("  ")
        self.assertIn("infer_url", str(ctx.exception))
        self.assertIn(get_action_chunk_adapter.DEFAULT_INFER_URL, str(ctx.exception))


class TestOnRequest(unittest.TestCase):
    """GetActionChunkAdapter._on_request: HTTP call shaping and response translation."""

    @classmethod
    def setUpClass(cls) -> None:
        rclpy.init()

    @classmethod
    def tearDownClass(cls) -> None:
        rclpy.shutdown()

    def setUp(self) -> None:
        self.node = GetActionChunkAdapter()

    def tearDown(self) -> None:
        self.node.destroy_node()

    @patch(SESSION_POST)
    def test_successful_chunk_populates_trajectory(self, mock_post: MagicMock) -> None:
        """A valid /infer response becomes a JointTrajectory with matching joint names."""
        stub_json(
            mock_post,
            {
                "action_chunk": [[0.5, 0.6], [0.7, 0.8]],
                "dt": 0.05,
            },
        )
        request = make_request()
        response = self.node._on_request(request, GetActionChunk.Response())

        self.assertEqual(
            response.status, GetActionChunk.Response.CHUNK_PRODUCED, response.message
        )
        self.assertEqual(list(response.chunk.joint_names), ["j1", "j2"])
        self.assertEqual(len(response.chunk.points), 2)
        self.assertEqual(list(response.chunk.points[0].positions), [0.5, 0.6])
        self.assertAlmostEqual(response.native_control_period, 0.05)

    @patch(SESSION_POST)
    def test_prompt_forwarded_as_task_field(self, mock_post: MagicMock) -> None:
        """The fixed request.prompt field is sent to /infer under the 'task' key."""
        stub_json(
            mock_post,
            {
                "action_chunk": [[0.0, 0.0]],
                "dt": 0.05,
            },
        )
        request = make_request(prompt="pick the red cube")
        self.node._on_request(request, GetActionChunk.Response())

        sent_payload = mock_post.call_args.kwargs["json"]
        self.assertEqual(sent_payload["task"], "pick the red cube")

    @patch(SESSION_POST)
    def test_observation_forwarded_in_payload(self, mock_post: MagicMock) -> None:
        """The joint positions, image names, and new_episode flag all reach the
        payload, posted to the node's configured infer_url."""
        stub_json(
            mock_post,
            {
                "action_chunk": [[0.0, 0.0]],
                "dt": 0.05,
            },
        )
        request = make_request(new_episode=True)
        self.node._on_request(request, GetActionChunk.Response())

        (url,) = mock_post.call_args.args
        self.assertEqual(url, self.node.infer_url)
        sent_payload = mock_post.call_args.kwargs["json"]
        self.assertEqual(sent_payload["state"], [0.1, 0.2])
        self.assertEqual(list(sent_payload["images"].keys()), ["front"])
        self.assertTrue(sent_payload["new_episode"])

    def test_unsupported_image_encoding_fails_without_http_call(self) -> None:
        """An image the encoder can't handle fails locally; /infer is never called."""
        request = make_request(images=[make_image("mono8")])
        with patch(SESSION_POST) as mock_post:
            response = self.node._on_request(request, GetActionChunk.Response())
            mock_post.assert_not_called()
        self.assertEqual(response.status, GetActionChunk.Response.ERROR)
        self.assertIn("image encode failed", response.message)

    def test_images_and_image_names_length_mismatch_fails_without_http_call(
        self,
    ) -> None:
        """A malformed request (arrays not lined up by index, per the .srv contract)
        fails locally instead of raising IndexError out of the service callback."""
        request = make_request(images=[make_image("rgb8"), make_image("rgb8")])
        with patch(SESSION_POST) as mock_post:
            response = self.node._on_request(request, GetActionChunk.Response())
            mock_post.assert_not_called()
        self.assertEqual(response.status, GetActionChunk.Response.ERROR)
        self.assertIn("length mismatch", response.message)

    def test_state_name_position_length_mismatch_fails_without_http_call(self) -> None:
        """Mismatched robot_state arrays fail locally naming the real defect;
        the payload state comes from positions while the chunk is validated and
        labeled with the joint names."""
        request = make_request(
            robot_state=JointState(name=["j1", "j2", "j3"], position=[0.1, 0.2])
        )
        with patch(SESSION_POST) as mock_post:
            response = self.node._on_request(request, GetActionChunk.Response())
            mock_post.assert_not_called()
        self.assertEqual(response.status, GetActionChunk.Response.ERROR)
        self.assertIn("2 positions", response.message)
        self.assertIn("3 joint names", response.message)

    @patch(SESSION_POST)
    def test_connection_error_names_the_server_and_a_check_command(
        self, mock_post: MagicMock
    ) -> None:
        """A refused connection tells the operator which server is down and how to check it."""
        mock_post.side_effect = requests.ConnectionError("refused")
        response = self.node._on_request(make_request(), GetActionChunk.Response())

        self.assertEqual(response.status, GetActionChunk.Response.ERROR)
        self.assertIn("/infer request failed", response.message)
        self.assertIn("inference_server", response.message)
        self.assertIn("--with-inference-server", response.message)

    @patch(SESSION_POST)
    def test_request_carries_inference_key_bearer_token(
        self, mock_post: MagicMock
    ) -> None:
        """/infer requests present MOVEIT_INFERENCE_KEY as the bearer token the
        server's auth check expects."""
        stub_json(
            mock_post,
            {
                "action_chunk": [[0.0, 0.0]],
                "dt": 0.05,
            },
        )
        with patch.dict(os.environ, {"MOVEIT_INFERENCE_KEY": "secret-key"}):
            node = GetActionChunkAdapter()
        try:
            node._on_request(make_request(), GetActionChunk.Response())
        finally:
            node.destroy_node()

        self.assertEqual(node._session.headers["Authorization"], "Bearer secret-key")

    @patch(SESSION_POST)
    def test_missing_key_sends_no_authorization_header(
        self, mock_post: MagicMock
    ) -> None:
        """Without an inference key the request goes out bare, even when the
        frontend key is set; the server's 401 detail then names the fix through
        the normal error path."""
        stub_json(
            mock_post,
            {
                "action_chunk": [[0.0, 0.0]],
                "dt": 0.05,
            },
        )
        with patch.dict(os.environ, {"MOVEIT_FRONTEND_KEY": "root-secret"}):
            os.environ.pop("MOVEIT_INFERENCE_KEY", None)
            node = GetActionChunkAdapter()
        try:
            node._on_request(make_request(), GetActionChunk.Response())
        finally:
            node.destroy_node()

        self.assertNotIn("Authorization", node._session.headers)

    @patch(SESSION_POST)
    def test_local_401_without_a_key_names_the_runtime_side(
        self, mock_post: MagicMock
    ) -> None:
        """A 401 on a bare request tells the operator the Runtime lacks the key."""
        stub_json(
            mock_post,
            {"error": "/infer requires MOVEIT_INFERENCE_KEY as a bearer token"},
            status=401,
        )
        with patch.dict(os.environ):
            os.environ.pop("MOVEIT_INFERENCE_KEY", None)
            node = GetActionChunkAdapter()
        try:
            response = node._on_request(make_request(), GetActionChunk.Response())
        finally:
            node.destroy_node()

        self.assertEqual(response.status, GetActionChunk.Response.ERROR)
        self.assertIn("HTTP 401", response.message)
        self.assertIn(
            "MOVEIT_INFERENCE_KEY is not set in the Runtime", response.message
        )
        self.assertIn("moveit_pro run", response.message)

    @patch(SESSION_POST)
    def test_local_401_with_a_key_points_at_the_server(
        self, mock_post: MagicMock
    ) -> None:
        """A rejected key means the server holds a different one, so restart it."""
        stub_json(
            mock_post,
            {"error": "/infer requires MOVEIT_INFERENCE_KEY as a bearer token"},
            status=401,
        )
        with patch.dict(os.environ, {"MOVEIT_INFERENCE_KEY": "k" * 64}):
            node = GetActionChunkAdapter()
        try:
            response = node._on_request(make_request(), GetActionChunk.Response())
        finally:
            node.destroy_node()

        self.assertEqual(response.status, GetActionChunk.Response.ERROR)
        self.assertIn("--only-inference-server", response.message)
        self.assertIn("MOVEIT_INFERENCE_KEY", response.message)

    @patch(SESSION_POST)
    def test_timeout_surfaces_as_failure(self, mock_post: MagicMock) -> None:
        """A network-level failure other than a refused connection is reported, not raised."""
        mock_post.side_effect = requests.Timeout("timed out")
        response = self.node._on_request(make_request(), GetActionChunk.Response())

        self.assertEqual(response.status, GetActionChunk.Response.ERROR)
        self.assertIn("/infer timed out", response.message)
        # The configured budget is named, so the operator knows what to raise.
        self.assertIn("9s", response.message)

    @patch(SESSION_POST)
    def test_http_timeout_tracks_the_parameter(self, mock_post: MagicMock) -> None:
        """The connect + read split sums to the http_timeout parameter.

        DeadlineAdapter turns the pair into one urllib3 total, so neither a name
        with several dead addresses nor a silent server can spend more than the
        sum. This also checks trust_env, because the TLS tests build
        their own session and would not catch the node leaving it on."""
        stub_json(
            mock_post,
            {
                "action_chunk": [[0.0, 0.0]],
                "dt": 0.05,
            },
        )
        self.node._on_request(make_request(), GetActionChunk.Response())

        connect_s, read_s = mock_post.call_args.kwargs["timeout"]
        self.assertEqual(connect_s, 2.0)
        self.assertEqual(connect_s + read_s, 9.0)
        for scheme in ("https://", "http://"):
            self.assertIsInstance(
                self.node._session.get_adapter(scheme),
                get_action_chunk_adapter.DeadlineAdapter,
            )
        self.assertFalse(
            self.node._session.trust_env,
            "An ambient proxy or netrc entry must not reach observations.",
        )
        self.assertEqual(
            self.node._session.headers["Accept-Encoding"],
            "identity",
            "A compressed body would bypass the response size cap.",
        )

    @patch(SESSION_POST)
    def test_server_error_field_surfaces_as_failure(self, mock_post: MagicMock) -> None:
        """A server-reported {"error": ...} body fails the request rather than being ignored.

        This is also the path that relays "still loading the model" (503) and
        "model load failed" (500) to the operator."""
        stub_json(mock_post, {"error": "checkpoint not loaded"}, status=503)
        response = self.node._on_request(make_request(), GetActionChunk.Response())

        self.assertEqual(response.status, GetActionChunk.Response.ERROR)
        self.assertIn("checkpoint not loaded", response.message)

    @patch(SESSION_POST)
    def test_huge_server_error_is_truncated(self, mock_post: MagicMock) -> None:
        """A runaway server error string is clipped before it reaches the UI."""
        stub_json(mock_post, {"error": "x" * 100_000}, status=500)
        response = self.node._on_request(make_request(), GetActionChunk.Response())

        self.assertEqual(response.status, GetActionChunk.Response.ERROR)
        self.assertLess(len(response.message), 3000)
        self.assertIn("[...]", response.message)

    @patch(SESSION_POST)
    def test_non_json_error_response_reports_http_status(
        self, mock_post: MagicMock
    ) -> None:
        """A non-JSON body on a failed status still produces a diagnosable message."""
        mock_post.return_value = FakeResponse(
            status=502, body=b"<html><body>502 Bad Gateway</body></html>"
        )
        response = self.node._on_request(make_request(), GetActionChunk.Response())

        self.assertEqual(response.status, GetActionChunk.Response.ERROR)
        self.assertIn("HTTP 502", response.message)
        self.assertIn("no JSON detail", response.message)

    @patch(SESSION_POST)
    def test_missing_dt_fails(self, mock_post: MagicMock) -> None:
        """A response with a chunk but no dt is rejected rather than defaulting silently."""
        stub_json(mock_post, {"action_chunk": [[0.0, 0.0]]})
        response = self.node._on_request(make_request(), GetActionChunk.Response())

        self.assertEqual(response.status, GetActionChunk.Response.ERROR)
        self.assertIn("missing", response.message)

    @patch(SESSION_POST)
    def test_chunk_width_mismatch_fails(self, mock_post: MagicMock) -> None:
        """A chunk whose column count doesn't match the requested joint count is rejected."""
        stub_json(
            mock_post,
            {
                "action_chunk": [[0.0, 0.0, 0.0]],  # 3 columns, request has 2 joints
                "dt": 0.05,
            },
        )
        response = self.node._on_request(make_request(), GetActionChunk.Response())

        self.assertEqual(response.status, GetActionChunk.Response.ERROR)
        self.assertIn("does not match", response.message)

    @patch(SESSION_POST)
    def test_ragged_chunk_fails_naming_action_chunk(self, mock_post: MagicMock) -> None:
        """Rows of different widths are refused even when the first row matches."""
        stub_json(
            mock_post,
            {
                # row 0 matches the request's 2 joints; row 1 is the actual offender.
                "action_chunk": [[0.0, 0.0], [0.0, 0.0, 0.0]],
                "dt": 0.05,
            },
        )
        response = self.node._on_request(make_request(), GetActionChunk.Response())

        self.assertEqual(response.status, GetActionChunk.Response.ERROR)
        self.assertIn("action_chunk is not a rectangular array", response.message)

    @patch(SESSION_POST)
    def test_scalar_chunk_rows_fail_naming_action_chunk(
        self, mock_post: MagicMock
    ) -> None:
        """Rows that aren't lists (a flat or scalar chunk from a custom server)
        are rejected naming action_chunk, not as a generic adapter failure."""
        stub_json(
            mock_post,
            {
                "action_chunk": [0.0, 0.1],
                "dt": 0.05,
            },
        )
        response = self.node._on_request(make_request(), GetActionChunk.Response())

        self.assertEqual(response.status, GetActionChunk.Response.ERROR)
        self.assertIn("action_chunk is 1-dimensional", response.message)

    @patch(SESSION_POST)
    def test_non_numeric_chunk_values_fail_naming_action_chunk(
        self, mock_post: MagicMock
    ) -> None:
        """String action values are rejected naming the chunk, not as a raw
        numpy conversion error."""
        stub_json(
            mock_post,
            {
                "action_chunk": [["a", "b"]],
                "dt": 0.05,
            },
        )
        response = self.node._on_request(make_request(), GetActionChunk.Response())

        self.assertEqual(response.status, GetActionChunk.Response.ERROR)
        self.assertIn(
            "action_chunk is not a rectangular array of numbers", response.message
        )

    @patch(SESSION_POST)
    def test_non_finite_chunk_fails(self, mock_post: MagicMock) -> None:
        """NaN/inf action values are rejected before they reach the controller."""
        stub_json(
            mock_post,
            {
                "action_chunk": [[0.0, float("nan")]],
                "dt": 0.05,
            },
        )
        response = self.node._on_request(make_request(), GetActionChunk.Response())

        self.assertEqual(response.status, GetActionChunk.Response.ERROR)
        self.assertIn("non-finite", response.message)

    @patch(SESSION_POST)
    def test_non_positive_dt_fails(self, mock_post: MagicMock) -> None:
        """A dt of zero (or below) is rejected: it cannot pace chunk playback."""
        stub_json(
            mock_post,
            {
                "action_chunk": [[0.0, 0.0]],
                "dt": 0.0,
            },
        )
        response = self.node._on_request(make_request(), GetActionChunk.Response())

        self.assertEqual(response.status, GetActionChunk.Response.ERROR)
        self.assertIn("invalid dt", response.message)

    @patch(SESSION_POST)
    def test_non_finite_dt_fails(self, mock_post: MagicMock) -> None:
        """A NaN dt is rejected before it reaches trajectory timing."""
        stub_json(
            mock_post,
            {
                "action_chunk": [[0.0, 0.0]],
                "dt": float("nan"),
            },
        )
        response = self.node._on_request(make_request(), GetActionChunk.Response())

        self.assertEqual(response.status, GetActionChunk.Response.ERROR)
        self.assertIn("invalid dt", response.message)

    @patch(SESSION_POST)
    def test_previous_chunk_reshaped_and_forwarded(self, mock_post: MagicMock) -> None:
        """A populated previous_action_chunk is reshaped from its flat layout before sending."""
        stub_json(
            mock_post,
            {
                "action_chunk": [[0.0, 0.0]],
                "dt": 0.05,
            },
        )
        prev = Float64MultiArray(
            layout=MultiArrayLayout(
                dim=[
                    MultiArrayDimension(label="steps", size=2, stride=4),
                    MultiArrayDimension(label="dims", size=2, stride=2),
                ]
            ),
            data=[1.0, 2.0, 3.0, 4.0],
        )
        request = make_request(previous_action_chunk=prev, frozen_prefix_steps=3)
        self.node._on_request(request, GetActionChunk.Response())

        sent_payload = mock_post.call_args.kwargs["json"]
        self.assertEqual(sent_payload["prev_chunk_left_over"], [[1.0, 2.0], [3.0, 4.0]])
        self.assertEqual(sent_payload["inference_delay"], 3)

    def test_previous_chunk_with_malformed_layout_fails_without_http_call(self) -> None:
        """A populated previous_action_chunk whose layout is not the contract's two
        dimensions fails loudly instead of silently dropping the RTC carryover."""
        prev = Float64MultiArray(
            layout=MultiArrayLayout(
                dim=[MultiArrayDimension(label="flat", size=4, stride=4)]
            ),
            data=[1.0, 2.0, 3.0, 4.0],
        )
        request = make_request(previous_action_chunk=prev, frozen_prefix_steps=3)
        with patch(SESSION_POST) as mock_post:
            response = self.node._on_request(request, GetActionChunk.Response())
            mock_post.assert_not_called()

        self.assertEqual(response.status, GetActionChunk.Response.ERROR)
        self.assertIn("1 dimensions", response.message)
        self.assertIn("2 (steps, action width)", response.message)

    def test_previous_chunk_with_wrong_element_count_fails_without_http_call(
        self,
    ) -> None:
        """A previous_action_chunk whose data length contradicts its declared
        layout fails with the mismatch named, not a raw reshape error."""
        prev = Float64MultiArray(
            layout=MultiArrayLayout(
                dim=[
                    MultiArrayDimension(label="steps", size=2, stride=6),
                    MultiArrayDimension(label="dims", size=3, stride=3),
                ]
            ),
            data=[1.0, 2.0, 3.0, 4.0],
        )
        request = make_request(previous_action_chunk=prev, frozen_prefix_steps=3)
        with patch(SESSION_POST) as mock_post:
            response = self.node._on_request(request, GetActionChunk.Response())
            mock_post.assert_not_called()

        self.assertEqual(response.status, GetActionChunk.Response.ERROR)
        self.assertIn("4 values", response.message)
        self.assertIn("2x3", response.message)

    @patch(SESSION_POST)
    def test_empty_previous_chunk_omits_rtc_fields(self, mock_post: MagicMock) -> None:
        """The first call of an episode (empty previous_action_chunk) sends no RTC carryover."""
        stub_json(
            mock_post,
            {
                "action_chunk": [[0.0, 0.0]],
                "dt": 0.05,
            },
        )
        self.node._on_request(make_request(), GetActionChunk.Response())

        sent_payload = mock_post.call_args.kwargs["json"]
        self.assertNotIn("prev_chunk_left_over", sent_payload)
        self.assertNotIn("inference_delay", sent_payload)

    @patch(SESSION_POST)
    def test_zero_guidance_horizon_omits_the_field(self, mock_post: MagicMock) -> None:
        """guidance_horizon=0 defers to the server's own RTC default, per the .srv contract."""
        stub_json(
            mock_post,
            {
                "action_chunk": [[0.0, 0.0]],
                "dt": 0.05,
            },
        )
        self.node._on_request(
            make_request(guidance_horizon=0), GetActionChunk.Response()
        )

        sent_payload = mock_post.call_args.kwargs["json"]
        self.assertNotIn("guidance_horizon", sent_payload)

    @patch(SESSION_POST)
    def test_nonzero_guidance_horizon_is_forwarded(self, mock_post: MagicMock) -> None:
        """A nonzero guidance_horizon from the Objective is forwarded as the soft-guidance
        width; the server, which knows the inference delay, maps it onto lerobot's horizon.
        """
        stub_json(
            mock_post,
            {
                "action_chunk": [[0.0, 0.0]],
                "dt": 0.05,
            },
        )
        self.node._on_request(
            make_request(guidance_horizon=7), GetActionChunk.Response()
        )

        sent_payload = mock_post.call_args.kwargs["json"]
        self.assertEqual(sent_payload["guidance_horizon"], 7)

    @patch(SESSION_POST)
    def test_action_chunk_raw_echoed_as_policy_action_chunk(
        self, mock_post: MagicMock
    ) -> None:
        """When the server echoes action_chunk_raw, it is reflected back as policy_action_chunk."""
        stub_json(
            mock_post,
            {
                "action_chunk": [[0.5, 0.6], [0.7, 0.8]],
                "action_chunk_raw": [[-0.1, 0.1], [0.2, -0.2]],
                "dt": 0.05,
            },
        )
        response = self.node._on_request(make_request(), GetActionChunk.Response())

        arr = response.policy_action_chunk
        self.assertEqual([d.size for d in arr.layout.dim], [2, 2])
        self.assertEqual(list(arr.data), [-0.1, 0.1, 0.2, -0.2])

    @patch(SESSION_POST)
    def test_ragged_action_chunk_raw_fails_the_request(
        self, mock_post: MagicMock
    ) -> None:
        """A malformed RTC echo fails the whole request naming the field, never
        success with the echo silently missing."""
        stub_json(
            mock_post,
            {
                "action_chunk": [[0.5, 0.6]],
                "action_chunk_raw": [[0.1, 0.2], [0.3]],
                "dt": 0.05,
            },
        )
        response = self.node._on_request(make_request(), GetActionChunk.Response())

        self.assertEqual(response.status, GetActionChunk.Response.ERROR)
        self.assertIn("action_chunk_raw", response.message)

    @patch(SESSION_POST)
    def test_zero_width_action_chunk_raw_fails_the_request(
        self, mock_post: MagicMock
    ) -> None:
        """An RTC echo with zero-width rows is rejected, not fed back as a
        structurally valid but empty carryover."""
        stub_json(
            mock_post,
            {
                "action_chunk": [[0.5, 0.6]],
                "action_chunk_raw": [[], []],
                "dt": 0.05,
            },
        )
        response = self.node._on_request(make_request(), GetActionChunk.Response())

        self.assertEqual(response.status, GetActionChunk.Response.ERROR)
        self.assertIn("action_chunk_raw", response.message)

    @patch(SESSION_POST)
    def test_empty_action_chunk_raw_succeeds_with_empty_echo(
        self, mock_post: MagicMock
    ) -> None:
        """A present-but-empty echo means no RTC carryover, same as an absent one."""
        stub_json(
            mock_post,
            {
                "action_chunk": [[0.5, 0.6]],
                "action_chunk_raw": [],
                "dt": 0.05,
            },
        )
        response = self.node._on_request(make_request(), GetActionChunk.Response())

        self.assertEqual(
            response.status, GetActionChunk.Response.CHUNK_PRODUCED, response.message
        )
        self.assertEqual(list(response.policy_action_chunk.data), [])

    @patch(SESSION_POST)
    def test_non_finite_action_chunk_raw_fails_the_request(
        self, mock_post: MagicMock
    ) -> None:
        """A NaN in the RTC echo is rejected here, where the field is named,
        not one call later as a confusing previous-chunk error."""
        stub_json(
            mock_post,
            {
                "action_chunk": [[0.5, 0.6]],
                "action_chunk_raw": [[0.1, float("nan")]],
                "dt": 0.05,
            },
        )
        response = self.node._on_request(make_request(), GetActionChunk.Response())

        self.assertEqual(response.status, GetActionChunk.Response.ERROR)
        self.assertIn("action_chunk_raw", response.message)
        self.assertIn("non-finite", response.message)

    @patch(SESSION_POST)
    def test_non_numeric_action_chunk_raw_fails_the_request(
        self, mock_post: MagicMock
    ) -> None:
        """Rectangular but non-numeric echo content is reported against the
        field, not as a generic adapter failure."""
        stub_json(
            mock_post,
            {
                "action_chunk": [[0.5, 0.6]],
                "action_chunk_raw": [["a", "b"]],
                "dt": 0.05,
            },
        )
        response = self.node._on_request(make_request(), GetActionChunk.Response())

        self.assertEqual(response.status, GetActionChunk.Response.ERROR)
        self.assertIn(
            "action_chunk_raw is not a rectangular array of numbers", response.message
        )

    @patch(SESSION_POST)
    def test_no_action_chunk_raw_leaves_policy_action_chunk_empty(
        self, mock_post: MagicMock
    ) -> None:
        """A policy without RTC support (no action_chunk_raw) leaves policy_action_chunk unset."""
        stub_json(
            mock_post,
            {
                "action_chunk": [[0.5, 0.6]],
                "dt": 0.05,
            },
        )
        response = self.node._on_request(make_request(), GetActionChunk.Response())

        self.assertEqual(list(response.policy_action_chunk.data), [])


class TestRemoteTransportSetup(unittest.TestCase):
    """Constructor wiring for an HTTPS endpoint: trust and hints."""

    @classmethod
    def setUpClass(cls) -> None:
        rclpy.init()

    @classmethod
    def tearDownClass(cls) -> None:
        rclpy.shutdown()

    def _remote_node(self, environment: dict):
        """Build a node against an HTTPS endpoint with a controlled environment."""
        with patch.object(
            get_action_chunk_adapter, "DEFAULT_INFER_URL", "https://gpu.example/infer"
        ), patch.dict(os.environ, environment, clear=False):
            for name in ("MOVEIT_INFERENCE_KEY", "MOVEIT_INFERENCE_CA_FILE"):
                if name not in environment:
                    os.environ.pop(name, None)
            return GetActionChunkAdapter()

    def test_unreadable_ca_file_refuses_to_start(self) -> None:
        """A trust anchor that cannot be read would silently fall back to the system store."""
        with self.assertRaises(ValueError) as error:
            self._remote_node({"MOVEIT_INFERENCE_CA_FILE": "/nonexistent/ca.pem"})

        self.assertIn("/nonexistent/ca.pem", str(error.exception))
        self.assertIn("MOVEIT_INFERENCE_CA_FILE on the host", str(error.exception))

    def test_blank_inference_key_never_sends_the_root_key(self) -> None:
        """A blank inference key sends a bare request, never the frontend key,
        and the 401 hint then points at the Runtime rather than the GPU host."""
        node = self._remote_node(
            {"MOVEIT_INFERENCE_KEY": "  ", "MOVEIT_FRONTEND_KEY": "root-secret"}
        )
        try:
            self.assertNotIn("Authorization", node._session.headers)
            self.assertIn("moveit_pro run", node._auth_hint)
            self.assertNotIn("inference host", node._auth_hint)
        finally:
            node.destroy_node()

    def test_configured_ca_file_becomes_the_only_trust_anchor(self) -> None:
        """Verification uses the mounted certificate rather than the system store,
        and the root credential never reaches an off-box server."""
        ca_file = Path(__file__).resolve()
        node = self._remote_node(
            {
                "MOVEIT_INFERENCE_KEY": "k" * 64,
                "MOVEIT_INFERENCE_CA_FILE": str(ca_file),
                "MOVEIT_FRONTEND_KEY": "root-secret",
            }
        )
        try:
            self.assertEqual(node._session.verify, str(ca_file))
            self.assertEqual(
                node._session.headers["Authorization"], "Bearer " + "k" * 64
            )
            # The credential is provisioned by hand on the other host, so the
            # rejection message has to name the command that prints it.
            self.assertIn("moveit_pro inference-key", node._auth_hint)
        finally:
            node.destroy_node()


class TestChunkShape(unittest.TestCase):
    """Shape validation, which decides what can reach the controller."""

    @classmethod
    def setUpClass(cls) -> None:
        rclpy.init()

    @classmethod
    def tearDownClass(cls) -> None:
        rclpy.shutdown()

    def setUp(self) -> None:
        self.node = GetActionChunkAdapter()

    def tearDown(self) -> None:
        self.node.destroy_node()

    def test_nested_rows_are_refused_by_name(self) -> None:
        """A 3-D chunk passes the width check, so the array shape is checked too.

        Without this it reaches float() on a list and fails as a bare TypeError
        naming neither action_chunk nor the shape.
        """
        with self.assertRaises(get_action_chunk_adapter.RequestError) as error:
            self.node._validate_chunk(
                {"action_chunk": [[[1.0, 2.0], [3.0, 4.0]]], "dt": 0.1}, 2
            )

        self.assertIn("action_chunk", str(error.exception))
        self.assertIn("3-dimensional", str(error.exception))

    @patch(SESSION_POST)
    def test_nested_rtc_echo_is_refused_by_name(self, mock_post: MagicMock) -> None:
        """A 3-D echo would otherwise fail unpacking its own shape."""
        stub_json(
            mock_post,
            {
                "action_chunk": [[0.0, 0.0]],
                "action_chunk_raw": [[[0.1, 0.2], [0.3, 0.4]]],
                "dt": 0.05,
            },
        )
        response = self.node._on_request(make_request(), GetActionChunk.Response())

        self.assertEqual(response.status, GetActionChunk.Response.ERROR)
        self.assertIn("action_chunk_raw", response.message)
        self.assertIn("3-dimensional", response.message)


if __name__ == "__main__":
    unittest.main()
