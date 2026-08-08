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
    """resolve_infer_url: requests carry the deployment key over plain HTTP, so
    the target has to stay on this machine."""

    def test_default_url_is_accepted(self) -> None:
        """The shipped default must pass its own check."""
        self.assertEqual(
            resolve_infer_url(get_action_chunk_adapter.DEFAULT_INFER_URL),
            get_action_chunk_adapter.DEFAULT_INFER_URL,
        )

    def test_ipv6_loopback_is_accepted(self) -> None:
        """A server bound to ::1 is just as local as one on 127.0.0.1."""
        url = "http://[::1]:8973/infer"
        self.assertEqual(resolve_infer_url(url), url)

    def test_loopback_range_beyond_the_first_address_is_accepted(self) -> None:
        """The whole 127.0.0.0/8 range is loopback, not just 127.0.0.1."""
        url = "http://127.0.0.5:8973/infer"
        self.assertEqual(resolve_infer_url(url), url)

    def test_remote_host_is_rejected(self) -> None:
        """The shared key would go out in cleartext to a machine we do not own."""
        with self.assertRaises(ValueError) as ctx:
            resolve_infer_url("http://10.0.0.7:8973/infer")
        self.assertIn("loopback", str(ctx.exception))

    def test_remote_https_host_is_rejected(self) -> None:
        """TLS is not the bar: serving off this machine is not supported."""
        with self.assertRaises(ValueError):
            resolve_infer_url("https://10.0.0.7:8973/infer")

    def test_https_is_rejected_even_for_loopback(self) -> None:
        """The server listens without TLS, so an https target could only ever
        fail the handshake. Rejecting it here names the scheme as the problem,
        rather than surfacing later as an unreachable server."""
        for url in ("https://127.0.0.1:8973/infer", "https://[::1]:8973/infer"):
            with self.assertRaises(ValueError, msg=url) as ctx:
                resolve_infer_url(url)
            self.assertIn("http URL", str(ctx.exception), msg=url)

    def test_hostname_is_rejected_even_when_it_names_loopback(self) -> None:
        """'localhost' resolves at connect time, so accepting it would accept
        whatever the resolver returns then, not what is checked here."""
        with self.assertRaises(ValueError) as ctx:
            resolve_infer_url("http://localhost:8973/infer")
        self.assertIn("IP literal", str(ctx.exception))

    def test_userinfo_cannot_disguise_a_remote_host(self) -> None:
        """Everything before '@' is userinfo: this URL addresses evil.com."""
        with self.assertRaises(ValueError) as ctx:
            resolve_infer_url("http://127.0.0.1@evil.com/infer")
        self.assertIn("no credentials", str(ctx.exception))

    def test_backslash_in_the_authority_is_rejected(self) -> None:
        """urlsplit reads the backslash as an ordinary userinfo character and
        reports 127.0.0.1, while the HTTP client ends the authority there and
        connects to evil.example. Accepting this sends the deployment key to
        that host in cleartext."""
        for url in (
            "http://evil.example\\@127.0.0.1:8973/infer",
            "http://evil.example\\@[::1]:8973/infer",
        ):
            with self.assertRaises(ValueError, msg=url) as ctx:
                resolve_infer_url(url)
            self.assertIn("no credentials", str(ctx.exception), msg=url)

    def test_the_returned_url_is_rebuilt_from_the_checked_parts(self) -> None:
        """urlsplit drops tab and newline characters before it parses, so a URL
        handed back as given can still carry bytes that were never checked."""
        self.assertEqual(
            resolve_infer_url("http://127.0.0.1:8973/infer\n"),
            "http://127.0.0.1:8973/infer",
        )

    def test_a_port_the_client_cannot_use_is_rejected_here(self) -> None:
        """Out-of-range and non-numeric ports otherwise surface much later, as
        a per-request client error that does not name the parameter at fault."""
        for url in ("http://127.0.0.1:99999/infer", "http://127.0.0.1:8973-8975/infer"):
            with self.assertRaises(ValueError, msg=url) as ctx:
                resolve_infer_url(url)
            self.assertIn("invalid port", str(ctx.exception), msg=url)

    def test_ipv4_mapped_ipv6_cannot_disguise_a_remote_host(self) -> None:
        """::ffff:10.0.0.7 parses as an IP literal but is not loopback."""
        with self.assertRaises(ValueError) as ctx:
            resolve_infer_url("http://[::ffff:10.0.0.7]:8973/infer")
        self.assertIn("loopback", str(ctx.exception))

    def test_alternate_spellings_of_loopback_are_rejected(self) -> None:
        """Decimal and hex forms of 127.0.0.1 are what a bypass looks like;
        requiring dotted-quad keeps the accepted set to one spelling."""
        for url in ("http://2130706433:8973/infer", "http://0x7f000001:8973/infer"):
            with self.assertRaises(ValueError, msg=url):
                resolve_infer_url(url)

    def test_non_http_scheme_is_rejected(self) -> None:
        """The adapter POSTs with requests; a file:// target is not a server."""
        with self.assertRaises(ValueError) as ctx:
            resolve_infer_url("file:///etc/passwd")
        self.assertIn("http URL", str(ctx.exception))


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

    @patch("get_action_chunk_adapter.requests.post")
    def test_successful_chunk_populates_trajectory(self, mock_post: MagicMock) -> None:
        """A valid /infer response becomes a JointTrajectory with matching joint names."""
        mock_post.return_value.json.return_value = {
            "action_chunk": [[0.5, 0.6], [0.7, 0.8]],
            "dt": 0.05,
        }
        request = make_request()
        response = self.node._on_request(request, GetActionChunk.Response())

        self.assertEqual(
            response.status, GetActionChunk.Response.CHUNK_PRODUCED, response.message
        )
        self.assertEqual(list(response.chunk.joint_names), ["j1", "j2"])
        self.assertEqual(len(response.chunk.points), 2)
        self.assertEqual(list(response.chunk.points[0].positions), [0.5, 0.6])
        self.assertAlmostEqual(response.native_control_period, 0.05)

    @patch("get_action_chunk_adapter.requests.post")
    def test_prompt_forwarded_as_task_field(self, mock_post: MagicMock) -> None:
        """The fixed request.prompt field is sent to /infer under the 'task' key."""
        mock_post.return_value.json.return_value = {
            "action_chunk": [[0.0, 0.0]],
            "dt": 0.05,
        }
        request = make_request(prompt="pick the red cube")
        self.node._on_request(request, GetActionChunk.Response())

        sent_payload = mock_post.call_args.kwargs["json"]
        self.assertEqual(sent_payload["task"], "pick the red cube")

    @patch("get_action_chunk_adapter.requests.post")
    def test_observation_forwarded_in_payload(self, mock_post: MagicMock) -> None:
        """The joint positions, image names, and new_episode flag all reach the
        payload, posted to the node's configured infer_url."""
        mock_post.return_value.json.return_value = {
            "action_chunk": [[0.0, 0.0]],
            "dt": 0.05,
        }
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
        with patch("get_action_chunk_adapter.requests.post") as mock_post:
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
        with patch("get_action_chunk_adapter.requests.post") as mock_post:
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
        with patch("get_action_chunk_adapter.requests.post") as mock_post:
            response = self.node._on_request(request, GetActionChunk.Response())
            mock_post.assert_not_called()
        self.assertEqual(response.status, GetActionChunk.Response.ERROR)
        self.assertIn("2 positions", response.message)
        self.assertIn("3 joint names", response.message)

    @patch("get_action_chunk_adapter.requests.post")
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

    @patch("get_action_chunk_adapter.requests.post")
    def test_request_carries_frontend_key_bearer_token(
        self, mock_post: MagicMock
    ) -> None:
        """/infer requests present MOVEIT_FRONTEND_KEY as the bearer token the
        server's auth check expects."""
        mock_post.return_value.json.return_value = {
            "action_chunk": [[0.0, 0.0]],
            "dt": 0.05,
        }
        with patch.dict(os.environ, {"MOVEIT_FRONTEND_KEY": "secret-key"}):
            node = GetActionChunkAdapter()
        try:
            node._on_request(make_request(), GetActionChunk.Response())
        finally:
            node.destroy_node()

        self.assertEqual(
            mock_post.call_args.kwargs["headers"],
            {"Authorization": "Bearer secret-key"},
        )

    @patch("get_action_chunk_adapter.requests.post")
    def test_missing_key_sends_no_authorization_header(
        self, mock_post: MagicMock
    ) -> None:
        """Without a key the request goes out bare; the server's 401 detail then
        names the fix through the normal error path."""
        mock_post.return_value.json.return_value = {
            "action_chunk": [[0.0, 0.0]],
            "dt": 0.05,
        }
        with patch.dict(os.environ):
            os.environ.pop("MOVEIT_FRONTEND_KEY", None)
            node = GetActionChunkAdapter()
        try:
            node._on_request(make_request(), GetActionChunk.Response())
        finally:
            node.destroy_node()

        self.assertEqual(mock_post.call_args.kwargs["headers"], {})

    @patch("get_action_chunk_adapter.requests.post")
    def test_timeout_surfaces_as_failure(self, mock_post: MagicMock) -> None:
        """A network-level failure other than a refused connection is reported, not raised."""
        mock_post.side_effect = requests.Timeout("timed out")
        response = self.node._on_request(make_request(), GetActionChunk.Response())

        self.assertEqual(response.status, GetActionChunk.Response.ERROR)
        self.assertIn("/infer request failed", response.message)

    @patch("get_action_chunk_adapter.requests.post")
    def test_http_timeout_tracks_the_parameter(self, mock_post: MagicMock) -> None:
        """The connect + read split sums to the http_timeout parameter (9.0
        default, strictly below the caller's 10.0 service timeout), so a hung
        server cannot wedge the single-threaded node past the caller."""
        mock_post.return_value.json.return_value = {
            "action_chunk": [[0.0, 0.0]],
            "dt": 0.05,
        }
        self.node._on_request(make_request(), GetActionChunk.Response())

        connect_s, read_s = mock_post.call_args.kwargs["timeout"]
        self.assertEqual(connect_s, 3.0)
        self.assertEqual(connect_s + read_s, 9.0)

    @patch("get_action_chunk_adapter.requests.post")
    def test_server_error_field_surfaces_as_failure(self, mock_post: MagicMock) -> None:
        """A server-reported {"error": ...} body fails the request rather than being ignored.

        This is also the path that relays "still loading the model" (503) and
        "model load failed" (500) to the operator."""
        mock_post.return_value.json.return_value = {"error": "checkpoint not loaded"}
        response = self.node._on_request(make_request(), GetActionChunk.Response())

        self.assertEqual(response.status, GetActionChunk.Response.ERROR)
        self.assertIn("checkpoint not loaded", response.message)

    @patch("get_action_chunk_adapter.requests.post")
    def test_huge_server_error_is_truncated(self, mock_post: MagicMock) -> None:
        """A runaway server error string is clipped before it reaches the UI."""
        mock_post.return_value.json.return_value = {"error": "x" * 100_000}
        response = self.node._on_request(make_request(), GetActionChunk.Response())

        self.assertEqual(response.status, GetActionChunk.Response.ERROR)
        self.assertLess(len(response.message), 3000)
        self.assertIn("[...]", response.message)

    @patch("get_action_chunk_adapter.requests.post")
    def test_non_json_error_response_reports_http_status(
        self, mock_post: MagicMock
    ) -> None:
        """A non-JSON body on a failed status still produces a diagnosable message."""
        mock_post.return_value.json.side_effect = ValueError("no json")
        mock_post.return_value.ok = False
        mock_post.return_value.status_code = 502
        response = self.node._on_request(make_request(), GetActionChunk.Response())

        self.assertEqual(response.status, GetActionChunk.Response.ERROR)
        self.assertIn("HTTP 502", response.message)

    @patch("get_action_chunk_adapter.requests.post")
    def test_missing_dt_fails(self, mock_post: MagicMock) -> None:
        """A response with a chunk but no dt is rejected rather than defaulting silently."""
        mock_post.return_value.json.return_value = {"action_chunk": [[0.0, 0.0]]}
        response = self.node._on_request(make_request(), GetActionChunk.Response())

        self.assertEqual(response.status, GetActionChunk.Response.ERROR)
        self.assertIn("missing", response.message)

    @patch("get_action_chunk_adapter.requests.post")
    def test_chunk_width_mismatch_fails(self, mock_post: MagicMock) -> None:
        """A chunk whose column count doesn't match the requested joint count is rejected."""
        mock_post.return_value.json.return_value = {
            "action_chunk": [[0.0, 0.0, 0.0]],  # 3 columns, request has 2 joints
            "dt": 0.05,
        }
        response = self.node._on_request(make_request(), GetActionChunk.Response())

        self.assertEqual(response.status, GetActionChunk.Response.ERROR)
        self.assertIn("does not match", response.message)

    @patch("get_action_chunk_adapter.requests.post")
    def test_chunk_width_mismatch_reports_the_offending_row(
        self, mock_post: MagicMock
    ) -> None:
        """The error names the row that actually mismatched, not always row 0."""
        mock_post.return_value.json.return_value = {
            # row 0 matches the request's 2 joints; row 1 is the actual offender.
            "action_chunk": [[0.0, 0.0], [0.0, 0.0, 0.0]],
            "dt": 0.05,
        }
        response = self.node._on_request(make_request(), GetActionChunk.Response())

        self.assertEqual(response.status, GetActionChunk.Response.ERROR)
        self.assertIn("chunk width 3", response.message)

    @patch("get_action_chunk_adapter.requests.post")
    def test_scalar_chunk_rows_fail_naming_action_chunk(
        self, mock_post: MagicMock
    ) -> None:
        """Rows that aren't lists (a flat or scalar chunk from a custom server)
        are rejected naming action_chunk, not as a generic adapter failure."""
        mock_post.return_value.json.return_value = {
            "action_chunk": [0.0, 0.1],
            "dt": 0.05,
        }
        response = self.node._on_request(make_request(), GetActionChunk.Response())

        self.assertEqual(response.status, GetActionChunk.Response.ERROR)
        self.assertIn("action_chunk is not a steps x dims array", response.message)

    @patch("get_action_chunk_adapter.requests.post")
    def test_non_numeric_chunk_values_fail_naming_action_chunk(
        self, mock_post: MagicMock
    ) -> None:
        """String action values are rejected naming the chunk, not as a raw
        numpy conversion error."""
        mock_post.return_value.json.return_value = {
            "action_chunk": [["a", "b"]],
            "dt": 0.05,
        }
        response = self.node._on_request(make_request(), GetActionChunk.Response())

        self.assertEqual(response.status, GetActionChunk.Response.ERROR)
        self.assertIn("non-numeric action values", response.message)

    @patch("get_action_chunk_adapter.requests.post")
    def test_non_finite_chunk_fails(self, mock_post: MagicMock) -> None:
        """NaN/inf action values are rejected before they reach the controller."""
        mock_post.return_value.json.return_value = {
            "action_chunk": [[0.0, float("nan")]],
            "dt": 0.05,
        }
        response = self.node._on_request(make_request(), GetActionChunk.Response())

        self.assertEqual(response.status, GetActionChunk.Response.ERROR)
        self.assertIn("non-finite", response.message)

    @patch("get_action_chunk_adapter.requests.post")
    def test_non_positive_dt_fails(self, mock_post: MagicMock) -> None:
        """A dt of zero (or below) is rejected: it cannot pace chunk playback."""
        mock_post.return_value.json.return_value = {
            "action_chunk": [[0.0, 0.0]],
            "dt": 0.0,
        }
        response = self.node._on_request(make_request(), GetActionChunk.Response())

        self.assertEqual(response.status, GetActionChunk.Response.ERROR)
        self.assertIn("invalid dt", response.message)

    @patch("get_action_chunk_adapter.requests.post")
    def test_non_finite_dt_fails(self, mock_post: MagicMock) -> None:
        """A NaN dt is rejected before it reaches trajectory timing."""
        mock_post.return_value.json.return_value = {
            "action_chunk": [[0.0, 0.0]],
            "dt": float("nan"),
        }
        response = self.node._on_request(make_request(), GetActionChunk.Response())

        self.assertEqual(response.status, GetActionChunk.Response.ERROR)
        self.assertIn("invalid dt", response.message)

    @patch("get_action_chunk_adapter.requests.post")
    def test_previous_chunk_reshaped_and_forwarded(self, mock_post: MagicMock) -> None:
        """A populated previous_action_chunk is reshaped from its flat layout before sending."""
        mock_post.return_value.json.return_value = {
            "action_chunk": [[0.0, 0.0]],
            "dt": 0.05,
        }
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
        with patch("get_action_chunk_adapter.requests.post") as mock_post:
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
        with patch("get_action_chunk_adapter.requests.post") as mock_post:
            response = self.node._on_request(request, GetActionChunk.Response())
            mock_post.assert_not_called()

        self.assertEqual(response.status, GetActionChunk.Response.ERROR)
        self.assertIn("4 values", response.message)
        self.assertIn("2x3", response.message)

    @patch("get_action_chunk_adapter.requests.post")
    def test_empty_previous_chunk_omits_rtc_fields(self, mock_post: MagicMock) -> None:
        """The first call of an episode (empty previous_action_chunk) sends no RTC carryover."""
        mock_post.return_value.json.return_value = {
            "action_chunk": [[0.0, 0.0]],
            "dt": 0.05,
        }
        self.node._on_request(make_request(), GetActionChunk.Response())

        sent_payload = mock_post.call_args.kwargs["json"]
        self.assertNotIn("prev_chunk_left_over", sent_payload)
        self.assertNotIn("inference_delay", sent_payload)

    @patch("get_action_chunk_adapter.requests.post")
    def test_zero_guidance_horizon_omits_the_field(self, mock_post: MagicMock) -> None:
        """guidance_horizon=0 defers to the server's own RTC default, per the .srv contract."""
        mock_post.return_value.json.return_value = {
            "action_chunk": [[0.0, 0.0]],
            "dt": 0.05,
        }
        self.node._on_request(
            make_request(guidance_horizon=0), GetActionChunk.Response()
        )

        sent_payload = mock_post.call_args.kwargs["json"]
        self.assertNotIn("guidance_horizon", sent_payload)

    @patch("get_action_chunk_adapter.requests.post")
    def test_nonzero_guidance_horizon_is_forwarded(self, mock_post: MagicMock) -> None:
        """A nonzero guidance_horizon from the Objective is forwarded as the soft-guidance
        width; the server, which knows the inference delay, maps it onto lerobot's horizon.
        """
        mock_post.return_value.json.return_value = {
            "action_chunk": [[0.0, 0.0]],
            "dt": 0.05,
        }
        self.node._on_request(
            make_request(guidance_horizon=7), GetActionChunk.Response()
        )

        sent_payload = mock_post.call_args.kwargs["json"]
        self.assertEqual(sent_payload["guidance_horizon"], 7)

    @patch("get_action_chunk_adapter.requests.post")
    def test_action_chunk_raw_echoed_as_policy_action_chunk(
        self, mock_post: MagicMock
    ) -> None:
        """When the server echoes action_chunk_raw, it is reflected back as policy_action_chunk."""
        mock_post.return_value.json.return_value = {
            "action_chunk": [[0.5, 0.6], [0.7, 0.8]],
            "action_chunk_raw": [[-0.1, 0.1], [0.2, -0.2]],
            "dt": 0.05,
        }
        response = self.node._on_request(make_request(), GetActionChunk.Response())

        arr = response.policy_action_chunk
        self.assertEqual([d.size for d in arr.layout.dim], [2, 2])
        self.assertEqual(list(arr.data), [-0.1, 0.1, 0.2, -0.2])

    @patch("get_action_chunk_adapter.requests.post")
    def test_ragged_action_chunk_raw_fails_the_request(
        self, mock_post: MagicMock
    ) -> None:
        """A malformed RTC echo fails the whole request naming the field, never
        success with the echo silently missing."""
        mock_post.return_value.json.return_value = {
            "action_chunk": [[0.5, 0.6]],
            "action_chunk_raw": [[0.1, 0.2], [0.3]],
            "dt": 0.05,
        }
        response = self.node._on_request(make_request(), GetActionChunk.Response())

        self.assertEqual(response.status, GetActionChunk.Response.ERROR)
        self.assertIn("action_chunk_raw", response.message)

    @patch("get_action_chunk_adapter.requests.post")
    def test_zero_width_action_chunk_raw_fails_the_request(
        self, mock_post: MagicMock
    ) -> None:
        """An RTC echo with zero-width rows is rejected, not fed back as a
        structurally valid but empty carryover."""
        mock_post.return_value.json.return_value = {
            "action_chunk": [[0.5, 0.6]],
            "action_chunk_raw": [[], []],
            "dt": 0.05,
        }
        response = self.node._on_request(make_request(), GetActionChunk.Response())

        self.assertEqual(response.status, GetActionChunk.Response.ERROR)
        self.assertIn("action_chunk_raw", response.message)

    @patch("get_action_chunk_adapter.requests.post")
    def test_empty_action_chunk_raw_succeeds_with_empty_echo(
        self, mock_post: MagicMock
    ) -> None:
        """A present-but-empty echo means no RTC carryover, same as an absent one."""
        mock_post.return_value.json.return_value = {
            "action_chunk": [[0.5, 0.6]],
            "action_chunk_raw": [],
            "dt": 0.05,
        }
        response = self.node._on_request(make_request(), GetActionChunk.Response())

        self.assertEqual(
            response.status, GetActionChunk.Response.CHUNK_PRODUCED, response.message
        )
        self.assertEqual(list(response.policy_action_chunk.data), [])

    @patch("get_action_chunk_adapter.requests.post")
    def test_non_finite_action_chunk_raw_fails_the_request(
        self, mock_post: MagicMock
    ) -> None:
        """A NaN in the RTC echo is rejected here, where the field is named,
        not one call later as a confusing previous-chunk error."""
        mock_post.return_value.json.return_value = {
            "action_chunk": [[0.5, 0.6]],
            "action_chunk_raw": [[0.1, float("nan")]],
            "dt": 0.05,
        }
        response = self.node._on_request(make_request(), GetActionChunk.Response())

        self.assertEqual(response.status, GetActionChunk.Response.ERROR)
        self.assertIn("action_chunk_raw", response.message)
        self.assertIn("non-finite", response.message)

    @patch("get_action_chunk_adapter.requests.post")
    def test_non_numeric_action_chunk_raw_fails_the_request(
        self, mock_post: MagicMock
    ) -> None:
        """Rectangular but non-numeric echo content is reported against the
        field, not as a generic adapter failure."""
        mock_post.return_value.json.return_value = {
            "action_chunk": [[0.5, 0.6]],
            "action_chunk_raw": [["a", "b"]],
            "dt": 0.05,
        }
        response = self.node._on_request(make_request(), GetActionChunk.Response())

        self.assertEqual(response.status, GetActionChunk.Response.ERROR)
        self.assertIn("action_chunk_raw", response.message)
        self.assertIn("non-numeric", response.message)

    @patch("get_action_chunk_adapter.requests.post")
    def test_no_action_chunk_raw_leaves_policy_action_chunk_empty(
        self, mock_post: MagicMock
    ) -> None:
        """A policy without RTC support (no action_chunk_raw) leaves policy_action_chunk unset."""
        mock_post.return_value.json.return_value = {
            "action_chunk": [[0.5, 0.6]],
            "dt": 0.05,
        }
        response = self.node._on_request(make_request(), GetActionChunk.Response())

        self.assertEqual(list(response.policy_action_chunk.data), [])


if __name__ == "__main__":
    unittest.main()
