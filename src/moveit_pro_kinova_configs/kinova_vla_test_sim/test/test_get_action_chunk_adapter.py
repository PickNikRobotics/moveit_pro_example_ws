#!/usr/bin/env python3
"""Tests for the GetActionChunk bridge: image encoding and request/response mapping."""

import importlib.util
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

        self.assertTrue(response.success, response.message)
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

    def test_unsupported_image_encoding_fails_without_http_call(self) -> None:
        """An image the encoder can't handle fails locally; /infer is never called."""
        request = make_request(images=[make_image("mono8")])
        with patch("get_action_chunk_adapter.requests.post") as mock_post:
            response = self.node._on_request(request, GetActionChunk.Response())
            mock_post.assert_not_called()
        self.assertFalse(response.success)
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
        self.assertFalse(response.success)
        self.assertIn("length mismatch", response.message)

    @patch("get_action_chunk_adapter.requests.post")
    def test_http_error_surfaces_as_failure(self, mock_post: MagicMock) -> None:
        """A network-level failure calling /infer is reported, not raised."""
        mock_post.side_effect = requests.ConnectionError("refused")
        response = self.node._on_request(make_request(), GetActionChunk.Response())

        self.assertFalse(response.success)
        self.assertIn("/infer request failed", response.message)

    @patch("get_action_chunk_adapter.requests.post")
    def test_server_error_field_surfaces_as_failure(self, mock_post: MagicMock) -> None:
        """A server-reported {"error": ...} body fails the request rather than being ignored."""
        mock_post.return_value.json.return_value = {"error": "checkpoint not loaded"}
        response = self.node._on_request(make_request(), GetActionChunk.Response())

        self.assertFalse(response.success)
        self.assertIn("checkpoint not loaded", response.message)

    @patch("get_action_chunk_adapter.requests.post")
    def test_missing_dt_fails(self, mock_post: MagicMock) -> None:
        """A response with a chunk but no dt is rejected rather than defaulting silently."""
        mock_post.return_value.json.return_value = {"action_chunk": [[0.0, 0.0]]}
        response = self.node._on_request(make_request(), GetActionChunk.Response())

        self.assertFalse(response.success)
        self.assertIn("missing", response.message)

    @patch("get_action_chunk_adapter.requests.post")
    def test_chunk_width_mismatch_fails(self, mock_post: MagicMock) -> None:
        """A chunk whose column count doesn't match the requested joint count is rejected."""
        mock_post.return_value.json.return_value = {
            "action_chunk": [[0.0, 0.0, 0.0]],  # 3 columns, request has 2 joints
            "dt": 0.05,
        }
        response = self.node._on_request(make_request(), GetActionChunk.Response())

        self.assertFalse(response.success)
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

        self.assertFalse(response.success)
        self.assertIn("chunk width 3", response.message)

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
    def test_zero_guidance_horizon_omits_execution_horizon(
        self, mock_post: MagicMock
    ) -> None:
        """guidance_horizon=0 defers to the server's own RTC default, per the .srv contract."""
        mock_post.return_value.json.return_value = {
            "action_chunk": [[0.0, 0.0]],
            "dt": 0.05,
        }
        self.node._on_request(
            make_request(guidance_horizon=0), GetActionChunk.Response()
        )

        sent_payload = mock_post.call_args.kwargs["json"]
        self.assertNotIn("execution_horizon", sent_payload)

    @patch("get_action_chunk_adapter.requests.post")
    def test_nonzero_guidance_horizon_overrides_execution_horizon(
        self, mock_post: MagicMock
    ) -> None:
        """A nonzero guidance_horizon from the Objective is forwarded as execution_horizon."""
        mock_post.return_value.json.return_value = {
            "action_chunk": [[0.0, 0.0]],
            "dt": 0.05,
        }
        self.node._on_request(
            make_request(guidance_horizon=7), GetActionChunk.Response()
        )

        sent_payload = mock_post.call_args.kwargs["json"]
        self.assertEqual(sent_payload["execution_horizon"], 7)

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
