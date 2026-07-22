#!/usr/bin/env python3
"""Tests for serve_policy.py: image decoding and the /infer, /health HTTP handlers.

Runs in the same Python environment as serve_policy.py itself (lerobot/torch/cv2),
not the ROS workspace's pytest suite — see docker/README.md for how to run this.
"""

import base64
import http.client
import json
import threading
import unittest

import cv2
import numpy as np
import torch
from http.server import ThreadingHTTPServer

from serve_policy import decode_image, make_handler


def encode_bgr_jpeg_b64(bgr: np.ndarray) -> str:
    ok, buf = cv2.imencode(".jpg", bgr)
    assert ok
    return base64.b64encode(buf.tobytes()).decode("ascii")


class TestDecodeImage(unittest.TestCase):
    """decode_image: base64 JPEG -> CHW float32 [0,1] RGB tensor."""

    def test_invalid_base64_raises(self) -> None:
        """Malformed image bytes fail loudly (ValueError) instead of returning garbage."""
        with self.assertRaises(ValueError):
            decode_image(base64.b64encode(b"not a jpeg").decode("ascii"))

    def test_output_shape_and_dtype(self) -> None:
        """A 4x2 BGR frame decodes to a (3, 4, 2) float32 tensor scaled to [0, 1]."""
        bgr = np.zeros((4, 2, 3), dtype=np.uint8)
        tensor = decode_image(encode_bgr_jpeg_b64(bgr))

        self.assertEqual(tuple(tensor.shape), (3, 4, 2))
        self.assertEqual(tensor.dtype, torch.float32)
        self.assertGreaterEqual(float(tensor.min()), 0.0)
        self.assertLessEqual(float(tensor.max()), 1.0)

    def test_bgr_to_rgb_channel_order(self) -> None:
        """A pure-blue BGR frame decodes with the red channel near zero (BGR -> RGB swap)."""
        bgr = np.zeros((8, 8, 3), dtype=np.uint8)
        bgr[:, :, 0] = 255  # BGR channel 0 = blue
        tensor = decode_image(encode_bgr_jpeg_b64(bgr))

        # channel 0 = red after the BGR->RGB swap, so it should stay dark despite the
        # source being fully saturated on the blue channel; channel 2 = blue, saturated.
        self.assertLess(float(tensor[0].mean()), 0.2)
        self.assertGreater(float(tensor[2].mean()), 0.8)


class TestHttpHandler(unittest.TestCase):
    """The /health and /infer HTTP endpoints, isolated from real policy inference."""

    class FakeServer:
        """Stands in for PolicyServer: same .lock/.infer(payload) contract, no model."""

        def __init__(self, infer_result=None, infer_error=None) -> None:
            self.lock = threading.Lock()
            self._infer_result = infer_result
            self._infer_error = infer_error

        def infer(self, payload: dict) -> dict:
            if self._infer_error is not None:
                raise self._infer_error
            return self._infer_result

    def _start(
        self, fake_server: "TestHttpHandler.FakeServer"
    ) -> http.client.HTTPConnection:
        httpd = ThreadingHTTPServer(("127.0.0.1", 0), make_handler(fake_server))
        threading.Thread(target=httpd.serve_forever, daemon=True).start()
        self.addCleanup(httpd.shutdown)
        return http.client.HTTPConnection("127.0.0.1", httpd.server_address[1])

    def test_health_returns_ok(self) -> None:
        """GET /health reports ok without touching the (possibly still-loading) server."""
        conn = self._start(self.FakeServer())
        conn.request("GET", "/health")
        resp = conn.getresponse()

        self.assertEqual(resp.status, 200)
        self.assertEqual(json.loads(resp.read())["status"], "ok")

    def test_unknown_path_is_404(self) -> None:
        """A request to any path other than /health or /infer is rejected, not routed."""
        conn = self._start(self.FakeServer())
        conn.request("GET", "/unknown")
        resp = conn.getresponse()

        self.assertEqual(resp.status, 404)

    def test_infer_success_returns_result(self) -> None:
        """A valid POST /infer body is forwarded to server.infer() and its result returned."""
        conn = self._start(
            self.FakeServer(infer_result={"action_chunk": [[0.1]], "dt": 0.05})
        )
        body = json.dumps({"task": "warmup"}).encode()
        conn.request("POST", "/infer", body=body)
        resp = conn.getresponse()

        self.assertEqual(resp.status, 200)
        self.assertEqual(json.loads(resp.read())["dt"], 0.05)

    def test_infer_malformed_json_is_400(self) -> None:
        """A body that isn't valid JSON is rejected before it ever reaches server.infer()."""
        conn = self._start(self.FakeServer())
        conn.request("POST", "/infer", body=b"not json")
        resp = conn.getresponse()

        self.assertEqual(resp.status, 400)

    def test_infer_exception_returns_200_with_error_field(self) -> None:
        """server.infer() raising is caught and reported as {"error": ...}, not a 500.

        The bridge (get_action_chunk_adapter.py) treats any 2xx body containing an
        "error" key as a failed chunk; a raw 500 would instead surface as an unhandled
        requests.HTTPError from raise_for_status().
        """
        conn = self._start(self.FakeServer(infer_error=RuntimeError("model not warm")))
        conn.request("POST", "/infer", body=json.dumps({"task": "x"}).encode())
        resp = conn.getresponse()

        self.assertEqual(resp.status, 200)
        self.assertIn("model not warm", json.loads(resp.read())["error"])


if __name__ == "__main__":
    unittest.main()
