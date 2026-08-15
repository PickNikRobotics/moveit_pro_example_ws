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

"""Tests for vla_inference_server.py: resolvers, image decoding, and the HTTP state machine.

Runs in the same Python environment as vla_inference_server.py itself (lerobot/torch/cv2),
not the ROS workspace's pytest suite (see README.md for how to run this).
"""

import argparse
import base64
import http.client
import json
import os
import tempfile
import threading
import unittest
from unittest.mock import patch
from http.server import ThreadingHTTPServer

import cv2
import numpy as np
import torch
import yaml
from lerobot.configs.types import RTCAttentionSchedule
from lerobot.processor import RenameObservationsProcessorStep

from vla_inference_server import (
    REQUEST_SOCKET_TIMEOUT_SECONDS,
    ServerState,
    apply_frontend_key,
    decode_image_b64,
    hub_access_error_message,
    load_policy,
    load_serving_config,
    make_handler,
    native_camera_map,
    parse_args,
    request_camera_names,
    resolve_default,
    resolve_device,
    resolve_fps,
    resolve_rtc_horizon,
    resolve_rtc_schedule,
    torch_accelerator_backend,
)


def encode_bgr_jpeg_b64(bgr: np.ndarray) -> str:
    ok, buf = cv2.imencode(".jpg", bgr)
    assert ok
    return base64.b64encode(buf.tobytes()).decode("ascii")


class TestResolveDevice(unittest.TestCase):
    """resolve_device: auto-selection and fail-loud explicit requests."""

    def test_auto_prefers_cuda_when_available(self) -> None:
        """device=auto on a GPU host serves on cuda, never silently on cpu."""
        self.assertEqual(resolve_device("auto", cuda_available=True), "cuda")

    def test_auto_falls_back_to_cpu(self) -> None:
        """device=auto without a GPU serves on cpu."""
        self.assertEqual(resolve_device("auto", cuda_available=False), "cpu")

    def test_explicit_cuda_without_gpu_raises(self) -> None:
        """An explicit cuda request on a CPU-only host is a startup error."""
        with self.assertRaises(ValueError):
            resolve_device("cuda", cuda_available=False)

    def test_explicit_cpu_always_honored(self) -> None:
        """An explicit cpu request is honored even when a GPU exists."""
        self.assertEqual(resolve_device("cpu", cuda_available=True), "cpu")


class TestTorchAcceleratorBackend(unittest.TestCase):
    """torch_accelerator_backend: build provenance, independent of device spelling."""

    @staticmethod
    def _torch_version(*, hip=None, cuda=None):
        return type(
            "FakeTorch",
            (),
            {"version": type("Version", (), {"hip": hip, "cuda": cuda})()},
        )()

    def test_rocm_build_wins_even_though_device_is_cuda(self) -> None:
        """ROCm is identified from build metadata, not torch's shared cuda API."""
        module = self._torch_version(hip="7.2.2", cuda=None)
        self.assertEqual(torch_accelerator_backend(module), "rocm")

    def test_cuda_build_is_identified(self) -> None:
        """A CUDA-only distribution reports the NVIDIA backend."""
        module = self._torch_version(hip=None, cuda="13.0")
        self.assertEqual(torch_accelerator_backend(module), "cuda")

    def test_cpu_build_is_identified(self) -> None:
        """A distribution with neither GPU build marker reports CPU."""
        module = self._torch_version(hip=None, cuda=None)
        self.assertEqual(torch_accelerator_backend(module), "cpu")


class TestLoadServingConfig(unittest.TestCase):
    """load_serving_config: tolerant of missing/empty, loud on malformed."""

    def _write(self, text: str) -> str:
        with tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False) as handle:
            handle.write(text)
            path = handle.name
        self.addCleanup(os.unlink, path)
        return path

    def test_missing_file_returns_empty(self) -> None:
        """A path with no file yields {}, so env and built-in defaults still apply."""
        # GIVEN a path that does not exist
        # WHEN loading the serving config
        result = load_serving_config("/nonexistent/vla_serving.yaml")

        # THEN it is an empty dict, not an error
        self.assertEqual(result, {})

    def test_empty_file_returns_empty(self) -> None:
        """An empty YAML file is tolerated the same as a missing one."""
        # GIVEN an empty file
        path = self._write("")

        # WHEN loading it
        # THEN it yields {} rather than raising
        self.assertEqual(load_serving_config(path), {})

    def test_valid_file_parses_each_knob_with_native_type(self) -> None:
        """A well-formed file parses keys with their YAML-native types."""
        # GIVEN a well-formed serving config
        path = self._write("checkpoint: org/model\nfps: 10.0\nstate_dim: 8\n")

        # WHEN loading it
        config = load_serving_config(path)

        # THEN each knob carries its native type
        self.assertEqual(config["checkpoint"], "org/model")
        self.assertEqual(config["fps"], 10.0)
        self.assertEqual(config["state_dim"], 8)

    def test_malformed_file_raises(self) -> None:
        """Broken YAML fails loudly instead of silently serving the wrong model."""
        # GIVEN a syntactically broken YAML file
        path = self._write("checkpoint: [unterminated\n")

        # WHEN loading it
        # THEN it raises, so the loader thread can park in the error state
        with self.assertRaises(yaml.YAMLError):
            load_serving_config(path)

    def test_non_mapping_file_raises(self) -> None:
        """A top-level list is rejected, since knobs are looked up by key."""
        # GIVEN a YAML file whose top level is a list
        path = self._write("- checkpoint\n- fps\n")

        # WHEN loading it
        # THEN it is rejected with a message naming the expected shape
        with self.assertRaises(ValueError):
            load_serving_config(path)


class TestResolveDefault(unittest.TestCase):
    """resolve_default: YAML > built-in for an argparse default."""

    def test_yaml_beats_builtin(self) -> None:
        """A YAML value wins over the built-in."""
        # GIVEN a YAML value
        # WHEN resolving the default
        # THEN the YAML value is used
        self.assertEqual(resolve_default("cpu", "auto"), "cpu")

    def test_builtin_used_when_yaml_absent(self) -> None:
        """With no YAML value, the built-in is returned."""
        # GIVEN no YAML value
        # WHEN resolving
        # THEN the built-in default is returned
        self.assertEqual(resolve_default(None, "auto"), "auto")

    def test_yaml_zero_is_honored_over_builtin(self) -> None:
        """A YAML value of 0 (the fps/state_dim auto sentinel) is honored, not skipped."""
        # GIVEN a YAML value of 0 and a non-zero built-in
        # WHEN resolving
        # THEN 0 is returned, not treated as absent
        self.assertEqual(resolve_default(0, 8), 0)


class TestParseArgsCoercion(unittest.TestCase):
    """parse_args: type-invalid YAML values defer to the error state, never crash."""

    def test_non_numeric_yaml_values_park_in_config_error(self) -> None:
        """A non-numeric fps or state_dim in the YAML lands in config_error with the
        built-in default applied, so the socket still binds and the loader thread
        reports the typo through /health instead of a pre-bind crash loop."""
        # GIVEN a serving config whose fps and state_dim are not numbers
        with tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False) as f:
            f.write("fps: ten\nstate_dim: [7, 1]\n")
            path = f.name
        try:
            # WHEN parsing arguments against that config
            with patch("sys.argv", ["vla_inference_server.py", "--config", path]):
                args = parse_args()
        finally:
            os.unlink(path)

        # THEN both bad values are reported and the built-ins are used
        self.assertIn("fps", args.config_error)
        self.assertIn("state_dim", args.config_error)
        self.assertEqual(args.fps, 0.0)
        self.assertEqual(args.state_dim, 0)


class TestLoadPolicyMissingCheckpoint(unittest.TestCase):
    """load_policy: an unset checkpoint parks the error state, never exits."""

    def test_empty_checkpoint_parks_error_state(self) -> None:
        """With no checkpoint configured, the loader thread parks in the error
        state naming the fix, so the socket stays bound and /health plus the
        objective's UI messages report it instead of the process exiting."""
        # GIVEN parsed args with a readable config but no checkpoint
        state = ServerState()
        args = argparse.Namespace(config_error="", checkpoint="")

        # WHEN the loader runs
        load_policy(state, args)

        # THEN the server is parked in the error state with actionable detail
        self.assertEqual(state.status, "error")
        self.assertIn("checkpoint", state.detail)
        self.assertIn("vla_serving.yaml", state.detail)


class TestResolveFps(unittest.TestCase):
    """resolve_fps: explicit value wins; unresolvable rate is an error."""

    def test_explicit_fps_wins(self) -> None:
        """A positive fps skips the checkpoint lookup entirely."""
        self.assertEqual(resolve_fps("/nonexistent", 10.0), 10.0)

    def test_unresolvable_fps_raises(self) -> None:
        """fps=0 with no readable train_config.json is a startup error, not a default."""
        with self.assertRaises(ValueError):
            resolve_fps("nonexistent-checkpoint", 0.0)


class TestHubAccessErrorMessage(unittest.TestCase):
    """hub_access_error_message: each access failure names its own fix."""

    def test_gated_without_token_says_export_it(self) -> None:
        """A gated repo with no token points at exporting HF_TOKEN."""
        message = hub_access_error_message("org/model", gated=True, token_present=False)
        self.assertIn("HF_TOKEN is not set", message)
        self.assertIn("export HF_TOKEN", message)

    def test_gated_with_token_says_accept_the_license(self) -> None:
        """A gated repo with a token present points at license acceptance, not the token."""
        message = hub_access_error_message("org/model", gated=True, token_present=True)
        self.assertIn("has not been granted access", message)
        self.assertNotIn("export HF_TOKEN", message)

    def test_not_found_without_token_mentions_private_repos(self) -> None:
        """An unknown repo without a token flags both a typo and the private case."""
        message = hub_access_error_message(
            "org/model", gated=False, token_present=False
        )
        self.assertIn("vla_serving.yaml", message)
        self.assertIn("private repo", message)

    def test_not_found_with_token_points_at_the_name(self) -> None:
        """An unknown repo with a token present points at the checkpoint name."""
        message = hub_access_error_message("org/typo", gated=False, token_present=True)
        self.assertIn("org/typo", message)
        self.assertIn("vla_serving.yaml", message)


class TestResolveRtcHorizon(unittest.TestCase):
    """resolve_rtc_horizon: the service's guidance width -> lerobot's absolute horizon."""

    def test_request_width_extends_past_the_prefix(self) -> None:
        """The guided region ends inference_delay + width steps into the chunk, so a
        width smaller than the delay can never shrink the frozen prefix."""
        self.assertEqual(resolve_rtc_horizon(9, 8, 12), 17)

    def test_zero_width_uses_the_server_default(self) -> None:
        """guidance_horizon=0 defers to the server's configured width, per the contract."""
        self.assertEqual(resolve_rtc_horizon(9, 0, 12), 21)

    def test_zero_delay_passes_the_width_through(self) -> None:
        """With no frozen prefix the horizon is just the guidance width."""
        self.assertEqual(resolve_rtc_horizon(0, 8, 12), 8)


class TestResolveRtcSchedule(unittest.TestCase):
    """resolve_rtc_schedule: named schedules resolve; typos name the valid values."""

    def test_known_schedule_resolves(self) -> None:
        """A valid schedule name maps onto lerobot's enum."""
        self.assertEqual(resolve_rtc_schedule("EXP"), RTCAttentionSchedule.EXP)

    def test_unknown_schedule_names_the_valid_values(self) -> None:
        """A typo'd schedule fails with a message listing the valid names and
        pointing at the knob's file."""
        with self.assertRaises(ValueError) as ctx:
            resolve_rtc_schedule("exp")
        self.assertIn("EXP", str(ctx.exception))
        self.assertIn("vla_serving.yaml", str(ctx.exception))


class TestDecodeImageB64(unittest.TestCase):
    """decode_image_b64: base64 JPEG -> CHW float32 [0,1] RGB tensor."""

    def test_invalid_base64_raises(self) -> None:
        """Malformed image bytes fail loudly (ValueError) instead of returning garbage."""
        with self.assertRaises(ValueError):
            decode_image_b64(base64.b64encode(b"not a jpeg").decode("ascii"))

    def test_output_shape_and_dtype(self) -> None:
        """A 4x2 BGR frame decodes to a (3, 4, 2) float32 tensor scaled to [0, 1]."""
        bgr = np.zeros((4, 2, 3), dtype=np.uint8)
        tensor = decode_image_b64(encode_bgr_jpeg_b64(bgr))

        self.assertEqual(tuple(tensor.shape), (3, 4, 2))
        self.assertEqual(tensor.dtype, torch.float32)
        self.assertGreaterEqual(float(tensor.min()), 0.0)
        self.assertLessEqual(float(tensor.max()), 1.0)

    def test_bgr_to_rgb_channel_order(self) -> None:
        """A pure-blue BGR frame decodes with the red channel near zero (BGR -> RGB swap)."""
        bgr = np.zeros((8, 8, 3), dtype=np.uint8)
        bgr[:, :, 0] = 255  # BGR channel 0 = blue
        tensor = decode_image_b64(encode_bgr_jpeg_b64(bgr))

        # channel 0 = red after the BGR->RGB swap, so it should stay dark despite the
        # source being fully saturated on the blue channel; channel 2 = blue, saturated.
        self.assertLess(float(tensor[0].mean()), 0.2)
        self.assertGreater(float(tensor[2].mean()), 0.8)


class TestNativeCameraMap(unittest.TestCase):
    """native_camera_map: dataset-native camera names from the preprocessor pipeline."""

    def test_rename_step_yields_prefix_stripped_image_map(self) -> None:
        """Image entries lose the feature prefix; non-image entries are ignored."""
        step = RenameObservationsProcessorStep(
            rename_map={
                "observation.images.overview": "observation.images.base_0_rgb",
                "observation.images.scene": "observation.images.right_wrist_0_rgb",
                "observation.env_state": "observation.state",
            }
        )
        self.assertEqual(
            native_camera_map([step]),
            {"overview": "base_0_rgb", "scene": "right_wrist_0_rgb"},
        )

    def test_pipeline_without_rename_step_yields_empty_map(self) -> None:
        """A checkpoint whose dataset already used the slot names offers no aliases."""
        self.assertEqual(native_camera_map([object()]), {})

    def test_first_image_renaming_step_wins_and_warns(self) -> None:
        """With two image-renaming steps the first defines the request names,
        and the ambiguity is logged next to the load's request-names line."""
        first = RenameObservationsProcessorStep(
            rename_map={"observation.images.front": "observation.images.scene"}
        )
        second = RenameObservationsProcessorStep(
            rename_map={"observation.images.top": "observation.images.scene"}
        )
        with patch("vla_inference_server.log") as mock_log:
            result = native_camera_map([first, second])

        self.assertEqual(result, {"front": "scene"})
        self.assertIn("WARNING", mock_log.call_args[0][0])

    def test_non_image_rename_step_does_not_mask_a_later_image_one(self) -> None:
        """A step renaming only state keys is skipped; the image-renaming step
        behind it still defines the camera names, with no ambiguity warning."""
        state_only = RenameObservationsProcessorStep(
            rename_map={"observation.env_state": "observation.state"}
        )
        images = RenameObservationsProcessorStep(
            rename_map={"observation.images.front": "observation.images.scene"}
        )
        with patch("vla_inference_server.log") as mock_log:
            result = native_camera_map([state_only, images])

        self.assertEqual(result, {"front": "scene"})
        mock_log.assert_not_called()


class TestRequestCameraNames(unittest.TestCase):
    """request_camera_names: the /infer image keys for a checkpoint, in order."""

    def test_partial_rename_mixes_native_and_slot_names(self) -> None:
        """A camera the checkpoint renames takes its dataset name; one it does
        not rename keeps its slot name, in checkpoint-declared order."""
        self.assertEqual(
            request_camera_names(["scene", "aux"], {"front": "scene"}),
            ["front", "aux"],
        )

    def test_no_rename_map_keeps_slot_names(self) -> None:
        """Without a rename step the config.json slot names are the request names."""
        self.assertEqual(request_camera_names(["a", "b"], {}), ["a", "b"])

    def test_many_to_one_rename_warns_and_uses_the_last(self) -> None:
        """Two dataset names mapping onto one slot cannot both be honored; the
        collision is logged and the last one becomes the request name."""
        with patch("vla_inference_server.log") as mock_log:
            result = request_camera_names(["scene"], {"a": "scene", "b": "scene"})

        self.assertEqual(result, ["b"])
        self.assertIn("WARNING", mock_log.call_args[0][0])


class FakeRunner:
    """Stands in for PolicyRunner: same expected_state_dim/infer contract, no model."""

    def __init__(
        self,
        infer_error: Exception | None = None,
        camera_keys: list | None = None,
        native_map: dict | None = None,
    ) -> None:
        self.device = "cpu"
        self._infer_error = infer_error
        # Like PolicyRunner, derived once at construction.
        self.request_names = request_camera_names(
            camera_keys if camera_keys is not None else ["scene"],
            native_map if native_map is not None else {},
        )

    def expected_state_dim(self) -> int:
        return 2

    def infer(
        self, images, state, prompt, prev_chunk, inference_delay, guidance_horizon
    ):
        if self._infer_error is not None:
            raise self._infer_error
        return np.array([[0.1, 0.2]]), np.array([[0.5, 0.5]])


class TestApplyFrontendKey(unittest.TestCase):
    """Fail-closed handling of the MOVEIT_FRONTEND_KEY environment value."""

    def test_blank_or_missing_key_parks_error_state(self) -> None:
        """An unset or blank key parks the server so /health names the fix."""
        for raw_key in (None, "", "   "):
            state = ServerState()
            self.assertFalse(apply_frontend_key(state, raw_key))
            self.assertEqual(state.status, "error")
            self.assertIn("MOVEIT_FRONTEND_KEY", state.detail)
            # /health serves the detail without a token; it must never name a
            # usable key value, only point at the docs.
            self.assertNotIn("moveit-secret-key", state.detail)

    def test_valid_key_is_stored_stripped(self) -> None:
        """A usable key is stored without surrounding whitespace."""
        state = ServerState()
        self.assertTrue(apply_frontend_key(state, "  secret-key \n"))
        self.assertEqual(state.frontend_key, "secret-key")
        self.assertEqual(state.status, "loading")


class TestHttpStateMachine(unittest.TestCase):
    """/health and /infer across the loading -> ready/error lifecycle."""

    # Auth key served by every test server; _infer presents it by default.
    TEST_KEY = "test-frontend-key"

    def _start(self, state: ServerState) -> http.client.HTTPConnection:
        state.frontend_key = self.TEST_KEY
        httpd = ThreadingHTTPServer(("127.0.0.1", 0), make_handler(state))
        threading.Thread(target=httpd.serve_forever, daemon=True).start()
        # LIFO: shutdown() stops the serve loop first, then server_close()
        # frees the listening socket.
        self.addCleanup(httpd.server_close)
        self.addCleanup(httpd.shutdown)
        return http.client.HTTPConnection("127.0.0.1", httpd.server_address[1])

    def _ready_state(self, runner: FakeRunner | None = None) -> ServerState:
        state = ServerState()
        # The loader thread resolves fps before flipping to "ready"; mirror that here.
        state.fps = 20.0
        state.runner = runner or FakeRunner()
        state.status = "ready"
        return state

    def _auth_header(self) -> dict:
        return {"Authorization": f"Bearer {self.TEST_KEY}"}

    def _infer(self, conn: http.client.HTTPConnection, payload: dict):
        conn.request(
            "POST",
            "/infer",
            body=json.dumps(payload).encode(),
            headers=self._auth_header(),
        )
        resp = conn.getresponse()
        return resp.status, json.loads(resp.read())

    def _valid_payload(self) -> dict:
        blank = encode_bgr_jpeg_b64(np.zeros((4, 4, 3), dtype=np.uint8))
        # Request image names are the checkpoint's own camera keys; FakeRunner
        # expects "scene", so the observation supplies "scene".
        return {"state": [0.0, 0.0], "task": "stack", "images": {"scene": blank}}

    def test_handler_sets_connection_timeout(self) -> None:
        """A half-open connection cannot park its handler thread forever: the
        handler applies a socket timeout to every connection."""
        handler_cls = make_handler(ServerState())
        self.assertEqual(handler_cls.timeout, REQUEST_SOCKET_TIMEOUT_SECONDS)
        self.assertGreater(REQUEST_SOCKET_TIMEOUT_SECONDS, 0)

    def test_health_reports_loading(self) -> None:
        """GET /health during model load reports 'loading', usable as a startup probe."""
        conn = self._start(ServerState())
        conn.request("GET", "/health")
        resp = conn.getresponse()

        self.assertEqual(resp.status, 200)
        self.assertEqual(json.loads(resp.read())["status"], "loading")

    def test_health_reports_error_with_detail(self) -> None:
        """GET /health after a failed load carries the load error for diagnosis."""
        state = ServerState()
        state.status = "error"
        state.detail = "ValueError: no config.json"
        conn = self._start(state)
        conn.request("GET", "/health")
        body = json.loads(conn.getresponse().read())

        self.assertEqual(body["status"], "error")
        self.assertIn("no config.json", body["detail"])

    def test_unknown_path_is_404(self) -> None:
        """A request to any path other than /health or /infer is rejected, not routed."""
        conn = self._start(self._ready_state())
        conn.request("GET", "/unknown")
        self.assertEqual(conn.getresponse().status, 404)

    def test_infer_while_loading_is_503_with_message(self) -> None:
        """POST /infer during model load answers 503 'still loading', which the
        adapter relays verbatim to the MoveIt Pro UI."""
        conn = self._start(ServerState())
        status, body = self._infer(conn, {"task": "x"})

        self.assertEqual(status, 503)
        self.assertIn("still loading", body["error"])

    def test_infer_after_failed_load_is_500_with_detail(self) -> None:
        """POST /infer after a failed load relays the load error, not a generic 500."""
        state = ServerState()
        state.status = "error"
        state.detail = "ValueError: checkpoint directory does not exist"
        conn = self._start(state)
        status, body = self._infer(conn, {"task": "x"})

        self.assertEqual(status, 500)
        self.assertIn("checkpoint directory does not exist", body["error"])

    def test_infer_without_token_is_401(self) -> None:
        """POST /infer without the shared key is rejected before any other check."""
        conn = self._start(self._ready_state())
        conn.request("POST", "/infer", body=b"{}")
        resp = conn.getresponse()

        self.assertEqual(resp.status, 401)
        self.assertIn("MOVEIT_FRONTEND_KEY", json.loads(resp.read())["error"])

    def test_infer_with_wrong_token_is_401(self) -> None:
        """A mismatched key is rejected the same as a missing one."""
        conn = self._start(self._ready_state())
        conn.request(
            "POST",
            "/infer",
            body=b"{}",
            headers={"Authorization": "Bearer wrong-key"},
        )
        self.assertEqual(conn.getresponse().status, 401)

    def test_infer_while_loading_still_requires_token(self) -> None:
        """Auth wraps the whole endpoint: an unauthenticated probe cannot even
        distinguish the loading state."""
        conn = self._start(ServerState())
        conn.request("POST", "/infer", body=b"{}")
        self.assertEqual(conn.getresponse().status, 401)

    def test_infer_with_non_ascii_token_is_401(self) -> None:
        """A non-ASCII token gets a clean 401, not a dropped connection:
        compare_digest on str raises TypeError for non-ASCII input."""
        conn = self._start(self._ready_state())
        conn.request(
            "POST",
            "/infer",
            body=b"{}",
            headers={"Authorization": "Bearer café-key"},
        )
        self.assertEqual(conn.getresponse().status, 401)

    def test_infer_accepts_case_insensitive_bearer_scheme(self) -> None:
        """The auth scheme is case-insensitive per RFC 7235."""
        conn = self._start(self._ready_state())
        conn.request(
            "POST",
            "/infer",
            body=json.dumps(self._valid_payload()).encode(),
            headers={"Authorization": f"bEaReR {self.TEST_KEY}"},
        )
        self.assertEqual(conn.getresponse().status, 200)

    def test_health_needs_no_token(self) -> None:
        """GET /health stays token-free so health probes keep working."""
        conn = self._start(self._ready_state())
        conn.request("GET", "/health")
        resp = conn.getresponse()

        self.assertEqual(resp.status, 200)
        self.assertEqual(json.loads(resp.read())["status"], "ready")

    def test_health_reports_accelerator_provenance_and_warmup(self) -> None:
        """A ready health response proves which binary backend ran the model."""
        state = self._ready_state()
        state.accelerator = "rocm"
        state.torch_version = "2.10.0+rocm7.2.2"
        state.warmup_metrics = {"steady_seconds": 1.25, "chunk_steps": 50}
        conn = self._start(state)
        conn.request("GET", "/health")
        body = json.loads(conn.getresponse().read())

        self.assertEqual(body["device"], "cpu")
        self.assertEqual(body["accelerator"], "rocm")
        self.assertEqual(body["torch_version"], "2.10.0+rocm7.2.2")
        self.assertEqual(body["warmup"]["chunk_steps"], 50)

    def test_infer_malformed_json_is_400(self) -> None:
        """A body that isn't valid JSON is rejected before it reaches the policy."""
        conn = self._start(self._ready_state())
        conn.request("POST", "/infer", body=b"not json", headers=self._auth_header())
        self.assertEqual(conn.getresponse().status, 400)

    def test_infer_negative_content_length_is_400(self) -> None:
        """A negative Content-Length is rejected before any body read."""
        conn = self._start(self._ready_state())
        conn.putrequest("POST", "/infer", skip_accept_encoding=True)
        conn.putheader("Authorization", f"Bearer {self.TEST_KEY}")
        conn.putheader("Content-Length", "-1")
        conn.endheaders()
        self.assertEqual(conn.getresponse().status, 400)

    def test_infer_oversized_body_is_413(self) -> None:
        """A declared body size over the ceiling is rejected before it is read."""
        conn = self._start(self._ready_state())
        conn.request(
            "POST",
            "/infer",
            body=b"x",
            headers={"Content-Length": str(2**40), **self._auth_header()},
        )
        self.assertEqual(conn.getresponse().status, 413)

    def test_infer_success_returns_chunk_and_dt(self) -> None:
        """A valid POST /infer runs the policy and returns the chunk with dt=1/fps."""
        conn = self._start(self._ready_state())
        status, body = self._infer(conn, self._valid_payload())

        self.assertEqual(status, 200)
        self.assertEqual(body["action_chunk"], [[0.1, 0.2]])
        self.assertEqual(body["action_chunk_raw"], [[0.5, 0.5]])
        self.assertAlmostEqual(body["dt"], 0.05)

    def test_infer_missing_expected_camera_is_400_naming_it(self) -> None:
        """Images that omit one of the checkpoint's cameras are rejected as the
        caller's error (400), because lerobot would otherwise zero-fill the
        camera and run the policy blind."""
        conn = self._start(self._ready_state())  # FakeRunner expects "scene"
        blank = encode_bgr_jpeg_b64(np.zeros((4, 4, 3), dtype=np.uint8))
        # The request supplies "front", not the checkpoint's expected "scene".
        status, body = self._infer(
            conn, {"state": [0.0, 0.0], "task": "stack", "images": {"front": blank}}
        )

        self.assertEqual(status, 400)
        self.assertIn("scene", body["error"])
        self.assertIn("image_names", body["error"])

    def test_infer_native_camera_names_accepted(self) -> None:
        """A complete set of dataset-native names serves a chunk: the checkpoint's
        own rename step maps them onto the model slots."""
        runner = FakeRunner(native_map={"front": "scene"})
        conn = self._start(self._ready_state(runner))
        blank = encode_bgr_jpeg_b64(np.zeros((4, 4, 3), dtype=np.uint8))
        status, body = self._infer(
            conn, {"state": [0.0, 0.0], "task": "stack", "images": {"front": blank}}
        )

        self.assertEqual(status, 200)
        self.assertIn("action_chunk", body)

    def test_infer_slot_names_on_renaming_checkpoint_rejected_with_fix(self) -> None:
        """A renaming checkpoint takes its dataset camera names only; the
        config.json slot names are refused with a message naming the names to
        use instead."""
        runner = FakeRunner(native_map={"front": "scene"})
        conn = self._start(self._ready_state(runner))
        blank = encode_bgr_jpeg_b64(np.zeros((4, 4, 3), dtype=np.uint8))
        status, body = self._infer(
            conn, {"state": [0.0, 0.0], "task": "stack", "images": {"scene": blank}}
        )

        self.assertEqual(status, 400)
        self.assertIn("front", body["error"])
        self.assertIn("image_names", body["error"])

    def test_infer_partial_rename_native_full_set_accepted(self) -> None:
        """A checkpoint renaming only some cameras accepts the mixed native set:
        the dataset name where one exists, the slot name where it does not."""
        runner = FakeRunner(camera_keys=["scene", "aux"], native_map={"front": "scene"})
        conn = self._start(self._ready_state(runner))
        blank = encode_bgr_jpeg_b64(np.zeros((4, 4, 3), dtype=np.uint8))
        status, body = self._infer(
            conn,
            {
                "state": [0.0, 0.0],
                "task": "stack",
                "images": {"front": blank, "aux": blank},
            },
        )

        self.assertEqual(status, 200)
        self.assertIn("action_chunk", body)

    def test_infer_extra_camera_name_is_400_naming_it(self) -> None:
        """A request carrying both a dataset name and the slot it renames to is
        refused: lerobot's rename step would silently overwrite one with the
        other and feed the policy the wrong camera."""
        runner = FakeRunner(native_map={"front": "scene"})
        conn = self._start(self._ready_state(runner))
        blank = encode_bgr_jpeg_b64(np.zeros((4, 4, 3), dtype=np.uint8))
        status, body = self._infer(
            conn,
            {
                "state": [0.0, 0.0],
                "task": "stack",
                "images": {"front": blank, "scene": blank},
            },
        )

        self.assertEqual(status, 400)
        self.assertIn("unexpected", body["error"])
        self.assertIn("scene", body["error"])

    def test_infer_partial_rename_missing_unrenamed_camera_is_400(self) -> None:
        """Covering only the renamed camera is refused: the unrenamed one would
        be silently zero-filled and the policy would run partially blind."""
        runner = FakeRunner(camera_keys=["scene", "aux"], native_map={"front": "scene"})
        conn = self._start(self._ready_state(runner))
        blank = encode_bgr_jpeg_b64(np.zeros((4, 4, 3), dtype=np.uint8))
        status, body = self._infer(
            conn, {"state": [0.0, 0.0], "task": "stack", "images": {"front": blank}}
        )

        self.assertEqual(status, 400)
        self.assertIn("aux", body["error"])

    def test_infer_state_width_mismatch_is_400_with_both_widths(self) -> None:
        """A state narrower than the checkpoint expects is the caller's error
        (400) and names both widths."""
        conn = self._start(self._ready_state())
        payload = self._valid_payload()
        payload["state"] = [0.0]
        status, body = self._infer(conn, payload)

        self.assertEqual(status, 400)
        self.assertIn("1-dim state", body["error"])
        self.assertIn("expects 2", body["error"])

    def test_infer_non_list_state_is_400(self) -> None:
        """A scalar or string state is the caller's error (400), not an
        internal 500 from torch.tensor."""
        conn = self._start(self._ready_state())
        payload = self._valid_payload()
        payload["state"] = "oops"
        status, body = self._infer(conn, payload)

        self.assertEqual(status, 400)
        self.assertIn("must be a list", body["error"])

    def test_infer_non_string_image_value_is_400(self) -> None:
        """A non-string camera value is the caller's error (400), not a
        TypeError-turned-500 from base64."""
        conn = self._start(self._ready_state())
        payload = self._valid_payload()
        payload["images"]["scene"] = 5
        status, body = self._infer(conn, payload)

        self.assertEqual(status, 400)
        self.assertIn("base64", body["error"])

    def test_infer_non_finite_state_is_400(self) -> None:
        """A NaN joint position from a degraded publisher is the caller's
        error (400), not policy input."""
        conn = self._start(self._ready_state())
        payload = self._valid_payload()
        payload["state"] = [0.0, float("nan")]
        status, body = self._infer(conn, payload)

        self.assertEqual(status, 400)
        self.assertIn("finite", body["error"])

    def test_infer_non_finite_prev_chunk_is_400(self) -> None:
        """A NaN in the RTC carryover is the caller's error (400), not
        guidance input."""
        conn = self._start(self._ready_state())
        payload = self._valid_payload()
        payload["prev_chunk_left_over"] = [[0.1, float("nan")]]
        status, body = self._infer(conn, payload)

        self.assertEqual(status, 400)
        self.assertIn("prev_chunk_left_over", body["error"])

    def test_infer_exception_is_500_with_error_field(self) -> None:
        """A policy exception returns 500 with {"error": ...}; the adapter parses the
        body before checking the status, so the detail still reaches the operator."""
        conn = self._start(self._ready_state(FakeRunner(RuntimeError("cuda OOM"))))
        status, body = self._infer(conn, self._valid_payload())

        self.assertEqual(status, 500)
        self.assertIn("cuda OOM", body["error"])


if __name__ == "__main__":
    unittest.main()
