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
import hashlib
import http.client
import json
import os
import tempfile
import threading
import unittest
from http.server import ThreadingHTTPServer
from types import SimpleNamespace
from unittest.mock import MagicMock, patch

import cv2
import numpy as np
import torch
import yaml
from lerobot.configs.types import RTCAttentionSchedule
from lerobot.policies.pi05.modeling_pi05 import PI05Policy
from lerobot.processor import RenameObservationsProcessorStep
from safetensors.torch import save_file
from vla_inference_server import (
    INFER_ABANDONED_SECONDS,
    RELOAD_IDLE_SECONDS,
    REQUEST_SOCKET_TIMEOUT_SECONDS,
    PolicyRunner,
    ServerState,
    apply_bearer_key,
    decode_image_b64,
    hub_access_error_message,
    load_checkpoint_file,
    load_hf_token_file,
    load_policy,
    parse_serving_config,
    read_serving_config,
    make_handler,
    native_camera_map,
    parse_args,
    request_camera_names,
    restart_process,
    resolve_default,
    resolve_device,
    resolve_fps,
    resolve_rtc_horizon,
    resolve_rtc_schedule,
    watch_runtime_inputs,
)


def load_serving_config(path: str) -> dict:
    """Read and parse a serving config the way the server's startup does."""
    return parse_serving_config(path, read_serving_config(path))


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


class TestHuggingFaceTokenFile(unittest.TestCase):
    """The product-owned secret file is safe and environment-compatible."""

    def test_owner_only_token_is_loaded_when_environment_is_absent(self) -> None:
        with tempfile.NamedTemporaryFile("w", delete=False) as handle:
            handle.write("hf_private_token\n")
            path = handle.name
        self.addCleanup(os.unlink, path)
        os.chmod(path, 0o600)

        with patch.dict(os.environ, {}, clear=True):
            self.assertEqual(load_hf_token_file(path), "")
            self.assertEqual(os.environ["HF_TOKEN"], "hf_private_token")

    def test_environment_token_remains_an_explicit_override(self) -> None:
        with tempfile.NamedTemporaryFile("w", delete=False) as handle:
            handle.write("hf_file_token")
            path = handle.name
        self.addCleanup(os.unlink, path)

        with patch.dict(os.environ, {"HF_TOKEN": "hf_environment_token"}, clear=True):
            self.assertEqual(load_hf_token_file(path), "")
            self.assertEqual(os.environ["HF_TOKEN"], "hf_environment_token")

    def test_group_readable_token_is_rejected_without_echoing_it(self) -> None:
        with tempfile.NamedTemporaryFile("w", delete=False) as handle:
            handle.write("hf_must_not_escape")
            path = handle.name
        self.addCleanup(os.unlink, path)
        os.chmod(path, 0o640)

        with patch.dict(os.environ, {}, clear=True):
            error = load_hf_token_file(path)

        self.assertIn("owner-only", error)
        self.assertNotIn("hf_must_not_escape", error)


class TestAutomaticReload(unittest.TestCase):
    """Serving input replacement triggers one process reload."""

    def test_restart_reloads_a_file_token_instead_of_inheriting_it(self) -> None:
        """A removed or replaced token file cannot leave the prior token active."""
        with patch.dict(
            os.environ,
            {
                "HF_TOKEN": "hf_old_file_token",
                "MOVEIT_PRO_HF_TOKEN_FROM_FILE": "1",
                "KEEP_ME": "yes",
            },
            clear=True,
        ):
            with patch("os.execve") as execve:
                restart_process()

        environment = execve.call_args.args[2]
        self.assertNotIn("HF_TOKEN", environment)
        self.assertNotIn("MOVEIT_PRO_HF_TOKEN_FROM_FILE", environment)
        self.assertEqual(environment["KEEP_ME"], "yes")

    def test_config_replacement_restarts_after_debounce(self) -> None:
        with tempfile.NamedTemporaryFile("w", delete=False) as handle:
            handle.write("checkpoint: first\n")
            path = handle.name
        self.addCleanup(os.unlink, path)
        waits = 0

        def replace_after_first_wait(_seconds: float) -> None:
            nonlocal waits
            waits += 1
            if waits == 1:
                replacement = f"{path}.replacement"
                with open(replacement, "w", encoding="utf-8") as handle:
                    handle.write("checkpoint: second\n")
                os.replace(replacement, path)

        restart = MagicMock()
        watch_runtime_inputs([path], wait=replace_after_first_wait, restart=restart)

        restart.assert_called_once_with()
        self.assertEqual(waits, 2)

    def _watch_replaced_config(self, state: ServerState, clock) -> tuple:
        """Run the watcher over a config replaced during its first wait."""
        with tempfile.NamedTemporaryFile("w", delete=False) as handle:
            handle.write("checkpoint: first\n")
            path = handle.name
        self.addCleanup(os.unlink, path)
        waits = 0

        def replace_after_first_wait(_seconds: float) -> None:
            nonlocal waits
            waits += 1
            if waits == 1:
                replacement = f"{path}.replacement"
                with open(replacement, "w", encoding="utf-8") as handle:
                    handle.write("checkpoint: second\n")
                os.replace(replacement, path)

        restart = MagicMock()
        watch_runtime_inputs(
            [path],
            state=state,
            wait=replace_after_first_wait,
            restart=restart,
            clock=clock,
        )
        return restart, waits

    def test_idle_window_is_pinned_to_the_reload_idle_constant(self) -> None:
        """The server is busy until exactly RELOAD_IDLE_SECONDS after an /infer."""
        self.assertTrue(
            ServerState().try_begin_reload(0.0), "No inference yet means idle"
        )

        state = ServerState()
        state.last_infer_monotonic = 100.0
        self.assertFalse(state.try_begin_reload(100.0 + RELOAD_IDLE_SECONDS - 0.001))
        self.assertFalse(state.reloading, "A refused reload must not block /infer")
        self.assertTrue(state.try_begin_reload(100.0 + RELOAD_IDLE_SECONDS))

    def test_in_flight_inference_holds_off_a_reload_until_abandoned(self) -> None:
        """A live /infer blocks the reload; a hung one cannot block it forever."""
        state = ServerState()
        with patch("vla_inference_server.time.monotonic", return_value=100.0):
            self.assertTrue(state.begin_infer())

        self.assertFalse(
            state.try_begin_reload(100.0 + INFER_ABANDONED_SECONDS - 0.001)
        )
        self.assertTrue(state.try_begin_reload(100.0 + INFER_ABANDONED_SECONDS))

    def test_retries_behind_a_hung_inference_do_not_hold_off_a_reload(self) -> None:
        """Requests queued behind a hung /infer cannot keep its reload away."""
        state = ServerState()
        with patch("vla_inference_server.time.monotonic", return_value=100.0):
            self.assertTrue(state.begin_infer())
        # A retry arrives on another handler thread just before the check.
        retry_at = 100.0 + INFER_ABANDONED_SECONDS - 0.001
        with patch("vla_inference_server.time.monotonic", return_value=retry_at):
            retry = threading.Thread(target=state.begin_infer)
            retry.start()
            retry.join()
        self.assertEqual(state.inflight_infers, 2)

        self.assertFalse(state.try_begin_reload(retry_at))
        self.assertTrue(state.try_begin_reload(100.0 + INFER_ABANDONED_SECONDS))

    def test_finished_inference_releases_the_in_flight_count(self) -> None:
        state = ServerState()
        self.assertTrue(state.begin_infer())
        state.end_infer()

        self.assertEqual(state.inflight_infers, 0)
        self.assertIsNotNone(state.last_infer_monotonic)

    def test_committed_reload_refuses_new_inference(self) -> None:
        """No /infer can start between the idle check and the process replacement."""
        state = ServerState()
        self.assertTrue(state.try_begin_reload(0.0))

        self.assertFalse(state.begin_infer())
        self.assertEqual(state.inflight_infers, 0)

    def test_idle_server_restarts_right_after_the_debounce(self) -> None:
        """Without a running policy the reload is as prompt as with no state."""
        state = ServerState()

        restart, waits = self._watch_replaced_config(state, clock=lambda: 0.0)

        restart.assert_called_once_with()
        self.assertEqual(waits, 2)
        self.assertFalse(state.reload_pending)

    def test_failed_process_replacement_keeps_serving_and_retries(self) -> None:
        """An execve failure must not leave /infer refused for good."""
        state = ServerState()
        with tempfile.NamedTemporaryFile("w", delete=False) as handle:
            handle.write("checkpoint: first\n")
            path = handle.name
        self.addCleanup(os.unlink, path)
        waits = 0
        reloading_after_failure = []

        def wait(_seconds: float) -> None:
            nonlocal waits
            waits += 1
            if waits == 1:
                with open(path, "a", encoding="utf-8") as handle:
                    handle.write("fps: 10.0\n")
            if waits == 3:
                reloading_after_failure.append(state.reloading)

        restart = MagicMock(side_effect=[OSError("text file busy"), None])
        watch_runtime_inputs(
            [path], state=state, wait=wait, restart=restart, clock=lambda: 0.0
        )

        self.assertEqual(restart.call_count, 2)
        self.assertEqual(reloading_after_failure, [False])
        # Poll and debounce for the failed attempt, then again for the retry.
        self.assertEqual(waits, 4)

    def test_reload_is_deferred_until_the_running_policy_stops(self) -> None:
        """A config replaced mid-run does not pull the model out from under it."""
        state = ServerState()
        state.last_infer_monotonic = 100.0
        busy = 100.0 + RELOAD_IDLE_SECONDS - 0.001
        idle = 100.0 + RELOAD_IDLE_SECONDS
        # One check after the debounce, two while deferred, then the idle one.
        times = iter([busy, busy, busy, idle])
        restart, waits = self._watch_replaced_config(state, clock=lambda: next(times))

        restart.assert_called_once_with()
        # Poll, debounce, then one wait per busy check inside the deferral.
        self.assertEqual(waits, 4)
        self.assertTrue(state.reload_pending, "/status must report the deferral")


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

    def test_trainer_handoff_fields_are_loaded_from_yaml(self) -> None:
        """The server carries Trainer's exact revision and token requirement."""
        revision = "a" * 40
        text = (
            "checkpoint: acme/model\n"
            f"checkpoint_revision: {revision}\n"
            "checkpoint_requires_hf_token: true\n"
            "moveit_pro_trainer_handoff_version: 1\n"
        )
        with tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False) as f:
            f.write(text)
            path = f.name
        try:
            with patch("sys.argv", ["vla_inference_server.py", "--config", path]):
                args = parse_args()
        finally:
            os.unlink(path)

        self.assertEqual(args.checkpoint_revision, revision)
        self.assertTrue(args.checkpoint_requires_hf_token)
        self.assertEqual(args.trainer_handoff_version, 1)
        # Trainer hashes the same bytes, so /status can name the loaded file.
        self.assertEqual(
            args.config_revision, hashlib.sha256(text.encode()).hexdigest()
        )
        self.assertEqual(args.config_error, "")

    def test_non_integer_handoff_version_parks_in_config_error(self) -> None:
        """A version the server cannot compare is reported, not echoed as-is."""
        with tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False) as f:
            f.write("moveit_pro_trainer_handoff_version: true\n")
            path = f.name
        try:
            with patch("sys.argv", ["vla_inference_server.py", "--config", path]):
                args = parse_args()
        finally:
            os.unlink(path)

        self.assertIsNone(args.trainer_handoff_version)
        self.assertIn("moveit_pro_trainer_handoff_version", args.config_error)

    def test_an_unreadable_config_parks_in_config_error(self) -> None:
        """A read failure parks post-bind like a malformed file, not in a crash."""
        with tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False) as f:
            f.write("checkpoint: acme/model\n")
            path = f.name
        try:
            with patch("sys.argv", ["vla_inference_server.py", "--config", path]):
                with patch(
                    "pathlib.Path.read_bytes", side_effect=PermissionError("denied")
                ):
                    args = parse_args()
        finally:
            os.unlink(path)

        self.assertIsNone(args.config_revision)
        self.assertIn("PermissionError", args.config_error)

    def test_legacy_yaml_keeps_mutable_public_checkpoint_defaults(self) -> None:
        """Existing serving files need no new fields and retain their behavior."""
        with tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False) as f:
            f.write("checkpoint: PickNikRobotics/public-model\nfps: 10.0\n")
            path = f.name
        try:
            with patch("sys.argv", ["vla_inference_server.py", "--config", path]):
                args = parse_args()
        finally:
            os.unlink(path)

        self.assertEqual(args.checkpoint_revision, "")
        self.assertFalse(args.checkpoint_requires_hf_token)
        self.assertIsNone(args.trainer_handoff_version)
        self.assertEqual(args.config_error, "")

    def test_invalid_checkpoint_revision_parks_in_config_error(self) -> None:
        """A mutable branch name cannot masquerade as an immutable handoff."""
        with tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False) as f:
            f.write("checkpoint_revision: main\n")
            path = f.name
        try:
            with patch("sys.argv", ["vla_inference_server.py", "--config", path]):
                args = parse_args()
        finally:
            os.unlink(path)

        self.assertIn("40-character", args.config_error)
        self.assertEqual(args.checkpoint_revision, "")

    def test_cli_rejects_mutable_checkpoint_revision(self) -> None:
        """The CLI cannot bypass the immutable-revision contract."""
        with tempfile.NamedTemporaryFile("w", suffix=".yaml") as config:
            with (
                patch(
                    "sys.argv",
                    [
                        "vla_inference_server.py",
                        "--config",
                        config.name,
                        "--checkpoint-revision",
                        "main",
                    ],
                ),
                self.assertRaises(SystemExit),
            ):
                parse_args()

    def test_cli_can_disable_yaml_hub_token_requirement(self) -> None:
        """An explicit CLI false overrides a private-checkpoint YAML default."""
        with tempfile.NamedTemporaryFile("w", suffix=".yaml") as config:
            config.write("checkpoint_requires_hf_token: true\n")
            config.flush()
            with patch(
                "sys.argv",
                [
                    "vla_inference_server.py",
                    "--config",
                    config.name,
                    "--no-checkpoint-requires-hf-token",
                ],
            ):
                args = parse_args()

        self.assertFalse(args.checkpoint_requires_hf_token)


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

    def test_required_hub_token_fails_closed_before_checkpoint_access(self) -> None:
        """A private Trainer checkpoint is never loaded through anonymous Hub access."""
        state = ServerState()
        args = argparse.Namespace(
            config_error="",
            checkpoint="acme/private-model",
            checkpoint_revision="a" * 40,
            checkpoint_requires_hf_token=True,
        )

        with patch.dict(os.environ, {}, clear=True):
            load_policy(state, args)

        self.assertEqual(state.status, "error")
        self.assertIn("requires Hugging Face model access", state.detail)

    def test_local_checkpoint_rejects_revision_before_metadata_resolution(self) -> None:
        """Explicit metadata cannot let a Hub-only revision reach a local loader."""
        state = ServerState()
        with tempfile.TemporaryDirectory() as checkpoint:
            args = argparse.Namespace(
                config_error="",
                checkpoint=checkpoint,
                checkpoint_revision="a" * 40,
                checkpoint_requires_hf_token=False,
                fps=10.0,
                policy_class="pi05",
                device="cpu",
                guidance_horizon=8,
                rtc_schedule="EXP",
                state_dim=8,
            )
            with patch("vla_inference_server.PolicyRunner") as runner:
                load_policy(state, args)

        self.assertEqual(state.status, "error")
        self.assertIn("local checkpoint directory", state.detail)
        runner.assert_not_called()


class TestReloadIdleWindow(unittest.TestCase):
    """load_policy sizes the reload idle window from the checkpoint's own chunk."""

    def _load(self, fps: float, warmup) -> ServerState:
        state = ServerState()
        args = argparse.Namespace(
            config_error="",
            checkpoint="acme/model",
            checkpoint_revision="",
            checkpoint_requires_hf_token=False,
            fps=fps,
            policy_class="pi05",
            device="cpu",
            guidance_horizon=8,
            rtc_schedule="EXP",
            state_dim=8,
        )
        runner = MagicMock()
        runner.policy.config.input_features = {}
        runner.warmup = warmup
        with patch("vla_inference_server.PolicyRunner", return_value=runner):
            load_policy(state, args)
        self.assertEqual(state.status, "ready", state.detail)
        return state

    def test_window_is_one_chunk_of_playback_but_never_below_the_floor(self) -> None:
        """No valid commit can leave /infer quiet for longer than one chunk."""
        cases = [
            # fps, chunk steps, expected window
            (5.0, 100, 20.0),
            (10.0, 50, RELOAD_IDLE_SECONDS),
            (50.0, 50, RELOAD_IDLE_SECONDS),
        ]
        for fps, chunk_steps, expected in cases:
            with self.subTest(fps=fps, chunk_steps=chunk_steps):
                state = self._load(fps, MagicMock(return_value=(1.0, 0.1, chunk_steps)))
                self.assertEqual(state.reload_idle_seconds, expected)

    def test_a_long_chunk_holds_off_a_reload_for_its_whole_playback(self) -> None:
        """A 20 second chunk is still a running policy 19 seconds after /infer."""
        state = self._load(5.0, MagicMock(return_value=(1.0, 0.1, 100)))
        state.last_infer_monotonic = 100.0

        self.assertFalse(state.try_begin_reload(100.0 + 20.0 - 0.001))
        self.assertTrue(state.try_begin_reload(100.0 + 20.0))

    def test_failed_warmup_keeps_the_floor(self) -> None:
        """Without a measured chunk the server still serves, on the default window."""
        state = self._load(5.0, MagicMock(side_effect=RuntimeError("padded state")))

        self.assertEqual(state.reload_idle_seconds, RELOAD_IDLE_SECONDS)


class TestImmutableCheckpointLoading(unittest.TestCase):
    """Every checkpoint file and LeRobot loader reads the same Hub commit."""

    def test_checkpoint_json_download_uses_the_revision(self) -> None:
        revision = "a" * 40
        with tempfile.NamedTemporaryFile("w", suffix=".json", delete=False) as f:
            json.dump({"type": "pi05"}, f)
            path = f.name
        self.addCleanup(os.unlink, path)

        with patch(
            "vla_inference_server.hf_hub_download", return_value=path
        ) as download:
            result = load_checkpoint_file("acme/model", "config.json", revision)

        self.assertEqual(result, {"type": "pi05"})
        download.assert_called_once_with(
            repo_id="acme/model", filename="config.json", revision=revision
        )

    def test_policy_and_processors_load_the_pinned_snapshot(self) -> None:
        """No LeRobot loader is handed a repo id it could resolve to another commit."""
        revision = "b" * 40
        snapshot = "/hf/hub/models--acme--model/snapshots/" + revision
        config = SimpleNamespace(input_features={}, rtc_config=None)
        policy = MagicMock(config=config)
        policy_class = SimpleNamespace(from_pretrained=MagicMock(return_value=policy))
        pre = SimpleNamespace(steps=[])
        post = SimpleNamespace(steps=[])

        with (
            patch("vla_inference_server.get_policy_class", return_value=policy_class),
            patch(
                "vla_inference_server.snapshot_download", return_value=snapshot
            ) as download,
            patch(
                "vla_inference_server.make_pre_post_processors",
                return_value=(pre, post),
            ) as processors,
        ):
            PolicyRunner(
                "acme/model",
                "pi05",
                "cpu",
                8,
                "EXP",
                8,
                checkpoint_revision=revision,
            )

        download.assert_called_once_with("acme/model", revision=revision)
        policy_class.from_pretrained.assert_called_once_with(snapshot)
        processors.assert_called_once_with(
            policy_cfg=config,
            pretrained_path=snapshot,
            preprocessor_overrides={"device_processor": {"device": "cpu"}},
            postprocessor_overrides={"device_processor": {"device": "cpu"}},
        )

    def test_policy_runner_legacy_call_uses_existing_mutable_defaults(self) -> None:
        """The original positional constructor remains valid and follows Hub HEAD."""
        config = SimpleNamespace(input_features={}, rtc_config=None)
        policy = MagicMock(config=config)
        policy_class = SimpleNamespace(from_pretrained=MagicMock(return_value=policy))

        with (
            patch("vla_inference_server.get_policy_class", return_value=policy_class),
            patch("vla_inference_server.snapshot_download") as download,
            patch(
                "vla_inference_server.make_pre_post_processors",
                return_value=(SimpleNamespace(steps=[]), SimpleNamespace(steps=[])),
            ) as processors,
        ):
            PolicyRunner("acme/model", "pi05", "cpu", 8, "EXP", 8)

        download.assert_not_called()
        policy_class.from_pretrained.assert_called_once_with("acme/model")
        self.assertEqual(processors.call_args.kwargs["pretrained_path"], "acme/model")

    def test_pi05_loader_reads_weights_from_a_snapshot_directory(self) -> None:
        """LeRobot's real pi0.5 weight resolution serves the directory it is given.

        That loader drops `revision` on its weights download, so pinning relies
        on a local snapshot resolving to its own model.safetensors.
        """
        with tempfile.TemporaryDirectory() as snapshot:
            save_file(
                {"model.marker": torch.tensor([979.0])},
                os.path.join(snapshot, "model.safetensors"),
            )
            with (
                patch.object(PI05Policy, "__init__", return_value=None),
                patch.object(PI05Policy, "config", SimpleNamespace(), create=True),
                patch.object(
                    PI05Policy,
                    "_fix_pytorch_state_dict_keys",
                    side_effect=lambda state_dict, _config: state_dict,
                ),
                patch.object(
                    PI05Policy, "load_state_dict", return_value=([], [])
                ) as load,
            ):
                PI05Policy.from_pretrained(snapshot, config=SimpleNamespace())

        load.assert_called_once()
        self.assertEqual(load.call_args.args[0]["model.marker"].item(), 979.0)


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


class ObservedState(ServerState):
    """Signals when the handler finishes recording an /infer."""

    def __init__(self) -> None:
        super().__init__()
        self.infer_ended = threading.Event()

    def end_infer(self) -> None:
        super().end_infer()
        self.infer_ended.set()


class TestApplyBearerKey(unittest.TestCase):
    """Fail-closed handling of the MOVEIT_INFERENCE_KEY environment value."""

    def test_blank_or_missing_key_parks_error_state(self) -> None:
        """An unset or blank key parks the server so /health names the fix.

        The frontend key is not a fallback, so an environment carrying only
        that key parks the server too.
        """
        for environment in (
            {},
            {"MOVEIT_INFERENCE_KEY": ""},
            {"MOVEIT_INFERENCE_KEY": "   "},
            {"MOVEIT_FRONTEND_KEY": "f" * 64},
        ):
            with self.subTest(environment=environment):
                state = ServerState()
                self.assertFalse(apply_bearer_key(state, environment))
                self.assertEqual(state.status, "error")
                self.assertEqual(state.bearer_key, "")
                self.assertIn("MOVEIT_INFERENCE_KEY", state.detail)
                self.assertIn("moveit_pro run", state.detail)
                # /health serves the detail without a token; it must never
                # name a key value, only the commands that set one.
                self.assertNotIn("f" * 64, state.detail)

    def test_valid_key_is_stored_stripped(self) -> None:
        """A usable key is stored without surrounding whitespace."""
        state = ServerState()
        self.assertTrue(
            apply_bearer_key(state, {"MOVEIT_INFERENCE_KEY": "  secret-key \n"})
        )
        self.assertEqual(state.bearer_key, "secret-key")
        self.assertEqual(state.status, "loading")


class TestInferenceKeyAuthentication(unittest.TestCase):
    """Only the provisioned inference key is accepted."""

    def test_only_provisioned_key_is_accepted(self) -> None:
        """/infer and /status reject a missing or mismatched key."""
        state = ServerState()
        self.assertTrue(apply_bearer_key(state, {"MOVEIT_INFERENCE_KEY": "a" * 64}))
        server = ThreadingHTTPServer(("127.0.0.1", 0), make_handler(state))
        worker = threading.Thread(target=server.serve_forever, daemon=True)
        worker.start()
        self.addCleanup(worker.join)
        self.addCleanup(server.server_close)
        self.addCleanup(server.shutdown)
        connection = http.client.HTTPConnection(
            "127.0.0.1", server.server_port, timeout=2
        )
        self.addCleanup(connection.close)
        for path, method in (("/infer", "POST"), ("/status", "GET")):
            for credential in ("", "b" * 64, "a" * 64):
                with self.subTest(path=path, credential=credential):
                    connection.request(
                        method,
                        path,
                        body="{}",
                        headers={"Authorization": "Bearer " + credential},
                    )
                    response = connection.getresponse()
                    response.read()
                    if credential == "a" * 64:
                        expected = 503 if method == "POST" else 200
                    else:
                        expected = 401
                    self.assertEqual(response.status, expected)


class TestHttpStateMachine(unittest.TestCase):
    """/health and /infer across the loading -> ready/error lifecycle."""

    # Auth key served by every test server; _infer presents it by default.
    TEST_KEY = "test-inference-key"

    def _start(self, state: ServerState) -> http.client.HTTPConnection:
        state.bearer_key = self.TEST_KEY
        httpd = ThreadingHTTPServer(("127.0.0.1", 0), make_handler(state))
        threading.Thread(target=httpd.serve_forever, daemon=True).start()
        # LIFO: shutdown() stops the serve loop first, then server_close()
        # frees the listening socket.
        self.addCleanup(httpd.server_close)
        self.addCleanup(httpd.shutdown)
        return http.client.HTTPConnection("127.0.0.1", httpd.server_address[1])

    def _ready_state(
        self, runner: FakeRunner | None = None, state: ServerState | None = None
    ) -> ServerState:
        state = state or ServerState()
        # The loader thread resolves fps before flipping to "ready"; mirror that here.
        state.fps = 20.0
        state.runner = runner or FakeRunner()
        state.checkpoint = "acme/model"
        state.checkpoint_revision = "a" * 40
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
        """POST /infer without the inference key is rejected before any other check."""
        conn = self._start(self._ready_state())
        conn.request("POST", "/infer", body=b"{}")
        resp = conn.getresponse()

        self.assertEqual(resp.status, 401)
        self.assertIn("MOVEIT_INFERENCE_KEY", json.loads(resp.read())["error"])

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

    def test_status_requires_token_and_reports_loaded_revision(self) -> None:
        """The Runtime can confirm the exact model without exposing it publicly."""
        state = self._ready_state()
        state.config_revision = "c" * 64
        state.trainer_handoff_version = 1
        conn = self._start(state)
        conn.request("GET", "/status")
        self.assertEqual(conn.getresponse().status, 401)

        conn.request("GET", "/status", headers=self._auth_header())
        response = conn.getresponse()
        body = json.loads(response.read())

        self.assertEqual(response.status, 200)
        self.assertEqual(body["state"], "ready")
        self.assertEqual(body["checkpoint"], "acme/model")
        self.assertEqual(body["checkpointRevision"], "a" * 40)
        self.assertIs(body["reloadPending"], False)
        # Trainer compares these with the file it wrote and the contract it needs.
        self.assertEqual(body["configRevision"], "c" * 64)
        self.assertEqual(body["trainerHandoffVersion"], 1)

    def test_status_reports_a_deferred_reload(self) -> None:
        """A reload waiting on the running policy is visible, not a silent stall."""
        state = self._ready_state()
        state.reload_pending = True
        conn = self._start(state)

        conn.request("GET", "/status", headers=self._auth_header())
        body = json.loads(conn.getresponse().read())

        self.assertIs(body["reloadPending"], True)
        self.assertEqual(body["state"], "ready")

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

    def _assert_infer_recorded(self, runner: FakeRunner, expected_status: int) -> None:
        state = self._ready_state(runner, ObservedState())
        conn = self._start(state)

        status, _body = self._infer(conn, self._valid_payload())

        self.assertEqual(status, expected_status)
        # The handler stamps the end after it responds; wait for that stamp.
        self.assertTrue(state.infer_ended.wait(timeout=5.0))
        self.assertEqual(state.inflight_infers, 0)
        self.assertIsNotNone(state.last_infer_monotonic)
        self.assertFalse(
            state.try_begin_reload(state.last_infer_monotonic),
            "A just-served /infer must hold off a reload",
        )

    def test_served_infer_holds_off_a_reload(self) -> None:
        """A successful /infer marks the server busy for the reload watcher."""
        self._assert_infer_recorded(FakeRunner(), 200)

    def test_failed_infer_releases_the_in_flight_count(self) -> None:
        """A policy exception cannot leave the server permanently busy."""
        self._assert_infer_recorded(FakeRunner(RuntimeError("cuda OOM")), 500)

    def test_infer_during_a_committed_reload_is_503(self) -> None:
        """A request racing the process replacement is refused, not dropped."""
        state = self._ready_state()
        self.assertTrue(state.try_begin_reload(0.0))
        conn = self._start(state)

        status, body = self._infer(conn, self._valid_payload())

        self.assertEqual(status, 503)
        self.assertIn("reloading", body["error"])

    def test_rejected_infer_does_not_hold_off_a_reload(self) -> None:
        """An unauthenticated caller cannot postpone a reload indefinitely."""
        state = self._ready_state()
        conn = self._start(state)

        conn.request("POST", "/infer", body=b"{}")
        response = conn.getresponse()
        response.read()

        self.assertEqual(response.status, 401)
        self.assertIsNone(state.last_infer_monotonic)
        self.assertEqual(state.inflight_infers, 0)


if __name__ == "__main__":
    unittest.main()
