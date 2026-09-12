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

from safetensors import safe_open
from safetensors.torch import load_file, save_file
import torchao
from torchao.prototype.safetensors.safetensors_support import (
    flatten_tensor_state_dict,
    unflatten_tensor_state_dict,
)
from torchao.quantization import Int8Tensor, Int8WeightOnlyConfig, quantize_

from vla_inference_server import (
    INT8_MARKER,
    MODEL_WEIGHTS_FILE,
    QUANTIZATION_KEY,
    REQUEST_SOCKET_TIMEOUT_SECONDS,
    TORCHAO_VERSION_KEY,
    ServerState,
    apply_frontend_key,
    assign_quantized_weights,
    decode_image_b64,
    hub_access_error_message,
    load_full_policy,
    load_policy,
    load_serving_config,
    make_handler,
    native_camera_map,
    parse_args,
    read_quantization,
    read_weights_metadata,
    request_camera_names,
    resolve_default,
    resolve_device,
    resolve_fps,
    resolve_quantization,
    resolve_rtc_horizon,
    resolve_rtc_schedule,
    resolve_weights_file,
    save_quantized_checkpoint,
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

    def test_non_boolean_int8_parks_in_config_error(self) -> None:
        """A quoted or misspelled int8 lands in config_error with the flag off.
        Coercing it with bool() would read every non-empty string as true, so the
        server would quantize the policy the operator asked to leave alone."""
        # GIVEN a serving config whose int8 is a string rather than a boolean
        with tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False) as f:
            f.write('int8: "false"\n')
            path = f.name
        try:
            # WHEN parsing arguments against that config
            with patch("sys.argv", ["vla_inference_server.py", "--config", path]):
                args = parse_args()
        finally:
            os.unlink(path)

        # THEN the value is reported and the flag stays off
        self.assertIn("int8", args.config_error)
        self.assertFalse(args.int8)

    def test_boolean_int8_is_honored(self) -> None:
        """A real YAML boolean reaches the flag, so the knob works as documented."""
        # GIVEN a serving config asking for int8
        with tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False) as f:
            f.write("int8: true\n")
            path = f.name
        try:
            # WHEN parsing arguments against that config
            with patch("sys.argv", ["vla_inference_server.py", "--config", path]):
                args = parse_args()
        finally:
            os.unlink(path)

        # THEN the flag is on and nothing is reported
        self.assertTrue(args.int8)
        self.assertEqual(args.config_error, "")


class TestReadQuantization(unittest.TestCase):
    """read_quantization: the marker that decides which loader a file goes to.

    An ordinary checkpoint and a quantized one are both model.safetensors, and
    handing the second to the loader written for the first fails deep inside
    torch on a name it does not recognize. The marker is read before either
    loader runs.
    """

    def test_ordinary_weights_declare_nothing(self) -> None:
        """A file written without our metadata reads as unquantized, not as unknown."""
        # GIVEN a weights file saved the way lerobot saves one
        with tempfile.TemporaryDirectory() as directory:
            path = os.path.join(directory, MODEL_WEIGHTS_FILE)
            save_file({"weight": torch.zeros(2, 2)}, path)

            # WHEN its quantization is read
            # THEN it declares none
            self.assertEqual(read_quantization(path), "")

    def test_marked_weights_declare_int8(self) -> None:
        """The marker survives the round trip through safetensors' metadata."""
        # GIVEN a weights file carrying the marker
        with tempfile.TemporaryDirectory() as directory:
            path = os.path.join(directory, MODEL_WEIGHTS_FILE)
            save_file(
                {"weight": torch.zeros(2, 2)},
                path,
                metadata={QUANTIZATION_KEY: INT8_MARKER},
            )

            # WHEN its quantization is read
            # THEN it is the one that was written
            self.assertEqual(read_quantization(path), INT8_MARKER)

    def test_unquantized_weights_resolve_to_the_ordinary_loader(self) -> None:
        """Declaring nothing resolves to "", which is the full-width path."""
        # GIVEN a weights file saved the way lerobot saves one
        with tempfile.TemporaryDirectory() as directory:
            path = os.path.join(directory, MODEL_WEIGHTS_FILE)
            save_file({"weight": torch.zeros(2, 2)}, path)

            # WHEN the loader to use is resolved
            # THEN it is the ordinary one
            self.assertEqual(resolve_quantization(path), "")

    def test_an_unreadable_quantization_is_refused(self) -> None:
        """A marker this server does not know raises instead of falling through.

        Falling through hands a file in some other format to the full-width
        loader, which fails much later and deep inside torch. The marker exists
        to route the file, so an unroutable one is an error here.
        """
        # GIVEN a weights file declaring a quantization this server cannot read
        with tempfile.TemporaryDirectory() as directory:
            path = os.path.join(directory, MODEL_WEIGHTS_FILE)
            save_file(
                {"weight": torch.zeros(2, 2)},
                path,
                metadata={QUANTIZATION_KEY: "int4"},
            )

            # WHEN the loader to use is resolved
            with self.assertRaises(ValueError) as caught:
                resolve_quantization(path)

            # THEN the error names what was declared and what is readable
            message = str(caught.exception)
            self.assertIn("int4", message)
            self.assertIn(INT8_MARKER, message)

    def test_local_checkpoint_resolves_without_the_hub(self) -> None:
        """A local directory resolves to its own file, so an offline host still loads."""
        # GIVEN a checkpoint directory on disk
        with tempfile.TemporaryDirectory() as directory:
            path = os.path.join(directory, MODEL_WEIGHTS_FILE)
            save_file({"weight": torch.zeros(2, 2)}, path)

            # WHEN the weights file is resolved from the directory
            # THEN it is that file, with no hub lookup
            self.assertEqual(resolve_weights_file(directory), path)

    def test_directory_without_weights_resolves_to_nothing(self) -> None:
        """A directory missing its weights file defers, rather than asking the hub.

        Treating the path as a repo id gets an HF validation error about
        alphanumeric characters, where lerobot's own load names the file it
        could not find.
        """
        # GIVEN a checkpoint directory with no weights in it
        with tempfile.TemporaryDirectory() as directory:
            # WHEN the weights file is resolved from it
            # THEN nothing is resolved and nothing is fetched
            self.assertEqual(resolve_weights_file(directory), "")


class TestQuantizedRoundTrip(unittest.TestCase):
    """Saving quantized weights and loading them back into a fresh model.

    Two assumptions the pre-quantized checkpoint rests on, neither of them
    obvious: torchao's flattening is a prototype API, and assign=True is what
    keeps an int8 weight int8, where an ordinary load would copy it into the
    skeleton's own full-width tensor and quietly undo the quantization. Two
    small linear layers exercise both in milliseconds, where a real pi0.5
    checkpoint costs minutes.
    """

    @staticmethod
    def build_model() -> torch.nn.Module:
        torch.manual_seed(0)
        return torch.nn.Sequential(torch.nn.Linear(64, 32), torch.nn.Linear(32, 8))

    @staticmethod
    def build_quantized() -> torch.nn.Module:
        model = TestQuantizedRoundTrip.build_model()
        quantize_(model, Int8WeightOnlyConfig(version=2, set_inductor_config=False))
        return model

    @staticmethod
    def write_source(directory: str) -> str:
        """A source checkpoint: full-width weights plus the sidecars beside them."""
        source = os.path.join(directory, "source")
        os.makedirs(source)
        for name in ("config.json", "policy_preprocessor.json"):
            with open(os.path.join(source, name), "w") as f:
                f.write("{}")
        save_file({"old": torch.ones(9, 9)}, os.path.join(source, MODEL_WEIGHTS_FILE))
        return source

    def test_int8_weights_survive_save_and_load(self) -> None:
        """The loaded model is still quantized and answers exactly as the saved one did."""
        # GIVEN a quantized model and what it makes of a fixed input
        model = self.build_quantized()
        sample = torch.randn(4, 64)
        with torch.no_grad():
            expected = model(sample)

        with tempfile.TemporaryDirectory() as directory:
            source = self.write_source(directory)
            out = os.path.join(directory, "out")

            # WHEN it is written out and read back into a full-width skeleton
            save_quantized_checkpoint(model, source, out)
            skeleton = self.build_model()
            skeleton.requires_grad_(False)
            assign_quantized_weights(skeleton, os.path.join(out, MODEL_WEIGHTS_FILE))

            # THEN the weights are still int8, and the answers are unchanged
            self.assertIsInstance(skeleton[0].weight, Int8Tensor)
            self.assertEqual(skeleton[0].weight.qdata.dtype, torch.int8)
            with torch.no_grad():
                self.assertTrue(torch.equal(skeleton(sample), expected))

    def test_copying_weights_instead_of_assigning_them_is_refused(self) -> None:
        """An ordinary load raises rather than quietly dequantizing.

        assign=True is the whole mechanism, so what happens without it is worth
        pinning: the failure is loud, not a model that silently serves at full
        width.
        """
        with tempfile.TemporaryDirectory() as directory:
            # GIVEN a written quantized checkpoint
            out = os.path.join(directory, "out")
            save_quantized_checkpoint(
                self.build_quantized(), self.write_source(directory), out
            )
            written = os.path.join(out, MODEL_WEIGHTS_FILE)
            state_dict, _ = unflatten_tensor_state_dict(
                load_file(written), read_weights_metadata(written)
            )

            # WHEN its tensors are copied into a skeleton rather than assigned
            # THEN the load fails
            skeleton = self.build_model()
            with self.assertRaises(Exception):
                skeleton.load_state_dict(state_dict, assign=False, strict=True)

    def test_tensors_the_metadata_misses_name_the_torchao_versions(self) -> None:
        """A file whose metadata does not cover its tensors fails with both versions."""
        with tempfile.TemporaryDirectory() as directory:
            # GIVEN a weights file carrying a tensor its metadata omits
            model = self.build_quantized()
            tensors, metadata = flatten_tensor_state_dict(model.state_dict())
            tensors["unaccounted"] = torch.zeros(2, 2)
            metadata[TORCHAO_VERSION_KEY] = "0.0.1"
            path = os.path.join(directory, MODEL_WEIGHTS_FILE)
            save_file(tensors, path, metadata=metadata)

            # WHEN it is assigned into a skeleton
            with self.assertRaises(ValueError) as caught:
                assign_quantized_weights(self.build_model(), path)

            # THEN the error names the tensor and both torchao versions
            message = str(caught.exception)
            self.assertIn("unaccounted", message)
            self.assertIn("0.0.1", message)
            self.assertIn(torchao.__version__, message)

    def test_saving_keeps_the_checkpoint_whole(self) -> None:
        """Everything but the weights is carried across, and the old weights are not.

        The processors load from the same directory as the weights, so a
        checkpoint that arrives without them loads into a policy that cannot
        normalize an observation.
        """
        with tempfile.TemporaryDirectory() as directory:
            # GIVEN a source checkpoint with the files a policy needs beside its weights
            source = self.write_source(directory)

            # WHEN a quantized policy is written out against it
            out = os.path.join(directory, "out")
            copied = save_quantized_checkpoint(self.build_quantized(), source, out)

            # THEN the sidecars came along, the weights are the new ones, and
            # the file says what it is
            self.assertEqual(copied, ["config.json", "policy_preprocessor.json"])
            written = os.path.join(out, MODEL_WEIGHTS_FILE)
            self.assertNotIn("old", load_file(written))
            self.assertEqual(read_quantization(written), INT8_MARKER)
            self.assertEqual(
                read_weights_metadata(written)[TORCHAO_VERSION_KEY],
                torchao.__version__,
            )

    def test_everything_written_is_readable_by_another_user(self) -> None:
        """Weights and sidecars alike land 0644, not at whatever wrote them.

        safetensors writes 0600 whatever the umask is, and a copy carries the
        source's mode, so a checkpoint served by a container running as a
        different uid needs both made readable.
        """
        with tempfile.TemporaryDirectory() as directory:
            # GIVEN a source checkpoint whose sidecar is readable only by its owner
            source = self.write_source(directory)
            os.chmod(os.path.join(source, "config.json"), 0o600)

            # WHEN a quantized policy is written out against it
            out = os.path.join(directory, "out")
            save_quantized_checkpoint(self.build_quantized(), source, out)

            # THEN the directory is traversable and every file in it is readable
            self.assertEqual(os.stat(out).st_mode & 0o777, 0o755)
            for name in os.listdir(out):
                self.assertEqual(
                    os.stat(os.path.join(out, name)).st_mode & 0o777,
                    0o644,
                    f"{name} is not readable by the serving user",
                )

    def test_full_width_weights_are_not_marked_quantized(self) -> None:
        """Saving an unquantized policy is refused, so the marker stays a fact.

        The server reads the marker to decide how to load, and reports it on
        /health, so a checkpoint that claims int8 without being int8 serves at
        full width while saying otherwise.
        """
        with tempfile.TemporaryDirectory() as directory:
            # GIVEN a model that was never quantized
            source = self.write_source(directory)
            out = os.path.join(directory, "out")

            # WHEN it is written out as a quantized checkpoint
            # THEN the write is refused, and nothing is left behind
            with self.assertRaises(ValueError):
                save_quantized_checkpoint(self.build_model(), source, out)
            self.assertFalse(os.path.exists(out))

    def test_writing_over_an_existing_checkpoint_is_refused(self) -> None:
        """An existing destination is left alone rather than half-overwritten."""
        with tempfile.TemporaryDirectory() as directory:
            # GIVEN a destination that already exists
            source = self.write_source(directory)
            out = os.path.join(directory, "out")
            os.makedirs(out)

            # WHEN a quantized policy is written to it
            # THEN the write is refused
            with self.assertRaises(FileExistsError):
                save_quantized_checkpoint(self.build_quantized(), source, out)

    def test_quantizing_a_quantized_checkpoint_is_refused(self) -> None:
        """A written checkpoint is not a source the full-width loader can read.

        lerobot reports the state-dict mismatch and carries on, so without this
        the second pass would quantize untrained weights and write a checkpoint
        that looks entirely valid.
        """
        with tempfile.TemporaryDirectory() as directory:
            # GIVEN a checkpoint this server has already quantized
            source = self.write_source(directory)
            out = os.path.join(directory, "out")
            save_quantized_checkpoint(self.build_quantized(), source, out)

            # WHEN it is handed back to the full-width loader
            # THEN the read is refused before any weight is touched
            with self.assertRaises(ValueError) as caught:
                load_full_policy(out, "pi05", None, int8=True)
            self.assertIn(INT8_MARKER, str(caught.exception))

    def test_subdirectories_in_the_source_are_carried_across(self) -> None:
        """A nested processor directory comes along instead of being dropped."""
        with tempfile.TemporaryDirectory() as directory:
            # GIVEN a source checkpoint with a nested directory
            source = self.write_source(directory)
            os.makedirs(os.path.join(source, "tokenizer"))
            with open(os.path.join(source, "tokenizer", "vocab.json"), "w") as f:
                f.write("{}")

            # WHEN a quantized policy is written out against it
            out = os.path.join(directory, "out")
            copied = save_quantized_checkpoint(self.build_quantized(), source, out)

            # THEN the directory and its contents came too
            self.assertIn("tokenizer", copied)
            self.assertTrue(
                os.path.isfile(os.path.join(out, "tokenizer", "vocab.json"))
            )


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
        self.int8 = False
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
