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

"""CPU-only native backend tests; fake sessions never load QAIRT or model weights."""

import ctypes
import hashlib
import http.client
import json
import os
import tempfile
import threading
import unittest
from contextlib import contextmanager
from http.server import ThreadingHTTPServer
from pathlib import Path
from unittest.mock import MagicMock, patch

import numpy as np
import torch
import yaml
from lerobot.configs.types import RTCAttentionSchedule
from lerobot.policies.pi05.modeling_pi05 import resize_with_pad_torch
from lerobot.policies.rtc.configuration_rtc import RTCConfig
from lerobot.policies.rtc.modeling_rtc import RTCProcessor

from test_vla_inference_server import encode_bgr_jpeg_b64
from vla_inference_server import (
    QNN_LOAD_ABANDONED_SECONDS,
    PolicyRunner,
    QnnPolicyRunner,
    ServerState,
    load_policy,
    make_handler,
    parse_args,
    run_inference,
    watch_runtime_inputs,
)
from vla_qnn_export import sampling_values
from vla_qnn_policy import (
    FOUR_GRAPH_ABI,
    FOUR_GRAPH_ORDER,
    SIX_GRAPH_ABI,
    SIX_GRAPH_ORDER,
    graph_ports,
    guide_native_update,
    prepare_rtc,
    resize_with_pad,
    validate_bundle,
)
from vla_qnn_runtime import (
    ERROR,
    Hardware,
    Infrastructure,
    NativeModel,
    NativeRuntime,
    Perf,
    Platform,
    PlatformV1,
    Power,
    Tensor,
    function,
)


def expert_update(actions):
    """Deterministic fake already-integrated expert output, not neural inference."""
    return actions * np.float32(0.99) - np.float32(0.002)


@contextmanager
def http_connection(state):
    """Serve the real handler on container loopback, without a production server."""
    server = ThreadingHTTPServer(("127.0.0.1", 0), make_handler(state))
    server.daemon_threads = False
    worker = threading.Thread(target=server.serve_forever, daemon=True)
    worker.start()
    connection = http.client.HTTPConnection("127.0.0.1", server.server_port, timeout=5)
    try:
        yield connection
    finally:
        connection.close()
        server.shutdown()
        server.server_close()
        worker.join(5)
        assert not worker.is_alive(), "HTTP fixture did not shut down"


def call(connection, path, payload=None):
    """Return the HTTP status and JSON body of one authenticated GET, or POST of payload."""
    connection.request(
        "GET" if payload is None else "POST",
        path,
        body=None if payload is None else json.dumps(payload),
        headers={"Authorization": "Bearer qnn-test-key"},
    )
    response = connection.getresponse()
    return response.status, json.loads(response.read())


class FakeTokenizer:
    """Capture the model's actual host prompt and return fixed typed graph inputs."""

    def __init__(self):
        self.calls = []

    def __call__(self, prompts, **kwargs):
        self.calls.append((prompts, kwargs))
        return {
            "input_ids": np.ones((1, 200), dtype=np.int64),
            "attention_mask": np.ones((1, 200), dtype=np.int64),
        }


class FakeModel:
    """Native session double with the real graph ABI and deterministic outputs."""

    def __init__(self, name, device, ports):
        self.name = name
        self.device_id = device
        self.calls = []
        inputs, outputs = ports
        self.input_names = list(inputs)
        # Force the alias adapter to handle compiler-renamed/reordered outputs.
        self.output_names = [f"output_{i}" for i in reversed(range(len(outputs)))]
        self.source_outputs = list(outputs)
        self.input_refs = [
            {"shape": shape, "dtype": dtype} for shape, dtype in inputs.values()
        ]
        self.output_refs = [
            {
                "shape": outputs[self.source_outputs[int(name.split("_")[1])]][0],
                "dtype": outputs[self.source_outputs[int(name.split("_")[1])]][1],
            }
            for name in self.output_names
        ]

    def run(self, feed):
        if set(feed) != set(self.input_names):
            raise AssertionError(f"wrong fake graph inputs for {self.name}")
        for name, spec in zip(self.input_names, self.input_refs, strict=True):
            if feed[name].shape != spec["shape"] or feed[name].dtype != spec["dtype"]:
                raise AssertionError(
                    f"wrong fake graph shape/dtype for {self.name}/{name}"
                )
        self.calls.append(
            {
                name: feed[name].copy()
                for name in ("x_t", "time_step", "image", "lang_tokens", "lang_mask")
                if name in feed
            }
        )
        result = {}
        for name, spec in zip(self.output_names, self.output_refs, strict=True):
            alias = self.source_outputs[int(name.split("_")[1])]
            result[name] = (
                expert_update(feed["x_t"])
                if alias == "action_emb"
                else np.zeros(spec["shape"], dtype=spec["dtype"])
            )
        return result


class FakeRuntime:
    """Record native load/close calls; no board, QAIRT, host neural work or sockets."""

    def __init__(self, ports):
        self.ports = ports
        self.models = {}
        self.closed = False

    def load(self, binary, abi, device, *, name="model.bin"):
        assert isinstance(binary, bytes)
        assert Path(abi).is_file()
        model = FakeModel(name, device, self.ports[name])
        self.models[name] = model
        return model

    def open(self):
        pass

    def close(self):
        self.closed = True


class QnnFixture(unittest.TestCase):
    """Construct a complete local compiled bundle with fake binary/session data."""

    def setUp(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.root = Path(directory.name)
        self.config = {
            "schema": 1,
            "graph_abi": SIX_GRAPH_ABI,
            "purpose": "simulation_only_kinova_finetune",
            "model_id": "test/pi05",
            "model_revision": "a" * 40,
            "camera_names": ["overview", "wrist", "scene"],
            "request_state_dim": 8,
            "model_action_dim": 8,
            "state_columns": list(range(8)),
            "normalization": "QUANTILES",
            "chunk_size": 50,
            "max_action_dim": 32,
            "denoising_steps": 10,
            "expert_output": "internally_integrated_actions",
            "dt": 0.1,
            "action_mapping": {"mode": "trained_absolute_quantiles"},
            "noise_seed": 265,
            "stats": "stats.json",
            "tokenizer": "tokenizer",
            "graphs": {name: name for name in SIX_GRAPH_ORDER},
            "device_placement": {
                name: 1 if name in ("vision_encoder", "token_emb") else 0
                for name in SIX_GRAPH_ORDER
            },
            "precision": {
                "vision_encoder": "fp16",
                "token_emb": "fp16",
                "backbone": "fp16_split",
                "action_expert": "fp16_native_rms",
            },
        }
        self.stats = {
            "observation.state.q01": [0.0] * 8,
            "observation.state.q99": [1.0] * 8,
            "action.q01": [-2.0] * 8,
            "action.q99": [4.0] * 8,
        }
        (self.root / "stats.json").write_text(json.dumps(self.stats))
        (self.root / "tokenizer").mkdir()
        self.bundle = self.root / "bundle.json"
        self.tokenizer = FakeTokenizer()
        self.ports = graph_ports(True)
        self.stage_bundle()

    def stage_bundle(self):
        for name in self.config["graphs"]:
            directory = self.root / name
            directory.mkdir(exist_ok=True)
            binary = b"fake native context: " + name.encode()
            (directory / "model.bin").write_bytes(binary)
            (directory / "abi.json").write_text("{}")
            outputs = self.ports[name][1]
            manifest = {
                "context_sha256": hashlib.sha256(binary).hexdigest(),
                "output_aliases": {
                    f"output_{i}": name for i, name in enumerate(outputs)
                },
            }
            if name == "action_expert":
                steps = self.config["denoising_steps"]
                # Fake export descriptor: tests do not attest real native compilation.
                manifest["sampling"] = {
                    **sampling_values(self.config["chunk_size"], steps),
                    "context_sha256": manifest["context_sha256"],
                    "source_sha256": {"model.onnx": "b" * 64},
                }
            (directory / "manifest.json").write_text(json.dumps(manifest))
        self.bundle.write_text(json.dumps(self.config))

    @contextmanager
    def native(self, runtime):
        """Serve QnnPolicy's native runtime and local tokenizer from doubles."""
        with patch("vla_qnn_runtime.NativeRuntime", return_value=runtime), patch(
            "vla_qnn_policy.AutoTokenizer.from_pretrained", return_value=self.tokenizer
        ) as tokenizer_load:
            yield tokenizer_load

    def runner(self, runtime=None, load=True, **overrides):
        runtime = runtime or FakeRuntime(self.ports)
        values = dict(
            bundle=str(self.bundle),
            checkpoint="test/pi05",
            policy_type="pi05",
            revision="",
            state_dim=8,
            fps=10.0,
            guidance_horizon=8,
            rtc_schedule="EXP",
        )
        values.update(overrides)
        with self.native(runtime) as tokenizer_load:
            runner = QnnPolicyRunner(**values)
            if load:
                runner.load()
        tokenizer_load.assert_called_once_with(
            str(self.root / "tokenizer"), local_files_only=True
        )
        self.addCleanup(self.close_runner, runner)
        return runner, runtime

    @staticmethod
    def close_runner(runner):
        try:
            runner.close()
        except RuntimeError:
            pass  # Deliberate failed-free doubles retain ownership, without retry.

    def payload(self):
        blank = encode_bgr_jpeg_b64(np.zeros((4, 8, 3), dtype=np.uint8))
        return {
            "state": [0.0] * 8,
            "task": " stack_cubes\n ",
            "images": {name: blank for name in self.config["camera_names"]},
        }

    def ready_state(self, runner):
        state = ServerState()
        state.runner = runner
        state.fps = runner.policy.fps
        state.status = "ready"
        state.checkpoint = self.config["model_id"]
        state.checkpoint_revision = self.config["model_revision"]
        state.bearer_key = "qnn-test-key"
        return state

    def watch(self, state, restart=None, clock=lambda: 0):
        """Replace the watched YAML once; any automatic retry poll fails the test."""
        serving = self.root / "watched.yaml"
        serving.write_text("backend: qnn\n")
        closed_at_restart = []
        waits = 0

        def wait(_seconds):
            nonlocal waits
            waits += 1
            if waits > 2:
                raise AssertionError("QNN watcher automatically retried")
            serving.write_text("backend: pytorch\n")

        def record():
            closed_at_restart.append(self.runtime.closed)

        watch_runtime_inputs(
            [str(serving)],
            state=state,
            wait=wait,
            restart=restart or record,
            clock=clock,
        )
        return closed_at_restart

    def parsed_args(self, extra=(), **fields):
        serving = self.root / "vla_serving.yaml"
        serving.write_text(
            yaml.safe_dump(
                {
                    "checkpoint": self.config["model_id"],
                    "backend": "qnn",
                    "qnn_bundle": str(self.bundle),
                    "fps": 10.0,
                    "state_dim": 8,
                    **fields,
                }
            )
        )
        with patch(
            "sys.argv", ["vla_inference_server.py", "--config", str(serving), *extra]
        ):
            return parse_args()


class TestQnnSelection(QnnFixture):
    def load_offline(self, args, runtime):
        """Load through load_policy with every Hugging Face entry point refused."""
        state = ServerState()
        hub_refused = AssertionError("the QNN load contacted the Hugging Face Hub")
        environment = {
            key: value for key, value in os.environ.items() if key != "HF_TOKEN"
        }
        environment["HF_HUB_OFFLINE"] = "1"
        environment["HF_HUB_CACHE"] = str(self.root / "empty_hub_cache")
        with patch.dict(os.environ, environment, clear=True), self.native(
            runtime
        ), patch(
            "vla_inference_server.hf_hub_download", side_effect=hub_refused
        ), patch(
            "vla_inference_server.snapshot_download", side_effect=hub_refused
        ), patch(
            "vla_inference_server.try_to_load_from_cache", side_effect=hub_refused
        ):
            load_policy(state, args)
        return state

    def test_yaml_selects_qnn_without_loading_pytorch_or_hub(self):
        args = self.parsed_args()
        runtime = FakeRuntime(self.ports)
        with patch("vla_inference_server.PolicyRunner") as pytorch, patch(
            "vla_inference_server.resolve_policy_type"
        ) as family, patch("vla_inference_server.resolve_fps") as fps, patch(
            "vla_inference_server.torch.cuda.is_available",
            side_effect=AssertionError("no GPU probe"),
        ):
            state = self.load_offline(args, runtime)
        self.addCleanup(runtime.close)
        self.assertEqual(state.status, "ready", state.detail)
        self.assertIsInstance(state.runner, QnnPolicyRunner)
        self.assertEqual(state.runner.device, "npu:qnn_htp")
        self.assertEqual(state.checkpoint_revision, "a" * 40)
        self.assertEqual(state.fps, 10.0)
        self.assertFalse(state.runner.int8)
        self.assertEqual(set(runtime.models), set(SIX_GRAPH_ORDER))
        pytorch.assert_not_called()
        family.assert_not_called()
        fps.assert_not_called()
        self.assertEqual((state.state_names, state.action_names), ([], []))

    def test_serving_config_names_order_state_and_label_actions_offline(self):
        trained = [f"joint_{i}" for i in range(8)]
        args = self.parsed_args(state_names=trained, action_names=trained)
        state = self.load_offline(args, FakeRuntime(self.ports))
        self.assertEqual(state.status, "ready", state.detail)
        payload = self.payload()
        # joint_0, the first trained joint, is last in the request and at q99.
        payload["state"] = [0.0] * 7 + [1.0]
        payload["state_names"] = list(reversed(trained))
        response = run_inference(state, payload)
        self.assertEqual(response["action_names"], trained)
        prompt = self.tokenizer.calls[-1][0][0]
        bins = prompt.split("State: ")[1].split(";")[0].split()
        self.assertEqual(len(bins), 8)
        self.assertNotEqual(bins[0], "0")
        self.assertEqual(bins[1:], ["0"] * 7)
        payload["state_names"] = ["other_joint", *reversed(trained[:-1])]
        with self.assertRaisesRegex(ValueError, "missing \\['joint_7'\\]"):
            run_inference(state, payload)

    def test_serving_config_names_of_the_wrong_width_close_contexts_and_park(self):
        trained = [f"joint_{i}" for i in range(8)]
        for fields, message in (
            (
                {"state_names": trained[:7], "action_names": trained[:7]},
                "state_names length 7",
            ),
            (
                {"state_names": trained, "action_names": trained[:7]},
                "action_names length 7 does not match the 8 values",
            ),
            ({"state_names": trained}, "sets only one of state_names"),
        ):
            with self.subTest(fields=fields):
                runtime = FakeRuntime(self.ports)
                state = self.load_offline(self.parsed_args(**fields), runtime)
                self.assertEqual(state.status, "error")
                self.assertIn(message, state.detail)
                self.assertIn("restart the inference server explicitly", state.detail)
                self.assertTrue(runtime.closed)
                self.assertIsNone(state.runner)

    def test_default_and_cli_override_keep_pytorch_path(self):
        """CLI precedence reaches PyTorch with no QAIRT, ignoring an unused invalid bundle."""
        for unused in (str(self.bundle), 17, ["wrong"], {"path": "wrong"}):
            with self.subTest(qnn_bundle=unused):
                args = self.parsed_args(
                    ("--backend", "pytorch", "--device", "cpu"), qnn_bundle=unused
                )
                self.assertEqual(args.config_error, "")
                args.policy_class = "pi05"
                runner = MagicMock(spec=PolicyRunner)
                runner.request_names = []
                runner.warmup.return_value = (0.1, 0.1, 50, 8)
                runner.expected_camera_keys.return_value = []
                runner.expected_state_dim.return_value = 8
                state = ServerState()
                with patch(
                    "vla_inference_server.PolicyRunner", return_value=runner
                ) as pytorch, patch(
                    "vla_qnn_runtime.NativeRuntime",
                    side_effect=AssertionError("optional QAIRT loaded"),
                ), patch(
                    "vla_inference_server.resolve_policy_names", return_value=([], [])
                ) as names:
                    load_policy(state, args)
                self.assertEqual(state.status, "ready", state.detail)
                self.assertIs(state.runner, runner)
                pytorch.assert_called_once()
                names.assert_called_once_with("test/pi05", "", [], [])
        with patch(
            "sys.argv",
            ["vla_inference_server.py", "--config", str(self.root / "absent.yaml")],
        ):
            self.assertEqual(parse_args().backend, "pytorch")

    def test_missing_bundle_and_non_pi05_family_are_refused(self):
        for overrides, message in (
            ({"bundle": ""}, "qnn_bundle"),
            ({"policy_type": "groot"}, "pi05 only"),
            ({"policy_type": "smolvla"}, "pi05 only"),
        ):
            with self.subTest(overrides=overrides), self.assertRaisesRegex(
                ValueError, message
            ):
                self.runner(**overrides)

    def test_invalid_yaml_backend_and_bundle_park_after_bind(self):
        for changes, message in (
            ({"backend": "qnnn"}, "backend"),
            ({"backend": "qnn", "qnn_bundle": ["wrong"]}, "qnn_bundle"),
        ):
            serving = self.root / "bad.yaml"
            serving.write_text(yaml.safe_dump(changes))
            with patch(
                "sys.argv", ["vla_inference_server.py", "--config", str(serving)]
            ):
                args = parse_args()
            self.assertIn(message, args.config_error)
            state = ServerState()
            load_policy(state, args)
            self.assertEqual(state.status, "error")

    def test_missing_qairt_is_actionable_and_has_no_fallback(self):
        args = self.parsed_args()
        state = ServerState()
        with patch(
            "vla_qnn_policy.AutoTokenizer.from_pretrained", return_value=self.tokenizer
        ), patch(
            "vla_qnn_runtime.c.CDLL", side_effect=OSError("library absent")
        ), patch(
            "vla_inference_server.PolicyRunner"
        ) as pytorch:
            load_policy(state, args)
        self.assertEqual(state.status, "error")
        self.assertIn("QAIRT 2.49", state.detail)
        self.assertIn("QNN_HTP_LIBRARY", state.detail)
        self.assertIn("CPU/GPU fallback is disabled", state.detail)
        pytorch.assert_not_called()

    def test_native_warmup_failure_closes_contexts_and_parks(self):
        args = self.parsed_args()
        runtime = FakeRuntime(self.ports)
        state = ServerState()
        with self.native(runtime), patch.object(
            QnnPolicyRunner, "warmup", side_effect=RuntimeError("native graph failed")
        ):
            load_policy(state, args)
        self.assertEqual(state.status, "error")
        self.assertIn("native graph failed", state.detail)
        self.assertTrue(runtime.closed)
        self.assertIsNone(state.runner)

    def test_checkpoint_width_rate_and_revision_must_match_bundle(self):
        for overrides in (
            {"checkpoint": "other/policy"},
            {"revision": "b" * 40},
            {"state_dim": 7},
            {"fps": 20.0},
            {"fps": float("nan")},
        ):
            with self.subTest(overrides=overrides), self.assertRaises(ValueError):
                self.runner(**overrides)
        runner, _ = self.runner(state_dim=0, fps=0)
        self.assertEqual(runner.expected_state_dim(), 8)
        self.assertEqual(runner.policy.fps, 10.0)

    def test_context_hash_or_port_contract_change_is_refused(self):
        directory = self.root / "action_expert"
        (directory / "model.bin").write_bytes(b"replaced context")
        with self.assertRaisesRegex(ValueError, "hash changed"):
            self.runner()
        self.stage_bundle()
        self.ports["action_expert"][0]["time_step"] = ((2,), np.dtype("float32"))
        runtime = FakeRuntime(self.ports)
        with self.native(runtime), self.assertRaisesRegex(ValueError, "graph ABI"):
            runner = QnnPolicyRunner(
                str(self.bundle), "test/pi05", "pi05", "", 8, 10.0, 8, "EXP"
            )
            try:
                runner.load()
            finally:
                runner.close()
        self.assertTrue(runtime.closed)

    def test_native_load_uses_verified_snapshot_after_file_replacement(self):
        runtime = FakeRuntime(self.ports)
        original_load = runtime.load
        loaded = []
        expected = (self.root / "vision_encoder" / "model.bin").read_bytes()

        def replace(binary, abi, device, **kwargs):
            name = kwargs["name"]
            if name == "vision_encoder":
                (self.root / name / "model.bin").write_bytes(
                    b"replacement after hash check"
                )
                loaded.append(binary)
            return original_load(binary, abi, device, **kwargs)

        runtime.load = replace
        self.runner(runtime)
        self.assertEqual(loaded, [expected])

    def test_bundle_refuses_unsupported_sampling_mapping_and_devices(self):
        changes = {
            "schema": True,
            "graph_abi": "groot",
            "denoising_steps": 0,
            "expert_output": "velocity",
            "normalization": "MEAN_STD",
            "action_mapping": {"mode": "deltas"},
            "camera_names": ["scene"] * 3,
            "state_columns": list(reversed(range(8))),
            "model_action_dim": 33,
            "noise_seed": True,
            "dt": -0.1,
            "device_placement": {name: 0 for name in SIX_GRAPH_ORDER},
        }
        for field, value in changes.items():
            with self.subTest(field=field), self.assertRaisesRegex(ValueError, "QNN"):
                validate_bundle({**self.config, field: value})


class TestQnnSamplingContract(QnnFixture):
    def expert_manifest(self, change):
        path = self.root / "action_expert" / "manifest.json"
        manifest = json.loads(path.read_text())
        change(manifest)
        path.write_text(json.dumps(manifest))

    def test_alternate_chunk_and_steps_drive_shapes_noise_and_native_schedule(self):
        for chunk, steps in ((16, 4), (72, 12)):
            with self.subTest(chunk=chunk, steps=steps):
                self.config.update(chunk_size=chunk, denoising_steps=steps)
                self.ports = graph_ports(True, chunk)
                self.stage_bundle()
                runner, runtime = self.runner(guidance_horizon=chunk)
                expected = np.random.default_rng(265).standard_normal(
                    (1, chunk, 32), dtype=np.float32
                )
                for _ in range(steps):
                    expected = expert_update(expected)
                result = run_inference(self.ready_state(runner), self.payload())
                np.testing.assert_array_equal(
                    result["action_chunk_raw"], expected[0, :, :8]
                )
                calls = runtime.models["action_expert"].calls
                self.assertEqual(len(calls), steps)
                np.testing.assert_array_equal(
                    [call["time_step"][0] for call in calls],
                    np.asarray([1 - i / steps for i in range(steps)], dtype=np.float32),
                )
                self.assertEqual(result["dt"], 0.1)

    def test_manifest_only_dimension_changes_are_refused(self):
        for field, value in (("chunk_size", 16), ("denoising_steps", 4)):
            with self.subTest(field=field):
                self.config[field] = value
                self.bundle.write_text(json.dumps(self.config))
                with self.assertRaisesRegex(ValueError, "sampling.*" + field):
                    self.runner()
                self.config.update(chunk_size=50, denoising_steps=10)
                self.bundle.write_text(json.dumps(self.config))

    def test_sampling_must_match_context_conditioning_and_integration(self):
        for field, value in (
            ("context_sha256", "c" * 64),
            ("conditioning_rows", 10),
            ("integration_dt", -0.2),
            ("timesteps", [1.0] * 10),
            ("schedule", "continuous_time"),
            ("expert_output", "velocity"),
        ):
            with self.subTest(field=field):
                self.stage_bundle()
                self.expert_manifest(lambda m: m["sampling"].update({field: value}))
                with self.assertRaisesRegex(ValueError, "sampling"):
                    self.runner()

    def test_unknown_context_requires_sampling_even_with_legacy_dimensions(self):
        self.expert_manifest(lambda m: m.pop("sampling"))
        with self.assertRaisesRegex(ValueError, "sampling"):
            self.runner()

    def test_alternate_chunk_must_match_every_chaining_port(self):
        self.config.update(chunk_size=16, denoising_steps=4)
        self.stage_bundle()  # Keep the fake graph IO at 50, unlike its descriptor.
        with self.assertRaisesRegex(ValueError, "graph ABI"):
            self.runner()

    def test_alternate_rtc_uses_the_compiled_integration_interval(self):
        self.config.update(chunk_size=16, denoising_steps=4)
        self.ports = graph_ports(True, 16)
        self.stage_bundle()
        runner, runtime = self.runner()
        previous = np.zeros((12, 8), dtype=np.float32)
        expected = np.random.default_rng(265).standard_normal(
            (1, 16, 32), dtype=np.float32
        )
        for step in range(4):
            expected = guide_native_update(
                expected,
                expert_update(expected),
                float(np.float32(1 - step / 4)),
                previous,
                2,
                10,
                np.float32(-0.25),
                runner.policy.rtc,
            )
        result = run_inference(
            self.ready_state(runner),
            {
                **self.payload(),
                "prev_chunk_left_over": previous.tolist(),
                "inference_delay": 2,
                "guidance_horizon": 8,
            },
        )
        np.testing.assert_array_equal(result["action_chunk_raw"], expected[0, :, :8])
        self.assertEqual(len(runtime.models["action_expert"].calls), 4)
        with self.assertRaisesRegex(ValueError, "0..16"):
            self.runner(guidance_horizon=17)


class TestQnnInference(QnnFixture):
    def test_off_path_preserves_integrated_updates_and_noise_advancement(self):
        runner, runtime = self.runner()
        state = self.ready_state(runner)
        rng = np.random.default_rng(265)
        # A first RTC chunk (a width but no carryover) is not guided either.
        for payload in (self.payload(), {**self.payload(), "guidance_horizon": 18}):
            expected = rng.standard_normal((1, 50, 32), dtype=np.float32)
            noise = expected.copy()
            for _step in range(10):
                expected = expert_update(expected)
            with patch(
                "vla_qnn_policy.guide_native_update",
                side_effect=AssertionError("off path guided"),
            ):
                result = run_inference(state, payload)
            np.testing.assert_array_equal(
                result["action_chunk_raw"], expected[0, :, :8]
            )
            np.testing.assert_array_equal(
                result["action_chunk"], (expected[0, :, :8] + 1) * 3 - 2
            )
            np.testing.assert_array_equal(
                runtime.models["action_expert"].calls[-10]["x_t"], noise
            )
            self.assertEqual(result["dt"], 0.1)
        self.assertEqual(len(runtime.models["action_expert"].calls), 20)
        self.assertEqual(len(runtime.models["vision_encoder"].calls), 6)
        self.assertEqual(len(runtime.models["token_emb"].calls), 2)

    def test_warmup_preserves_first_request_seed(self):
        runner, runtime = self.runner()
        runner.warmup()
        run_inference(self.ready_state(runner), self.payload())
        expected = np.random.default_rng(265).standard_normal(
            (1, 50, 32), dtype=np.float32
        )
        np.testing.assert_array_equal(
            runtime.models["action_expert"].calls[-10]["x_t"], expected
        )

    def test_rtc_guides_after_each_native_update_and_maps_width_to_end(self):
        runner, runtime = self.runner()
        rng = np.random.default_rng(11)
        previous = rng.normal(size=(30, 8)).astype(np.float32)
        payload = {
            **self.payload(),
            "prev_chunk_left_over": previous.tolist(),
            "inference_delay": 4,
            "guidance_horizon": 18,
        }
        expected = np.random.default_rng(265).standard_normal(
            (1, 50, 32), dtype=np.float32
        )
        expected_inputs = []
        for step in range(10):
            expected_inputs.append(expected.copy())
            expected = guide_native_update(
                expected,
                expert_update(expected),
                float(np.float32(1 - step / 10)),
                previous,
                4,
                22,
                np.float32(-0.1),
                runner.policy.rtc,
            )
        result = run_inference(self.ready_state(runner), payload)
        for index, feed in enumerate(runtime.models["action_expert"].calls):
            np.testing.assert_array_equal(feed["x_t"], expected_inputs[index])
        np.testing.assert_array_equal(result["action_chunk_raw"], expected[0, :, :8])
        self.assertEqual(len(runtime.models["action_expert"].calls), 10)

    def test_malformed_carryover_never_calls_native_sessions(self):
        runner, runtime = self.runner()
        state = self.ready_state(runner)
        bad_values = [
            "",
            False,
            {},
            [0.0] * 8,
            [[0.0] * 7],
            [[True] * 8],
            [[float("nan")] * 8],
            [[1e200] * 8],
            [[10**1000] * 8],
            [[0.0] * 8] * 51,
        ]
        for value in bad_values:
            with self.subTest(value=str(value)[:80]), self.assertRaisesRegex(
                ValueError, "RTC"
            ):
                run_inference(state, {**self.payload(), "prev_chunk_left_over": value})
        for field in ("guidance_horizon", "inference_delay"):
            for value in (-1, 51, True, 1.5, "18", None):
                with self.subTest(field=field, value=value), self.assertRaisesRegex(
                    ValueError, "RTC"
                ):
                    run_inference(state, {**self.payload(), field: value})
        self.assertTrue(all(not model.calls for model in runtime.models.values()))

    def test_zero_request_width_resolves_to_configured_default(self):
        runner, _runtime = self.runner(guidance_horizon=13)
        images = {name: torch.zeros(3, 224, 224) for name in runner.request_names}
        previous = np.zeros((30, 8), dtype=np.float32)
        with patch.object(
            runner.policy, "infer", return_value=(previous, previous)
        ) as infer:
            runner.infer(images, [0.0] * 8, "stack", previous, 4, 0)
        self.assertEqual(infer.call_args.args[-1], 17)

    def test_camera_order_prompt_and_image_math_match_stock(self):
        runner, runtime = self.runner()
        run_inference(self.ready_state(runner), self.payload())
        prompts, kwargs = self.tokenizer.calls[-1]
        self.assertEqual(
            prompts, ["Task: stack cubes, State: 0 0 0 0 0 0 0 0;\nAction: "]
        )
        self.assertEqual(kwargs["padding_side"], "right")
        self.assertEqual(kwargs["max_length"], 200)
        self.assertEqual(
            runtime.models["token_emb"].calls[0]["lang_tokens"].dtype, np.int32
        )
        for call in runtime.models["vision_encoder"].calls:
            np.testing.assert_array_equal(
                call["image"], -np.ones((1, 3, 224, 224), dtype=np.float32)
            )
        image = torch.linspace(0, 1, 3 * 15 * 31).reshape(1, 3, 15, 31)
        torch.testing.assert_close(
            resize_with_pad(image),
            resize_with_pad_torch(image, 224, 224),
            rtol=0,
            atol=0,
        )

    def test_four_graph_bundle_uses_same_action_contract(self):
        self.config["graph_abi"] = FOUR_GRAPH_ABI
        self.config["graphs"] = {name: name for name in FOUR_GRAPH_ORDER}
        self.config["device_placement"] = {
            name: 1 if name in ("vision_encoder", "token_emb") else 0
            for name in FOUR_GRAPH_ORDER
        }
        self.config["precision"]["backbone"] = "w8a16"
        self.ports = graph_ports(False)
        self.stage_bundle()
        runner, runtime = self.runner()
        result = run_inference(self.ready_state(runner), self.payload())
        self.assertEqual(set(runtime.models), set(FOUR_GRAPH_ORDER))
        self.assertFalse(runner.int8)
        self.assertEqual(runner.precision["backbone"], "w8a16")
        self.assertEqual(np.shape(result["action_chunk_raw"]), (50, 8))
        self.assertEqual(result["dt"], 0.1)

    def test_http_status_and_rtc_round_trip_use_existing_contract(self):
        runner, runtime = self.runner()
        with http_connection(self.ready_state(runner)) as connection:
            code, status = call(connection, "/status")
            self.assertEqual(code, 200)
            self.assertEqual(status["device"], "npu:qnn_htp")
            self.assertIs(status["int8"], False)
            self.assertEqual(status["qnnPrecision"], self.config["precision"])
            self.assertEqual(status["checkpointRevision"], "a" * 40)
            self.assertIn("configRevision", status)
            _, health = call(connection, "/health")
            self.assertEqual(health["qnnPrecision"], self.config["precision"])
            self.assertIs(health["int8"], False)
            payload = {**self.payload(), "guidance_horizon": 18}
            code, first = call(connection, "/infer", payload)
            self.assertEqual(code, 200)
            payload.update(
                prev_chunk_left_over=first["action_chunk_raw"][20:], inference_delay=4
            )
            code, second = call(connection, "/infer", payload)
            self.assertEqual(code, 200)
            self.assertEqual(np.shape(second["action_chunk_raw"]), (50, 8))
            self.assertEqual(second["dt"], 0.1)
            payload["prev_chunk_left_over"] = [[True] * 8]
            self.assertEqual(call(connection, "/infer", payload)[0], 400)
        self.assertEqual(len(runtime.models["action_expert"].calls), 20)


class TestQnnReload(QnnFixture):
    def test_failed_warmup_cleanup_retains_runner_and_requires_explicit_restart(self):
        args = self.parsed_args()
        self.runtime = FakeRuntime(self.ports)
        original_close = self.runtime.close
        self.runtime.close = MagicMock(side_effect=RuntimeError("native free failed"))
        state = ServerState()
        with self.native(self.runtime), patch.object(
            QnnPolicyRunner, "warmup", side_effect=RuntimeError("native warmup failed")
        ):
            load_policy(state, args)
        self.assertEqual(state.status, "error")
        self.assertIsNone(state.runner)
        self.assertIsInstance(state.cleanup_runner, QnnPolicyRunner)
        self.assertTrue(state.cleanup_runner.release_started)
        retained = state.cleanup_runner
        self.runtime.close.side_effect = original_close
        self.assertEqual(self.watch(state), [])
        self.assertIs(state.cleanup_runner, retained)
        self.runtime.close.assert_called_once()
        self.assertIn("restart the inference server explicitly", state.detail)
        self.assertFalse(state.begin_infer())

    def test_cleanup_failure_during_reload_is_attempted_once_without_exec(self):
        runner, self.runtime = self.runner()
        state = self.ready_state(runner)
        self.runtime.close = MagicMock(side_effect=RuntimeError("context free failed"))
        self.assertEqual(self.watch(state), [])
        self.runtime.close.assert_called_once()
        self.assertIs(state.cleanup_runner, runner)
        self.assertEqual(state.status, "error")
        self.assertIn("context free failed", state.detail)
        self.assertIn("restart the inference server explicitly", state.detail)
        self.assertFalse(state.begin_infer())

    def test_repeated_close_never_retries_failed_native_cleanup(self):
        runner, self.runtime = self.runner()
        self.runtime.close = MagicMock(side_effect=RuntimeError("native free failed"))
        for _ in range(2):
            with self.assertRaisesRegex(RuntimeError, "native free failed"):
                runner.close()
        self.runtime.close.assert_called_once()
        with self.assertRaisesRegex(RuntimeError, "restart"):
            runner.infer({}, [], "stack", None, 0, 0)

    def test_partial_load_and_failed_inference_are_never_reused(self):
        runner, self.runtime = self.runner()
        with patch.object(
            runner.policy, "infer", side_effect=RuntimeError("HTP execute failed")
        ):
            state = self.ready_state(runner)
            with self.assertRaisesRegex(RuntimeError, "HTP execute failed"):
                run_inference(state, self.payload())
        self.assertEqual(state.status, "error")
        self.assertIn("restart the inference server explicitly", state.detail)
        self.assertTrue(self.runtime.closed)
        with self.assertRaisesRegex(RuntimeError, "restart"):
            runner.load()
        with self.assertRaisesRegex(RuntimeError, "restart"):
            runner.infer({}, [], "stack", None, 0, 0)

    def test_queued_request_publishes_recorded_native_failure(self):
        runner, self.runtime = self.runner()
        state = self.ready_state(runner)
        runner.failed, runner.failure_detail = True, "HTP execute failed"
        with self.assertRaisesRegex(RuntimeError, "restart"):
            run_inference(state, self.payload())
        self.assertIn("HTP execute failed", state.detail)

    def test_failed_load_is_retained_but_never_inferred_or_retried(self):
        args = self.parsed_args()
        self.runtime = FakeRuntime(self.ports)
        load = self.runtime.load

        def fail_last(binary, abi, device, **kwargs):
            if kwargs["name"] == "action_expert":
                raise RuntimeError("context deserialization failed")
            return load(binary, abi, device, **kwargs)

        self.runtime.load = fail_last
        self.runtime.close = MagicMock(side_effect=RuntimeError("contextFree failed"))
        state = ServerState()
        with self.native(self.runtime):
            load_policy(state, args)
        self.assertEqual(state.status, "error")
        self.assertIn("context deserialization failed", state.detail)
        self.assertIn("contextFree failed", state.detail)
        self.assertIn("restart the inference server explicitly", state.detail)
        self.assertIsNone(state.runner)
        retained = state.cleanup_runner
        self.assertTrue(retained.failed)
        self.assertEqual(self.watch(state), [])
        self.assertIs(state.cleanup_runner, retained)
        self.runtime.close.assert_called_once()
        with self.assertRaisesRegex(RuntimeError, "restart"):
            retained.infer({}, [], "stack", None, 0, 0)

    def test_unloaded_runner_refuses_inference_without_native_calls(self):
        runner, runtime = self.runner(load=False)
        with self.assertRaisesRegex(RuntimeError, "incomplete"):
            runner.infer({}, [], "stack", None, 0, 0)
        self.assertFalse(runtime.models)

    def test_failed_exec_before_native_runner_publication_requires_restart(self):
        state = ServerState()
        state.backend = "qnn"
        state.native_load_started = -QNN_LOAD_ABANDONED_SECONDS - 1
        restart = MagicMock(side_effect=OSError("replacement unavailable"))
        self.watch(state, restart)
        restart.assert_called_once()
        self.assertEqual(state.status, "error")
        self.assertIn("replacement unavailable", state.detail)
        self.assertFalse(state.begin_native_load())

    def test_watcher_failure_during_loader_cleanup_preserves_both_details(self):
        args = self.parsed_args()
        self.runtime = FakeRuntime(self.ports)
        state = ServerState()
        attempts = []

        def close():
            attempts.append(True)
            state.fail_qnn(
                "QNN process replacement failed (late watcher error)",
                state.cleanup_runner,
            )
            raise RuntimeError("contextFree failed")

        self.runtime.close = close
        with self.native(self.runtime), patch.object(
            QnnPolicyRunner,
            "warmup",
            side_effect=RuntimeError("original warmup failed"),
        ):
            load_policy(state, args)
        self.assertEqual(state.status, "error")
        self.assertIn("original warmup failed", state.detail)
        self.assertIn("contextFree failed", state.detail)
        self.assertEqual(attempts, [True])

    def test_hung_release_and_failed_exec_never_retry_after_call_unblocks(self):
        runner, self.runtime = self.runner()
        state = self.ready_state(runner)
        restart = MagicMock(side_effect=OSError("replacement unavailable"))
        runner.lock.acquire()
        try:
            with patch("vla_inference_server.QNN_RELEASE_TIMEOUT_SECONDS", 0.01):
                self.watch(state, restart)
        finally:
            runner.lock.release()
        self.assertTrue(runner.release_started)
        self.assertFalse(self.runtime.closed)
        self.assertFalse(runner.close())
        self.assertEqual(self.watch(state), [])
        restart.assert_called_once()
        self.assertIs(state.cleanup_runner, runner)
        with self.assertRaisesRegex(RuntimeError, "restart"):
            runner.infer({}, [], "stack", None, 0, 0)

    def test_expired_load_preserves_failed_replacement_error_before_publication(self):
        """A watcher releasing the warm runner must win over the loader's ready write."""
        args = self.parsed_args()
        self.runtime = FakeRuntime(self.ports)
        state = ServerState()
        state.bearer_key = "qnn-test-key"
        original_warmup = QnnPolicyRunner.warmup
        failure_details = []

        def restart():
            self.assertTrue(self.runtime.closed)
            raise OSError("replacement unavailable")

        def warmup(runner):
            timings = original_warmup(runner)
            self.assertIs(state.cleanup_runner, runner)
            expired = state.native_load_started + QNN_LOAD_ABANDONED_SECONDS + 1
            self.watch(state, restart, clock=lambda: expired)
            failure_details.append(state.detail)
            self.assertTrue(runner.release_started)
            self.assertFalse(state.begin_infer())
            return timings

        with self.native(self.runtime), patch.object(QnnPolicyRunner, "warmup", warmup):
            load_policy(state, args)

        self.assertEqual(state.status, "error")
        self.assertEqual(state.detail, failure_details[0])
        self.assertIn("replacement unavailable", state.detail)
        self.assertIsNone(state.runner)
        self.assertIsInstance(state.cleanup_runner, QnnPolicyRunner)
        self.assertIsNone(state.native_load_started)

        with http_connection(state) as connection:
            _, health = call(connection, "/health")
            self.assertEqual(health["status"], "error")
            self.assertEqual(health["detail"], failure_details[0])
            for _ in range(3):
                code, error = call(connection, "/infer", self.payload())
                self.assertEqual(code, 500)
                self.assertIn("replacement unavailable", error["error"])
        # Refusals must not postpone the still-pending reload by marking activity.
        self.assertIsNone(state.last_infer_monotonic)
        self.assertEqual(state.inflight_infers, 0)
        self.assertEqual(self.watch(state), [])

    def test_expired_load_does_not_publish_while_reload_is_committed(self):
        args = self.parsed_args()
        self.runtime = FakeRuntime(self.ports)
        state = ServerState()
        original_warmup = QnnPolicyRunner.warmup

        def warmup(runner):
            timings = original_warmup(runner)
            expired = state.native_load_started + QNN_LOAD_ABANDONED_SECONDS + 1
            self.assertTrue(state.try_begin_reload(expired))
            self.assertFalse(runner.release_started)
            return timings

        with self.native(self.runtime), patch.object(QnnPolicyRunner, "warmup", warmup):
            load_policy(state, args)

        self.assertEqual(state.status, "loading")
        self.assertIsNone(state.runner)
        self.assertIsInstance(state.cleanup_runner, QnnPolicyRunner)
        self.assertFalse(state.begin_infer())
        self.assertEqual(self.watch(state), [True])

    def test_pytorch_load_finishing_during_failed_reload_keeps_serving(self):
        state = ServerState()
        runner = object.__new__(PolicyRunner)
        serving = self.root / "watched.yaml"
        serving.write_text("backend: pytorch\n")
        status_at_retry = []

        def restart():
            if not status_at_retry:
                status_at_retry.append(None)
                # The loader finishes after the reload commit, before exec fails.
                self.assertTrue(state.publish_ready(runner))
                raise OSError("text file busy")
            status_at_retry.append((state.status, state.runner))

        watch_runtime_inputs(
            [str(serving)],
            state=state,
            wait=lambda _seconds: serving.write_text("backend: qnn\n"),
            restart=restart,
            clock=lambda: 0,
        )
        self.assertEqual(status_at_retry[1], ("ready", runner))

    def test_release_between_warmup_passes_keeps_failed_replacement_error(self):
        args = self.parsed_args()
        self.runtime = FakeRuntime(self.ports)
        state = ServerState()
        original_infer = QnnPolicyRunner.infer
        failure_details = []
        restart = MagicMock(side_effect=OSError("replacement unavailable"))

        def infer(runner, *args):
            result = original_infer(runner, *args)
            if not failure_details:
                expired = state.native_load_started + QNN_LOAD_ABANDONED_SECONDS + 1
                self.watch(state, restart, clock=lambda: expired)
                failure_details.append(state.detail)
            return result

        with self.native(self.runtime), patch.object(QnnPolicyRunner, "infer", infer):
            load_policy(state, args)

        self.assertEqual(state.status, "error")
        self.assertIn("replacement unavailable", failure_details[0])
        self.assertEqual(state.detail, failure_details[0])
        self.assertIsNone(state.runner)

    def test_reload_frees_native_sessions_before_replacing_the_process(self):
        runner, self.runtime = self.runner()
        self.assertEqual(self.watch(self.ready_state(runner)), [True])

    def test_reload_during_load_waits_to_free_the_new_sessions(self):
        args = self.parsed_args()
        self.runtime = FakeRuntime(self.ports)
        state = ServerState()
        warming = threading.Event()
        finish = threading.Event()
        waits = 0

        def warmup(_runner):
            warming.set()
            finish.wait(10)
            return 0.1, 0.1, 50, 8

        def wait(seconds):
            nonlocal waits
            waits += 1
            serving.write_text("backend: pytorch\n")
            if waits == 4:
                finish.set()
                loader.join(10)

        def restart():
            closed_at_restart.append((state.status, self.runtime.closed))

        serving = self.root / "watched.yaml"
        serving.write_text("backend: qnn\n")
        closed_at_restart = []
        with self.native(self.runtime), patch.object(QnnPolicyRunner, "warmup", warmup):
            loader = threading.Thread(target=load_policy, args=(state, args))
            loader.start()
            self.assertTrue(warming.wait(10))
            watch_runtime_inputs(
                [str(serving)], state=state, wait=wait, restart=restart
            )
        self.addCleanup(state.runner.policy.close)
        self.assertEqual(closed_at_restart, [("ready", True)])
        self.assertTrue(state.reload_pending)

    def test_failed_exec_after_release_parks_without_retry(self):
        runner, self.runtime = self.runner()
        state = self.ready_state(runner)
        restart = MagicMock(side_effect=OSError("text file busy"))
        self.watch(state, restart)
        restart.assert_called_once()
        self.assertEqual(state.status, "error")
        self.assertIn("text file busy", state.detail)
        self.assertIn("restart the inference server explicitly", state.detail)
        self.assertIs(state.cleanup_runner, runner)
        self.assertFalse(state.begin_infer())
        self.assertEqual(self.watch(state), [])

    def test_reload_over_a_hung_infer_does_not_wait_for_its_sessions(self):
        runner, self.runtime = self.runner()
        runner.lock.acquire()
        self.addCleanup(runner.lock.release)
        with patch("vla_inference_server.QNN_RELEASE_TIMEOUT_SECONDS", 0.01):
            self.assertEqual(self.watch(self.ready_state(runner)), [False])


def bare_runtime(**attributes):
    """NativeRuntime state without a QAIRT library; tests install fake entry points."""
    runtime = NativeRuntime.__new__(NativeRuntime)
    runtime.lock = threading.RLock()
    runtime.backend = ctypes.c_void_p(8)
    runtime.models, runtime.devices, runtime.power = [], {}, {}
    runtime.failed, runtime.release_started, runtime.cleanup_error = False, False, None
    runtime.platform_info = None
    vars(runtime).update(attributes)
    return runtime


def platform_runtime(info_status=0, create_status=0, free_status=0):
    """Drive NativeRuntime.device through the pinned ctypes API, with fake handles/statuses."""
    hardware = Hardware()
    hardware.version = hardware.v1.num_cores = 1
    hardware.v1.cores = hardware.v1.extension = 1
    platform = Platform(1, PlatformV1(1, ctypes.pointer(hardware)))

    def info(_logger, output):
        output._obj.contents = platform
        return info_status

    def create(_logger, _configs, output):
        output._obj.value = 7
        return create_status

    def message(status, output):
        output._obj.value = f"native-status-{status}".encode()
        return 0

    return bare_runtime(
        functions={
            "platform_info": MagicMock(side_effect=info),
            "device_create": MagicMock(side_effect=create),
            "platform_free": MagicMock(return_value=free_status),
            "error_message": message,
            "device_free": MagicMock(return_value=0),
            "backend_free": MagicMock(return_value=0),
        }
    )


class TestNativeContextOwnership(unittest.TestCase):
    def runtime(self, fail_retrieve=False, fail_free=False):
        runtime = bare_runtime(devices={0: ctypes.c_void_p(7)})
        runtime.device = lambda _device: runtime.devices[0]
        observed = []
        frees = []

        def check(status, operation):
            if status:
                raise RuntimeError(operation + " failed")

        def create(_backend, _device, config, data, size, context, _profile):
            observed.append((ctypes.string_at(data, size), config))
            context._obj.value = 11
            return 0

        def retrieve(_context, _name, graph):
            if fail_retrieve:
                return 1
            graph._obj.value = 12
            return 0

        def free(_context, _profile):
            frees.append(True)
            return 1 if fail_free and len(frees) == 1 else 0

        runtime.check = check
        runtime.functions = {
            "context_create": create,
            "graph_retrieve": retrieve,
            "context_free": free,
            "device_free": MagicMock(return_value=0),
            "backend_free": MagicMock(return_value=0),
        }
        return runtime, observed, frees

    def abi(self, root):
        row = TestNativeTensorAbi().row()
        output = json.loads(json.dumps(row))
        output["info"].update(
            name="actions",
            type="QNN_TENSOR_TYPE_APP_READ",
            dataType="QNN_DATATYPE_FLOAT_32",
        )
        path = root / "abi.json"
        path.write_text(
            json.dumps(
                {
                    "info": {
                        "backendId": 6,
                        "numGraphs": 1,
                        "graphs": [
                            {
                                "info": {
                                    "graphName": "test",
                                    "graphInputs": [row],
                                    "graphOutputs": [output],
                                }
                            }
                        ],
                    }
                }
            )
        )
        return path

    def test_partial_context_stays_owned_when_graph_and_cleanup_fail(self):
        runtime, _observed, frees = self.runtime(fail_retrieve=True, fail_free=True)
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaises(RuntimeError):
                runtime.load(b"validated snapshot", self.abi(Path(directory)), 0)
            self.assertEqual(len(runtime.models), 1)
            self.assertTrue(runtime.models[0].context)
            self.assertEqual(frees, [])  # Cleanup belongs to the owning runner.
            retained = runtime.models[0]
            for _ in range(2):
                with self.assertRaisesRegex(RuntimeError, "contextFree"):
                    runtime.close()
            self.assertEqual(len(frees), 1)
            self.assertEqual(runtime.models, [retained])
            runtime.functions["device_free"].assert_not_called()
            runtime.functions["backend_free"].assert_not_called()
            with self.assertRaisesRegex(RuntimeError, "restart"):
                runtime.load(b"another context", self.abi(Path(directory)), 0)
            with self.assertRaisesRegex(RuntimeError, "restart"):
                retained.run({})

    def test_partial_backend_allocation_is_owned_before_one_cleanup_attempt(self):
        runtime, _observed, _frees = self.runtime()
        runtime.backend = ctypes.c_void_p()

        def create(_logger, _config, backend):
            backend._obj.value = 42
            return 1

        runtime.functions["backend_create"] = create
        runtime.functions["backend_free"].return_value = 1
        with self.assertRaisesRegex(RuntimeError, "backendCreate"):
            runtime.open()
        self.assertTrue(runtime.failed)
        self.assertEqual(runtime.backend.value, 42)
        for _ in range(2):
            with self.assertRaisesRegex(RuntimeError, "backendFree"):
                runtime.close()
        runtime.functions["backend_free"].assert_called_once()
        self.assertEqual(runtime.backend.value, 42)

    def test_native_context_loads_snapshot_with_default_config(self):
        runtime, observed, _frees = self.runtime()
        with tempfile.TemporaryDirectory() as directory:
            runtime.load(b"validated snapshot", self.abi(Path(directory)), 0)
            self.assertEqual(observed, [(b"validated snapshot", None)])
            runtime.close()


class TestNativeBurst(unittest.TestCase):
    """Drive NativeRuntime.device through fake QNN entry points, no QAIRT."""

    def device(self, environment):
        runtime = platform_runtime()
        modes = []

        def create_id(_device, _core, client):
            client[0] = 3
            return 0

        def set_power(_client, values):
            modes.append(values[0].contents.value.dcvs.mode)
            return 0

        callbacks = [
            ctypes.CFUNCTYPE(*types)(callback)
            for types, callback in (
                (
                    (
                        ERROR,
                        ctypes.c_uint32,
                        ctypes.c_uint32,
                        ctypes.POINTER(ctypes.c_uint32),
                    ),
                    create_id,
                ),
                ((ERROR, ctypes.c_uint32), lambda _client: 0),
                (
                    (ERROR, ctypes.c_uint32, ctypes.POINTER(ctypes.POINTER(Power))),
                    set_power,
                ),
            )
        ]
        addresses = [ctypes.cast(cb, ctypes.c_void_p).value for cb in callbacks]
        infrastructure = Infrastructure(0, Perf(*addresses, None))

        def infrastructure_info(result):
            result._obj.contents = infrastructure
            return 0

        runtime.functions["infrastructure"] = infrastructure_info
        with patch.dict(os.environ):
            os.environ.pop("QNN_HTP_BURST", None)
            os.environ.update(environment)
            handle = runtime.device(0)
        self.assertEqual(handle.value, 7)
        return modes, runtime.power

    def test_burst_is_applied_by_default(self):
        for environment in ({}, {"QNN_HTP_BURST": "1"}):
            with self.subTest(environment=environment):
                modes, power = self.device(environment)
                self.assertEqual(modes, [0x10])
                self.assertEqual(list(power), [0])

    def test_burst_can_be_turned_off(self):
        modes, power = self.device({"QNN_HTP_BURST": "0"})
        self.assertEqual(modes, [])
        self.assertEqual(power, {})


class TestNativeRtcArithmetic(unittest.TestCase):
    def test_guidance_matches_stock_for_every_schedule_and_native_step(self):
        rng = np.random.default_rng(42)
        actions = rng.normal(size=(1, 50, 32)).astype(np.float32)
        updated = rng.normal(size=actions.shape).astype(np.float32)
        dt = np.float32(-0.1)
        for schedule in RTCAttentionSchedule:
            for delay, width, rows in (
                (0, 18, 50),
                (2, 18, 30),
                (12, 18, 8),
                (22, 18, 28),
                (48, 18, 2),
            ):
                previous = rng.normal(size=(rows, 8)).astype(np.float32)
                processor = RTCProcessor(
                    RTCConfig(enabled=True, prefix_attention_schedule=schedule)
                )
                for step in range(11):
                    with self.subTest(
                        schedule=schedule, delay=delay, rows=rows, step=step
                    ):
                        timestep = float(np.float32(1 - step / 10))
                        velocity = torch.from_numpy((updated - actions) / dt)
                        guided = processor.denoise_step(
                            torch.from_numpy(actions),
                            torch.from_numpy(previous),
                            delay,
                            timestep,
                            lambda _x: velocity,
                            execution_horizon=delay + width,
                        )
                        expected = (
                            (torch.from_numpy(actions) + dt * guided).detach().numpy()
                        )
                        actual = guide_native_update(
                            actions,
                            updated,
                            timestep,
                            previous,
                            delay,
                            delay + width,
                            dt,
                            processor,
                        )
                        np.testing.assert_allclose(actual, expected, rtol=0, atol=4e-6)

    def test_no_carryover_returns_the_exact_native_update(self):
        updated = np.ones((1, 50, 32), dtype=np.float32)
        self.assertIs(
            guide_native_update(
                np.zeros_like(updated), updated, 1, None, 0, 0, np.float32(-0.1), None
            ),
            updated,
        )

    def test_empty_carryover_starts_without_guidance(self):
        for previous in (None, []):
            self.assertEqual(
                prepare_rtc(
                    {"prev_chunk_left_over": previous, "guidance_horizon": 18}, 50, 8
                ),
                (None, 0, 18),
            )

    def test_invalid_native_schedule_is_refused(self):
        actions = np.zeros((1, 50, 32), dtype=np.float32)
        processor = RTCProcessor(RTCConfig(enabled=True))
        for timestep, dt in ((float("nan"), -0.1), (1.1, -0.1), (0.5, 0.0)):
            with self.subTest(timestep=timestep, dt=dt), self.assertRaisesRegex(
                ValueError, "schedule"
            ):
                guide_native_update(
                    actions,
                    actions,
                    timestep,
                    np.zeros((20, 8), dtype=np.float32),
                    2,
                    18,
                    np.float32(dt),
                    processor,
                )


class TestNativeTensorAbi(unittest.TestCase):
    def row(self):
        return {
            "version": "QNN_TENSOR_VERSION_2",
            "info": {
                "id": 19,
                "name": "tokens",
                "dataType": "QNN_DATATYPE_INT_32",
                "quantizeParams": {
                    "definition": "QNN_DEFINITION_UNDEFINED",
                    "quantizationEncoding": "QNN_QUANTIZATION_ENCODING_UNDEFINED",
                },
                "rank": 2,
                "dimensions": [1, 200],
                "type": "QNN_TENSOR_TYPE_APP_WRITE",
                "dataFormat": "QNN_TENSOR_DATA_FORMAT_FLAT_BUFFER",
            },
        }

    def test_tensor_preserves_integer_id_dtype_and_shape(self):
        tensors, refs = NativeModel.tensor_array([self.row()], True)
        self.assertEqual(tensors[0].data.v2.id, 19)
        self.assertEqual(tensors[0].data.v2.dtype, 0x0032)
        self.assertEqual(tensors[0].data.v2.name, b"tokens")
        self.assertEqual(refs[0]["shape"], (1, 200))
        self.assertEqual(refs[0]["dtype"], np.int32)
        # The ctypes tensor is the native ABI itself, not implementation text.
        self.assertEqual(ctypes.sizeof(Tensor), 144)

    def test_unknown_quantized_dynamic_or_duplicate_ports_are_refused(self):
        for field, value in (
            ("dataType", "QNN_DATATYPE_UFIXED_POINT_8"),
            ("id", 2**32),
            ("rank", 3),
            ("dimensions", [1, 0]),
            ("dataFormat", "sparse"),
            ("type", "QNN_TENSOR_TYPE_APP_READ"),
        ):
            row = self.row()
            row["info"][field] = value
            with self.subTest(field=field), self.assertRaises(ValueError):
                NativeModel.tensor_array([row], True)
        row = self.row()
        row["info"]["quantizeParams"]["definition"] = "QNN_DEFINITION_DEFINED"
        with self.assertRaises(ValueError):
            NativeModel.tensor_array([row], True)
        with self.assertRaises(ValueError):
            NativeModel.tensor_array([self.row(), self.row()], True)
        with self.assertRaises(RuntimeError):
            function(None, ctypes.c_uint64)

    def test_missing_library_does_not_prevent_cpu_tensor_construction(self):
        with patch("vla_qnn_runtime.c.CDLL", side_effect=OSError("not installed")):
            NativeModel.tensor_array([self.row()], True)
            with self.assertRaisesRegex(RuntimeError, "QAIRT 2.49"):
                NativeRuntime()

    def test_native_model_binds_buffers_and_refuses_wrong_dtype(self):
        model = NativeModel.__new__(NativeModel)
        model.context = ctypes.c_void_p(1)
        model.graph = ctypes.c_void_p(2)
        model.graph_name = "test_graph"
        model.device_id = 0
        model.input_names = ["tokens"]
        model.output_names = ["actions"]
        model.inputs, model.input_refs = NativeModel.tensor_array([self.row()], True)
        output_row = self.row()
        output_row["info"].update(
            name="actions",
            type="QNN_TENSOR_TYPE_APP_READ",
            dataType="QNN_DATATYPE_FLOAT_32",
        )
        model.outputs, model.output_refs = NativeModel.tensor_array([output_row], False)
        calls = []

        def execute(
            _graph, inputs, _input_count, outputs, _output_count, _profile, _signal
        ):
            tokens = np.ctypeslib.as_array(
                ctypes.cast(
                    inputs[0].data.v2.memory.client.data, ctypes.POINTER(ctypes.c_int32)
                ),
                shape=(200,),
            )
            result = np.ctypeslib.as_array(
                ctypes.cast(
                    outputs[0].data.v2.memory.client.data,
                    ctypes.POINTER(ctypes.c_float),
                ),
                shape=(200,),
            )
            result[:] = tokens * 0.5
            calls.append(True)
            return 0

        runtime = MagicMock()
        runtime.failed = False
        runtime.release_started = False
        runtime.lock = threading.RLock()
        runtime.functions = {"graph_execute": execute}
        model.runtime = runtime
        result = model.run({"tokens": np.ones((1, 200), dtype=np.int32)})
        np.testing.assert_array_equal(
            result["actions"], np.full((1, 200), 0.5, dtype=np.float32)
        )
        model.output_refs[0]["buffer"].fill(0)
        np.testing.assert_array_equal(result["actions"], 0.5)
        for values in (
            {"tokens": np.ones((1, 200), dtype=np.float32)},
            {"tokens": np.ones((1, 199), dtype=np.int32)},
            {"other": np.ones((1, 200), dtype=np.int32)},
        ):
            with self.assertRaises(ValueError):
                model.run(values)
        self.assertEqual(len(calls), 1)
        runtime.functions["graph_execute"] = MagicMock(return_value=0)  # NaN prefill.
        with self.assertRaisesRegex(RuntimeError, "nonfinite"):
            model.run({"tokens": np.ones((1, 200), dtype=np.int32)})
        self.assertTrue(runtime.failed)
        runtime.failed = False
        runtime.functions["graph_execute"] = MagicMock(return_value=1)
        runtime.check.side_effect = RuntimeError("graphExecute failed")
        with self.assertRaisesRegex(RuntimeError, "graphExecute"):
            model.run({"tokens": np.ones((1, 200), dtype=np.int32)})
        self.assertTrue(runtime.failed)
        with self.assertRaisesRegex(RuntimeError, "restart"):
            model.run({"tokens": np.ones((1, 200), dtype=np.int32)})
        runtime.functions["graph_execute"].assert_called_once()

    def test_cleanup_keeps_dependencies_owned_when_context_free_fails(self):
        model = MagicMock()
        model.close.side_effect = RuntimeError("context busy")
        runtime = bare_runtime(
            models=[model],
            devices={0: ctypes.c_void_p(2)},
            functions={"device_free": MagicMock(), "backend_free": MagicMock()},
        )
        with self.assertRaisesRegex(RuntimeError, "context busy"):
            runtime.close()
        self.assertEqual(runtime.models, [model])
        runtime.functions["device_free"].assert_not_called()
        runtime.functions["backend_free"].assert_not_called()


if __name__ == "__main__":
    unittest.main()
