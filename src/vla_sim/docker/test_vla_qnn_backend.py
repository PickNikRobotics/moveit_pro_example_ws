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
import tempfile
import threading
import unittest
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
    PolicyRunner,
    QnnPolicyRunner,
    ServerState,
    load_policy,
    make_handler,
    parse_args,
    run_inference,
)
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
from vla_qnn_runtime import NativeModel, NativeRuntime, Tensor, function


def expert_update(actions):
    """Deterministic fake already-integrated expert output, not neural inference."""
    return actions * np.float32(0.99) - np.float32(0.002)


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

    def load(self, binary, abi, device):
        assert Path(abi).is_file()
        name = Path(binary).parent.name
        model = FakeModel(name, device, self.ports[name])
        self.models[name] = model
        return model

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
            (directory / "manifest.json").write_text(
                json.dumps(
                    {
                        "context_sha256": hashlib.sha256(binary).hexdigest(),
                        "output_aliases": {
                            f"output_{i}": name for i, name in enumerate(outputs)
                        },
                    }
                )
            )
        self.bundle.write_text(json.dumps(self.config))

    def runner(self, **overrides):
        runtime = FakeRuntime(self.ports)
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
        with patch("vla_qnn_runtime.NativeRuntime", return_value=runtime), patch(
            "vla_qnn_policy.AutoTokenizer.from_pretrained", return_value=self.tokenizer
        ) as tokenizer_load:
            runner = QnnPolicyRunner(**values)
        tokenizer_load.assert_called_once_with(
            str(self.root / "tokenizer"), local_files_only=True
        )
        self.addCleanup(runner.policy.close)
        return runner, runtime

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

    def parsed_args(self, extra=()):
        serving = self.root / "vla_serving.yaml"
        serving.write_text(
            yaml.safe_dump(
                {
                    "checkpoint": self.config["model_id"],
                    "backend": "qnn",
                    "qnn_bundle": str(self.bundle),
                    "fps": 10.0,
                    "state_dim": 8,
                }
            )
        )
        with patch(
            "sys.argv", ["vla_inference_server.py", "--config", str(serving), *extra]
        ):
            return parse_args()


class TestQnnSelection(QnnFixture):
    def test_yaml_selects_qnn_without_loading_pytorch_or_hub(self):
        args = self.parsed_args()
        runtime = FakeRuntime(self.ports)
        state = ServerState()
        with patch("vla_qnn_runtime.NativeRuntime", return_value=runtime), patch(
            "vla_qnn_policy.AutoTokenizer.from_pretrained", return_value=self.tokenizer
        ), patch("vla_inference_server.PolicyRunner") as pytorch, patch(
            "vla_inference_server.resolve_policy_type"
        ) as family, patch(
            "vla_inference_server.resolve_fps"
        ) as fps, patch(
            "vla_inference_server.torch.cuda.is_available",
            side_effect=AssertionError("no GPU probe"),
        ):
            load_policy(state, args)
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

    def test_default_and_cli_override_keep_pytorch_path(self):
        args = self.parsed_args(("--backend", "pytorch"))
        runner = MagicMock(spec=PolicyRunner)
        runner.request_names = []
        runner.warmup.return_value = (0.1, 0.1, 50)
        runner.expected_camera_keys.return_value = []
        runner.expected_state_dim.return_value = 8
        state = ServerState()
        with patch(
            "vla_inference_server.PolicyRunner", return_value=runner
        ) as pytorch, patch(
            "vla_qnn_runtime.NativeRuntime",
            side_effect=AssertionError("optional QAIRT loaded"),
        ):
            args.policy_class = "pi05"
            args.device = "cpu"
            load_policy(state, args)
        self.assertEqual(state.status, "ready", state.detail)
        pytorch.assert_called_once()
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
            ({"qnn_bundle": ["wrong"]}, "qnn_bundle"),
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
        with patch("vla_qnn_runtime.NativeRuntime", return_value=runtime), patch(
            "vla_qnn_policy.AutoTokenizer.from_pretrained", return_value=self.tokenizer
        ), patch.object(
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
        with patch("vla_qnn_runtime.NativeRuntime", return_value=runtime), patch(
            "vla_qnn_policy.AutoTokenizer.from_pretrained", return_value=self.tokenizer
        ), self.assertRaisesRegex(ValueError, "graph ABI"):
            QnnPolicyRunner(
                str(self.bundle), "test/pi05", "pi05", "", 8, 10.0, 8, "EXP"
            )
        self.assertTrue(runtime.closed)

    def test_bundle_refuses_unsupported_sampling_mapping_and_devices(self):
        changes = {
            "schema": True,
            "graph_abi": "groot",
            "denoising_steps": 9,
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


class TestQnnInference(QnnFixture):
    def test_off_path_preserves_integrated_updates_and_noise_advancement(self):
        runner, runtime = self.runner()
        state = self.ready_state(runner)
        rng = np.random.default_rng(265)
        for _ in range(2):
            expected = rng.standard_normal((1, 50, 32), dtype=np.float32)
            noise = expected.copy()
            for _step in range(10):
                expected = expert_update(expected)
            with patch(
                "vla_qnn_policy.guide_native_update",
                side_effect=AssertionError("off path guided"),
            ):
                result = run_inference(state, self.payload())
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

    def test_first_rtc_chunk_has_raw_echo_but_no_guidance(self):
        runner, _ = self.runner()
        payload = {**self.payload(), "guidance_horizon": 18}
        with patch(
            "vla_qnn_policy.guide_native_update",
            side_effect=AssertionError("first chunk guided"),
        ):
            result = run_inference(self.ready_state(runner), payload)
        self.assertEqual(np.shape(result["action_chunk_raw"]), (50, 8))
        self.assertNotEqual(result["action_chunk_raw"], result["action_chunk"])

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
        self.assertTrue(runner.int8)
        self.assertEqual(np.shape(result["action_chunk_raw"]), (50, 8))
        self.assertEqual(result["dt"], 0.1)

    def test_http_status_and_rtc_round_trip_use_existing_contract(self):
        runner, runtime = self.runner()
        state = self.ready_state(runner)
        server = ThreadingHTTPServer(("127.0.0.1", 0), make_handler(state))
        worker = threading.Thread(target=server.serve_forever, daemon=True)
        worker.start()
        self.addCleanup(worker.join)
        self.addCleanup(server.server_close)
        self.addCleanup(server.shutdown)
        conn = http.client.HTTPConnection("127.0.0.1", server.server_port, timeout=10)
        self.addCleanup(conn.close)
        headers = {"Authorization": "Bearer " + state.bearer_key}
        conn.request("GET", "/status", headers=headers)
        response = conn.getresponse()
        status = json.loads(response.read())
        self.assertEqual(response.status, 200)
        self.assertEqual(status["device"], "npu:qnn_htp")
        self.assertEqual(status["checkpointRevision"], "a" * 40)
        self.assertIn("configRevision", status)
        conn.request(
            "POST",
            "/infer",
            body=json.dumps({**self.payload(), "guidance_horizon": 18}),
            headers=headers,
        )
        response = conn.getresponse()
        first = json.loads(response.read())
        self.assertEqual(response.status, 200)
        tail = first["action_chunk_raw"][20:]
        conn.request(
            "POST",
            "/infer",
            body=json.dumps(
                {
                    **self.payload(),
                    "prev_chunk_left_over": tail,
                    "inference_delay": 4,
                    "guidance_horizon": 18,
                }
            ),
            headers=headers,
        )
        response = conn.getresponse()
        second = json.loads(response.read())
        self.assertEqual(response.status, 200)
        self.assertEqual(np.shape(second["action_chunk_raw"]), (50, 8))
        self.assertEqual(second["dt"], 0.1)
        conn.request(
            "POST",
            "/infer",
            body=json.dumps({**self.payload(), "prev_chunk_left_over": [[True] * 8]}),
            headers=headers,
        )
        response = conn.getresponse()
        response.read()
        self.assertEqual(response.status, 400)
        self.assertEqual(len(runtime.models["action_expert"].calls), 20)


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
        model.call_count = 0
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
        self.assertEqual(model.call_count, 1)

    def test_cleanup_keeps_dependencies_owned_when_context_free_fails(self):
        runtime = NativeRuntime.__new__(NativeRuntime)
        runtime.lock = threading.RLock()
        model = MagicMock()
        model.close.side_effect = RuntimeError("context busy")
        runtime.models = [model]
        runtime.devices = {0: ctypes.c_void_p(2)}
        runtime.power = {}
        runtime.backend = ctypes.c_void_p(3)
        runtime.functions = {"device_free": MagicMock(), "backend_free": MagicMock()}
        with self.assertRaisesRegex(RuntimeError, "context busy"):
            runtime.close()
        self.assertEqual(runtime.models, [model])
        runtime.functions["device_free"].assert_not_called()
        runtime.functions["backend_free"].assert_not_called()


if __name__ == "__main__":
    unittest.main()
