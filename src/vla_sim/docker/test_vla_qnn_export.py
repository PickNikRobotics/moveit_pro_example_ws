#!/usr/bin/env python3

# Copyright 2026 PickNik Inc.
# SPDX-License-Identifier: BSD-3-Clause

"""Semantic export-contract tests with tiny ONNX experts, never real native models."""

import hashlib
import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import numpy as np
import onnx
from onnx import TensorProto, helper, numpy_helper
from onnx.reference import ReferenceEvaluator

from vla_qnn_export import (
    LEGACY_EXPERT_SHA256,
    emit_groot_sampling,
    emit_sampling,
    groot_sampling_values,
    inspect_sampling,
    sampling_values,
    validate_groot_sampling,
    validate_sampling,
)


def expert(chunk=16, steps=4):
    """A runnable fixed-grid integrated expert seam, not a neural/native export."""
    constants = {
        "one": np.asarray(1, np.float32),
        "steps": np.asarray(steps, np.float32),
        "zero": np.asarray(0, np.int64),
        "last": np.asarray(steps, np.int64),
        "dt": np.asarray(-1 / steps, np.float32),
        "conditioning": np.broadcast_to(
            np.arange(steps + 1, dtype=np.float32)[:, None], (steps + 1, 1024)
        ).copy(),
        "slice_start": np.asarray([0], np.int64),
        "slice_end": np.asarray([32], np.int64),
        "slice_axis": np.asarray([1], np.int64),
        "velocity_shape": np.asarray([1, 1, 32], np.int64),
    }
    nodes = [
        helper.make_node("Sub", ["one", "time_step"], ["remaining"]),
        helper.make_node("Mul", ["remaining", "steps"], ["scaled"]),
        helper.make_node("Round", ["scaled"], ["rounded"]),
        helper.make_node("Cast", ["rounded"], ["index"], to=TensorProto.INT64),
        helper.make_node("Clip", ["index", "zero", "last"], ["clipped"]),
        helper.make_node("Gather", ["conditioning", "clipped"], ["condition"], axis=0),
        helper.make_node(
            "Slice",
            ["condition", "slice_start", "slice_end", "slice_axis"],
            ["narrow_condition"],
        ),
        helper.make_node(
            "Reshape", ["narrow_condition", "velocity_shape"], ["velocity"]
        ),
        helper.make_node("Mul", ["velocity", "dt"], ["integrated"]),
        helper.make_node("Add", ["x_t", "integrated"], ["action_emb"]),
    ]
    graph = helper.make_graph(
        nodes,
        "tiny_sampling_expert",
        [
            helper.make_tensor_value_info("x_t", TensorProto.FLOAT, [1, chunk, 32]),
            helper.make_tensor_value_info("time_step", TensorProto.FLOAT, [1]),
        ],
        [
            helper.make_tensor_value_info(
                "action_emb", TensorProto.FLOAT, [1, chunk, 32]
            )
        ],
        [numpy_helper.from_array(value, name) for name, value in constants.items()],
    )
    return helper.make_model(graph, opset_imports=[helper.make_opsetid("", 20)])


class TestQnnExportSampling(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        self.source = self.root / "expert.onnx"
        self.context = self.root / "model.bin"
        self.context.write_bytes(b"fake compiled context, not a real QNN artifact")

    def save(self, model):
        onnx.checker.check_model(model)
        onnx.save(model, self.source)

    def test_emitted_alternate_contract_runs_actual_onnx_grid_and_integrated_updates(
        self,
    ):
        for chunk, steps in ((16, 4), (72, 12), (50, 10)):
            with self.subTest(chunk=chunk, steps=steps):
                model = expert(chunk, steps)
                self.save(model)
                output = self.root / f"sampling-{steps}.json"
                value = emit_sampling(self.source, self.context, output)
                self.assertEqual(json.loads(output.read_text()), value)
                digest = hashlib.sha256(self.context.read_bytes()).hexdigest()
                self.assertEqual(value["context_sha256"], digest)
                self.assertEqual(
                    value["source_sha256"][self.source.name],
                    hashlib.sha256(self.source.read_bytes()).hexdigest(),
                )
                self.assertEqual(
                    validate_sampling(value, sampling_values(chunk, steps), digest),
                    value,
                )
                evaluator = ReferenceEvaluator(model)
                actions = np.ones((1, chunk, 32), dtype=np.float32)
                for index, time in enumerate(value["timesteps"]):
                    (updated,) = evaluator.run(
                        None,
                        {"x_t": actions, "time_step": np.asarray([time], np.float32)},
                    )
                    np.testing.assert_array_equal(
                        updated,
                        actions
                        + np.float32(index) * np.float32(value["integration_dt"]),
                    )
                    actions = updated

    def test_emission_hashes_external_conditioning_bytes_without_neural_execution(self):
        model = expert()
        onnx.save_model(
            model,
            self.source,
            save_as_external_data=True,
            all_tensors_to_one_file=True,
            location="weights.bin",
            size_threshold=1000,
        )
        value = emit_sampling(self.source, self.context, self.root / "sampling.json")
        self.assertEqual(
            value["source_sha256"]["weights.bin"],
            hashlib.sha256((self.root / "weights.bin").read_bytes()).hexdigest(),
        )

    def test_export_refuses_grid_scale_clip_and_integration_mismatches(self):
        for constant, array in (
            ("steps", np.asarray(10, np.float32)),
            ("dt", np.asarray(-0.1, np.float32)),
            ("last", np.asarray(3, np.int64)),
            ("conditioning", np.zeros((4, 1024), np.float32)),
        ):
            with self.subTest(constant=constant):
                model = expert()
                tensor = next(t for t in model.graph.initializer if t.name == constant)
                tensor.CopyFrom(numpy_helper.from_array(array, constant))
                self.save(model)
                with self.assertRaisesRegex(ValueError, "QNN sampling"):
                    inspect_sampling(self.source)

    def test_export_refuses_velocity_output_and_dead_conditioning(self):
        for output in ("velocity", "dead_conditioning"):
            with self.subTest(output=output):
                model = expert()
                node = model.graph.node[-1]
                if output == "velocity":
                    node.CopyFrom(
                        helper.make_node("Identity", ["integrated"], ["action_emb"])
                    )
                else:
                    model.graph.node[-2].input[0] = "x_t"
                self.save(model)
                with self.assertRaisesRegex(ValueError, "QNN sampling"):
                    inspect_sampling(self.source)

    def test_export_refuses_additional_time_semantics(self):
        model = expert()
        model.graph.node.append(
            helper.make_node("Identity", ["time_step"], ["other_time"])
        )
        self.save(model)
        with self.assertRaisesRegex(ValueError, "additional consumers"):
            inspect_sampling(self.source)

    def test_source_emission_never_overwrites_an_existing_descriptor(self):
        self.save(expert())
        output = self.root / "sampling.json"
        value = emit_sampling(self.source, self.context, output)
        with self.assertRaises(FileExistsError):
            emit_sampling(self.source, self.context, output)
        self.assertEqual(json.loads(output.read_text()), value)

    def test_only_known_legacy_expert_can_omit_sampling_metadata(self):
        config = sampling_values(50, 10)
        value = validate_sampling(None, config, LEGACY_EXPERT_SHA256)
        self.assertEqual(value["integration_dt"], float(np.float32(-0.1)))
        with self.assertRaisesRegex(ValueError, "required"):
            validate_sampling(None, config, "a" * 64)
        with self.assertRaisesRegex(ValueError, "chunk_size"):
            validate_sampling(None, sampling_values(16, 10), LEGACY_EXPERT_SHA256)

    def test_known_legacy_digest_cannot_be_relabelled_as_an_alternate_expert(self):
        value = {
            **sampling_values(16, 4),
            "context_sha256": LEGACY_EXPERT_SHA256,
            "source_sha256": {"expert.onnx": "a" * 64},
        }
        with self.assertRaisesRegex(ValueError, "retained 50/10"):
            validate_sampling(value, sampling_values(16, 4), LEGACY_EXPERT_SHA256)

    def test_groot_schedule_comes_from_its_model_and_policy_configs(self):
        model = self.root / "model_config.json"
        policy = self.root / "config.json"
        model.write_text(
            json.dumps(
                {
                    "action_horizon": 40,
                    "max_action_dim": 132,
                    "num_inference_timesteps": 4,
                    "num_timestep_buckets": 1000,
                    "rtc_ramp_rate": 6.0,
                }
            )
        )
        policy.write_text(
            json.dumps(
                {"chunk_size": 40, "num_inference_timesteps": 8, "rtc_ramp_rate": None}
            )
        )
        output = self.root / "sampling.json"
        value = emit_groot_sampling(model, policy, self.context, output)
        self.assertEqual(json.loads(output.read_text()), value)
        self.assertEqual(value["timesteps"], [0, 125, 250, 375, 500, 625, 750, 875])
        self.assertEqual(
            (value["integration_dt"], value["rtc_ramp_rate"]), (0.125, 6.0)
        )
        digest = value["context_sha256"]
        config = {"chunk_size": 40, "max_action_dim": 132, "denoising_steps": 8}
        self.assertIs(validate_groot_sampling(value, config, digest), value)
        for field, change in (
            ("context_sha256", "e" * 64),
            ("timesteps", [0] * 8),
            ("integration_dt", 0.25),
            ("timestep_buckets", 4),
            ("schedule", "uniform_euler_round_clip.v1"),
        ):
            with self.subTest(field=field), self.assertRaisesRegex(
                ValueError, "sampling"
            ):
                validate_groot_sampling({**value, field: change}, config, digest)
        with self.assertRaisesRegex(ValueError, "denoising_steps"):
            validate_groot_sampling(value, {**config, "denoising_steps": 4}, digest)
        with self.assertRaises(FileExistsError):
            emit_groot_sampling(model, policy, self.context, output)
        with self.assertRaisesRegex(ValueError, "--groot-policy-config"):
            emit_groot_sampling(model, None, self.context, self.root / "other.json")
        unverified = {**value, "source_sha256": {"model_config.json": "d" * 64}}
        with self.assertRaisesRegex(ValueError, "--groot-policy-config"):
            validate_groot_sampling(unverified, config, digest)

    def groot_configs(self, model_fields=None, policy_fields=None):
        model = self.root / "model_config.json"
        policy = self.root / "config.json"
        model.write_text(
            json.dumps(
                {
                    "action_horizon": 40,
                    "max_action_dim": 132,
                    "num_inference_timesteps": 8,
                    "num_timestep_buckets": 1000,
                    "rtc_ramp_rate": 6.0,
                    **(model_fields or {}),
                }
            )
        )
        policy.write_text(json.dumps(policy_fields or {}))
        return model, policy

    def test_groot_export_refuses_a_policy_cap_below_the_native_horizon(self):
        output = self.root / "sampling.json"
        for cap in ("chunk_size", "n_action_steps"):
            with self.subTest(cap=cap), self.assertRaisesRegex(
                ValueError,
                f"{cap} \\(16\\) is below the model's action_horizon \\(40\\)",
            ):
                emit_groot_sampling(
                    *self.groot_configs(policy_fields={cap: 16}), self.context, output
                )
        self.assertFalse(output.exists())
        model, policy = self.groot_configs(
            policy_fields={"chunk_size": 40, "n_action_steps": 40}
        )
        value = emit_groot_sampling(model, policy, self.context, output)
        self.assertEqual((value["chunk_size"], value["action_horizon"]), (40, 40))
        self.assertEqual(value["policy_caps"], {"chunk_size": 40, "n_action_steps": 40})
        config = {"chunk_size": 40, "max_action_dim": 132, "denoising_steps": 8}
        digest = value["context_sha256"]
        self.assertIs(validate_groot_sampling(value, config, digest), value)
        # Serving enforces the same rule on the recorded metadata.
        shortened = {
            **value,
            **groot_sampling_values(16, 132, 8, 1000, 6.0),
            "policy_caps": {"n_action_steps": 16},
        }
        with self.assertRaisesRegex(ValueError, "action_horizon"):
            validate_groot_sampling(shortened, {**config, "chunk_size": 16}, digest)
        capped = {**value, "policy_caps": {"n_action_steps": 16}}
        with self.assertRaisesRegex(ValueError, "below the model's action_horizon"):
            validate_groot_sampling(capped, config, digest)

    def test_groot_export_applies_the_serving_schedule_checks(self):
        output = self.root / "sampling.json"
        for fields in (
            {"num_timestep_buckets": 4},
            {"rtc_ramp_rate": float("nan")},
            {"rtc_ramp_rate": float("inf")},
        ):
            with self.subTest(fields=fields), self.assertRaisesRegex(
                ValueError, "timestep_buckets/rtc_ramp_rate"
            ):
                emit_groot_sampling(
                    *self.groot_configs(model_fields=fields), self.context, output
                )
        self.assertFalse(output.exists())

    def test_failed_sidecar_write_leaves_nothing_and_a_retry_succeeds(self):
        model, policy = self.groot_configs()
        output = self.root / "sampling.json"
        with patch("vla_qnn_export.json.dumps", side_effect=OSError("disk full")):
            with self.assertRaisesRegex(OSError, "disk full"):
                emit_groot_sampling(model, policy, self.context, output)
        self.assertEqual(
            sorted(p.name for p in self.root.iterdir() if "sampling" in p.name), []
        )
        value = emit_groot_sampling(model, policy, self.context, output)
        self.assertEqual(json.loads(output.read_text()), value)
        self.assertEqual(
            sorted(p.name for p in self.root.iterdir() if "sampling" in p.name),
            ["sampling.json"],
        )


if __name__ == "__main__":
    unittest.main()
