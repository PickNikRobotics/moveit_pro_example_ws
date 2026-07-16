#!/usr/bin/env python3

# Copyright 2026 PickNik Inc.
# All rights reserved.
#
# Unauthorized copying of this code base via any medium is strictly prohibited.
# Proprietary and confidential.

"""Contract tests for lazy eager and corrected TensorRT feature adapters."""

import hashlib
import json
from pathlib import Path

import numpy as np
import pytest

from vo_vslam_evaluation_harness.adapters import (
    EagerAlikedLightGlueAdapter,
    TensorRTAlikedLightGlueAdapter,
    TensorRTConfig,
    load_verified_tensorrt_engine,
    normalize_tensorrt_outputs,
)
from vo_vslam_evaluation_harness.core import FeatureSet


class FakeBackend:
    """Minimal eager or TensorRT runtime used to validate adapter behavior."""

    def extract(self, image: np.ndarray) -> dict[str, np.ndarray]:
        """Return stable candidate output arrays."""
        return {
            "keypoints": np.array([[10.0, 20.0]], dtype=np.float32),
            "descriptors": np.ones((1, 4), dtype=np.float32),
            "scores": np.array([0.9], dtype=np.float32),
        }

    def match(self, first: FeatureSet, second: FeatureSet) -> np.ndarray:
        """Return one valid feature match."""
        return np.array([[0, 0]], dtype=np.int64)


def test_eager_runtime_is_lazy_and_normalizes_feature_shapes() -> None:
    """Import-heavy eager runtime creation is deferred until the first frame."""
    calls = 0

    def factory() -> FakeBackend:
        nonlocal calls
        calls += 1
        return FakeBackend()

    adapter = EagerAlikedLightGlueAdapter(backend_factory=factory)
    assert calls == 0

    features = adapter.extract(np.zeros((32, 48), dtype=np.uint8))

    assert calls == 1
    assert features.keypoints.shape == (1, 2)
    assert features.descriptors.shape == (1, 4)
    assert features.scores.shape == (1,)
    assert features.image_size == (48, 32)


def test_corrected_tensorrt_contract_denormalizes_and_filters_outputs() -> None:
    """Corrected ALIKED coordinates and eager-equivalent score filtering are applied."""
    outputs = {
        "keypoints": np.array([[[-1.0, -1.0], [1.0, 1.0]]], dtype=np.float32),
        "descriptors": np.arange(8, dtype=np.float32).reshape(1, 2, 4),
        "scores": np.array([[0.1, 0.9]], dtype=np.float32),
    }

    features = normalize_tensorrt_outputs(
        outputs,
        original_size=(100, 50),
        processed_size=(50, 25),
        scales=np.array([0.5, 0.5], dtype=np.float32),
        score_threshold=0.2,
    )

    np.testing.assert_allclose(features.keypoints, [[98.5, 48.5]])
    np.testing.assert_allclose(features.descriptors, [[4.0, 5.0, 6.0, 7.0]])
    np.testing.assert_allclose(features.scores, [0.9])


@pytest.mark.parametrize("precision", ["fp32_tf32", "fp32_strict", "fp16"])
def test_tensorrt_precision_profiles_are_explicit(precision: str) -> None:
    """Primary, parity-reference, and optional precision modes are selectable."""
    config = TensorRTConfig(engine_path="/tmp/aliked.engine", precision=precision)
    adapter = TensorRTAlikedLightGlueAdapter(
        config, backend_factory=lambda _: FakeBackend()
    )

    assert adapter.name == f"aliked_tensorrt_{precision}_lightglue"


def test_tensorrt_defaults_to_corrected_fp32_tf32_and_rejects_unknown_precision() -> (
    None
):
    """The measured corrected profile and resize contract are the safe defaults."""
    config = TensorRTConfig(engine_path="model.engine")

    assert config.precision == "fp32_tf32"
    assert config.preprocess_resize == 1024
    with pytest.raises(ValueError, match="precision"):
        TensorRTConfig(engine_path="model.engine", precision="int8")


def test_tensorrt_metadata_binds_engine_hash_and_profile(tmp_path: Path) -> None:
    """A real backend cannot report a profile unrelated to its engine artifact."""
    engine = tmp_path / "aliked.engine"
    engine.write_bytes(b"engine artifact")
    digest = hashlib.sha256(engine.read_bytes()).hexdigest()
    metadata = tmp_path / "aliked.engine.json"
    metadata.write_text(
        json.dumps(
            {
                "schema_version": 1,
                "engine_sha256": digest,
                "precision": "fp32_tf32",
                "preprocess_resize": 1024,
                "score_threshold": 0.2,
            }
        ),
        encoding="utf-8",
    )
    config = TensorRTConfig(
        str(engine), engine_metadata_path=str(metadata), precision="fp32_tf32"
    )

    verified_bytes, verified_digest = load_verified_tensorrt_engine(config, engine)
    assert verified_bytes == b"engine artifact"
    assert verified_digest == digest

    relabeled = TensorRTConfig(
        str(engine), engine_metadata_path=str(metadata), precision="fp16"
    )
    with pytest.raises(RuntimeError, match="precision"):
        load_verified_tensorrt_engine(relabeled, engine)


@pytest.mark.parametrize("score_threshold", [float("nan"), float("inf"), -0.1, 1.1])
def test_tensorrt_config_rejects_invalid_score_threshold(
    score_threshold: float,
) -> None:
    """Filtering thresholds must remain finite probabilities."""
    with pytest.raises(ValueError, match="score_threshold"):
        TensorRTConfig("engine", score_threshold=score_threshold)


@pytest.mark.parametrize("resize", [float("nan"), float("inf"), 1024.0, True, 0])
def test_tensorrt_config_requires_positive_integral_resize(resize: object) -> None:
    """Tensor shapes cannot use non-integral or non-finite configuration."""
    with pytest.raises(ValueError, match="preprocess_resize"):
        TensorRTConfig("engine", preprocess_resize=resize)  # type: ignore[arg-type]


def test_tensorrt_metadata_must_be_json_object(tmp_path: Path) -> None:
    """Malformed metadata follows the documented RuntimeError path."""
    engine = tmp_path / "engine"
    metadata = tmp_path / "engine.json"
    engine.write_bytes(b"engine")
    metadata.write_text("[]", encoding="utf-8")
    config = TensorRTConfig(str(engine), engine_metadata_path=str(metadata))

    with pytest.raises(RuntimeError, match="JSON object"):
        load_verified_tensorrt_engine(config, engine)


def test_adapter_rejects_malformed_candidate_output() -> None:
    """Candidate output shape mismatches fail at the adapter boundary."""
    with pytest.raises(ValueError, match="descriptors"):
        FeatureSet(np.zeros((2, 2)), np.zeros((1, 4)), np.ones(2))


def test_feature_set_rejects_nonfinite_candidate_output() -> None:
    """NaN candidate output never reaches stereo geometry or PnP."""
    with pytest.raises(ValueError, match="finite"):
        FeatureSet(
            np.array([[float("nan"), 0.0]], dtype=np.float32),
            np.zeros((1, 128), dtype=np.float32),
            np.ones(1, dtype=np.float32),
        )
