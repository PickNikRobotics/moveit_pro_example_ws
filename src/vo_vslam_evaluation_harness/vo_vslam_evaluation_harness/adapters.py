# Copyright 2026 PickNik Inc.
# All rights reserved.
#
# Unauthorized copying of this code base via any medium is strictly prohibited.
# Proprietary and confidential.

"""Lazy ALIKED + LightGlue eager and corrected TensorRT adapters."""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
from math import isfinite
from numbers import Integral
from pathlib import Path
from typing import Any, Callable, Protocol

import cv2
import numpy as np

from .core import FeatureSet


class _Backend(Protocol):
    def extract(self, image: np.ndarray) -> dict[str, np.ndarray]:
        """Return unbatched keypoints, descriptors, and scores."""
        raise NotImplementedError

    def match(self, first: FeatureSet, second: FeatureSet) -> np.ndarray:
        """Return matched feature index pairs."""
        raise NotImplementedError


def _feature_set(output: dict[str, np.ndarray]) -> FeatureSet:
    try:
        keypoints = np.asarray(output["keypoints"], dtype=np.float32)
        descriptors = np.asarray(output["descriptors"], dtype=np.float32)
        scores = np.asarray(output["scores"], dtype=np.float32)
    except KeyError as error:
        raise ValueError(
            "candidate output must contain keypoints, descriptors, and scores"
        ) from error
    if keypoints.ndim == 3 and keypoints.shape[0] == 1:
        keypoints = keypoints[0]
    if descriptors.ndim == 3 and descriptors.shape[0] == 1:
        descriptors = descriptors[0]
    if scores.ndim == 2 and scores.shape[0] == 1:
        scores = scores[0]
    return FeatureSet(keypoints, descriptors, scores)


class _LazyAdapter:
    """Shared lazy backend lifecycle and shape normalization."""

    name = ""

    def __init__(self, backend_factory: Callable[[], _Backend]) -> None:
        self._backend_factory = backend_factory
        self._backend: _Backend | None = None

    def _get_backend(self) -> _Backend:
        if self._backend is None:
            self._backend = self._backend_factory()
        return self._backend

    def extract(self, image: np.ndarray) -> FeatureSet:
        """Extract and validate a normalized feature set."""
        image = np.asarray(image)
        if image.ndim < 2:
            raise ValueError("image must have height and width dimensions")
        features = _feature_set(self._get_backend().extract(image))
        return FeatureSet(
            features.keypoints,
            features.descriptors,
            features.scores,
            image_size=(int(image.shape[1]), int(image.shape[0])),
        )

    def match(self, first: FeatureSet, second: FeatureSet) -> np.ndarray:
        """Match normalized feature sets with the candidate runtime."""
        return np.asarray(
            self._get_backend().match(first, second), dtype=np.int64
        ).reshape(-1, 2)


class EagerAlikedLightGlueAdapter(_LazyAdapter):
    """Lazy eager PyTorch ALIKED + LightGlue reference/fallback adapter."""

    name = "aliked_eager_lightglue"

    def __init__(
        self,
        max_keypoints: int = 768,
        model_name: str = "aliked-n16",
        lightglue_layers: int = 9,
        depth_confidence: float = 0.95,
        width_confidence: float = 0.99,
        filter_threshold: float = 0.1,
        device: str = "cuda",
        backend_factory: Callable[[], _Backend] | None = None,
    ) -> None:
        factory = backend_factory or (
            lambda: _EagerBackend(
                max_keypoints,
                model_name,
                lightglue_layers,
                depth_confidence,
                width_confidence,
                filter_threshold,
                device,
            )
        )
        super().__init__(factory)


class _EagerBackend:
    """Import-heavy PyTorch implementation instantiated only on first use."""

    def __init__(
        self,
        max_keypoints: int,
        model_name: str,
        lightglue_layers: int,
        depth_confidence: float,
        width_confidence: float,
        filter_threshold: float,
        device_name: str,
    ) -> None:
        try:
            import torch
            from lightglue import ALIKED, LightGlue
            from lightglue.utils import rbd
        except ImportError as error:
            raise RuntimeError(
                "eager adapter requires torch and the lightglue package"
            ) from error
        if device_name == "cuda" and not torch.cuda.is_available():
            raise RuntimeError("CUDA eager runtime requested but CUDA is unavailable")
        self._torch = torch
        self._rbd = rbd
        self._device = torch.device(device_name)
        self._extractor = (
            ALIKED(model_name=model_name, max_num_keypoints=max_keypoints)
            .eval()
            .to(self._device)
        )
        self._matcher = (
            LightGlue(
                features="aliked",
                n_layers=lightglue_layers,
                depth_confidence=depth_confidence,
                width_confidence=width_confidence,
                filter_threshold=filter_threshold,
            )
            .eval()
            .to(self._device)
        )

    def _tensor(self, image: np.ndarray) -> Any:
        if image.ndim == 2:
            image = np.repeat(image[:, :, None], 3, axis=2)
        if image.ndim != 3 or image.shape[2] not in (1, 3, 4):
            raise ValueError("image must be grayscale or HxWxC")
        image = image[:, :, :3]
        tensor = self._torch.from_numpy(np.ascontiguousarray(image)).permute(2, 0, 1)
        return tensor.float().div(255.0).to(self._device)

    def extract(self, image: np.ndarray) -> dict[str, np.ndarray]:
        """Run eager ALIKED using the validated [0, 1] RGB contract."""
        with self._torch.inference_mode():
            features = self._extractor.extract(self._tensor(image))
        features = self._rbd(features)
        return {
            "keypoints": features["keypoints"].detach().cpu().numpy(),
            "descriptors": features["descriptors"].detach().cpu().numpy(),
            "scores": features["keypoint_scores"].detach().cpu().numpy(),
        }

    def _torch_features(self, features: FeatureSet) -> dict[str, Any]:
        if features.image_size is None:
            raise ValueError("LightGlue features require image_size metadata")
        return {
            "keypoints": self._torch.from_numpy(features.keypoints).to(self._device)[
                None
            ],
            "descriptors": self._torch.from_numpy(features.descriptors).to(
                self._device
            )[None],
            "keypoint_scores": self._torch.from_numpy(features.scores).to(self._device)[
                None
            ],
            "image_size": self._torch.tensor(
                [features.image_size], dtype=self._torch.float32, device=self._device
            ),
        }

    def match(self, first: FeatureSet, second: FeatureSet) -> np.ndarray:
        """Run LightGlue and return unbatched match indices."""
        with self._torch.inference_mode():
            output = self._matcher(
                {
                    "image0": self._torch_features(first),
                    "image1": self._torch_features(second),
                }
            )
        return self._rbd(output)["matches"].detach().cpu().numpy()


_PRECISIONS = {"fp32_tf32", "fp32_strict", "fp16"}


@dataclass(frozen=True)
class TensorRTConfig:
    """TensorRT engine and corrected eager-parity preprocessing contract."""

    engine_path: str
    engine_metadata_path: str = ""
    precision: str = "fp32_tf32"
    preprocess_resize: int | None = 1024
    score_threshold: float = 0.2

    def __post_init__(self) -> None:
        if self.precision not in _PRECISIONS:
            raise ValueError(
                f"precision must be one of {sorted(_PRECISIONS)}, got {self.precision!r}"
            )
        if self.preprocess_resize is not None and (
            not isinstance(self.preprocess_resize, Integral)
            or isinstance(self.preprocess_resize, bool)
            or self.preprocess_resize <= 0
        ):
            raise ValueError("preprocess_resize must be a positive integer or null")
        if not isfinite(self.score_threshold) or not 0.0 <= self.score_threshold <= 1.0:
            raise ValueError("score_threshold must be a finite probability")


def load_verified_tensorrt_engine(
    config: TensorRTConfig, engine_path: Path
) -> tuple[bytes, str]:
    """Read once and bind immutable engine bytes to a trusted runtime profile."""
    try:
        engine_bytes = engine_path.read_bytes()
    except OSError as error:
        raise RuntimeError(f"failed to read TensorRT engine: {engine_path}") from error
    metadata_path = Path(config.engine_metadata_path)
    if not metadata_path.is_file():
        raise RuntimeError(f"TensorRT engine metadata does not exist: {metadata_path}")
    try:
        metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise RuntimeError(
            f"invalid TensorRT engine metadata: {metadata_path}"
        ) from error
    if not isinstance(metadata, dict):
        raise RuntimeError("TensorRT engine metadata must be a JSON object")
    engine_sha256 = hashlib.sha256(engine_bytes).hexdigest()
    expected = {
        "schema_version": 1,
        "engine_sha256": engine_sha256,
        "precision": config.precision,
        "preprocess_resize": config.preprocess_resize,
        "score_threshold": config.score_threshold,
    }
    for key, value in expected.items():
        if metadata.get(key) != value:
            raise RuntimeError(
                f"TensorRT engine metadata mismatch for {key}: "
                f"expected {value!r}, got {metadata.get(key)!r}"
            )
    return engine_bytes, engine_sha256


def normalize_tensorrt_outputs(
    outputs: dict[str, np.ndarray],
    original_size: tuple[int, int],
    processed_size: tuple[int, int],
    scales: np.ndarray,
    score_threshold: float,
) -> FeatureSet:
    """Apply corrected ALIKED coordinate decoding and eager-equivalent filtering."""
    del original_size  # Encoded by the exact per-axis preprocessing scales.
    features = _feature_set(outputs)
    processed_width, processed_height = processed_size
    scales = np.asarray(scales, dtype=np.float32).reshape(2)
    if not np.all(np.isfinite(scales)) or np.any(scales <= 0.0):
        raise ValueError("preprocessing scales must be finite and positive")
    keypoints = features.keypoints.copy()
    wh_minus_one = np.array(
        [processed_width - 1, processed_height - 1], dtype=np.float32
    )
    keypoints = wh_minus_one * (keypoints + 1.0) / 2.0
    keypoints = (keypoints + 0.5) / scales[None] - 0.5
    keep = features.scores > score_threshold
    return FeatureSet(
        keypoints[keep], features.descriptors[keep], features.scores[keep]
    )


class TensorRTAlikedLightGlueAdapter(_LazyAdapter):
    """Lazy corrected TensorRT ALIKED extractor with eager LightGlue matching."""

    def __init__(
        self,
        config: TensorRTConfig,
        backend_factory: Callable[[TensorRTConfig], _Backend] | None = None,
    ) -> None:
        self.config = config
        self.name = f"aliked_tensorrt_{config.precision}_lightglue"
        factory = backend_factory or (lambda cfg: _TensorRTBackend(cfg))
        super().__init__(lambda: factory(config))

    @property
    def engine_sha256(self) -> str:
        """Return the digest verified by the initialized real backend."""
        digest = getattr(self._get_backend(), "engine_sha256", None)
        if not isinstance(digest, str):
            raise RuntimeError(
                "TensorRT backend did not report a verified engine digest"
            )
        return digest


class _TensorRTBackend:
    """Corrected fixed-shape ALIKED TensorRT runner and eager matcher."""

    def __init__(self, config: TensorRTConfig) -> None:
        engine_path = Path(config.engine_path)
        if not engine_path.is_file():
            raise RuntimeError(f"TensorRT engine does not exist: {engine_path}")
        engine_bytes, self.engine_sha256 = load_verified_tensorrt_engine(
            config, engine_path
        )
        try:
            import tensorrt as trt
            import torch
            from lightglue import LightGlue
            from lightglue.utils import rbd
        except ImportError as error:
            raise RuntimeError(
                "TensorRT adapter requires tensorrt, torch, and lightglue"
            ) from error
        if not torch.cuda.is_available():
            raise RuntimeError("TensorRT adapter requires an available CUDA device")
        self._config = config
        self._trt = trt
        self._torch = torch
        self._rbd = rbd
        self._device = torch.device("cuda")
        logger = trt.Logger(trt.Logger.WARNING)
        trt.init_libnvinfer_plugins(logger, "")
        runtime = trt.Runtime(logger)
        self._engine = runtime.deserialize_cuda_engine(engine_bytes)
        if self._engine is None:
            raise RuntimeError(f"failed to deserialize TensorRT engine: {engine_path}")
        self._context = self._engine.create_execution_context()
        if self._context is None:
            raise RuntimeError("failed to create TensorRT execution context")
        self._buffers: dict[str, Any] = {}
        self._input_name = ""
        self._output_names: list[str] = []
        for index in range(self._engine.num_io_tensors):
            name = self._engine.get_tensor_name(index)
            shape = tuple(int(value) for value in self._engine.get_tensor_shape(name))
            if any(value <= 0 for value in shape):
                raise RuntimeError(
                    "dynamic TensorRT shapes are unsupported by this harness"
                )
            numpy_dtype = trt.nptype(self._engine.get_tensor_dtype(name))
            torch_dtype = torch.from_numpy(np.empty((), dtype=numpy_dtype)).dtype
            buffer = torch.empty(shape, dtype=torch_dtype, device=self._device)
            self._buffers[name] = buffer
            self._context.set_tensor_address(name, int(buffer.data_ptr()))
            if self._engine.get_tensor_mode(name) == trt.TensorIOMode.INPUT:
                if self._input_name:
                    raise RuntimeError(
                        "TensorRT ALIKED engine must have exactly one input"
                    )
                self._input_name = name
            else:
                self._output_names.append(name)
        if not self._input_name:
            raise RuntimeError("TensorRT ALIKED engine has no input")
        if set(self._output_names) != {"keypoints", "descriptors", "scores"}:
            raise RuntimeError(
                "TensorRT ALIKED outputs must be keypoints, descriptors, and scores"
            )
        self._matcher = LightGlue(features="aliked").eval().to(self._device)

    def _preprocess(
        self, image: np.ndarray
    ) -> tuple[Any, tuple[int, int], tuple[int, int], np.ndarray]:
        if image.ndim == 2:
            image = np.repeat(image[:, :, None], 3, axis=2)
        if image.ndim != 3 or image.shape[2] < 3:
            raise ValueError("image must be grayscale or HxWxC")
        original_height, original_width = image.shape[:2]
        processed = image[:, :, :3]
        if self._config.preprocess_resize is not None:
            scale = self._config.preprocess_resize / max(
                original_width, original_height
            )
            processed_width = int(round(original_width * scale))
            processed_height = int(round(original_height * scale))
            processed = cv2.resize(processed, (processed_width, processed_height))
        processed_height, processed_width = processed.shape[:2]
        scales = np.array(
            [processed_width / original_width, processed_height / original_height],
            dtype=np.float32,
        )
        tensor = self._torch.from_numpy(np.ascontiguousarray(processed)).permute(
            2, 0, 1
        )[None]
        tensor = tensor.float().div(255.0).to(self._device)
        return (
            tensor,
            (original_width, original_height),
            (processed_width, processed_height),
            scales,
        )

    def extract(self, image: np.ndarray) -> dict[str, np.ndarray]:
        """Run fixed-shape TensorRT inference with corrected eager preprocessing."""
        tensor, original_size, processed_size, scales = self._preprocess(image)
        expected = self._buffers[self._input_name]
        if tuple(tensor.shape) != tuple(expected.shape):
            raise RuntimeError(
                "TensorRT input shape mismatch: "
                f"image {tuple(tensor.shape)}, engine {tuple(expected.shape)}"
            )
        expected.copy_(tensor)
        stream = self._torch.cuda.current_stream(device=self._device)
        if not self._context.execute_async_v3(stream_handle=stream.cuda_stream):
            raise RuntimeError("TensorRT execute_async_v3 failed")
        stream.synchronize()
        outputs = {
            name: self._buffers[name].detach().cpu().numpy().copy()
            for name in self._output_names
        }
        features = normalize_tensorrt_outputs(
            outputs,
            original_size,
            processed_size,
            scales,
            self._config.score_threshold,
        )
        return {
            "keypoints": features.keypoints,
            "descriptors": features.descriptors,
            "scores": features.scores,
        }

    def _torch_features(self, features: FeatureSet) -> dict[str, Any]:
        if features.image_size is None:
            raise ValueError("LightGlue features require image_size metadata")
        return {
            "keypoints": self._torch.from_numpy(features.keypoints).to(self._device)[
                None
            ],
            "descriptors": self._torch.from_numpy(features.descriptors).to(
                self._device
            )[None],
            "keypoint_scores": self._torch.from_numpy(features.scores).to(self._device)[
                None
            ],
            "image_size": self._torch.tensor(
                [features.image_size], dtype=self._torch.float32, device=self._device
            ),
        }

    def match(self, first: FeatureSet, second: FeatureSet) -> np.ndarray:
        """Match TensorRT features through eager LightGlue."""
        with self._torch.inference_mode():
            output = self._matcher(
                {
                    "image0": self._torch_features(first),
                    "image1": self._torch_features(second),
                }
            )
        return self._rbd(output)["matches"].detach().cpu().numpy()
