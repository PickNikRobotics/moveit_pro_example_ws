#!/usr/bin/env python3

# Copyright 2026 PickNik Inc.
# All rights reserved.
#
# Unauthorized copying of this code base via any medium is strictly prohibited.
# Proprietary and confidential.

"""Run a deterministic stereo sequence through the shared VO core."""

from __future__ import annotations

import argparse
from collections import Counter
import json
from math import isfinite
from pathlib import Path
from time import perf_counter

import cv2
import numpy as np

from vo_vslam_evaluation_harness.adapters import (
    EagerAlikedLightGlueAdapter,
    TensorRTAlikedLightGlueAdapter,
    TensorRTConfig,
)
from vo_vslam_evaluation_harness.calibration import StereoCalibration
from vo_vslam_evaluation_harness.core import StereoVisualOdometry, VOConfig
from vo_vslam_evaluation_harness.playback import validate_stereo_frame_keys


def main() -> None:
    """Process a sorted sequence and write measured completion diagnostics."""
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--runtime", choices=("eager", "fp32_tf32", "fp32_strict"), required=True
    )
    parser.add_argument("--dataset-root", type=Path, required=True)
    parser.add_argument("--engine", type=Path)
    parser.add_argument("--engine-metadata", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--max-frames", type=int, default=0)
    parser.add_argument("--fx", type=float, required=True)
    parser.add_argument("--fy", type=float, required=True)
    parser.add_argument("--cx", type=float, required=True)
    parser.add_argument("--cy", type=float, required=True)
    parser.add_argument("--baseline-m", type=float, required=True)
    parser.add_argument("--frame-rate-hz", type=float, default=10.0)
    arguments = parser.parse_args()

    left_paths = sorted((arguments.dataset_root / "image_left").glob("*.png"))
    right_paths = sorted((arguments.dataset_root / "image_right").glob("*.png"))
    if not left_paths or len(left_paths) != len(right_paths):
        raise ValueError("sequence must contain equal nonzero stereo image counts")
    validate_stereo_frame_keys(left_paths, right_paths)
    if not isfinite(arguments.frame_rate_hz) or arguments.frame_rate_hz <= 0.0:
        raise ValueError("frame-rate-hz must be finite and positive")
    if arguments.max_frames > 0:
        left_paths = left_paths[: arguments.max_frames]
        right_paths = right_paths[: arguments.max_frames]

    if arguments.runtime == "eager":
        adapter = EagerAlikedLightGlueAdapter()
        engine_sha256 = None
    else:
        if arguments.engine is None or arguments.engine_metadata is None:
            parser.error("--engine and --engine-metadata are required for TensorRT")
        adapter = TensorRTAlikedLightGlueAdapter(
            TensorRTConfig(
                str(arguments.engine),
                engine_metadata_path=str(arguments.engine_metadata),
                precision=arguments.runtime,
            )
        )
        engine_sha256 = None
    tracker = StereoVisualOdometry(
        adapter,
        StereoCalibration(
            fx=arguments.fx,
            fy=arguments.fy,
            cx=arguments.cx,
            cy=arguments.cy,
            baseline_m=arguments.baseline_m,
        ),
        VOConfig(
            min_disparity_px=0.5,
            max_depth_m=100.0,
            min_pnp_points=6,
            pnp_reprojection_error_px=0.75,
            pnp_confidence=0.999,
            pnp_iterations=100,
            reset_after_failures=3,
        ),
    )

    states: Counter[str] = Counter()
    failures: Counter[str] = Counter()
    total_latencies: list[float] = []
    last_pose = np.eye(4)
    image_shape: tuple[int, int] | None = None
    started = perf_counter()
    for index, (left_path, right_path) in enumerate(zip(left_paths, right_paths)):
        left = cv2.imread(str(left_path), cv2.IMREAD_GRAYSCALE)
        right = cv2.imread(str(right_path), cv2.IMREAD_GRAYSCALE)
        if left is None or right is None:
            raise RuntimeError(f"failed to read frame {index}")
        if left.shape != right.shape:
            raise ValueError(f"stereo image dimensions differ at frame {index}")
        if image_shape is None:
            image_shape = left.shape
        elif left.shape != image_shape:
            raise ValueError(f"image dimensions changed at frame {index}")
        result = tracker.process(left, right, index / arguments.frame_rate_hz)
        states[result.state.value] += 1
        if result.failure_reason:
            failures[result.failure_reason] += 1
        if "total" in result.stage_latency_ms:
            total_latencies.append(result.stage_latency_ms["total"])
        last_pose = result.world_from_camera

    elapsed = perf_counter() - started
    if isinstance(adapter, TensorRTAlikedLightGlueAdapter):
        engine_sha256 = adapter.engine_sha256
    summary = {
        "runtime": adapter.name,
        "engine_sha256": engine_sha256,
        "calibration": {
            "fx": arguments.fx,
            "fy": arguments.fy,
            "cx": arguments.cx,
            "cy": arguments.cy,
            "baseline_m": arguments.baseline_m,
        },
        "image_size": [image_shape[1], image_shape[0]] if image_shape else None,
        "timestamp_source": "synthetic_index",
        "frame_rate_hz": arguments.frame_rate_hz,
        "frames_completed": len(left_paths),
        "state_counts": dict(sorted(states.items())),
        "failure_counts": dict(sorted(failures.items())),
        "wall_seconds": elapsed,
        "mean_total_ms": float(np.mean(total_latencies)),
        "p95_total_ms": float(np.percentile(total_latencies, 95)),
        "final_world_from_camera": np.asarray(last_pose).tolist(),
    }
    arguments.output.parent.mkdir(parents=True, exist_ok=True)
    arguments.output.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n")
    print(json.dumps(summary, sort_keys=True))


if __name__ == "__main__":
    main()
