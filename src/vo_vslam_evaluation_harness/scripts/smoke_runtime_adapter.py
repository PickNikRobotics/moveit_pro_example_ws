#!/usr/bin/env python3

# Copyright 2026 PickNik Inc.
# All rights reserved.
#
# Unauthorized copying of this code base via any medium is strictly prohibited.
# Proprietary and confidential.

"""Smoke one stereo pair through a configured feature adapter."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2

from vo_vslam_evaluation_harness.adapters import (
    EagerAlikedLightGlueAdapter,
    TensorRTAlikedLightGlueAdapter,
    TensorRTConfig,
)


def main() -> None:
    """Extract and match one pair, printing measured counts as JSON."""
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--runtime", choices=("eager", "fp32_tf32", "fp32_strict"), required=True
    )
    parser.add_argument("--left", type=Path, required=True)
    parser.add_argument("--right", type=Path, required=True)
    parser.add_argument("--engine", type=Path)
    parser.add_argument("--engine-metadata", type=Path)
    arguments = parser.parse_args()

    left = cv2.imread(str(arguments.left), cv2.IMREAD_GRAYSCALE)
    right = cv2.imread(str(arguments.right), cv2.IMREAD_GRAYSCALE)
    if left is None or right is None:
        raise RuntimeError("failed to read one or both input images")
    if left.shape != right.shape:
        raise ValueError("left and right image dimensions differ")

    if arguments.runtime == "eager":
        adapter = EagerAlikedLightGlueAdapter()
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

    left_features = adapter.extract(left)
    right_features = adapter.extract(right)
    matches = adapter.match(left_features, right_features)
    print(
        json.dumps(
            {
                "runtime": adapter.name,
                "image_size": list(left_features.image_size or ()),
                "left_keypoints": int(left_features.keypoints.shape[0]),
                "right_keypoints": int(right_features.keypoints.shape[0]),
                "stereo_matches": int(matches.shape[0]),
            },
            sort_keys=True,
        )
    )


if __name__ == "__main__":
    main()
