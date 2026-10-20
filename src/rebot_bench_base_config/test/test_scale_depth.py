# Copyright 2026 PickNik Inc.
# SPDX-License-Identifier: BSD-3-Clause
import importlib.util
from pathlib import Path

import numpy as np
from sensor_msgs.msg import Image

spec = importlib.util.spec_from_file_location(
    "scale_depth", Path(__file__).parents[1] / "scripts/scale_depth.py"
)
scale_depth = importlib.util.module_from_spec(spec)
spec.loader.exec_module(scale_depth)


def image(values, encoding, dtype):
    data = np.array(values, dtype=dtype)
    return Image(
        height=1,
        width=len(values),
        encoding=encoding,
        step=data.nbytes,
        data=data.tobytes(),
    )


def test_millimetre_depth_scales_and_keeps_invalid_zero():
    out = scale_depth.scale_depth(image([0, 700, 1000], "16UC1", np.uint16), 0.96)
    assert np.frombuffer(bytes(out.data), np.uint16).tolist() == [0, 672, 960]
    assert (out.encoding, out.width, out.step) == ("16UC1", 3, 6)


def test_metre_depth_scales():
    out = scale_depth.scale_depth(image([0.5, 1.0], "32FC1", np.float32), 0.96)
    assert np.allclose(np.frombuffer(bytes(out.data), np.float32), [0.48, 0.96])
