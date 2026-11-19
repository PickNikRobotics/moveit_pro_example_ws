#!/usr/bin/env python3

# Copyright 2026 PickNik Inc.
# SPDX-License-Identifier: BSD-3-Clause

"""Export-time sampling metadata for the supported pi0.5 integrated expert.

Call emit_sampling() from the trusted context compiler/stager with the exact
ONNX source used to compile that context. This reads the graph's actual index,
conditioning-table extent and Euler constant, not serving knobs. ONNX is an
export-only dependency; the serving consumer needs only NumPy/standard library.
"""

import argparse
import hashlib
import json
import re
from pathlib import Path

import numpy as np

SCHEDULE = "uniform_euler_round_clip.v1"
# The retained native-RMS expert predates the descriptor. Its exporter fixes
# 50 rows / 10 updates. No other metadata-free binary inherits that schedule.
LEGACY_EXPERT_SHA256 = (
    "9a7c1e47c3a6fa14d34751fe6f418b9d08ca831436ec0cf1a9859aa35b76d125"
)


def sampling_values(chunk_size: int, steps: int) -> dict:
    """Describe the recognized uniform, already-integrated expert schedule."""
    return {
        "schema": 1,
        "schedule": SCHEDULE,
        "chunk_size": chunk_size,
        "max_action_dim": 32,
        "denoising_steps": steps,
        "conditioning_rows": steps + 1,
        "timesteps": [float(np.float32(1 - i / steps)) for i in range(steps)],
        "integration_dt": float(np.float32(-1 / steps)),
        "expert_output": "internally_integrated_actions",
    }


def validate_sampling(value, config: dict, context_sha256: str) -> dict:
    """Cross-check an export descriptor against the immutable context and bundle.

    The compiler/stager is trusted to pair its real source with its output. A
    context digest binds this descriptor to the bytes actually loaded; native
    IO checks independently verify the compiled chunk and chaining dimensions.
    This is not binary introspection or authentication of an untrusted exporter.
    """
    if value is None:
        if context_sha256 != LEGACY_EXPERT_SHA256:
            raise ValueError(
                "QNN sampling metadata is required for this context; emit it from the actual expert export"
            )
        value = sampling_values(50, 10)
    else:
        if not isinstance(value, dict) or value.get("context_sha256") != context_sha256:
            raise ValueError(
                "QNN sampling context_sha256 does not match the loaded expert"
            )
        sources = value.get("source_sha256")
        if (
            not isinstance(sources, dict)
            or not sources
            or any(
                not isinstance(name, str)
                or Path(name).name != name
                or not isinstance(digest, str)
                or not re.fullmatch(r"[0-9a-f]{64}", digest)
                for name, digest in sources.items()
            )
        ):
            raise ValueError(
                "QNN sampling source_sha256 must identify the compiled ONNX source files"
            )
    if context_sha256 == LEGACY_EXPERT_SHA256 and (
        value.get("chunk_size") != 50 or value.get("denoising_steps") != 10
    ):
        raise ValueError(
            "QNN sampling cannot relabel the retained 50/10 compiled expert"
        )
    if (
        type(value.get("schema")) is not int
        or value["schema"] != 1
        or value.get("schedule") != SCHEDULE
    ):
        raise ValueError(f"QNN sampling requires schema=1 and {SCHEDULE}")
    for field in ("chunk_size", "max_action_dim", "denoising_steps"):
        if type(value.get(field)) is not int or value[field] != config[field]:
            raise ValueError(f"QNN sampling {field} differs from the compiled bundle")
    steps = value["denoising_steps"]
    if (
        type(value.get("conditioning_rows")) is not int
        or value["conditioning_rows"] != steps + 1
    ):
        raise ValueError(
            "QNN sampling conditioning_rows must equal denoising_steps + 1"
        )
    if value.get("expert_output") != "internally_integrated_actions":
        raise ValueError("QNN sampling expert must return already-integrated actions")
    times = value.get("timesteps")
    dt = value.get("integration_dt")
    if (
        not isinstance(times, list)
        or len(times) != steps
        or any(type(t) not in (int, float) or not 0 <= t <= 1 for t in times)
        or type(dt) not in (int, float)
        or not -1 <= dt < 0
    ):
        raise ValueError("QNN sampling timesteps/integration_dt are invalid")
    expected = sampling_values(value["chunk_size"], steps)
    if not np.array_equal(
        np.asarray(times, dtype=np.float32), expected["timesteps"]
    ) or np.float32(dt) != np.float32(expected["integration_dt"]):
        raise ValueError(
            "QNN sampling conditioning grid and integrated Euler interval disagree"
        )
    return value


def inspect_sampling(source: Path) -> dict:
    """Read the supported ONNX semantics without loading or executing its weights.

    Only the fixed-grid FP32 source layout used by the four/six graph exporter
    is recognized. The neural body may contain the established precision/RMS
    rewrites; the schedule seam must remain Round -> Cast -> Clip -> Gather and
    the output must remain x_t + (negative constant * velocity).
    """
    import onnx
    from onnx import TensorProto, helper, numpy_helper

    source = Path(source)
    snapshot = source.read_bytes()
    model = onnx.load_model_from_string(snapshot)
    producers = {output: node for node in model.graph.node for output in node.output}
    initializers = {tensor.name: tensor for tensor in model.graph.initializer}

    def node(value, kind):
        found = producers.get(value)
        if (
            found is None
            or found.op_type != kind
            or found.domain not in ("", "ai.onnx")
        ):
            raise ValueError(f"QNN sampling export requires {kind} at {value}")
        return found

    def scalar(name, dtype=TensorProto.FLOAT):
        tensor = initializers.get(name)
        if (
            tensor is None
            or tensor.data_location == TensorProto.EXTERNAL
            or tensor.dims
            or tensor.data_type != dtype
        ):
            raise ValueError(
                f"QNN sampling export requires a typed inline scalar at {name}"
            )
        return numpy_helper.to_array(tensor).item()

    def shape(value):
        tensor = value.type.tensor_type
        dims = tuple(d.dim_value for d in tensor.shape.dim)
        if (
            tensor.elem_type != TensorProto.FLOAT
            or not dims
            or any(d <= 0 for d in dims)
        ):
            raise ValueError(
                f"QNN sampling export requires fixed Float32 IO at {value.name}"
            )
        return dims

    inputs = {value.name: value for value in model.graph.input}
    outputs = {value.name: value for value in model.graph.output}
    if (
        "x_t" not in inputs
        or "time_step" not in inputs
        or set(outputs) != {"action_emb"}
    ):
        raise ValueError("QNN sampling export requires x_t/time_step -> action_emb")
    actions = shape(inputs["x_t"])
    if (
        len(actions) != 3
        or actions[0] != 1
        or actions[2] != 32
        or shape(outputs["action_emb"]) != actions
        or shape(inputs["time_step"]) != (1,)
    ):
        raise ValueError(
            "QNN sampling export requires matching Float32 [1,chunk,32] actions and scalar time"
        )

    # Inspect the real output integration, never assume a scalar time IO makes
    # an expert compatible with a different host-loop count.
    addition = node("action_emb", "Add")
    if len(addition.input) != 2 or "x_t" not in addition.input:
        raise ValueError(
            "QNN sampling expert output is not an integrated action update"
        )
    product = node(next(name for name in addition.input if name != "x_t"), "Mul")
    constants = [
        name
        for name in product.input
        if name in initializers and not initializers[name].dims
    ]
    if len(product.input) != 2 or len(constants) != 1:
        raise ValueError("QNN sampling expert must have one baked Euler interval")
    dt = scalar(constants[0])
    if not isinstance(dt, float) or not np.isfinite(dt) or not -1 <= dt < 0:
        raise ValueError("QNN sampling expert Euler interval must be negative Float32")
    steps = round(-1 / dt)
    if (
        steps < 1
        or (steps + 1) * 1024 * 4 > 0xFFFFFFFF
        or np.float32(dt) != np.float32(-1 / steps)
    ):
        raise ValueError(
            "QNN sampling expert does not use the supported uniform Euler interval"
        )

    # The conditioning table is compiled, not generated by the serving loop.
    gathers = [
        n
        for n in model.graph.node
        if n.op_type == "Gather" and n.input and n.input[0] == "conditioning"
    ]
    table = initializers.get("conditioning")
    if (
        len(gathers) != 1
        or table is None
        or tuple(table.dims) != (steps + 1, 1024)
        or table.data_type != TensorProto.FLOAT
    ):
        raise ValueError(
            "QNN sampling conditioning table does not match the compiled update count"
        )
    gather = gathers[0]
    # Reject a dead table/index seam: its result must actually condition the
    # velocity that the graph integrates, not merely exist in unused nodes.
    velocity = next(name for name in product.input if name != constants[0])
    pending = [velocity]
    dependencies = set()
    while pending:
        value = pending.pop()
        if value in dependencies:
            continue
        dependencies.add(value)
        if value in producers:
            pending.extend(producers[value].input)
    if gather.output[0] not in dependencies:
        raise ValueError(
            "QNN sampling conditioning table does not drive the integrated expert"
        )
    if gather.domain not in ("", "ai.onnx") or any(
        a.name == "axis" and helper.get_attribute_value(a) != 0
        for a in gather.attribute
    ):
        raise ValueError("QNN sampling conditioning Gather must select time rows")
    index = gather.input[1]
    if index in producers and producers[index].op_type == "Reshape":
        index = producers[index].input[0]
    clip = node(index, "Clip")
    if (
        len(clip.input) != 3
        or scalar(clip.input[1], TensorProto.INT64) != 0
        or scalar(clip.input[2], TensorProto.INT64) != steps
    ):
        raise ValueError(
            "QNN sampling conditioning Clip differs from the compiled grid"
        )
    cast = node(clip.input[0], "Cast")
    if {a.name: helper.get_attribute_value(a) for a in cast.attribute} != {
        "to": TensorProto.INT64
    }:
        raise ValueError("QNN sampling conditioning index must cast to INT64")
    rounded = node(cast.input[0], "Round")
    scaled = node(rounded.input[0], "Mul")
    scale = [name for name in scaled.input if name in initializers]
    if len(scale) != 1 or scalar(scale[0]) != steps:
        raise ValueError(
            "QNN sampling conditioning index scale differs from Euler count"
        )
    sub = node(next(name for name in scaled.input if name != scale[0]), "Sub")
    if len(sub.input) != 2 or sub.input[1] != "time_step" or scalar(sub.input[0]) != 1:
        raise ValueError(
            "QNN sampling conditioning requires round((1-time_step)*steps)"
        )
    time_consumers = [n for n in model.graph.node if "time_step" in n.input]
    if len(time_consumers) != 1 or time_consumers[0] != sub:
        raise ValueError("QNN sampling time input has unsupported additional consumers")

    result = sampling_values(actions[1], steps)
    files = {source.name: hashlib.sha256(snapshot).hexdigest()}
    for tensor in model.graph.initializer:
        if tensor.data_location == TensorProto.EXTERNAL:
            fields = {entry.key: entry.value for entry in tensor.external_data}
            location = fields.get("location", "")
            if not location or Path(location).name != location:
                raise ValueError(
                    "QNN sampling source weights must be beside the ONNX graph"
                )
            if location not in files:
                with (source.parent / location).open("rb") as stream:
                    files[location] = hashlib.file_digest(stream, "sha256").hexdigest()
    return {**result, "source_sha256": files}


def emit_sampling(source: Path, context: Path, output: Path) -> dict:
    """Emit a sidecar at trusted compilation/staging time for that exact artifact.

    This helper never compiles or pairs arbitrary pre-existing exports by IO
    alone. The caller must supply the source used in its successful compilation.
    Copy the returned object into that expert's manifest.json `sampling` field.
    """
    value = inspect_sampling(source)
    with Path(context).open("rb") as stream:
        value["context_sha256"] = hashlib.file_digest(stream, "sha256").hexdigest()
    with Path(output).open("x") as stream:
        stream.write(json.dumps(value, indent=2) + "\n")
    return value


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--source",
        required=True,
        type=Path,
        help="exact ONNX source passed to the compiler",
    )
    parser.add_argument(
        "--context", required=True, type=Path, help="context produced from that source"
    )
    parser.add_argument(
        "--output", required=True, type=Path, help="new sampling JSON sidecar"
    )
    args = parser.parse_args()
    emit_sampling(args.source, args.context, args.output)


if __name__ == "__main__":
    main()
