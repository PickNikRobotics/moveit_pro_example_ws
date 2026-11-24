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

"""Host preprocessing/chaining for compiled pi0.5 and GR00T N1.7 QNN bundles.

Neural execution is exclusively native QnnGraph_execute. pi0.5 bundles use the
four-graph or six-graph ABI whose expert returns an already-integrated Euler
update; RTC adds only stock LeRobot's residual correction to that update.
GR00T N1.7 bundles use a six-graph ABI whose action step returns a velocity
that the host integrates with the stock flow schedule and RTC blend, or a
seven-graph ABI that splits that action step into two contexts at a DiT-block
residual, for boards whose HTP cannot allocate the whole FP16 step.
"""

import hashlib
import json
import math
import re
from pathlib import Path

import numpy as np
import torch
import torch.nn.functional as F
from lerobot.policies.rtc.modeling_rtc import RTCProcessor
from transformers import AutoTokenizer

from vla_qnn_export import validate_groot_sampling, validate_sampling

FOUR_GRAPH_ABI = "aihub.pi05.four_graphs.euler.v1"
FOUR_GRAPH_ORDER = ("vision_encoder", "token_emb", "backbone", "action_expert")
PREFIX_GROUPS = ((0, 6), (6, 12), (12, 18))
PREFIX_GRAPHS = tuple(f"prefix_{start}_{stop}" for start, stop in PREFIX_GROUPS)
SIX_GRAPH_ABI = "kinova.pi05.six_graphs.euler.v1"
SIX_GRAPH_ORDER = ("vision_encoder", "token_emb", *PREFIX_GRAPHS, "action_expert")
GROOT_ABI = "groot.n17.six_graphs.euler.v1"
GROOT_ORDER = (
    "vision",
    "tokens",
    "backbone_0_8",
    "backbone_8_16",
    "observation_features",
    "action_step",
)
GROOT_SPLIT_ABI = "groot.n17.seven_graphs.euler.v1"
GROOT_SPLIT_STEPS = ("action_step_0", "action_step_1")
GROOT_SPLIT_ORDER = (*GROOT_ORDER[:-1], *GROOT_SPLIT_STEPS)
GROOT_ORDERS = {GROOT_ABI: GROOT_ORDER, GROOT_SPLIT_ABI: GROOT_SPLIT_ORDER}
# Cosmos-Reason2 (Qwen3-VL) image patching and widths of the GR00T N1.7 graphs.
PATCH, TEMPORAL_PATCH, MERGE = 16, 2, 2
HIDDEN, STATE_HIDDEN, ROPE = 2048, 1536, 128


def finite_number(value) -> bool:
    """JSON numeric input, excluding booleans and unrepresentable Python integers."""
    try:
        return type(value) in (int, float) and math.isfinite(value)
    except OverflowError:
        return False


def validate_bundle(config: dict) -> tuple[str, ...]:
    """Refuse unsupported graph, sampling, normalization or device semantics."""
    if (
        not isinstance(config, dict)
        or type(config.get("schema")) is not int
        or config["schema"] != 1
    ):
        raise ValueError("QNN bundle must be a schema=1 JSON object")
    abi = config.get("graph_abi")
    if abi not in (FOUR_GRAPH_ABI, SIX_GRAPH_ABI):
        raise ValueError(f"QNN graph_abi must be {FOUR_GRAPH_ABI} or {SIX_GRAPH_ABI}")
    order = SIX_GRAPH_ORDER if abi == SIX_GRAPH_ABI else FOUR_GRAPH_ORDER
    validate_common(config, order)
    if type(config.get("max_action_dim")) is not int or config["max_action_dim"] != 32:
        raise ValueError(f"QNN {abi} requires max_action_dim=32")
    for field in ("request_state_dim", "model_action_dim"):
        if type(config.get(field)) is not int or not 1 <= config[field] <= 32:
            raise ValueError(f"QNN {field} must be an integer in 1..32")
    if config["model_action_dim"] != config["request_state_dim"]:
        raise ValueError("QNN absolute-joint actions must match request_state_dim")
    if config.get("state_columns") != list(range(config["request_state_dim"])) or any(
        type(column) is not int for column in config["state_columns"]
    ):
        raise ValueError(
            "QNN state_columns must preserve the full trained joint order (0..request_state_dim-1)"
        )
    if len(config["camera_names"]) != 3:
        raise ValueError(
            "QNN bundle must name three distinct trained cameras in graph order"
        )
    if config.get("normalization") != "QUANTILES" or config.get("action_mapping") != {
        "mode": "trained_absolute_quantiles"
    }:
        raise ValueError(
            "QNN bundle requires saved QUANTILES statistics and trained_absolute_quantiles actions; relative-action bundles are unsupported"
        )
    if config.get("expert_output") != "internally_integrated_actions":
        raise ValueError(
            "QNN expert_output must be internally_integrated_actions; no second Euler update is allowed"
        )
    precision = config.get("precision")
    if (
        not isinstance(precision, dict)
        or set(precision) != set(FOUR_GRAPH_ORDER)
        or precision["vision_encoder"] not in ("w8a16", "fp16")
        or precision["token_emb"] != "fp16"
        or precision["backbone"]
        not in (("fp16_split",) if abi == SIX_GRAPH_ABI else ("w4a16", "w8a16"))
        or precision["action_expert"] not in ("w8a16", "w16a16", "fp16_native_rms")
    ):
        raise ValueError(
            "QNN precision must describe the supported vision/token/backbone/expert compilation"
        )
    for field in ("stats", "tokenizer"):
        if not isinstance(config.get(field), str) or not config[field]:
            raise ValueError(f"QNN bundle must name a local {field} path")
    return order


def validate_common(config: dict, order: tuple) -> None:
    """Checks every compiled family shares: identity, timing, cameras and graphs."""
    if not isinstance(config.get("model_id"), str) or not config["model_id"]:
        raise ValueError("QNN bundle must identify its source checkpoint in model_id")
    if not isinstance(config.get("model_revision"), str) or not re.fullmatch(
        r"[0-9a-f]{40}", config["model_revision"]
    ):
        raise ValueError(
            "QNN model_revision must be the source checkpoint's full immutable Hub SHA"
        )
    for field in ("chunk_size", "denoising_steps"):
        if type(config.get(field)) is not int or not 1 <= config[field] < 0xFFFFFFFF:
            raise ValueError(
                f"QNN {field} must be a positive fixed export dimension/count"
            )
    names = config.get("camera_names")
    if (
        not isinstance(names, list)
        or not names
        or any(not isinstance(name, str) or not name for name in names)
        or len(set(names)) != len(names)
    ):
        raise ValueError("QNN bundle must name distinct trained cameras in graph order")
    dt = config.get("dt")
    if not finite_number(dt) or dt <= 0:
        raise ValueError(
            "QNN bundle dt must be finite positive seconds at the trained rate"
        )
    seed = config.get("noise_seed")
    if type(seed) is not int or seed < 0:
        raise ValueError("QNN noise_seed must be a non-negative integer")
    graphs = config.get("graphs")
    if (
        not isinstance(graphs, dict)
        or set(graphs) != set(order)
        or any(not isinstance(path, str) or not path for path in graphs.values())
    ):
        raise ValueError(
            f"QNN bundle must name exactly these graph directories: {order}"
        )
    # IQ9 boards expose HTP devices 0 and 1; IQ10 exposes one multi-core device 0.
    placement = config.get("device_placement")
    if (
        not isinstance(placement, dict)
        or set(placement) != set(order)
        or any(
            type(device) is not int or device not in (0, 1)
            for device in placement.values()
        )
    ):
        raise ValueError(
            "QNN device_placement must name HTP device 0 or 1 for every graph"
        )


def validate_groot_bundle(config: dict) -> tuple[str, ...]:
    """Refuse a GR00T N1.7 bundle outside the compiled six- or seven-graph contract."""
    if (
        not isinstance(config, dict)
        or type(config.get("schema")) is not int
        or config["schema"] != 1
        or config.get("graph_abi") not in GROOT_ORDERS
    ):
        raise ValueError(
            f"QNN GR00T bundle must be schema=1 with graph_abi {GROOT_ABI} "
            f"or {GROOT_SPLIT_ABI}"
        )
    order = GROOT_ORDERS[config["graph_abi"]]
    validate_common(config, order)
    for field in ("max_state_dim", "max_action_dim"):
        if type(config.get(field)) is not int or not 1 <= config[field] <= 4096:
            raise ValueError(f"QNN {field} must be the compiled padded width")
    for field, limit in (
        ("request_state_dim", config["max_state_dim"]),
        ("model_action_dim", config["max_action_dim"]),
    ):
        if type(config.get(field)) is not int or not 1 <= config[field] <= limit:
            raise ValueError(f"QNN {field} must be an integer in 1..{limit}")
    if config["model_action_dim"] != config["request_state_dim"]:
        raise ValueError("QNN GR00T model_action_dim must match request_state_dim")
    if not isinstance(config.get("prompt"), str) or not config["prompt"]:
        raise ValueError("QNN GR00T bundle must name the prompt its tokens encode")
    if config.get("normalization") != "MIN_MAX" or config.get("action_mapping") != {
        "mode": "trained_min_max"
    }:
        raise ValueError(
            "QNN GR00T bundle requires clipped MIN_MAX statistics and trained_min_max actions"
        )
    transform = config.get("image_transform")
    target = isinstance(transform, dict) and transform.get("image_target_size")
    if (
        not isinstance(target, list)
        or len(target) != 2
        or any(type(v) is not int or v < 1 or v % (PATCH * MERGE) for v in target)
        or set(transform)
        - {
            "image_crop_size",
            "image_target_size",
            "shortest_image_edge",
            "crop_fraction",
            "letter_box_transform",
        }
    ):
        raise ValueError(
            "QNN GR00T image_transform must be the checkpoint's default N1.7 transform "
            f"with an image_target_size divisible by {PATCH * MERGE}"
        )
    precision = config.get("precision")
    if (
        not isinstance(precision, dict)
        or set(precision) != set(order)
        or any(value not in ("fp16", "w8a16", "w4a16") for value in precision.values())
    ):
        raise ValueError("QNN precision must name fp16, w8a16 or w4a16 for every graph")
    for name in order[order.index("observation_features") + 1 :]:
        if precision[name] != "fp16":
            raise ValueError(
                f"QNN GR00T precision {name} must be fp16: a quantized action step "
                "drifts enough to trip ExecutePolicy's joint limits"
            )
    for field in ("stats", "static_inputs"):
        if not isinstance(config.get(field), str) or not config[field]:
            raise ValueError(f"QNN bundle must name a local {field} path")
    return order


def prepare_rtc(payload: dict, chunk_size: int, action_dim: int) -> tuple:
    """Validate the existing /infer carryover contract before any graph runs."""
    if not isinstance(payload, dict):
        raise ValueError("/infer RTC payload must be a JSON object")
    delay = payload.get("inference_delay", 0)
    width = payload.get("guidance_horizon", 0)
    for name, value in (("inference_delay", delay), ("guidance_horizon", width)):
        if type(value) is not int or not 0 <= value <= chunk_size:
            raise ValueError(
                f"/infer RTC {name} must be an integer in 0..{chunk_size} steps"
            )
    previous = payload.get("prev_chunk_left_over")
    if previous is None or (isinstance(previous, list) and not previous):
        return None, delay, width
    if (
        not isinstance(previous, list)
        or not 1 <= len(previous) <= chunk_size
        or any(
            not isinstance(row, list)
            or len(row) != action_dim
            or any(not finite_number(value) for value in row)
            for row in previous
        )
    ):
        raise ValueError(
            f"/infer RTC prev_chunk_left_over must contain finite normalized "
            f"rows [1..{chunk_size},{action_dim}]; send the previous action_chunk_raw tail"
        )
    with np.errstate(over="ignore"):
        values = np.asarray(previous, dtype=np.float32)
    if not np.isfinite(values).all():
        raise ValueError(
            "/infer RTC prev_chunk_left_over overflows Float32; discard malformed carryover"
        )
    return values, delay, width


def guide_native_update(
    actions, updated, timestep, previous, delay, horizon, dt, processor
):
    """Add stock detached-velocity RTC residual to a native integrated update.

    The native result is the base update, not a velocity. Recover velocity only
    to compute LeRobot's residual; add the residual to `updated`, never another
    Euler update. Without carryover return the native array unchanged.
    """
    if previous is None:
        return updated
    if (
        actions.shape != updated.shape
        or actions.ndim != 3
        or actions.dtype != np.float32
        or updated.dtype != np.float32
        or not np.isfinite(actions).all()
        or not np.isfinite(updated).all()
    ):
        raise RuntimeError("QNN RTC received invalid/nonfinite native updates")
    if (
        previous.ndim != 2
        or previous.dtype != np.float32
        or not 0 < len(previous) <= actions.shape[1]
        or not 0 < previous.shape[1] <= actions.shape[2]
        or not np.isfinite(previous).all()
    ):
        raise ValueError("QNN RTC requires validated Float32 carryover")
    if (
        not math.isfinite(timestep)
        or not 0 <= timestep <= 1
        or not math.isfinite(dt)
        or dt >= 0
    ):
        raise ValueError("QNN RTC requires the stock decreasing Euler schedule")
    with np.errstate(over="ignore", invalid="ignore"):
        velocity_array = (updated - actions) / dt
    if not np.isfinite(velocity_array).all():
        raise RuntimeError("QNN RTC recovered nonfinite velocity from native updates")
    velocity = torch.from_numpy(velocity_array)
    guided = processor.denoise_step(
        torch.from_numpy(actions),
        torch.from_numpy(previous),
        delay,
        float(timestep),
        lambda _x: velocity,
        execution_horizon=horizon,
    )
    result = updated + dt * (guided - velocity).detach().numpy()
    if not np.isfinite(result).all():
        # The native update and recovered velocity were finite. Refuse this
        # request's unrepresentable guidance without poisoning native sessions.
        raise ValueError(
            "QNN RTC carryover correction overflows Float32; discard this carryover chunk"
        )
    return result


def resize_with_pad(images: torch.Tensor) -> torch.Tensor:
    """LeRobot pi0.5's float NCHW bilinear resize/black pad to 224x224."""
    height, width = images.shape[-2:]
    ratio = max(width / 224, height / 224)
    target_h, target_w = int(height / ratio), int(width / ratio)
    if target_h == 0 or target_w == 0:
        raise ValueError(
            f"/infer camera resize of {height}x{width} produces a zero extent; send a less extreme aspect ratio"
        )
    resized = F.interpolate(
        images, size=(target_h, target_w), mode="bilinear", align_corners=False
    ).clamp(0, 1)
    top, extra_h = divmod(224 - target_h, 2)
    left, extra_w = divmod(224 - target_w, 2)
    return F.pad(resized, (left, left + extra_w, top, top + extra_h), value=0)


class NamedGraph:
    """Keep compiler output aliases inside the native-session boundary."""

    def __init__(self, runtime, artifact: Path, device: int, sampling=None):
        manifest = json.loads((artifact / "manifest.json").read_text())
        # Snapshot once: a later replacement of model.bin cannot swap the
        # bytes that were checked for different, ABI-compatible weights.
        binary = (artifact / "model.bin").read_bytes()
        digest = hashlib.sha256(binary).hexdigest()
        if digest != manifest["context_sha256"]:
            raise ValueError(
                f"QNN context hash changed for {artifact.name}; restore the compiled bundle"
            )
        if sampling is not None:
            self.sampling = sampling(manifest.get("sampling"), digest)
        self.model = runtime.load(
            binary, artifact / "abi.json", device, name=artifact.name
        )
        self.aliases = manifest["output_aliases"]
        if (
            not isinstance(self.aliases, dict)
            or set(self.aliases) != set(self.model.output_names)
            or len(set(self.aliases.values())) != len(self.aliases)
        ):
            raise ValueError(
                "QNN compiler output aliases are ambiguous; validate the bundle's ports"
            )

    def run(self, feed):
        """Expose every native output (all present and finite) under its ONNX name."""
        values = self.model.run(feed)
        return {self.aliases[name]: value for name, value in values.items()}


def graph_ports(split: bool, chunk_size: int = 50) -> dict:
    """Supported pi0.5 graph seams with the export's fixed action-chunk extent."""
    f32, i32 = np.dtype("float32"), np.dtype("int32")

    def floats(**shapes):
        return {name: (shape, f32) for name, shape in shapes.items()}

    caches = {
        f"{kind}_cache_l{i}": ((1, 968, 1, 256), f32)
        for kind in ("k", "v")
        for i in range(18)
    }
    backbone_inputs = floats(
        hidden_state=(1, 968, 2048),
        prefix_att_2d_masks=(1, 1, 968, 968),
        rope_emb_sin=(1, 968, 1, 128),
        rope_emb_cos=(1, 968, 1, 128),
    )
    ports = {
        "vision_encoder": (
            floats(image=(1, 3, 224, 224)),
            floats(img_embed=(1, 256, 2048)),
        ),
        "token_emb": (
            {
                "lang_tokens": ((1, 200), i32),
                **floats(
                    lang_mask=(1, 200),
                    img_embed1=(1, 256, 2048),
                    img_embed2=(1, 256, 2048),
                    img_embed3=(1, 256, 2048),
                ),
            },
            floats(
                prefix_emb=(1, 968, 2048),
                prefix_att_2d=(1, 1, 968, 968),
                prefix_sin=(1, 968, 1, 128),
                prefix_cos=(1, 968, 1, 128),
                suffix_sin=(1, chunk_size, 1, 128),
                suffix_cos=(1, chunk_size, 1, 128),
                full_att_4d=(1, 1, chunk_size, 968 + chunk_size),
            ),
        ),
        "action_expert": (
            {
                **floats(
                    x_t=(1, chunk_size, 32),
                    time_step=(1,),
                    full_att_4d=(1, 1, chunk_size, 968 + chunk_size),
                    rope_emb_sin=(1, chunk_size, 1, 128),
                    rope_emb_cos=(1, chunk_size, 1, 128),
                ),
                **{
                    f"{kind}_cache_l{i}": ((1, 968, 1, 256), f32)
                    for kind in ("key", "value")
                    for i in range(18)
                },
            },
            floats(action_emb=(1, chunk_size, 32)),
        ),
    }
    if split:
        for name, (start, stop) in zip(PREFIX_GRAPHS, PREFIX_GROUPS, strict=True):
            outputs = {
                f"{kind}_cache_l{i}": caches[f"{kind}_cache_l{i}"]
                for kind in ("k", "v")
                for i in range(start, stop)
            }
            if stop < 18:
                outputs.update(floats(hidden_state_out=(1, 968, 2048)))
            ports[name] = (backbone_inputs, outputs)
    else:
        ports["backbone"] = (backbone_inputs, caches)
    return ports


class BundlePolicy:
    """Bundle identity, resident native sessions and their one load/close."""

    family = ""
    warmup_prompt = "warmup"

    def __init__(self, bundle, checkpoint, revision, state_dim, fps, validate):
        self.path = path = Path(bundle).expanduser()
        self.config = config = json.loads(path.read_text())
        self.order = validate(config)
        if checkpoint != config["model_id"] or (
            revision and revision != config["model_revision"]
        ):
            raise ValueError(
                "QNN bundle model_id/model_revision do not match the configured checkpoint"
            )
        if state_dim and state_dim != config["request_state_dim"]:
            raise ValueError(
                "QNN state_dim differs from the compiled bundle's trained width"
            )
        self.fps = 1.0 / config["dt"]
        if not math.isfinite(self.fps):
            raise ValueError("QNN bundle dt produces a nonfinite training fps")
        if (
            not math.isfinite(fps)
            or fps < 0
            or (fps and not math.isclose(fps, self.fps, rel_tol=1e-9))
        ):
            raise ValueError("QNN fps differs from the compiled bundle's trained dt")
        self.rng = np.random.default_rng(config["noise_seed"])

    def local_path(self, value):
        item = Path(value).expanduser()
        return item if item.is_absolute() else self.path.parent / item

    def open_runtime(self):
        """Open QAIRT last, after every host-side input has been validated."""
        # Optional dependency: selecting PyTorch never imports this module or
        # opens a QAIRT library. Selecting QNN must fail, not silently fall back.
        from vla_qnn_runtime import NativeRuntime

        self.runtime = NativeRuntime()
        self.sessions = {}
        self.loaded = False
        self.failed = False
        self.closed = False
        self.artifacts = {
            name: self.local_path(self.config["graphs"][name]) for name in self.order
        }

    def load(self):
        """Load graphs after the runner owns this policy, retaining cleanup on failure.

        Cleanup belongs to the owning runner/load_policy path; a failed native
        free must not discard the runtime's retained handles.
        """
        if self.loaded or self.failed or self.closed:
            raise RuntimeError(
                "QNN policy cannot be reloaded; restart the inference server explicitly"
            )
        expert = self.order[-1]
        try:
            self.runtime.open()
            for name in self.order:
                self.sessions[name] = NamedGraph(
                    self.runtime,
                    self.artifacts[name],
                    self.config["device_placement"][name],
                    self.check_sampling if name == expert else None,
                )
            for name, (inputs, outputs) in self.graph_ports().items():
                session = self.sessions[name]
                model = session.model
                actual_inputs = {
                    key: (ref["shape"], ref["dtype"])
                    for key, ref in zip(
                        model.input_names, model.input_refs, strict=True
                    )
                }
                actual_outputs = {
                    session.aliases[key]: (ref["shape"], ref["dtype"])
                    for key, ref in zip(
                        model.output_names, model.output_refs, strict=True
                    )
                }
                if actual_inputs != inputs or actual_outputs != outputs:
                    raise ValueError(
                        f"QNN {name} input/output names, shapes or dtypes differ from the compiled graph ABI (chunk_size={self.config['chunk_size']})"
                    )
            self.sampling = self.sessions[expert].sampling
            self.loaded = True
        except Exception:
            self.failed = True
            raise

    def check_infer(self, task, state, previous) -> np.ndarray:
        """Shared request checks; returns the Float32 trained-order state."""
        config = self.config
        if not self.loaded or self.failed or self.closed:
            raise RuntimeError(
                "QNN policy is incomplete, failed or closed; restart the inference server explicitly"
            )
        if not isinstance(task, str):
            raise ValueError("/infer task must be a string")
        if len(state) != config["request_state_dim"] or any(
            not finite_number(value) for value in state
        ):
            raise ValueError(
                "/infer state must contain finite numeric positions in trained joint order"
            )
        if previous is not None and (
            not isinstance(previous, np.ndarray)
            or previous.dtype != np.float32
            or previous.ndim != 2
            or not 1 <= len(previous) <= config["chunk_size"]
            or previous.shape[1] != config["model_action_dim"]
            or not np.isfinite(previous).all()
        ):
            raise ValueError(
                "QNN RTC carryover must be validated Float32 normalized actions"
            )
        with np.errstate(over="ignore"):
            state_array = np.asarray(state, dtype=np.float32)
        if not np.isfinite(state_array).all():
            raise ValueError("/infer state overflows Float32")
        return state_array

    def close(self):
        """Release only this backend's resident native contexts."""
        self.closed = True
        self.runtime.close()


class QnnPolicy(BundlePolicy):
    """Resident compiled pi0.5 graphs, local trained preprocessing and RTC."""

    family = "pi05"

    def __init__(
        self,
        bundle: str,
        checkpoint: str,
        revision: str,
        state_dim: int,
        fps: float,
        rtc_config,
    ):
        super().__init__(bundle, checkpoint, revision, state_dim, fps, validate_bundle)
        config = self.config
        self.rtc = RTCProcessor(rtc_config)
        self.stats = json.loads(self.local_path(config["stats"]).read_text())
        for subject, width in (
            ("observation.state", config["request_state_dim"]),
            ("action", config["model_action_dim"]),
        ):
            for field in ("q01", "q99"):
                value = np.asarray(self.stats[subject + "." + field], dtype=np.float32)
                if value.shape != (width,) or not np.isfinite(value).all():
                    raise ValueError(
                        f"QNN {subject}.{field} must contain {width} finite trained values"
                    )
                self.stats[subject + "." + field] = value
            if np.any(self.stats[subject + ".q99"] < self.stats[subject + ".q01"]):
                raise ValueError(f"QNN {subject} q99 is below q01")
        tokenizer_dir = self.local_path(config["tokenizer"]).resolve()
        if not tokenizer_dir.is_dir():
            raise ValueError(
                f"QNN tokenizer must be an existing local directory: {tokenizer_dir}"
            )
        self.tokenizer = AutoTokenizer.from_pretrained(
            str(tokenizer_dir), local_files_only=True
        )
        self.open_runtime()

    def check_sampling(self, value, digest):
        return validate_sampling(value, self.config, digest)

    def graph_ports(self):
        return graph_ports(
            self.config["graph_abi"] == SIX_GRAPH_ABI, self.config["chunk_size"]
        )

    def run_backbone(self, feed):
        """Hide the validated residual split; preserve all 36 unchanged cache names."""
        if self.config["graph_abi"] == FOUR_GRAPH_ABI:
            return self.sessions["backbone"].run(feed)
        hidden = feed["hidden_state"]
        caches = {}
        for name in PREFIX_GRAPHS:
            values = self.sessions[name].run({**feed, "hidden_state": hidden})
            if "hidden_state_out" in values:
                hidden = values.pop("hidden_state_out")
            caches.update(values)
        return caches

    def infer(self, images, state, task, previous, delay, horizon):
        """Run native graph chaining and return physical plus normalized trained actions."""
        config = self.config
        state_array = self.check_infer(task, state, previous)
        low, high = (
            self.stats["observation.state." + field] for field in ("q01", "q99")
        )
        with np.errstate(over="ignore", invalid="ignore"):
            normalized = (
                2
                * (state_array - low)
                / np.where(high == low, np.float32(1e-8), high - low)
                - 1
            )
        if not np.isfinite(normalized).all():
            raise ValueError("/infer state normalization overflows Float32")
        bins = np.digitize(normalized, np.linspace(-1, 1, 257)[:-1]) - 1
        cleaned = task.strip().replace("_", " ").replace("\n", " ")
        prompt = f"Task: {cleaned}, State: {' '.join(map(str, bins))};\nAction: "
        encoded = self.tokenizer(
            [prompt],
            max_length=200,
            padding="max_length",
            padding_side="right",
            truncation=True,
            return_tensors="np",
        )
        processed = [
            (resize_with_pad(images[name].unsqueeze(0)) * 2.0 - 1.0).numpy()
            for name in config["camera_names"]
        ]
        embeddings = [
            self.sessions["vision_encoder"].run({"image": image})["img_embed"]
            for image in processed
        ]
        prefix = self.sessions["token_emb"].run(
            {
                "lang_tokens": encoded["input_ids"].astype(np.int32),
                "lang_mask": encoded["attention_mask"].astype(np.float32),
                **{f"img_embed{i + 1}": value for i, value in enumerate(embeddings)},
            }
        )
        caches = self.run_backbone(
            {
                "hidden_state": prefix["prefix_emb"],
                "prefix_att_2d_masks": prefix["prefix_att_2d"],
                "rope_emb_sin": prefix["prefix_sin"],
                "rope_emb_cos": prefix["prefix_cos"],
            }
        )
        expert = {
            "full_att_4d": prefix["full_att_4d"],
            "rope_emb_sin": prefix["suffix_sin"],
            "rope_emb_cos": prefix["suffix_cos"],
        }
        for i in range(18):
            expert[f"key_cache_l{i}"] = caches[f"k_cache_l{i}"]
            expert[f"value_cache_l{i}"] = caches[f"v_cache_l{i}"]
        action_shape = (1, self.sampling["chunk_size"], self.sampling["max_action_dim"])
        actions = self.rng.standard_normal(action_shape, dtype=np.float32)
        dt = np.float32(self.sampling["integration_dt"])
        for timestep in self.sampling["timesteps"]:
            expert["x_t"] = actions
            expert["time_step"] = np.asarray([timestep], dtype=np.float32)
            updated = self.sessions["action_expert"].run(expert)["action_emb"]
            actions = (
                updated
                if previous is None
                else guide_native_update(
                    actions,
                    updated,
                    float(expert["time_step"][0]),
                    previous,
                    delay,
                    horizon,
                    dt,
                    self.rtc,
                )
            )
        if (
            actions.shape != action_shape
            or actions.dtype != np.float32
            or not np.isfinite(actions).all()
        ):
            raise RuntimeError(
                f"QNN expert must return finite Float32 {action_shape} actions"
            )
        raw = actions[0, :, : config["model_action_dim"]]
        low, high = (self.stats["action." + field] for field in ("q01", "q99"))
        physical = (raw + np.float32(1)) * np.where(
            high == low, np.float32(1e-8), high - low
        ) / np.float32(2) + low
        if not np.isfinite(physical).all():
            raise RuntimeError(
                "QNN QUANTILES postprocessing returned nonfinite actions"
            )
        return physical, raw


def groot_ports(
    tokens: int,
    patches: int,
    chunk_size: int,
    action_dim: int,
    state_dim: int,
    split: bool = False,
) -> dict:
    """GR00T N1.7 graph seams at the export's fixed token/patch/chunk extents.

    With split, the action step is two graphs joined at a DiT-block residual:
    the state token plus one token per chunk row, at the DiT width.
    """
    f32, i32 = np.dtype("float32"), np.dtype("int32")
    sequence = ((1, tokens, HIDDEN), f32)
    image = ((patches // MERGE**2, HIDDEN), f32)
    bias = {
        "attention_bias": ((1, 1, tokens, tokens), f32),
        "rope_cos": ((1, tokens, ROPE), f32),
        "rope_sin": ((1, tokens, ROPE), f32),
    }
    deep = {f"deep{i}": sequence for i in range(3)}
    actions = ((1, chunk_size, action_dim), f32)
    step_inputs = {
        "timestep": ((1,), i32),
        "vl_features": sequence,
        "image_mask": ((1, tokens), f32),
        "valid_mask": ((1, tokens), f32),
    }
    state = {"state_features": ((1, 1, STATE_HIDDEN), f32)}
    if split:
        hidden = {"action_hidden": ((1, 1 + chunk_size, STATE_HIDDEN), f32)}
        steps = {
            "action_step_0": ({"actions": actions, **step_inputs, **state}, hidden),
            "action_step_1": ({**hidden, **step_inputs}, {"velocity": actions}),
        }
    else:
        steps = {
            "action_step": (
                {"actions": actions, **step_inputs, **state},
                {"velocity": actions},
            )
        }
    return {
        "vision": (
            {"patches": ((patches, 3 * TEMPORAL_PATCH * PATCH * PATCH), f32)},
            {"image_embed": image, **{f"deep{i}": image for i in range(3)}},
        ),
        "tokens": ({"token_ids": ((1, tokens), i32)}, {"token_embeds": sequence}),
        "backbone_0_8": (
            {"hidden": sequence, **bias, **deep},
            {"hidden_out": sequence},
        ),
        "backbone_8_16": (
            {"hidden": sequence, **bias},
            {"backbone_features": sequence},
        ),
        "observation_features": (
            {
                "backbone_features": sequence,
                "normalized_state": ((1, 1, state_dim), f32),
            },
            {"vl_features": sequence, "state_features": ((1, 1, STATE_HIDDEN), f32)},
        ),
        **steps,
    }


def min_max(value, low, high, clip=True):
    """GR00T N1.7's [-1, 1] min-max map; a constant column maps to 0."""
    span = high - low
    with np.errstate(over="ignore", invalid="ignore"):
        mapped = np.where(
            span != 0, 2 * (value - low) / np.where(span != 0, span, 1) - 1, 0
        ).astype(np.float32)
    return np.clip(mapped, -1, 1) if clip else mapped


class QnnGrootPolicy(BundlePolicy):
    """Resident compiled GR00T N1.7 graphs, stock host preprocessing, flow and RTC.

    The export fixes one prompt (its token, RoPE and attention inputs are static)
    and the embodiment, so a request for another prompt is refused.
    """

    family = "groot"

    def __init__(self, bundle, checkpoint, revision, state_dim, fps, ramp_rate=0.0):
        super().__init__(
            bundle, checkpoint, revision, state_dim, fps, validate_groot_bundle
        )
        config = self.config
        self.ramp_rate = ramp_rate
        self.warmup_prompt = config["prompt"]
        stats = json.loads(self.local_path(config["stats"]).read_text())
        horizon, width = config["chunk_size"], config["model_action_dim"]

        def array(key, shape):
            value = np.asarray(stats.get(key), dtype=np.float32)
            if value.shape != shape or not np.isfinite(value).all():
                raise ValueError(f"QNN {key} must contain finite {list(shape)} values")
            return value

        state_shape = (config["request_state_dim"],)
        self.state_low = array("observation.state.min", state_shape)
        self.state_high = array("observation.state.max", state_shape)
        self.action_low = array("action.min", (horizon, width))
        self.action_high = array("action.max", (horizon, width))
        relative = stats.get("action.relative")
        if (
            not isinstance(relative, list)
            or len(relative) != width
            or any(type(flag) is not bool for flag in relative)
        ):
            raise ValueError(
                f"QNN action.relative must flag each of the {width} action columns"
            )
        if np.any(self.state_high < self.state_low) or np.any(
            self.action_high < self.action_low
        ):
            raise ValueError("QNN GR00T statistics have max below min")
        self.relative = np.asarray(relative)
        with np.load(
            self.local_path(config["static_inputs"]), allow_pickle=False
        ) as data:
            self.static = {key: data[key].copy() for key in data.files}
        tokens = self.static.get("token_ids", np.zeros((0, 0))).shape[-1]
        target = config["image_transform"]["image_target_size"]
        rows = target[0] * target[1] // (PATCH * MERGE) ** 2
        f32 = np.float32
        expected = {
            "token_ids": ((1, tokens), np.int32),
            "image_mask": ((1, tokens), f32),
            "valid_mask": ((1, tokens), f32),
            "attention_bias": ((1, 1, tokens, tokens), f32),
            "rope_cos": ((1, tokens, ROPE), f32),
            "rope_sin": ((1, tokens, ROPE), f32),
        }
        if (
            set(self.static) != set(expected)
            or any(
                self.static[key].shape != shape or self.static[key].dtype != dtype
                for key, (shape, dtype) in expected.items()
            )
            or not all(np.isfinite(value).all() for value in self.static.values())
            or not np.isin(self.static["image_mask"], (0, 1)).all()
            or self.static["image_mask"].sum() != rows * len(config["camera_names"])
        ):
            raise ValueError(
                "QNN GR00T static_inputs must hold the export's token_ids, masks, "
                f"attention bias and RoPE, with {rows} image tokens per camera"
            )
        self.image_rows = self.static["image_mask"][0].astype(bool)
        # The stock N1.7 resize/center-crop, so preprocessing cannot drift from
        # the checkpoint's own processor.
        from lerobot.policies.groot.processor_groot import (
            _transform_n1_7_image_for_vlm_torch as transform,
        )

        self.transform = transform
        self.open_runtime()

    def check_sampling(self, value, digest):
        return validate_groot_sampling(value, self.config, digest)

    def graph_ports(self):
        target = self.config["image_transform"]["image_target_size"]
        return groot_ports(
            self.static["token_ids"].shape[1],
            target[0] * target[1] // PATCH**2,
            self.config["chunk_size"],
            self.config["max_action_dim"],
            self.config["max_state_dim"],
            split=self.order == GROOT_SPLIT_ORDER,
        )

    def patches(self, image: torch.Tensor) -> np.ndarray:
        """CHW [0, 1] camera image -> the Qwen3-VL patch rows the vision graph takes."""
        pixels = (image.clamp(0, 1) * 255.0).to(torch.uint8)
        pixels = self.transform(
            pixels,
            **{
                "image_crop_size": None,
                "shortest_image_edge": None,
                "crop_fraction": None,
                **self.config["image_transform"],
            },
        )
        height, width = self.config["image_transform"]["image_target_size"]
        if tuple(pixels.shape) != (3, height, width):
            raise ValueError(
                f"/infer camera image must be RGB, got {tuple(pixels.shape)}"
            )
        # Qwen's rescale and mean/std 0.5 normalization, fused as its fast processor does.
        pixels = ((pixels.float() - 127.5) / 127.5)[None].repeat(
            TEMPORAL_PATCH, 1, 1, 1
        )
        grid_h, grid_w = height // PATCH, width // PATCH
        pixels = pixels.reshape(
            TEMPORAL_PATCH,
            3,
            grid_h // MERGE,
            MERGE,
            PATCH,
            grid_w // MERGE,
            MERGE,
            PATCH,
        )
        pixels = pixels.permute(2, 5, 3, 6, 1, 0, 4, 7)
        return np.ascontiguousarray(
            pixels.reshape(grid_h * grid_w, -1).numpy(), dtype=np.float32
        )

    def rtc_prefix(self, previous, state):
        """Encode carryover as the model's normalized actions, as the PyTorch backend does.

        With relative columns the carryover is absolute and is re-anchored to this
        call's state with each step's stats. Trailing all-zero rows are dropped,
        as LeRobot's N1.7 RTC input preparation does.
        """
        if self.relative.any():
            rows = len(previous)
            offset = np.where(self.relative, state, np.float32(0))
            previous = min_max(
                previous - offset, self.action_low[:rows], self.action_high[:rows]
            )
        valid = np.flatnonzero(np.abs(previous).sum(axis=1) > 0)
        return previous[: valid[-1] + 1] if valid.size else None

    def infer(self, images, state, task, previous, delay, horizon):
        """Chain the native graphs; return physical actions and the RTC carryover."""
        config, static = self.config, self.static
        state_array = self.check_infer(task, state, previous)
        if task != config["prompt"]:
            raise ValueError(
                f"/infer task must be {config['prompt']!r}: this compiled GR00T bundle "
                "encodes that prompt's tokens; set the Objective's prompt to it"
            )
        normalized_state = np.zeros((1, 1, config["max_state_dim"]), np.float32)
        normalized_state[0, 0, : len(state_array)] = min_max(
            state_array, self.state_low, self.state_high
        )
        patches = [self.patches(images[name]) for name in config["camera_names"]]
        vision = [self.sessions["vision"].run({"patches": p}) for p in patches]
        tokens = self.sessions["tokens"].run({"token_ids": static["token_ids"]})
        hidden = tokens["token_embeds"].copy()
        hidden[0, self.image_rows] = np.concatenate([v["image_embed"] for v in vision])
        common = {
            key: static[key] for key in ("attention_bias", "rope_cos", "rope_sin")
        }
        first = {"hidden": hidden, **common}
        for i in range(3):
            first[f"deep{i}"] = np.zeros_like(hidden)
            first[f"deep{i}"][0, self.image_rows] = np.concatenate(
                [v[f"deep{i}"] for v in vision]
            )
        middle = self.sessions["backbone_0_8"].run(first)["hidden_out"]
        backbone = self.sessions["backbone_8_16"].run({"hidden": middle, **common})
        features = self.sessions["observation_features"].run(
            {
                "backbone_features": backbone["backbone_features"],
                "normalized_state": normalized_state,
            }
        )
        shape = (1, self.sampling["chunk_size"], self.sampling["max_action_dim"])
        actions = self.rng.standard_normal(shape, dtype=np.float32)
        strength = np.ones_like(actions)
        prefix = (
            None
            if previous is None
            else self.rtc_prefix(previous[:horizon], state_array)
        )
        if prefix is not None:
            # GR00T N1.7's native RTC: freeze the delayed rows, ramp in the rest.
            overlap, frozen = len(prefix), min(delay, len(prefix))
            actions[0, :overlap, : prefix.shape[1]] = prefix
            actions[0, :overlap, prefix.shape[1] :] = 0
            ramp_rate = self.ramp_rate or self.sampling["rtc_ramp_rate"]
            t = torch.linspace(0.0, 1.0, overlap - frozen + 2)
            ramp = 1 - torch.exp(-ramp_rate * t)
            ramp = (ramp / ramp[-1].clamp_min(1e-8)).numpy()
            strength[0, :frozen] = 0
            strength[0, frozen:overlap] = ramp[1:-1, None]
        dt = np.float32(self.sampling["integration_dt"])
        step = {
            "vl_features": features["vl_features"],
            "state_features": features["state_features"],
            "image_mask": static["image_mask"],
            "valid_mask": static["valid_mask"],
        }
        for timestep in self.sampling["timesteps"]:
            feed = {
                **step,
                "actions": actions,
                "timestep": np.asarray([timestep], dtype=np.int32),
            }
            velocity = self.action_step(feed)
            actions = actions + dt * velocity * strength
        if not np.isfinite(actions).all():
            raise RuntimeError("QNN GR00T flow integration returned nonfinite actions")
        raw = actions[0, :, : config["model_action_dim"]]
        span = self.action_high - self.action_low
        physical = (np.clip(raw, -1, 1) + 1) * np.float32(0.5) * span + self.action_low
        physical = physical + np.where(self.relative, state_array, np.float32(0))
        if not np.isfinite(physical).all():
            raise RuntimeError("QNN GR00T action decoding returned nonfinite actions")
        return physical, physical if self.relative.any() else raw

    def action_step(self, feed):
        """One flow-step velocity from the whole or the two-piece action step."""
        if self.order == GROOT_ORDER:
            return self.sessions["action_step"].run(feed)["velocity"]
        first, second = (self.sessions[name] for name in GROOT_SPLIT_STEPS)
        hidden = first.run(feed)["action_hidden"]
        rest = {
            key: feed[key]
            for key in ("timestep", "vl_features", "image_mask", "valid_mask")
        }
        return second.run({**rest, "action_hidden": hidden})["velocity"]


def check_relative_pairing(relative, state_names: list, action_names: list) -> None:
    """Refuse joint names that would anchor a relative action to another joint.

    QnnGrootPolicy adds and subtracts the state by column position, so each
    relative action column must name the same joint as the state column at
    that position. Without names, values already pair by list order.
    """
    if not state_names or not any(relative):
        return
    if len(state_names) != len(relative) or len(action_names) != len(relative):
        return  # load_policy reports the width mismatch itself.
    mismatched = [
        f"{action_names[i]} (state has {state_names[i]})"
        for i, flag in enumerate(relative)
        if flag and action_names[i] != state_names[i]
    ]
    if mismatched:
        raise ValueError(
            "QNN GR00T relative action columns must name the same joint as the "
            "state column at the same position; list action_names in "
            f"state_names order in vla_serving.yaml. Mismatched: {', '.join(mismatched)}"
        )


def bundle_family(bundle: str) -> str:
    """Read the compiled bundle's policy family without opening anything native."""
    config = json.loads(Path(bundle).expanduser().read_text())
    return (
        "groot"
        if isinstance(config, dict) and config.get("graph_abi") in GROOT_ORDERS
        else "pi05"
    )
