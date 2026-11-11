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

"""Host preprocessing/chaining for compiled pi0.5 QNN context bundles.

Neural execution is exclusively native QnnGraph_execute. Bundles use the
four-graph or six-graph ABI whose expert returns an already-integrated Euler
update. RTC adds only stock LeRobot's residual correction to that update.
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

FOUR_GRAPH_ABI = "aihub.pi05.four_graphs.euler.v1"
FOUR_GRAPH_ORDER = ("vision_encoder", "token_emb", "backbone", "action_expert")
PREFIX_GROUPS = ((0, 6), (6, 12), (12, 18))
PREFIX_GRAPHS = tuple(f"prefix_{start}_{stop}" for start, stop in PREFIX_GROUPS)
SIX_GRAPH_ABI = "kinova.pi05.six_graphs.euler.v1"
SIX_GRAPH_ORDER = ("vision_encoder", "token_emb", *PREFIX_GRAPHS, "action_expert")


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
    if not isinstance(config.get("model_id"), str) or not config["model_id"]:
        raise ValueError("QNN bundle must identify its source checkpoint in model_id")
    if not isinstance(config.get("model_revision"), str) or not re.fullmatch(
        r"[0-9a-f]{40}", config["model_revision"]
    ):
        raise ValueError(
            "QNN model_revision must be the source checkpoint's full immutable Hub SHA"
        )
    for field, expected in (
        ("chunk_size", 50),
        ("max_action_dim", 32),
        ("denoising_steps", 10),
    ):
        if type(config.get(field)) is not int or config[field] != expected:
            raise ValueError(f"QNN {abi} requires {field}={expected}")
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
    names = config.get("camera_names")
    if (
        not isinstance(names, list)
        or len(names) != 3
        or any(not isinstance(name, str) or not name for name in names)
        or len(set(names)) != 3
    ):
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
    placement = config.get("device_placement")
    expected_placement = {
        name: 1 if name in ("vision_encoder", "token_emb") else 0 for name in order
    }
    if placement != expected_placement or any(
        type(device) is not int for device in placement.values()
    ):
        raise ValueError(
            "QNN device_placement must pin vision/token to HTP1 and backbone/expert to HTP0"
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
        or previous.ndim != 2
        or not 0 < len(previous) <= actions.shape[1]
        or not 0 < previous.shape[1] <= actions.shape[2]
        or not np.isfinite(previous).all()
    ):
        raise ValueError(
            "QNN RTC requires matching finite Float32 native updates and validated carryover"
        )
    if (
        not math.isfinite(timestep)
        or not 0 <= timestep <= 1
        or not math.isfinite(dt)
        or dt >= 0
    ):
        raise ValueError("QNN RTC requires the stock decreasing Euler schedule")
    velocity = torch.from_numpy((updated - actions) / dt)
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
        raise RuntimeError(
            "QNN RTC residual produced nonfinite actions; discard this chunk"
        )
    return result


def resize_with_pad(images: torch.Tensor) -> torch.Tensor:
    """LeRobot pi0.5's float NCHW bilinear resize/black pad to 224x224."""
    height, width = images.shape[-2:]
    ratio = max(width / 224, height / 224)
    target_h, target_w = int(height / ratio), int(width / ratio)
    resized = F.interpolate(
        images, size=(target_h, target_w), mode="bilinear", align_corners=False
    ).clamp(0, 1)
    top, extra_h = divmod(224 - target_h, 2)
    left, extra_w = divmod(224 - target_w, 2)
    return F.pad(resized, (left, left + extra_w, top, top + extra_h), value=0)


class NamedGraph:
    """Keep compiler output aliases inside the native-session boundary."""

    def __init__(self, runtime, artifact: Path, device: int):
        manifest = json.loads((artifact / "manifest.json").read_text())
        with (artifact / "model.bin").open("rb") as stream:
            if (
                hashlib.file_digest(stream, "sha256").hexdigest()
                != manifest["context_sha256"]
            ):
                raise ValueError(
                    f"QNN context hash changed for {artifact.name}; restore the compiled bundle"
                )
        self.model = runtime.load(artifact / "model.bin", artifact / "abi.json", device)
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
        """Expose every real native output once under its source ONNX name."""
        values = self.model.run(feed)
        if set(values) != set(self.aliases) or any(
            not np.isfinite(value).all() for value in values.values()
        ):
            raise RuntimeError(
                "QNN graph returned missing/nonfinite outputs; discard this chunk"
            )
        return {self.aliases[name]: value for name, value in values.items()}


def graph_ports(split: bool) -> dict:
    """Fixed source-name/shape/dtype contract for the supported compiled ABIs."""
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
                suffix_sin=(1, 50, 1, 128),
                suffix_cos=(1, 50, 1, 128),
                full_att_4d=(1, 1, 50, 1018),
            ),
        ),
        "action_expert": (
            {
                **floats(
                    x_t=(1, 50, 32),
                    time_step=(1,),
                    full_att_4d=(1, 1, 50, 1018),
                    rope_emb_sin=(1, 50, 1, 128),
                    rope_emb_cos=(1, 50, 1, 128),
                ),
                **{
                    f"{kind}_cache_l{i}": ((1, 968, 1, 256), f32)
                    for kind in ("key", "value")
                    for i in range(18)
                },
            },
            floats(action_emb=(1, 50, 32)),
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


class QnnPolicy:
    """Resident compiled pi0.5 graphs, local trained preprocessing and RTC."""

    def __init__(
        self,
        bundle: str,
        checkpoint: str,
        revision: str,
        state_dim: int,
        fps: float,
        rtc_config,
    ):
        path = Path(bundle).expanduser()
        self.config = config = json.loads(path.read_text())
        self.order = validate_bundle(config)
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
        self.rtc = RTCProcessor(rtc_config)
        self.rng = np.random.default_rng(config["noise_seed"])

        def local_path(value):
            item = Path(value).expanduser()
            return item if item.is_absolute() else path.parent / item

        self.stats = json.loads(local_path(config["stats"]).read_text())
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
        self.tokenizer = AutoTokenizer.from_pretrained(
            str(local_path(config["tokenizer"])), local_files_only=True
        )
        # Optional dependency: selecting PyTorch never imports this module or
        # opens a QAIRT library. Selecting QNN must fail, not silently fall back.
        from vla_qnn_runtime import NativeRuntime

        self.runtime = NativeRuntime()
        self.sessions = {}
        try:
            for name in self.order:
                self.sessions[name] = NamedGraph(
                    self.runtime,
                    local_path(config["graphs"][name]),
                    config["device_placement"][name],
                )
            for name, (inputs, outputs) in graph_ports(
                config["graph_abi"] == SIX_GRAPH_ABI
            ).items():
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
                        f"QNN {name} input/output names, shapes or dtypes differ from the compiled graph ABI"
                    )
        except Exception:
            self.runtime.close()
            raise

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
        actions = self.rng.standard_normal((1, 50, 32), dtype=np.float32)
        dt = np.float32(-1.0 / config["denoising_steps"])
        for step in range(config["denoising_steps"]):
            expert["x_t"] = actions
            expert["time_step"] = np.asarray(
                [1.0 - step / config["denoising_steps"]], dtype=np.float32
            )
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
            actions.shape != (1, 50, 32)
            or actions.dtype != np.float32
            or not np.isfinite(actions).all()
        ):
            raise RuntimeError(
                "QNN expert must return finite Float32 [1,50,32] actions"
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

    def close(self):
        """Release only this backend's resident native contexts."""
        self.runtime.close()
