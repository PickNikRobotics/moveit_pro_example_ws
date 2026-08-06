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

"""LeRobot inference server for MoveIt Pro's ExecutePolicy.

Serves POST /infer and GET /health over HTTP. Runs in its own container (see
Dockerfile.vla_inference_server and the workspace docker-compose.yaml
`inference_server` service) so torch/lerobot stay out of the MoveIt Pro
images; the in-config adapter node (script/get_action_chunk_adapter.py)
bridges the /get_action_chunk ROS service to this server.

/infer requires the deployment's shared MOVEIT_FRONTEND_KEY as an
`Authorization: Bearer` token, the same key the MoveIt Pro web backend
endpoints use; a blank or unset key parks the server in the error state (fail
closed, matching those endpoints). /health stays token-free for health
probes. For a bare development run, export the documented dev key first
(`MOVEIT_FRONTEND_KEY=moveit-secret-key`).

The socket binds before the checkpoint loads: /health reports
loading|ready|error and /infer answers 503 (loading) or 500 (load failed) with
the same detail until the model is ready, so the adapter can tell the operator
exactly what is wrong from the MoveIt Pro UI. A bad or missing checkpoint
parks the server in the error state, surfacing the problem through the
service instead of exiting into a compose restart loop.

Each knob resolves as: an explicit CLI flag > the per-config model-serving
YAML (vla_serving.yaml, default /vla_config/vla_serving.yaml, mounted from
src/vla_sim/config/) > a built-in default. A missing or empty YAML file is
fine, so a bare `python vla_inference_server.py --checkpoint <dir-or-hf-id>`
still works for development; a malformed file or value parks the server in
the error state.
"""

import argparse
import base64
import hmac
import json
import math
import os
import threading
import time
import traceback
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import cv2
import numpy as np
import torch
import yaml
from huggingface_hub import hf_hub_download
from huggingface_hub.errors import GatedRepoError, RepositoryNotFoundError

from lerobot.configs.types import RTCAttentionSchedule
from lerobot.policies.factory import get_policy_class, make_pre_post_processors
from lerobot.policies.rtc.configuration_rtc import RTCConfig

# pi0.5 checkpoints save a processor pipeline that references
# 'relative_actions_processor', an alias lerobot does not always auto-register;
# without it make_pre_post_processors raises
# "Processor step 'relative_actions_processor' not found".
from lerobot.processor import ProcessorStepRegistry
from lerobot.processor.relative_action_processor import RelativeActionsProcessorStep

try:
    ProcessorStepRegistry.get("relative_actions_processor")
except Exception:
    ProcessorStepRegistry.register("relative_actions_processor")(
        RelativeActionsProcessorStep
    )

TESTED_POLICY_TYPES = ("smolvla", "pi05")

# Generous ceiling over a multi-camera base64 observation; bounds the memory
# one connection can demand before the body is read.
MAX_BODY_BYTES = 32 * 1024 * 1024

# Per-connection socket timeout. Each connection gets a handler thread, so
# without a timeout a client that opens a connection and never completes its
# request parks that thread forever. Generous over the largest loopback body
# read; inference time is not affected (no socket reads happen during it).
REQUEST_SOCKET_TIMEOUT_SECONDS = 30

# The per-config model-serving YAML, mounted read-only from the workspace's
# src/vla_sim/config/. Overridable with --config for a standalone `docker run`.
DEFAULT_CONFIG_PATH = "/vla_config/vla_serving.yaml"


def load_serving_config(path: str) -> dict:
    """Read the per-config model-serving YAML into a dict of knob values.

    A missing or empty file returns {}: built-in defaults still apply, so a
    bare `docker run` needs no config. A malformed file (or one whose top
    level is not a mapping) raises, so a typo in the operator's tuning surface
    fails loudly instead of silently serving with the wrong knobs.
    """
    file = Path(path).expanduser()
    if not file.is_file():
        log(f"no serving config at '{path}'; using built-in defaults")
        return {}
    loaded = yaml.safe_load(file.read_text())
    if loaded is None:
        log(f"serving config '{path}' is empty; using built-in defaults")
        return {}
    if not isinstance(loaded, dict):
        raise ValueError(
            f"serving config '{path}' must be a YAML mapping of knob names to "
            f"values, not a {type(loaded).__name__}"
        )
    return loaded


def resolve_default(yaml_value, builtin):
    """Pick an argparse default: the YAML value wins when present.

    argparse layers an explicit CLI flag on top of this, giving the full
    precedence CLI flag > YAML > built-in. A YAML value of 0 or "" is honored,
    since 0 is a meaningful "auto" sentinel for fps and state_dim.
    """
    return builtin if yaml_value is None else yaml_value


def load_checkpoint_file(checkpoint: str, filename: str) -> dict:
    """Read a JSON file from a local checkpoint directory or an HF repo.

    A local path wins when it exists; anything else must look like an HF repo
    id ("org/name"), fetched through the cache (HF_HUB_OFFLINE and HF_TOKEN
    apply).
    """
    path = Path(checkpoint).expanduser()
    if path.is_dir():
        file = path / filename
        if not file.is_file():
            raise FileNotFoundError(f"'{path}' has no {filename}")
        return json.loads(file.read_text())
    if "/" not in checkpoint:
        raise ValueError(
            f"checkpoint '{checkpoint}' is neither a local directory nor a "
            "Hugging Face repo id"
        )
    return json.loads(Path(hf_hub_download(checkpoint, filename)).read_text())


def resolve_policy_type(checkpoint: str, override: str) -> str:
    """Read the policy family from the checkpoint's config.json unless overridden."""
    if override:
        return override
    policy_type = load_checkpoint_file(checkpoint, "config.json").get("type", "")
    if not policy_type:
        raise ValueError(
            f"the config.json of '{checkpoint}' carries no 'type' field; "
            "set policy_class in vla_serving.yaml (or --policy-class) "
            "explicitly"
        )
    return policy_type


def resolve_fps(checkpoint: str, fps: float) -> float:
    """Resolve the policy's training rate, preferring the explicit value.

    The chunk is played at 1/fps seconds per step; a wrong value scales every
    commanded joint velocity, so an unresolvable rate is a startup error,
    never a silent default.
    """
    if fps > 0.0:
        return fps
    try:
        config = load_checkpoint_file(checkpoint, "train_config.json")
    except (GatedRepoError, RepositoryNotFoundError):
        # The tailored HF-access advice in load_policy beats a generic
        # missing-fps message.
        raise
    except Exception:
        config = {}
    from_config = config.get("dataset", {}).get("fps") or config.get("fps")
    if from_config and float(from_config) > 0:
        return float(from_config)
    raise ValueError(
        f"could not read the training fps from the train_config.json of "
        f"'{checkpoint}'; set fps in vla_serving.yaml (or --fps) to the "
        "rate the policy was trained at"
    )


def resolve_device(requested: str, cuda_available: bool) -> str:
    """Resolve the torch device, failing loudly when an explicit request can't be honored.

    'auto' picks cuda when torch reports a usable GPU and falls back to cpu.
    An explicit cuda request on a host without one is a startup error, never a
    silent cpu fallback, so pacing tuned for a GPU cannot quietly run an order
    of magnitude slower.
    """
    if requested == "auto":
        return "cuda" if cuda_available else "cpu"
    if requested.startswith("cuda") and not cuda_available:
        raise ValueError(
            f"device '{requested}' was requested but this torch build reports "
            "no usable GPU; run the container with the NVIDIA runtime "
            "(GPU serving is automatic on NVIDIA machines under the launcher) "
            "or set device: auto in vla_serving.yaml"
        )
    return requested


def resolve_rtc_horizon(
    inference_delay: int, guidance_horizon: int, default_guidance: int
) -> int:
    """Map the service's soft-guidance width onto lerobot's RTC horizon.

    lerobot's execution_horizon is the end index of the guided region measured
    from the chunk start (get_prefix_weights(start=inference_delay,
    end=execution_horizon)), while the GetActionChunk contract's
    guidance_horizon is that region's width past the frozen prefix, zero
    deferring to the server default. Passing the width through unconverted
    would shrink the frozen prefix whenever the width is smaller than the
    inference delay.
    """
    width = guidance_horizon if guidance_horizon > 0 else default_guidance
    return inference_delay + width


def resolve_rtc_schedule(name: str) -> RTCAttentionSchedule:
    """Map the rtc_schedule knob onto lerobot's enum, naming the valid values on a typo."""
    try:
        return RTCAttentionSchedule[name]
    except KeyError:
        valid = ", ".join(schedule.name for schedule in RTCAttentionSchedule)
        raise ValueError(
            f"rtc_schedule '{name}' is not a known RTC schedule; set "
            f"rtc_schedule in vla_serving.yaml (or pass --rtc-schedule) "
            f"to one of: {valid}"
        ) from None


def decode_image_b64(data: str) -> torch.Tensor:
    """base64 JPEG -> CHW float32 [0,1] RGB tensor."""
    buf = np.frombuffer(base64.b64decode(data), dtype=np.uint8)
    bgr = cv2.imdecode(buf, cv2.IMREAD_COLOR)
    if bgr is None:
        raise ValueError("cv2.imdecode failed on /infer image")
    rgb = np.ascontiguousarray(bgr[:, :, ::-1])
    return torch.from_numpy(rgb).float().permute(2, 0, 1) / 255.0


def native_camera_map(steps: list) -> dict:
    """Map the checkpoint's dataset-native camera names to its model slot names.

    Read from the first preprocessor step whose rename mapping touches image
    keys. The mapping is recognized by its shape (a rename_map dict) rather
    than by a lerobot class, so a lerobot relayout degrades to the model slot
    names instead of crashing the server before the socket binds. Empty when
    no step renames images (the dataset already used the model's slot names).
    """
    prefix = "observation.images."
    image_maps = []
    for step in steps:
        rename_map = getattr(step, "rename_map", None)
        if not isinstance(rename_map, dict):
            continue
        image_renames = {
            src.removeprefix(prefix): dst.removeprefix(prefix)
            for src, dst in rename_map.items()
            if src.startswith(prefix) and dst.startswith(prefix)
        }
        if image_renames:
            image_maps.append(image_renames)
    if len(image_maps) > 1:
        log(
            f"WARNING: {len(image_maps)} preprocessor steps rename cameras; "
            "the request names are derived from the first"
        )
    return image_maps[0] if image_maps else {}


def request_camera_names(slot_keys: list, native_map: dict) -> list:
    """The camera names an /infer request must carry, in checkpoint order.

    The dataset-native name where the checkpoint's rename step defines one, the
    model slot name for a camera it does not rename.
    """
    slot_to_native = {slot: native for native, slot in native_map.items()}
    if len(slot_to_native) < len(native_map):
        log(
            "WARNING: the checkpoint's rename map sends several dataset camera "
            "names to the same model slot; requests must use the last one"
        )
    return [slot_to_native.get(slot, slot) for slot in slot_keys]


class PolicyRunner:
    """Owns the loaded policy and serializes inference calls.

    Loading passes policy_cfg by keyword and overrides the device on both
    processors, which merged pi0.5 checkpoints need: their config declares a
    padded 32-dim state while the saved normalizer stats carry the trained
    width.
    """

    def __init__(
        self,
        checkpoint: str,
        policy_type: str,
        device: str,
        guidance_horizon: int,
        rtc_schedule: str,
        state_dim: int,
    ):
        self.device = device
        self.state_dim = state_dim
        self.guidance_horizon = guidance_horizon
        self.lock = threading.Lock()

        # Resolve before the slow checkpoint load so a schedule typo fails fast.
        schedule = resolve_rtc_schedule(rtc_schedule)

        self.policy = get_policy_class(policy_type).from_pretrained(checkpoint)
        self.policy.to(device)
        self.policy.eval()

        # infer() passes the horizon per call on every RTC request, so the
        # config's own execution_horizon never applies; only the enable and
        # schedule matter here.
        self.policy.config.rtc_config = RTCConfig(
            enabled=True,
            prefix_attention_schedule=schedule,
        )
        self.policy.init_rtc_processor()

        self.pre, self.post = make_pre_post_processors(
            policy_cfg=self.policy.config,
            pretrained_path=checkpoint,
            preprocessor_overrides={"device_processor": {"device": device}},
            postprocessor_overrides={"device_processor": {"device": device}},
        )

        # Derived once here, on the loader thread before the runner is
        # published, so the load log and every /infer validation report the
        # same set.
        self.request_names = request_camera_names(
            self.expected_camera_keys(), native_camera_map(self.pre.steps)
        )

    def expected_state_dim(self) -> int:
        # The state width config.json declares is not reliable: a checkpoint
        # can declare its pretraining base's width or its architecture's padded
        # maximum while the saved normalizer stats carry the width it was
        # actually trained on. state_dim is the caller asserting that trained
        # width; unset, the declared value is used and a mismatched checkpoint
        # fails at warmup.
        if self.state_dim > 0:
            return self.state_dim
        return int(self.policy.config.input_features["observation.state"].shape[0])

    def expected_camera_keys(self) -> list:
        """The camera keys the checkpoint was trained on, without the feature prefix."""
        return [
            key.removeprefix("observation.images.")
            for key in self.policy.config.input_features
            if "image" in key
        ]

    @torch.no_grad()
    def infer(
        self,
        images: dict,
        state: list,
        prompt: str,
        prev_chunk: np.ndarray | None,
        inference_delay: int,
        guidance_horizon: int,
    ) -> tuple[np.ndarray, np.ndarray]:
        """Run one inference. Returns (absolute_actions, normalized_actions), both (T, A)."""
        obs = {
            f"observation.images.{key}": tensor.to(self.device)
            for key, tensor in images.items()
        }
        obs["observation.state"] = torch.tensor(
            state, dtype=torch.float32, device=self.device
        )
        obs["task"] = prompt

        kwargs = {}
        if prev_chunk is not None and prev_chunk.size > 0:
            kwargs["prev_chunk_left_over"] = torch.tensor(
                prev_chunk, dtype=torch.float32, device=self.device
            )
            kwargs["inference_delay"] = inference_delay
            kwargs["execution_horizon"] = resolve_rtc_horizon(
                inference_delay, guidance_horizon, self.guidance_horizon
            )

        with self.lock:
            self.policy.reset()
            chunk = self.policy.predict_action_chunk(
                self.pre(obs), **kwargs
            )  # (1, T, A) normalized
            normalized = chunk.squeeze(0).detach().cpu().numpy()
            steps = [self.post(chunk[:, i, :]) for i in range(chunk.shape[1])]
            absolute = torch.stack(steps, dim=1).squeeze(0).detach().cpu().numpy()
        return absolute, normalized

    def warmup(self) -> tuple[float, float, int]:
        """Full-size warmup so the first real chunk does not pay model compile/cache costs.

        Returns (cold_s, steady_s, chunk_steps): the first pass carries the
        one-time compile/cache costs, the second approximates the per-request
        latency that execution pacing must absorb.
        """
        images = {key: torch.zeros(3, 224, 224) for key in self.expected_camera_keys()}
        state = [0.0] * self.expected_state_dim()
        start = time.perf_counter()
        chunk, _ = self.infer(images, state, "warmup", None, 0, 0)
        cold_s = time.perf_counter() - start
        start = time.perf_counter()
        self.infer(images, state, "warmup", None, 0, 0)
        steady_s = time.perf_counter() - start
        return cold_s, steady_s, chunk.shape[0]


class ServerState:
    """Load status shared between the loader thread and the HTTP handlers."""

    def __init__(self):
        self.status = "loading"
        self.detail = ""
        self.runner: PolicyRunner | None = None
        # Resolved by the loader thread; meaningful once status is "ready".
        self.fps = 0.0
        # Shared secret /infer requests must present; set from
        # MOVEIT_FRONTEND_KEY in main() before serve_forever() accepts any
        # request.
        self.frontend_key = ""


def log(message: str) -> None:
    print(f"[vla_inference_server] {message}", flush=True)


def hub_access_error_message(checkpoint: str, gated: bool, token_present: bool) -> str:
    """Actionable message for a Hugging Face download rejected for access reasons.

    The rejection can come from the checkpoint itself or from a gated
    dependency it resolves (the pi0.5 processor pulls the gated
    google/paligemma tokenizer). Whether HF_TOKEN is present decides the
    advice; the token value is never echoed.
    """
    if gated and token_present:
        return (
            f"a Hugging Face repo needed by checkpoint '{checkpoint}' is gated "
            "and the HF_TOKEN account has not been granted access: accept the "
            "model's license on huggingface.co with that account, then restart"
        )
    if gated:
        return (
            f"a Hugging Face repo needed by checkpoint '{checkpoint}' is gated "
            "and HF_TOKEN is not set in the environment: export HF_TOKEN with "
            "a token from an account that accepted the model's license, then "
            "restart"
        )
    if token_present:
        return (
            f"checkpoint '{checkpoint}' was not found on Hugging Face with the "
            "provided HF_TOKEN: check the checkpoint name in vla_serving.yaml "
            "and that the token's account can access the repo"
        )
    return (
        f"checkpoint '{checkpoint}' was not found on Hugging Face: check the "
        "checkpoint name in vla_serving.yaml; a private repo also needs "
        "HF_TOKEN exported in the environment"
    )


def load_policy(state: ServerState, args: argparse.Namespace) -> None:
    """Load + warm the policy in the background; on failure park in the error state."""
    try:
        # A malformed serving config was deferred out of parse_args so the
        # socket could bind first; surface it here like any other load failure.
        if args.config_error:
            raise ValueError(f"could not read the serving config: {args.config_error}")
        # A missing checkpoint parks in the error state like any other config
        # problem: no restart loop can supply the argument, and /health plus
        # the objective's UI messages then name the fix.
        if not args.checkpoint:
            raise ValueError(
                "set the checkpoint in vla_serving.yaml (or --checkpoint) to "
                "a local LeRobot checkpoint directory or an HF repo id"
            )
        # Resolving the fps can read the checkpoint's train_config.json (a
        # download for hub checkpoints), so it happens here rather than before
        # the socket binds, and an unresolvable rate parks in the error state
        # instead of exiting into a compose restart loop.
        state.fps = resolve_fps(args.checkpoint, args.fps)
        policy_type = resolve_policy_type(args.checkpoint, args.policy_class)
        device = resolve_device(args.device, torch.cuda.is_available())
        if policy_type not in TESTED_POLICY_TYPES:
            log(
                f"WARNING: policy family '{policy_type}' is untested with this "
                f"server (tested: {', '.join(TESTED_POLICY_TYPES)}); loading best-effort"
            )
        log(
            f"loading {policy_type} checkpoint '{args.checkpoint}' on '{device}' "
            f"(torch {torch.__version__}) ..."
        )
        runner = PolicyRunner(
            args.checkpoint,
            policy_type,
            device,
            args.guidance_horizon,
            args.rtc_schedule,
            args.state_dim,
        )

        image_features = [
            k for k in runner.policy.config.input_features if "image" in k
        ]
        log(
            f"checkpoint expects {len(image_features)} camera(s) {image_features} "
            f"(request names: {runner.request_names}) and a "
            f"{runner.expected_state_dim()}-dim state; chunk plays at "
            f"dt={1.0 / state.fps:.4f}s"
        )

        # Warmup pre-pays model compile/cache costs; a failure here is logged,
        # not fatal, because a padded-state pi0.5 config can reject the blank
        # observation while real requests, which carry the true shape, still
        # work (set state_dim in vla_serving.yaml to warm up cleanly).
        try:
            cold_s, steady_s, chunk_steps = runner.warmup()
            # Real-time chunking stays feasible only while one inference fits
            # into half a chunk of playback time: each seam must commit at
            # least latency/dt steps yet leave at least as many uncommitted,
            # capping tolerable latency at (chunk/2)*dt.
            budget_s = (chunk_steps / 2.0) / state.fps
            if steady_s > budget_s:
                log(
                    f"WARNING: inference takes {steady_s:.2f}s per "
                    f"{chunk_steps}-step chunk on '{device}', over the "
                    f"{budget_s:.2f}s real-time budget at {state.fps:g} fps; "
                    "execution will starve at chunk seams. Serve on a faster "
                    "device (the launcher uses the GPU automatically on NVIDIA "
                    "machines) or use a policy this machine can serve in time. The "
                    "objective's committed_action_steps x dt sets the "
                    "tighter per-run budget."
                )
            else:
                log(
                    f"warmup done: {steady_s:.2f}s per {chunk_steps}-step chunk "
                    f"(cold start {cold_s:.2f}s), within the {budget_s:.2f}s "
                    f"real-time budget at {state.fps:g} fps"
                )
        except Exception as exc:
            log(
                f"WARNING: warmup inference failed ({type(exc).__name__}: {exc}); "
                "continuing, the first request pays the cold cost"
            )

        state.runner = runner
        state.status = "ready"
        log("ready")
    except (GatedRepoError, RepositoryNotFoundError) as exc:
        traceback.print_exc()
        state.detail = hub_access_error_message(
            args.checkpoint,
            isinstance(exc, GatedRepoError),
            bool(os.environ.get("HF_TOKEN")),
        )
        state.status = "error"
        log(f"FATAL: model load failed: {state.detail}")
    except Exception as exc:
        traceback.print_exc()
        state.detail = f"{type(exc).__name__}: {exc}"
        state.status = "error"
        log(f"FATAL: model load failed: {state.detail}")


def run_inference(state: ServerState, payload: dict) -> dict:
    """Validate one /infer payload and run it through the loaded policy.

    new_episode in the payload is informational only: PolicyRunner resets per
    call, and the episode boundary is carried by an empty prev_chunk.
    """
    for key in ("state", "images", "task"):
        if key not in payload:
            raise ValueError(f"/infer payload is missing '{key}'")
    if not isinstance(payload["images"], dict) or not all(
        isinstance(v, str) for v in payload["images"].values()
    ):
        raise ValueError(
            "/infer payload 'images' must map camera names to base64-encoded JPEG strings"
        )
    if not isinstance(payload["state"], list) or not all(
        isinstance(v, (int, float)) and math.isfinite(v) for v in payload["state"]
    ):
        raise ValueError(
            "/infer payload 'state' must be a list of finite joint positions"
        )
    runner = state.runner
    expected_state = runner.expected_state_dim()
    robot_state = payload["state"]
    if len(robot_state) != expected_state:
        raise ValueError(
            f"request carries a {len(robot_state)}-dim state but the "
            f"checkpoint expects {expected_state} dims"
        )

    # Request image names are the checkpoint's own camera names: the dataset
    # names baked into its preprocessor rename step, or, for a camera the
    # checkpoint does not rename, its config.json slot name. Exactly that set
    # is required: lerobot zero-fills an expected camera the observation
    # lacks, and its rename step lets an unexpected name silently overwrite a
    # renamed camera's slot; either way the policy would run on wrong images,
    # so refuse before decoding anything.
    expected_names = runner.request_names
    missing = [name for name in expected_names if name not in payload["images"]]
    unexpected = sorted(payload["images"].keys() - set(expected_names))
    if missing or unexpected:
        problems = []
        if missing:
            problems.append(f"is missing camera(s) {missing}")
        if unexpected:
            problems.append(f"carries unexpected camera(s) {unexpected}")
        raise ValueError(
            f"the request {' and '.join(problems)} but this checkpoint takes "
            f"exactly {expected_names}; set each of the objective's "
            "image_names to the checkpoint's name for the camera on the "
            "matching image_topics entry"
        )
    images = {name: decode_image_b64(data) for name, data in payload["images"].items()}

    prev_chunk = None
    prev = payload.get("prev_chunk_left_over")
    if prev:
        try:
            prev_chunk = np.asarray(prev, dtype=float)
        except (TypeError, ValueError) as exc:
            raise ValueError(
                f"/infer payload 'prev_chunk_left_over' is not a numeric "
                f"array ({exc})"
            ) from exc
        if prev_chunk.ndim != 2 or not np.isfinite(prev_chunk).all():
            raise ValueError(
                "/infer payload 'prev_chunk_left_over' must be a 2-D array of "
                "finite numbers"
            )
    inference_delay = int(payload.get("inference_delay", 0))
    guidance_horizon = int(payload.get("guidance_horizon", 0))
    if inference_delay < 0 or guidance_horizon < 0:
        raise ValueError(
            "/infer payload 'inference_delay' and 'guidance_horizon' must be "
            "non-negative"
        )

    absolute, normalized = runner.infer(
        images,
        robot_state,
        payload["task"],
        prev_chunk,
        inference_delay,
        guidance_horizon,
    )
    return {
        "action_chunk": absolute.tolist(),
        "action_chunk_raw": normalized.tolist(),
        "dt": 1.0 / state.fps,
    }


def make_handler(state: ServerState):
    class Handler(BaseHTTPRequestHandler):
        # Applied to the connection socket by the base class, so a stalled
        # request read raises and frees the thread instead of parking it.
        timeout = REQUEST_SOCKET_TIMEOUT_SECONDS

        def _send(self, code: int, obj: dict) -> None:
            body = json.dumps(obj).encode()
            try:
                self.send_response(code)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
            except (BrokenPipeError, ConnectionResetError):
                # The client gave up waiting (its timeout is shorter than this
                # inference took); one line beats a stack trace per abort.
                log(f"client disconnected before the {code} response was sent")

        def do_GET(self):
            if self.path != "/health":
                self._send(404, {"error": "not found"})
                return
            health = {"status": state.status}
            if state.status == "error":
                health["detail"] = state.detail
            elif state.status == "ready":
                health["device"] = state.runner.device
            self._send(200, health)

        def _authorized(self) -> bool:
            # Same contract as the MoveIt Pro REST auth middleware: the shared
            # key as an `Authorization: Bearer` token, compared in constant
            # time. /health never reaches this check.
            header = self.headers.get("Authorization", "")
            scheme, _, token = header.partition(" ")
            if scheme.lower() != "bearer" or not token.strip():
                return False
            # Compare bytes: compare_digest raises TypeError on non-ASCII str,
            # and header values arrive latin-1-decoded, so a crafted header
            # would otherwise drop the connection instead of getting a 401.
            return hmac.compare_digest(
                token.strip().encode(), state.frontend_key.encode()
            )

        def do_POST(self):
            if self.path != "/infer":
                self._send(404, {"error": "not found"})
                return
            if not self._authorized():
                self._send(
                    401,
                    {
                        "error": "/infer requires the deployment's "
                        "MOVEIT_FRONTEND_KEY as an 'Authorization: Bearer' "
                        "token"
                    },
                )
                return
            if state.status == "loading":
                self._send(
                    503,
                    {
                        "error": "the inference server is still loading "
                        "the model; try again shortly"
                    },
                )
                return
            if state.status == "error":
                self._send(500, {"error": f"model load failed: {state.detail}"})
                return
            try:
                length = int(self.headers.get("Content-Length", 0))
                if length < 0:
                    # A negative length would make rfile.read() unbounded.
                    self._send(400, {"error": "invalid negative Content-Length"})
                    return
                if length > MAX_BODY_BYTES:
                    self._send(
                        413,
                        {
                            "error": f"request body of {length} bytes "
                            f"exceeds the {MAX_BODY_BYTES}-byte "
                            "limit"
                        },
                    )
                    return
                payload = json.loads(self.rfile.read(length))
            except ValueError as exc:
                self._send(400, {"error": f"bad request: {exc}"})
                return
            try:
                self._send(200, run_inference(state, payload))
            except ValueError as exc:
                # Request-shape problems (missing camera, state-width mismatch,
                # undecodable image) are the caller's error, not a server fault.
                self._send(400, {"error": f"{type(exc).__name__}: {exc}"})
            except Exception as exc:
                traceback.print_exc()
                self._send(500, {"error": f"{type(exc).__name__}: {exc}"})

        def log_message(self, *args):
            pass  # quiet; call counting is done adapter-side

    return Handler


def parse_args() -> argparse.Namespace:
    """CLI options resolving CLI flag > vla_serving.yaml > built-in default."""
    # Resolve the config path first so the YAML can seed the other defaults; a
    # bootstrap parser reads only that flag.
    bootstrap = argparse.ArgumentParser(add_help=False)
    bootstrap.add_argument("--config", default=DEFAULT_CONFIG_PATH)
    config_path = bootstrap.parse_known_args()[0].config

    config: dict = {}
    config_errors: list = []
    try:
        config = load_serving_config(config_path)
    except Exception as exc:
        # A malformed config must park in the error state after the socket
        # binds, not crash before it. Defer: build defaults from built-ins and
        # hand the error to the loader thread via the namespace.
        config_errors.append(f"{type(exc).__name__}: {exc}")

    def numeric_default(name: str, cast, builtin):
        # A non-numeric YAML value (the likeliest typo on the tuning surface)
        # must also park post-bind, not crash into a compose restart loop.
        value = resolve_default(config.get(name), builtin)
        try:
            return cast(value)
        except (TypeError, ValueError):
            config_errors.append(f"{name}: '{value}' is not a number")
            return builtin

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config",
        default=config_path,
        help="per-config model-serving YAML; missing or empty "
        "is fine, malformed parks the error state",
    )
    parser.add_argument(
        "--checkpoint",
        default=str(resolve_default(config.get("checkpoint"), "")),
        help="local LeRobot checkpoint directory or HF repo id",
    )
    parser.add_argument(
        "--policy-class",
        default=str(resolve_default(config.get("policy_class"), "")),
        help="lerobot policy family (pi05 | smolvla | ...); "
        "default: the checkpoint's config.json 'type'",
    )
    parser.add_argument(
        "--fps",
        type=float,
        default=numeric_default("fps", float, 0.0),
        help="training fps; response dt=1/fps; 0 reads the "
        "checkpoint's train_config.json",
    )
    parser.add_argument(
        "--device",
        default=str(resolve_default(config.get("device"), "auto")),
        help="torch device: auto | cpu | cuda",
    )
    parser.add_argument("--port", type=int, default=8973)
    parser.add_argument(
        "--state-dim",
        type=int,
        default=numeric_default("state_dim", int, 0),
        help="trained observation.state width when the "
        "checkpoint's config.json declares a padded one "
        "(0 = trust config.json)",
    )
    parser.add_argument(
        "--guidance-horizon",
        type=int,
        default=numeric_default("guidance_horizon", int, 8),
        help="RTC soft-guidance width in steps past the frozen "
        "prefix, used when a request's guidance_horizon "
        "is zero",
    )
    parser.add_argument(
        "--rtc-schedule",
        default=str(resolve_default(config.get("rtc_schedule"), "EXP")),
        help="RTC guidance-weight schedule: "
        + " | ".join(schedule.name for schedule in RTCAttentionSchedule),
    )
    args = parser.parse_args()
    args.config_error = "; ".join(config_errors)
    return args


def apply_frontend_key(state: ServerState, raw_key: str | None) -> bool:
    """Set the /infer auth key; park in the error state when it is blank.

    Fails closed like the other MOVEIT_FRONTEND_KEY consumers, but parks
    instead of exiting so /health names the fix rather than a compose restart
    loop hiding it.

    @param state: The server state to receive the key or the error.
    @param raw_key: The MOVEIT_FRONTEND_KEY environment value, or None.
    @return: True when the key is usable and the model load may proceed.
    """
    key = (raw_key or "").strip()
    if key:
        state.frontend_key = key
        return True
    # /health serves this text without a token, so it points at the docs
    # rather than naming any key value.
    state.detail = (
        "MOVEIT_FRONTEND_KEY is required: /infer authenticates with the "
        "deployment's shared key. Set it in the environment (see the MoveIt "
        "Pro endpoint authentication guide), then restart"
    )
    state.status = "error"
    return False


def main() -> None:
    # Compose forwards HF_TOKEN as an empty string when the host never set it;
    # drop it so the Hub client sees a genuinely absent token.
    if os.environ.get("HF_TOKEN") == "":
        del os.environ["HF_TOKEN"]
    args = parse_args()
    state = ServerState()
    httpd = ThreadingHTTPServer(("0.0.0.0", args.port), make_handler(state))
    if apply_frontend_key(state, os.environ.get("MOVEIT_FRONTEND_KEY")):
        threading.Thread(target=load_policy, args=(state, args), daemon=True).start()
        log(f"listening on 0.0.0.0:{args.port}; loading model ...")
    else:
        # The model is deliberately not loaded without a key.
        log(f"FATAL: {state.detail}")
    httpd.serve_forever()


if __name__ == "__main__":
    main()
