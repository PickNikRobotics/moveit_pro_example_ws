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

Serves POST /infer, GET /health, and GET /status over HTTP. Runs in its own
container (see Dockerfile.vla_inference_server and the workspace
docker-compose.yaml `inference_server` service) so torch/lerobot stay out of
the MoveIt Pro images; the in-config adapter node
(script/get_action_chunk_adapter.py) bridges the /get_action_chunk ROS service
to this server.

/infer and /status require MOVEIT_INFERENCE_KEY as an `Authorization: Bearer`
token. `moveit_pro run` derives that inference-only key from the deployment's
frontend key. A blank or unset key parks the server in the error state (fail
closed). /health needs no token, and the remote TLS proxy does not forward
/health.

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
import hashlib
import hmac
import json
import math
import os
import re
import stat
import sys
import threading
import time
import traceback
from collections.abc import Mapping
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import cv2
import numpy as np
import torch
import yaml
from huggingface_hub import hf_hub_download, snapshot_download
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
HUB_COMMIT_PATTERN = re.compile(r"^[0-9a-f]{40}$")
MAX_HF_TOKEN_BYTES = 4096
RELOAD_POLL_SECONDS = 1.0
# A reload replaces the process, which would fail a running policy's next
# /infer call. The reload waits until no /infer has been served for longer than
# the gap between calls within one policy run. ExecutePolicy calls every
# committed_action_steps * dt seconds, and a commit must be smaller than the
# chunk, so one chunk of playback bounds that gap for any objective. This is
# the floor, used until a warmup has measured the loaded checkpoint's chunk.
RELOAD_IDLE_SECONDS = 5.0
# An /infer still in flight after this long no longer holds off a reload: its
# caller gave up (the adapter's http_timeout is 9.0, under ExecutePolicy's
# policy_call_timeout of 10.0 in the stock objective), and a hung inference
# must not block the reload that would replace it.
INFER_ABANDONED_SECONDS = 30.0
INFERENCE_KEY_ENV = "MOVEIT_INFERENCE_KEY"
_HF_TOKEN_FILE_SOURCE_ENV = "MOVEIT_PRO_HF_TOKEN_FROM_FILE"


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


def serving_config_revision(path: str) -> str | None:
    """The sha256 of the serving config file's bytes, or None without a file.

    Trainer hashes the file it writes the same way, so /status can show
    whether this process loaded that file or some other mount.
    """
    file = Path(path).expanduser()
    if not file.is_file():
        return None
    return hashlib.sha256(file.read_bytes()).hexdigest()


def resolve_default(yaml_value, builtin):
    """Pick an argparse default: the YAML value wins when present.

    argparse layers an explicit CLI flag on top of this, giving the full
    precedence CLI flag > YAML > built-in. A YAML value of 0 or "" is honored,
    since 0 is a meaningful "auto" sentinel for fps and state_dim.
    """
    return builtin if yaml_value is None else yaml_value


def load_checkpoint_file(checkpoint: str, filename: str, revision: str = "") -> dict:
    """Read a JSON file from a local checkpoint directory or an HF repo.

    A local path wins when it exists; anything else must look like an HF repo
    id ("org/name"), fetched through the cache (HF_HUB_OFFLINE and HF_TOKEN
    apply).
    """
    path = Path(checkpoint).expanduser()
    if path.is_dir():
        if revision:
            raise ValueError(
                "checkpoint_revision applies only to a Hugging Face repo id, "
                "not a local checkpoint directory"
            )
        file = path / filename
        if not file.is_file():
            raise FileNotFoundError(f"'{path}' has no {filename}")
        return json.loads(file.read_text())
    if "/" not in checkpoint:
        raise ValueError(
            f"checkpoint '{checkpoint}' is neither a local directory nor a "
            "Hugging Face repo id"
        )
    return json.loads(
        Path(
            hf_hub_download(
                repo_id=checkpoint,
                filename=filename,
                revision=revision or None,
            )
        ).read_text()
    )


def resolve_policy_type(checkpoint: str, override: str, revision: str = "") -> str:
    """Read the policy family from the checkpoint's config.json unless overridden."""
    if override:
        return override
    policy_type = load_checkpoint_file(checkpoint, "config.json", revision).get(
        "type", ""
    )
    if not policy_type:
        raise ValueError(
            f"the config.json of '{checkpoint}' carries no 'type' field; "
            "set policy_class in vla_serving.yaml (or --policy-class) "
            "explicitly"
        )
    return policy_type


def resolve_fps(checkpoint: str, fps: float, revision: str = "") -> float:
    """Resolve the policy's training rate, preferring the explicit value.

    The chunk is played at 1/fps seconds per step; a wrong value scales every
    commanded joint velocity, so an unresolvable rate is a startup error,
    never a silent default.
    """
    if fps > 0.0:
        return fps
    try:
        config = load_checkpoint_file(checkpoint, "train_config.json", revision)
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
        checkpoint_revision: str = "",
    ):
        self.device = device
        self.state_dim = state_dim
        self.guidance_horizon = guidance_horizon
        self.lock = threading.Lock()

        # Resolve before the slow checkpoint load so a schedule typo fails fast.
        schedule = resolve_rtc_schedule(rtc_schedule)

        # LeRobot 0.6.0's pi0.5 loader reads the config at `revision` but
        # downloads the weights without it. A pinned checkpoint is therefore
        # resolved to its local snapshot first, and every loader reads that
        # directory, where no other commit's files can resolve.
        if checkpoint_revision:
            checkpoint = snapshot_download(checkpoint, revision=checkpoint_revision)

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
        # The selected checkpoint is available through the authenticated
        # /status endpoint while it loads, not through public /health.
        self.checkpoint = ""
        self.checkpoint_revision = ""
        # The serving config this process loaded, as /status reports it.
        self.config_revision: str | None = None
        self.trainer_handoff_version: int | None = None
        # main() sets this from MOVEIT_INFERENCE_KEY before serve_forever()
        # accepts a request.
        self.bearer_key = ""
        # Inference activity, written by handler threads and read by the
        # reload watcher.
        self.activity_lock = threading.Lock()
        # Start time of each in-flight /infer, keyed by its handler thread.
        self.infer_starts: dict[int, float] = {}
        self.last_infer_monotonic: float | None = None
        # How long /infer must stay quiet before a reload; see RELOAD_IDLE_SECONDS.
        self.reload_idle_seconds = RELOAD_IDLE_SECONDS
        # True while a changed serving input waits for the running policy to
        # stop. Advisory only: the watcher writes it and /status reads it
        # unlocked, and nothing else depends on it.
        self.reload_pending = False
        # Set once the watcher commits to a reload, so no /infer starts in the
        # moment before the process is replaced.
        self.reloading = False

    @property
    def inflight_infers(self) -> int:
        return len(self.infer_starts)

    def begin_infer(self) -> bool:
        """Record an /infer starting; False when a reload has been committed."""
        with self.activity_lock:
            if self.reloading:
                return False
            self.infer_starts[threading.get_ident()] = time.monotonic()
            return True

    def end_infer(self) -> None:
        with self.activity_lock:
            del self.infer_starts[threading.get_ident()]
            self.last_infer_monotonic = time.monotonic()

    def _idle(self, now: float) -> bool:
        # Judged on the oldest /infer: once it outlives INFER_ABANDONED_SECONDS
        # it is hung, and every later request is queued behind it on the
        # runner's lock, so none of them can hold off the reload that clears it.
        if (
            self.infer_starts
            and now - min(self.infer_starts.values()) < INFER_ABANDONED_SECONDS
        ):
            return False
        return (
            self.last_infer_monotonic is None
            or now - self.last_infer_monotonic >= self.reload_idle_seconds
        )

    def try_begin_reload(self, now: float) -> bool:
        """Commit to a reload once no live /infer is in flight or recently finished.

        The idle check and the commit share one lock acquisition, so an /infer
        cannot start between them.
        """
        with self.activity_lock:
            self.reloading = self._idle(now)
            return self.reloading

    def cancel_reload(self) -> None:
        """Accept /infer again after a committed reload failed to start."""
        with self.activity_lock:
            self.reloading = False


def log(message: str) -> None:
    print(f"[vla_inference_server] {message}", flush=True)


def load_hf_token_file(raw_path: str | None) -> str:
    """Load an owner-only token file when HF_TOKEN is not already set.

    Returns an empty string on success or absence. A non-empty result is safe
    to expose through /health and parks model loading in the error state.
    """
    environment_token = os.environ.get("HF_TOKEN", "").strip()
    if environment_token:
        os.environ.pop(_HF_TOKEN_FILE_SOURCE_ENV, None)
        os.environ["HF_TOKEN"] = environment_token
        return ""
    os.environ.pop("HF_TOKEN", None)
    path = (raw_path or "").strip()
    if not path:
        return ""
    try:
        descriptor = os.open(
            path, os.O_RDONLY | getattr(os, "O_NOFOLLOW", 0) | os.O_CLOEXEC
        )
    except FileNotFoundError:
        return ""
    except OSError:
        return "the Hugging Face serving credential could not be opened"
    try:
        with os.fdopen(descriptor, "rb") as handle:
            metadata = os.fstat(handle.fileno())
            if (
                not stat.S_ISREG(metadata.st_mode)
                or metadata.st_uid != os.geteuid()
                or metadata.st_mode & 0o077
            ):
                return "the Hugging Face serving credential is not owner-only"
            encoded = handle.read(MAX_HF_TOKEN_BYTES + 1)
    except OSError:
        return "the Hugging Face serving credential could not be read"
    if len(encoded) > MAX_HF_TOKEN_BYTES:
        return "the Hugging Face serving credential is too long"
    try:
        token = encoded.decode("utf-8").strip()
    except UnicodeError:
        return "the Hugging Face serving credential is not valid UTF-8"
    if not token:
        return ""
    if any(character.isspace() for character in token):
        return "the Hugging Face serving credential contains whitespace"
    os.environ["HF_TOKEN"] = token
    os.environ[_HF_TOKEN_FILE_SOURCE_ENV] = "1"
    return ""


def watched_file_signature(path: str) -> tuple[int, int, int] | None:
    """Return the replacement-sensitive signature of a watched file."""
    try:
        metadata = os.stat(path, follow_symlinks=False)
    except FileNotFoundError:
        return None
    except OSError:
        return (-1, -1, -1)
    return (metadata.st_ino, metadata.st_size, metadata.st_mtime_ns)


def restart_process() -> None:
    """Replace this process so model memory is released before reloading."""
    environment = os.environ.copy()
    if environment.pop(_HF_TOKEN_FILE_SOURCE_ENV, None) == "1":
        # The file is the source of truth. Do not let the old value become an
        # apparent explicit HF_TOKEN override in the replacement process.
        environment.pop("HF_TOKEN", None)
    os.execve(sys.executable, [sys.executable, *sys.argv], environment)


def watch_runtime_inputs(
    paths: list[str],
    *,
    state: ServerState | None = None,
    wait=time.sleep,
    restart=restart_process,
    clock=time.monotonic,
) -> None:
    """Restart after the serving config or credential is atomically replaced.

    With a state, the restart waits until the server is idle (see
    RELOAD_IDLE_SECONDS) so it does not fail a running policy, and /infer is
    refused from the moment the restart is committed.
    """
    baseline = [watched_file_signature(path) for path in paths]
    while True:
        wait(RELOAD_POLL_SECONDS)
        current = [watched_file_signature(path) for path in paths]
        if current != baseline:
            # Trainer writes the token and YAML as one user action. A short
            # debounce coalesces both replacements into one model reload.
            wait(RELOAD_POLL_SECONDS)
            if state is not None and not state.try_begin_reload(clock()):
                state.reload_pending = True
                log(
                    "serving configuration changed; reload deferred until the "
                    "running policy stops"
                )
                while not state.try_begin_reload(clock()):
                    wait(RELOAD_POLL_SECONDS)
            if state is not None and state.inflight_infers:
                log(
                    f"WARNING: reloading over {state.inflight_infers} /infer "
                    f"call(s) hung for more than {INFER_ABANDONED_SECONDS:g}s"
                )
            log("serving configuration changed; reloading the model")
            try:
                restart()
            except OSError as exc:
                # The inputs still differ from the baseline, so the next poll
                # retries; until then keep serving the loaded model.
                if state is not None:
                    state.cancel_reload()
                log(f"ERROR: the reload could not replace the process: {exc}")
                continue
            return


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
    state.checkpoint = args.checkpoint
    state.checkpoint_revision = getattr(args, "checkpoint_revision", "")
    state.config_revision = getattr(args, "config_revision", None)
    state.trainer_handoff_version = getattr(args, "trainer_handoff_version", None)
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
        if args.checkpoint_revision and Path(args.checkpoint).expanduser().is_dir():
            raise ValueError(
                "checkpoint_revision applies only to a Hugging Face repo id, "
                "not a local checkpoint directory"
            )
        if (
            args.checkpoint_requires_hf_token
            and not os.environ.get("HF_TOKEN", "").strip()
        ):
            raise ValueError(
                "this private Trainer checkpoint requires Hugging Face model access; "
                "configure it from the Trainer tab"
            )
        # Resolving the fps can read the checkpoint's train_config.json (a
        # download for hub checkpoints), so it happens here rather than before
        # the socket binds, and an unresolvable rate parks in the error state
        # instead of exiting into a compose restart loop.
        state.fps = resolve_fps(args.checkpoint, args.fps, args.checkpoint_revision)
        policy_type = resolve_policy_type(
            args.checkpoint, args.policy_class, args.checkpoint_revision
        )
        device = resolve_device(args.device, torch.cuda.is_available())
        if policy_type not in TESTED_POLICY_TYPES:
            log(
                f"WARNING: policy family '{policy_type}' is untested with this "
                f"server (tested: {', '.join(TESTED_POLICY_TYPES)}); loading best-effort"
            )
        log(
            f"loading {policy_type} checkpoint '{args.checkpoint}'"
            f"{f' at {args.checkpoint_revision}' if args.checkpoint_revision else ''} "
            f"on '{device}' "
            f"(torch {torch.__version__}) ..."
        )
        runner = PolicyRunner(
            args.checkpoint,
            policy_type,
            device,
            args.guidance_horizon,
            args.rtc_schedule,
            args.state_dim,
            checkpoint_revision=args.checkpoint_revision,
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
            state.reload_idle_seconds = max(
                RELOAD_IDLE_SECONDS, chunk_steps / state.fps
            )
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
                f"/infer payload 'prev_chunk_left_over' is not a numeric array ({exc})"
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
            if self.path == "/health":
                health = {"status": state.status}
                if state.status == "error":
                    health["detail"] = state.detail
                elif state.status == "ready":
                    health["device"] = state.runner.device
                self._send(200, health)
                return
            if self.path != "/status":
                self._send(404, {"error": "not found"})
                return
            if not self._authorized():
                self._send(
                    401,
                    {
                        "error": f"/status requires {INFERENCE_KEY_ENV} as a bearer token"
                    },
                )
                return
            health = {
                "state": state.status,
                "checkpoint": state.checkpoint,
                "checkpointRevision": state.checkpoint_revision,
                "reloadPending": state.reload_pending,
                "configRevision": state.config_revision,
                "trainerHandoffVersion": state.trainer_handoff_version,
            }
            if state.status == "error":
                health["detail"] = state.detail
            elif state.status == "ready":
                health["device"] = state.runner.device
            self._send(200, health)

        def _authorized(self) -> bool:
            # Compare against MOVEIT_INFERENCE_KEY in constant time. /health is
            # unauthenticated.
            header = self.headers.get("Authorization", "")
            scheme, _, token = header.partition(" ")
            if scheme.lower() != "bearer" or not token.strip():
                return False
            # Compare bytes: compare_digest raises TypeError on non-ASCII str,
            # and header values arrive latin-1-decoded, so a crafted header
            # would otherwise drop the connection instead of getting a 401.
            return hmac.compare_digest(
                token.strip().encode(), state.bearer_key.encode()
            )

        def do_POST(self):
            if self.path != "/infer":
                self._send(404, {"error": "not found"})
                return
            if not self._authorized():
                self._send(
                    401,
                    {"error": f"/infer requires {INFERENCE_KEY_ENV} as a bearer token"},
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
            if not state.begin_infer():
                self._send(
                    503,
                    {
                        "error": "the inference server is reloading the "
                        "model; try again shortly"
                    },
                )
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
            finally:
                state.end_infer()

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
    config_revision = serving_config_revision(config_path)
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
    checkpoint_revision = resolve_default(config.get("checkpoint_revision"), "")
    if not isinstance(checkpoint_revision, str):
        config_errors.append("checkpoint_revision must be a string")
        checkpoint_revision = ""
    elif checkpoint_revision and not HUB_COMMIT_PATTERN.fullmatch(checkpoint_revision):
        config_errors.append(
            "checkpoint_revision must be an empty value or a full 40-character "
            "lowercase Hugging Face commit SHA"
        )
        checkpoint_revision = ""

    def checkpoint_revision_arg(value: str) -> str:
        if value and not HUB_COMMIT_PATTERN.fullmatch(value):
            raise argparse.ArgumentTypeError(
                "checkpoint revision must be an empty value or a full "
                "40-character lowercase Hugging Face commit SHA"
            )
        return value

    parser.add_argument(
        "--checkpoint-revision",
        type=checkpoint_revision_arg,
        default=checkpoint_revision,
        help="exact Hugging Face commit SHA for the checkpoint",
    )
    requires_hf_token = resolve_default(
        config.get("checkpoint_requires_hf_token"), False
    )
    if not isinstance(requires_hf_token, bool):
        config_errors.append("checkpoint_requires_hf_token must be true or false")
        requires_hf_token = False
    parser.add_argument(
        "--checkpoint-requires-hf-token",
        action=argparse.BooleanOptionalAction,
        default=requires_hf_token,
        help="fail closed unless HF_TOKEN is present",
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
    # Echoed on /status so Trainer can check the running server, not just the
    # YAML it wrote: which file this process loaded and which handoff contract
    # that file declares.
    args.config_revision = config_revision
    handoff_version = config.get("moveit_pro_trainer_handoff_version")
    if handoff_version is not None and (
        isinstance(handoff_version, bool) or not isinstance(handoff_version, int)
    ):
        config_errors.append("moveit_pro_trainer_handoff_version must be an integer")
        handoff_version = None
    args.trainer_handoff_version = handoff_version
    args.config_error = "; ".join(config_errors)
    return args


def apply_bearer_key(state: ServerState, environment: Mapping[str, str]) -> bool:
    """Set the /infer and /status auth key; park in the error state when blank.

    Fails closed, but parks instead of exiting so /health names the fix rather
    than a compose restart loop hiding it. Only presence is checked.

    @param state: The server state to receive the key or the error.
    @param environment: The process environment, read only for MOVEIT_INFERENCE_KEY.
    @return: True when the key is usable and the model load may proceed.
    """
    key = environment.get(INFERENCE_KEY_ENV, "").strip()
    if key:
        state.bearer_key = key
        return True
    # /health serves this text without a token, so it names the two commands
    # that set the key rather than any key value.
    state.detail = (
        f"{INFERENCE_KEY_ENV} is not set, so /infer and /status refuse every "
        "request. Start the server through `moveit_pro run`, or set the variable "
        "to the output of `moveit_pro inference-key`, then restart it."
    )
    state.status = "error"
    return False


def main() -> None:
    token_file = os.environ.get("HF_TOKEN_FILE", "").strip()
    token_error = load_hf_token_file(token_file)
    args = parse_args()
    if token_error:
        args.config_error = "; ".join(
            part for part in (args.config_error, token_error) if part
        )
    state = ServerState()
    state.checkpoint = args.checkpoint
    state.checkpoint_revision = args.checkpoint_revision
    state.config_revision = args.config_revision
    state.trainer_handoff_version = args.trainer_handoff_version
    httpd = ThreadingHTTPServer(("0.0.0.0", args.port), make_handler(state))
    watched_paths = [args.config]
    if token_file:
        watched_paths.append(token_file)
    threading.Thread(
        target=watch_runtime_inputs,
        args=(watched_paths,),
        kwargs={"state": state},
        daemon=True,
    ).start()
    if apply_bearer_key(state, os.environ):
        threading.Thread(target=load_policy, args=(state, args), daemon=True).start()
        log(f"listening on 0.0.0.0:{args.port}; loading model ...")
    else:
        # The model is deliberately not loaded without a key.
        log(f"FATAL: {state.detail}")
    httpd.serve_forever()


if __name__ == "__main__":
    main()
