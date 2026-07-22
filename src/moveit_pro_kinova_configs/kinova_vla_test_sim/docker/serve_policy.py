#!/usr/bin/env python3
"""lerobot RTC inference server for MoveIt Pro ExecutePolicy (host side).

Serves POST /infer and GET /health over HTTP. Generalized over any lerobot RTC
policy via --policy-class (pi05 | smolvla | ...); feeds dataset-native camera
keys and lets each checkpoint's baked rename_map map them onto the model's
slots, so IMG_KEY_MAP is policy-agnostic.

Runs on the host; the in-container bridge node (get_action_chunk_adapter.py)
reaches it at http://172.17.0.1:<port>/infer.

  env -u LD_LIBRARY_PATH <python-with-lerobot> serve_policy.py \
      --checkpoint <path-to-merged-checkpoint> --policy-class pi05
"""
import argparse
import base64
import json
import threading
import traceback
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np
import torch

from lerobot.configs.types import RTCAttentionSchedule
from lerobot.policies.factory import get_policy_class, make_pre_post_processors
from lerobot.policies.rtc.configuration_rtc import RTCConfig

# pi0.5 checkpoints save a processor pipeline that references
# 'relative_actions_processor', an alias lerobot does not always auto-register.
# Register it before loading, else make_pre_post_processors raises
# "Processor step 'relative_actions_processor' not found".
from lerobot.processor import ProcessorStepRegistry
from lerobot.processor.relative_action_processor import RelativeActionsProcessorStep

try:
    ProcessorStepRegistry.get("relative_actions_processor")
except Exception:
    ProcessorStepRegistry.register("relative_actions_processor")(
        RelativeActionsProcessorStep
    )

# Request image key -> the checkpoint's dataset-native camera key. The training
# rename_map is baked into the saved processor, so make_pre_post_processors
# renames these to the model's slots itself; feed native keys and let it rename.
IMG_KEY_MAP = {"front": "scene", "wrist": "wrist", "overview": "overview"}


def decode_image(b64: str) -> torch.Tensor:
    """base64 JPEG -> CHW float32 [0,1] RGB tensor."""
    buf = np.frombuffer(base64.b64decode(b64), dtype=np.uint8)
    bgr = cv2.imdecode(buf, cv2.IMREAD_COLOR)  # HxWx3 BGR uint8
    if bgr is None:
        raise ValueError("cv2.imdecode failed on /infer image")
    rgb = np.ascontiguousarray(bgr[:, :, ::-1])
    return torch.from_numpy(rgb).float().permute(2, 0, 1) / 255.0


class PolicyServer:
    def __init__(
        self,
        checkpoint: str,
        device: str,
        fps: float,
        execution_horizon: int,
        policy_class: str = "pi05",
        state_dim: int = 0,
    ):
        self.device = device
        self.fps = fps
        self.state_dim = state_dim  # >0 overrides config shape for warmup
        self.lock = threading.Lock()  # serialize: one model, stateful RTC processor

        self.policy = get_policy_class(policy_class).from_pretrained(checkpoint)
        self.policy.to(device)
        self.policy.eval()

        # RTC guidance configured once, server-side; applies when the Objective
        # leaves guidance_horizon=0. A non-zero guidance_horizon value arrives per
        # call as execution_horizon.
        self.policy.config.rtc_config = RTCConfig(
            enabled=True,
            execution_horizon=execution_horizon,
            prefix_attention_schedule=RTCAttentionSchedule.EXP,
        )
        self.policy.init_rtc_processor()

        self.pre, self.post = make_pre_post_processors(
            policy_cfg=self.policy.config,
            pretrained_path=checkpoint,
            preprocessor_overrides={"device_processor": {"device": self.device}},
            postprocessor_overrides={"device_processor": {"device": self.device}},
        )

    @torch.no_grad()
    def infer(self, payload: dict) -> dict:
        obs = {
            f"observation.images.{IMG_KEY_MAP[k]}": decode_image(
                payload["images"][k]
            ).to(self.device)
            for k in IMG_KEY_MAP
        }
        obs["observation.state"] = torch.tensor(
            payload["state"], dtype=torch.float32, device=self.device
        )
        obs["task"] = payload["task"]

        kwargs = {}
        prev = payload.get("prev_chunk_left_over")
        if prev:
            kwargs["prev_chunk_left_over"] = torch.tensor(
                prev, dtype=torch.float32, device=self.device
            )
            kwargs["inference_delay"] = int(payload.get("inference_delay", 0))
            if payload.get("execution_horizon") is not None:
                kwargs["execution_horizon"] = int(payload["execution_horizon"])

        self.policy.reset()
        chunk = self.policy.predict_action_chunk(
            self.pre(obs), **kwargs
        )  # (1, T, A) normalized
        raw = chunk.squeeze(0).detach().cpu().numpy()
        steps = [self.post(chunk[:, i, :]) for i in range(chunk.shape[1])]
        actions = (
            torch.stack(steps, dim=1).squeeze(0).detach().cpu().numpy()
        )  # (T, A) absolute
        return {
            "action_chunk": actions.tolist(),
            "action_chunk_raw": raw.tolist(),
            "dt": 1.0 / self.fps,
        }

    def warmup(self):
        """Full-size warmup so the first real chunk does not pay for CUDA graph / cache setup."""
        h = w = 224
        blank = base64.b64encode(
            cv2.imencode(".jpg", np.zeros((h, w, 3), np.uint8))[1]
        ).decode("ascii")
        state_dim = self.state_dim or int(
            self.policy.config.input_features["observation.state"].shape[0]
        )
        self.infer(
            {
                "images": {k: blank for k in IMG_KEY_MAP},
                "state": [0.0] * state_dim,
                "task": "warmup",
            }
        )


def make_handler(server: PolicyServer):
    class Handler(BaseHTTPRequestHandler):
        def _send(self, code, obj):
            body = json.dumps(obj).encode()
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            if self.path == "/health":
                self._send(200, {"status": "ok", "warm": True})
            else:
                self._send(404, {"error": "not found"})

        def do_POST(self):
            if self.path != "/infer":
                self._send(404, {"error": "not found"})
                return
            try:
                n = int(self.headers.get("Content-Length", 0))
                payload = json.loads(self.rfile.read(n))
            except (ValueError, json.JSONDecodeError) as exc:
                self._send(400, {"error": f"bad request: {exc}"})
                return
            try:
                with server.lock:
                    result = server.infer(payload)
                self._send(200, result)
            except (
                Exception
            ) as exc:  # surfaced to the bridge as {"error": ...} -> success=false
                traceback.print_exc()
                self._send(200, {"error": f"{type(exc).__name__}: {exc}"})

        def log_message(self, *args):
            pass  # quiet; call counting is done adapter-side

    return Handler


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--checkpoint", required=True)
    ap.add_argument(
        "--policy-class",
        default="pi05",
        help="lerobot policy class: pi05 | smolvla | ...",
    )
    ap.add_argument(
        "--state-dim",
        type=int,
        default=0,
        help="override observation.state dim for warmup (0=use config)",
    )
    ap.add_argument(
        "--fps", type=float, default=10.0, help="training fps; response dt=1/fps"
    )
    ap.add_argument("--port", type=int, default=8973)
    ap.add_argument("--device", default="cuda")
    ap.add_argument(
        "--execution-horizon", type=int, default=20, help="server RTC default window"
    )
    args = ap.parse_args()

    print(
        f"[rtc-server] loading {args.policy_class} {args.checkpoint} on {args.device} ...",
        flush=True,
    )
    srv = PolicyServer(
        args.checkpoint,
        args.device,
        args.fps,
        args.execution_horizon,
        policy_class=args.policy_class,
        state_dim=args.state_dim,
    )
    print("[rtc-server] warmup inference ...", flush=True)
    try:
        srv.warmup()
    except (
        Exception
    ) as exc:  # warmup only pre-pays CUDA setup; never let it kill the server.
        traceback.print_exc()
        print(
            f"[rtc-server] WARNING: warmup skipped ({type(exc).__name__}: {exc}); "
            f"pass --state-dim for a merged checkpoint to warm up cleanly",
            flush=True,
        )
    httpd = ThreadingHTTPServer(("0.0.0.0", args.port), make_handler(srv))
    print(
        f"[rtc-server] ready on 0.0.0.0:{args.port} (dt={1.0/args.fps:.4f}, ehz={args.execution_horizon})",
        flush=True,
    )
    httpd.serve_forever()


if __name__ == "__main__":
    main()
