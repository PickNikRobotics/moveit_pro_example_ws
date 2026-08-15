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

"""Benchmark a live VLA inference server and emit JSON plus static HTML.

The synthetic mode measures the complete HTTP, image decode/preprocess, policy,
and postprocess path with deterministic 224x224 JPEG inputs. It verifies the
response contract and finite actions on every sample. It does not measure task
success; use a recorded or simulation-derived payload for representative model
quality experiments.
"""

import argparse
import base64
import hashlib
import html
import json
import math
import os
import statistics
import time
import urllib.error
import urllib.request
from datetime import datetime, timezone
from pathlib import Path
from urllib.parse import urlsplit, urlunsplit


def nearest_rank_percentile(values: list[float], percentile: float) -> float:
    """Return a nearest-rank percentile for a non-empty sample."""
    if not values:
        raise ValueError("cannot compute a percentile of an empty sample")
    if not 0.0 < percentile <= 100.0:
        raise ValueError("percentile must be in (0, 100]")
    ordered = sorted(values)
    rank = max(1, math.ceil(percentile / 100.0 * len(ordered)))
    return ordered[rank - 1]


def validate_response(body: dict) -> tuple[int, int, float]:
    """Validate the serving contract and return (steps, width, dt)."""
    chunk = body.get("action_chunk")
    if not isinstance(chunk, list) or not chunk:
        raise ValueError("response action_chunk must be a non-empty list")
    if not all(isinstance(row, list) and row for row in chunk):
        raise ValueError("response action_chunk rows must be non-empty lists")
    width = len(chunk[0])
    if any(len(row) != width for row in chunk):
        raise ValueError("response action_chunk must be rectangular")
    if not all(
        isinstance(value, (int, float)) and math.isfinite(value)
        for row in chunk
        for value in row
    ):
        raise ValueError("response action_chunk must contain finite numbers")
    dt = body.get("dt")
    if not isinstance(dt, (int, float)) or not math.isfinite(dt) or dt <= 0.0:
        raise ValueError("response dt must be a positive finite number")
    return len(chunk), width, float(dt)


def synthetic_payload(camera_names: list[str], state_dim: int, task: str) -> dict:
    """Create deterministic, full-resolution inputs for a throughput benchmark."""
    if not camera_names or any(not name for name in camera_names):
        raise ValueError("synthetic mode needs at least one non-empty camera name")
    if state_dim <= 0:
        raise ValueError("synthetic mode needs a positive state dimension")

    # Keep OpenCV and NumPy optional for JSON-payload mode and pure unit tests.
    import cv2
    import numpy as np

    axis = np.arange(224, dtype=np.uint8)
    horizontal = np.broadcast_to(axis, (224, 224))
    vertical = horizontal.T
    images = {}
    for index, name in enumerate(camera_names):
        blue = (horizontal + index * 37).astype(np.uint8)
        green = (vertical + index * 53).astype(np.uint8)
        red = ((horizontal // 2 + vertical // 2) + index * 71).astype(np.uint8)
        ok, encoded = cv2.imencode(".jpg", np.dstack((blue, green, red)))
        if not ok:
            raise RuntimeError(f"failed to encode synthetic camera '{name}'")
        images[name] = base64.b64encode(encoded.tobytes()).decode("ascii")
    return {"state": [0.0] * state_dim, "task": task, "images": images}


def health_url(infer_url: str) -> str:
    """Derive the sibling health endpoint without changing authority."""
    parts = urlsplit(infer_url)
    if parts.scheme not in ("http", "https") or not parts.netloc:
        raise ValueError("inference URL must be an absolute HTTP(S) URL")
    if parts.path != "/infer":
        raise ValueError("inference URL path must be exactly /infer")
    return urlunsplit((parts.scheme, parts.netloc, "/health", "", ""))


def request_json(
    method: str,
    url: str,
    timeout: float,
    token: str = "",
    payload: dict | None = None,
) -> dict:
    """Issue one JSON request and preserve a server error body's detail."""
    data = None if payload is None else json.dumps(payload).encode("utf-8")
    headers = {"Accept": "application/json"}
    if data is not None:
        headers["Content-Type"] = "application/json"
    if token:
        headers["Authorization"] = f"Bearer {token}"
    request = urllib.request.Request(url, data=data, headers=headers, method=method)
    try:
        with urllib.request.urlopen(request, timeout=timeout) as response:
            return json.loads(response.read())
    except urllib.error.HTTPError as exc:
        detail = exc.read().decode("utf-8", errors="replace")
        raise RuntimeError(
            f"{method} {url} returned HTTP {exc.code}: {detail}"
        ) from exc


def action_digest(body: dict) -> str:
    """Hash normalized model output for comparison without bloating the report."""
    value = body.get("action_chunk_raw", body["action_chunk"])
    encoded = json.dumps(value, separators=(",", ":"), allow_nan=False).encode()
    return hashlib.sha256(encoded).hexdigest()


def summarize(samples: list[dict], health: dict) -> dict:
    """Summarize validated samples and relate latency to the server budget."""
    durations = [sample["seconds"] for sample in samples]
    shapes = {(sample["chunk_steps"], sample["action_width"]) for sample in samples}
    summary = {
        "sample_count": len(samples),
        "mean_seconds": statistics.fmean(durations),
        "median_seconds": statistics.median(durations),
        "p95_seconds": nearest_rank_percentile(durations, 95.0),
        "min_seconds": min(durations),
        "max_seconds": max(durations),
        "median_chunks_per_second": 1.0 / statistics.median(durations),
        "response_shapes": [
            {"chunk_steps": steps, "action_width": width}
            for steps, width in sorted(shapes)
        ],
        "unique_action_digests": len({sample["action_sha256"] for sample in samples}),
    }
    budget = health.get("warmup", {}).get("realtime_budget_seconds")
    if isinstance(budget, (int, float)) and math.isfinite(budget):
        summary["realtime_budget_seconds"] = float(budget)
        summary["median_budget_margin_seconds"] = (
            float(budget) - summary["median_seconds"]
        )
        summary["median_within_realtime_budget"] = summary["median_seconds"] <= budget
    return summary


def run_benchmark(
    infer_url: str,
    token: str,
    payload: dict,
    warmups: int,
    samples: int,
    timeout: float,
) -> dict:
    """Run warmups and measured requests against a ready server."""
    if warmups < 0 or samples <= 0:
        raise ValueError("warmups must be non-negative and samples must be positive")
    health = request_json("GET", health_url(infer_url), timeout)
    if health.get("status") != "ready":
        raise RuntimeError(f"inference server is not ready: {health}")

    for _ in range(warmups):
        validate_response(request_json("POST", infer_url, timeout, token, payload))

    measured = []
    for index in range(samples):
        start = time.perf_counter()
        body = request_json("POST", infer_url, timeout, token, payload)
        elapsed = time.perf_counter() - start
        steps, width, dt = validate_response(body)
        measured.append(
            {
                "index": index,
                "seconds": elapsed,
                "chunk_steps": steps,
                "action_width": width,
                "dt": dt,
                "action_sha256": action_digest(body),
            }
        )

    report = {
        "schema_version": 1,
        "created_at": datetime.now(timezone.utc).isoformat(),
        "endpoint": infer_url,
        "health": health,
        "request": {
            "camera_names": list(payload.get("images", {}).keys()),
            "state_dim": len(payload.get("state", [])),
            "task": payload.get("task", ""),
            "json_bytes": len(json.dumps(payload).encode("utf-8")),
        },
        "benchmark": {"warmups": warmups, "samples": samples, "timeout": timeout},
        "samples": measured,
    }
    report["summary"] = summarize(measured, health)
    return report


def render_html(report: dict) -> str:
    """Render a dependency-free report that can be opened from disk."""
    summary = report["summary"]
    health = report["health"]
    request = report["request"]
    rows = "\n".join(
        "<tr>"
        f"<td>{sample['index']}</td>"
        f"<td>{sample['seconds']:.4f}</td>"
        f"<td>{sample['chunk_steps']} × {sample['action_width']}</td>"
        f"<td><code>{sample['action_sha256'][:12]}</code></td>"
        "</tr>"
        for sample in report["samples"]
    )
    margin = summary.get("median_budget_margin_seconds")
    margin_text = "not reported by server" if margin is None else f"{margin:.3f} s"
    raw_json = html.escape(json.dumps(report, indent=2, sort_keys=True))
    return f"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>MoveIt Pro VLA inference benchmark</title>
  <style>
    :root {{ color-scheme: light dark; font-family: system-ui, sans-serif; }}
    body {{ max-width: 960px; margin: 2rem auto; padding: 0 1rem; line-height: 1.45; }}
    .cards {{ display: grid; grid-template-columns: repeat(auto-fit, minmax(180px, 1fr)); gap: 1rem; }}
    .card {{ border: 1px solid #8886; border-radius: .5rem; padding: 1rem; }}
    .value {{ font-size: 1.5rem; font-weight: 650; }}
    table {{ width: 100%; border-collapse: collapse; }}
    th, td {{ border-bottom: 1px solid #8886; padding: .55rem; text-align: left; }}
    pre {{ overflow: auto; padding: 1rem; border-radius: .5rem; background: #8881; }}
    code {{ overflow-wrap: anywhere; }}
  </style>
</head>
<body>
  <h1>MoveIt Pro VLA inference benchmark</h1>
  <p>Created {html.escape(report['created_at'])}. Backend:
    <strong>{html.escape(str(health.get('accelerator', 'unknown')))}</strong>,
    device <code>{html.escape(str(health.get('device', 'unknown')))}</code>,
    Torch <code>{html.escape(str(health.get('torch_version', 'unknown')))}</code>.
  </p>
  <div class="cards">
    <div class="card"><div>Median</div><div class="value">{summary['median_seconds']:.3f} s</div></div>
    <div class="card"><div>P95</div><div class="value">{summary['p95_seconds']:.3f} s</div></div>
    <div class="card"><div>Budget margin</div><div class="value">{margin_text}</div></div>
    <div class="card"><div>Samples</div><div class="value">{summary['sample_count']}</div></div>
  </div>
  <h2>Request</h2>
  <p>{request['state_dim']}-dimensional state; cameras
    <code>{html.escape(', '.join(request['camera_names']))}</code>; task
    <code>{html.escape(str(request['task']))}</code>.</p>
  <h2>Samples</h2>
  <table><thead><tr><th>#</th><th>Seconds</th><th>Action shape</th><th>Action digest</th></tr></thead>
  <tbody>{rows}</tbody></table>
  <h2>Raw JSON</h2>
  <pre>{raw_json}</pre>
</body>
</html>
"""


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--url", default="http://127.0.0.1:8973/infer")
    parser.add_argument("--payload", type=Path)
    parser.add_argument("--camera", action="append", default=[])
    parser.add_argument("--state-dim", type=int, default=0)
    parser.add_argument("--task", default="stack the blue cube on the green cube")
    parser.add_argument("--warmups", type=int, default=1)
    parser.add_argument("--samples", type=int, default=5)
    parser.add_argument("--timeout", type=float, default=120.0)
    parser.add_argument("--json", type=Path, required=True)
    parser.add_argument("--html", type=Path, required=True)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    token = os.environ.get("MOVEIT_FRONTEND_KEY", "").strip()
    if not token:
        raise SystemExit("MOVEIT_FRONTEND_KEY must be set for /infer authentication")
    if args.payload:
        payload = json.loads(args.payload.read_text(encoding="utf-8"))
    else:
        payload = synthetic_payload(args.camera, args.state_dim, args.task)
    report = run_benchmark(
        args.url, token, payload, args.warmups, args.samples, args.timeout
    )
    args.json.parent.mkdir(parents=True, exist_ok=True)
    args.html.parent.mkdir(parents=True, exist_ok=True)
    args.json.write_text(
        json.dumps(report, indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    args.html.write_text(render_html(report), encoding="utf-8")
    print(json.dumps(report["summary"], indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
