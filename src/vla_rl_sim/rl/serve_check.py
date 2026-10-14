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
"""Serve a checkpoint this pipeline wrote with vla_sim's own inference server and request one chunk,
proving the artifact deploys through MoveIt Pro's ExecutePolicy path unchanged.
Usage: serve_check.py <checkpoint_dir> [device]"""

import base64
import json
import os
import subprocess
import sys
import tempfile
import time
import urllib.request
from pathlib import Path

import cv2
import numpy as np

import stack_env as se

ckpt = Path(sys.argv[1]).resolve()
device = sys.argv[2] if len(sys.argv) > 2 else "cpu"
server = se.WS_SRC / "vla_sim" / "docker" / "vla_inference_server.py"
key, port = "serve-check", 18973
cfg = Path(tempfile.mkdtemp()) / "vla_serving.yaml"
cfg.write_text(
    f"checkpoint: {ckpt}\npolicy_class: smolvla\nfps: 10.0\ndevice: {device}\nstate_dim: 8\n"
)
env_vars = {**os.environ, "MOVEIT_INFERENCE_KEY": key, "HF_HUB_OFFLINE": "1"}
proc = subprocess.Popen(
    [sys.executable, str(server), "--config", str(cfg), "--port", str(port)],
    env=env_vars,
    stdout=subprocess.PIPE,
    stderr=subprocess.STDOUT,
    text=True,
)
base = f"http://127.0.0.1:{port}"


def call(path, payload=None):
    req = urllib.request.Request(
        base + path,
        data=None if payload is None else json.dumps(payload).encode(),
        headers={"Authorization": f"Bearer {key}", "Content-Type": "application/json"},
    )
    with urllib.request.urlopen(req, timeout=300) as r:
        return json.loads(r.read())


try:
    for _ in range(600):
        try:
            st = call("/status")
            if st.get("state") in ("ready", "error") or st.get("ready"):
                break
        except Exception:
            pass
        if proc.poll() is not None:
            raise SystemExit("server exited:\n" + proc.stdout.read()[-3000:])
        time.sleep(1)
    print("status:", json.dumps(st)[:400])
    env = se.StackEnv(1)
    env.reset(np.random.default_rng(7))
    imgs, state = env.render()[0], env.joint_state()[0]
    payload = {
        "task": env.spec.prompt,
        "state": state.tolist(),
        "images": {
            n: base64.b64encode(
                cv2.imencode(".png", cv2.cvtColor(im, cv2.COLOR_RGB2BGR))[1]
            ).decode()
            for n, im in zip(env.spec.image_names, imgs, strict=True)
        },
    }
    t = time.perf_counter()
    out = call("/infer", payload)
    chunk = np.asarray(out["action_chunk"])
    print(
        f"/infer {time.perf_counter() - t:.2f}s chunk {chunk.shape} dt {out.get('dt')}"
    )
    print("first action  ", np.round(chunk[0], 3))
    print("current state ", np.round(state, 3))
    print(
        "max |first - state| (arm, rad)", float(np.abs(chunk[0, :7] - state[:7]).max())
    )
finally:
    proc.terminate()
    try:
        proc.wait(10)
    except subprocess.TimeoutExpired:
        proc.kill()
