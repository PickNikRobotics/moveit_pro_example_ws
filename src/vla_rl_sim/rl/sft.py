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
"""Supervised fine-tuning of smolvla_base on states-only scripted demos, rendering camera images on
demand from the stored MuJoCo states. Produces the base policy RL post-trains."""

import argparse
import json
import queue
import threading
import time
from pathlib import Path

import numpy as np
import torch

import stack_env as se
from vla import SmolVLA, stats_from


def main():
    p = argparse.ArgumentParser()
    p.add_argument(
        "--demos",
        type=Path,
        nargs="+",
        required=True,
        help="demos-format .npz files; successful episodes are kept",
    )
    p.add_argument("--num-demos", type=int, default=100)
    p.add_argument(
        "--base", default=str(Path.home() / ".cache/mjbatch_rl/smolvla_base")
    )
    p.add_argument(
        "--resume",
        type=Path,
        help="continue from a checkpoint this pipeline saved, keeping its normalization stats",
    )
    p.add_argument("--steps", type=int, default=2000)
    p.add_argument("--batch", type=int, default=16)
    p.add_argument("--lr", type=float, default=1e-4)
    p.add_argument("--warmup", type=int, default=100)
    p.add_argument("--out", type=Path, required=True)
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--save-every", type=int, default=1000)
    a = p.parse_args()

    parts = []
    for f in a.demos:
        z = np.load(f)
        keep = np.flatnonzero(z["success"])
        parts.append({k: z[k][keep] for k in ("sim_states", "states", "actions")})
        print(f"{f}: {len(keep)} successful of {len(z['success'])}")
    horizon = min(x["states"].shape[1] for x in parts)
    sim, states, actions = (
        np.concatenate([x[k][:, :horizon] for x in parts])[: a.num_demos]
        for k in ("sim_states", "states", "actions")
    )
    n_ep, horizon = states.shape[:2]
    print(f"{n_ep} successful demos x {horizon} steps")

    env = se.StackEnv(1)
    spec = env.spec
    if a.resume:
        vla = SmolVLA(a.resume, spec.image_names, env.image_size)
    else:
        vla = SmolVLA(
            a.base, spec.image_names, env.image_size, stats=stats_from(states, actions)
        )
    chunk = vla.chunk
    params = [q for q in vla.policy.parameters() if q.requires_grad]
    print(
        f"trainable params {sum(q.numel() for q in params) / 1e6:.1f}M of {sum(q.numel() for q in vla.policy.parameters()) / 1e6:.1f}M"
    )
    opt = torch.optim.AdamW(
        params, lr=a.lr, betas=(0.9, 0.95), eps=1e-8, weight_decay=1e-10
    )
    sched = torch.optim.lr_scheduler.LambdaLR(
        opt,
        lambda s: min(1.0, (s + 1) / a.warmup)
        * 0.5
        * (1 + np.cos(np.pi * min(s, a.steps) / a.steps)),
    )

    rng = np.random.default_rng(a.seed)
    if (
        a.resume
    ):  # sample demos and RL rollouts equally, so a small RL set is not swamped
        src_of = np.concatenate(
            [
                np.full(len(np.flatnonzero(np.load(f)["success"])), i)
                for i, f in enumerate(a.demos)
            ]
        )[: len(states)]
    else:
        src_of = np.zeros(len(states), int)
    by_src = [np.flatnonzero(src_of == i) for i in np.unique(src_of)]
    t_idx = np.arange(chunk)

    def make_batch():
        which = rng.integers(len(by_src), size=a.batch)
        ep = np.array([rng.choice(by_src[w]) for w in which])
        t = rng.integers(horizon, size=a.batch)
        idx = t[:, None] + t_idx[None]
        pad = idx >= horizon
        idx = np.minimum(idx, horizon - 1)
        imgs = env.render_states(sim[ep, t])
        return imgs, states[ep, t], actions[ep[:, None], idx], pad

    q: queue.Queue = queue.Queue(maxsize=4)
    stop = threading.Event()

    def producer():
        while not stop.is_set():
            q.put(make_batch())

    workers = [threading.Thread(target=producer, daemon=True) for _ in range(2)]
    for w in workers:
        w.start()

    vla.policy.train()
    log, t0 = [], time.perf_counter()
    for step in range(a.steps):
        imgs, st, act, pad = q.get()
        batch = vla.batch(imgs, st, spec.prompt, act, pad)
        with torch.autocast("cuda", dtype=torch.bfloat16):
            loss, _ = vla.policy.forward(batch)
        opt.zero_grad(set_to_none=True)
        loss.backward()
        torch.nn.utils.clip_grad_norm_(params, 10.0)
        opt.step()
        sched.step()
        log.append(loss.item())
        if (step + 1) % a.save_every == 0 and step + 1 < a.steps:
            vla.save(a.out / f"step_{step + 1}")
            vla.policy.train()
        if (step + 1) % 25 == 0:
            el = time.perf_counter() - t0
            print(
                f"step {step + 1} loss {np.mean(log[-25:]):.4f} lr {sched.get_last_lr()[0]:.2e} {el / (step + 1):.2f}s/step qsize {q.qsize()}",
                flush=True,
            )
    stop.set()
    vla.save(a.out)
    (a.out / "sft_log.json").write_text(
        json.dumps(
            {
                "loss": log,
                "num_demos": int(n_ep),
                "args": {k: str(v) for k, v in vars(a).items()},
            }
        )
    )
    print("saved", a.out)


if __name__ == "__main__":
    main()
