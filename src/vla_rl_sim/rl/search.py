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
"""Simulator-in-the-loop best-of-K search over the frozen VLA's flow noise.

Each decision: encode once, decode K noise samples, execute each chunk's committed steps in a
clone of the sim, keep the clone with the highest task progress (success first, then the shaping
potential). Measures the headroom noise steering has on this base policy, and produces successful
rollouts in the demos format for distilling back into the VLA weights (expert iteration).
"""
import argparse, json, time
from pathlib import Path
import numpy as np, torch
import stack_env as se
from vla import SmolVLA

p = argparse.ArgumentParser()
p.add_argument("--checkpoint", required=True)
p.add_argument("--k", type=int, default=8)
p.add_argument("--num-envs", type=int, default=16)
p.add_argument("--episodes", type=int, default=64)
p.add_argument("--seed", type=int, default=12345)
p.add_argument("--out", type=Path, required=True)
a = p.parse_args()
a.out.mkdir(parents=True, exist_ok=True)
spec = se.Spec.load()
vla = SmolVLA(a.checkpoint, spec.image_names, 480)
n, K = a.num_envs, a.k
env = se.StackEnv(n * K)  # replica r of env i is sim r * n + i
committed = spec.committed
decisions = int(np.ceil(spec.episode_steps / committed))
rng = np.random.default_rng(a.seed)
parts = {k: [] for k in ("sim_states", "states", "actions", "success")}
t0 = time.perf_counter()
for b in range(int(np.ceil(a.episodes / n))):
    env.reset(rng)
    # all replicas start from env i's layout
    env.state[:] = np.tile(env.state[:n], (K, 1))
    env.batch.forward()
    rec = {k: [] for k in ("sim_states", "states", "actions")}
    for d in range(decisions):
        imgs = env.render(range(n))
        st = env.joint_state()[:n]
        ctx = vla.encode(imgs, st, spec.prompt)
        chunks = np.concatenate(
            [vla.decode(ctx) for _ in range(K)]
        )  # (K*n, 50, 8), replica-major
        start = env.state.copy()
        traj_s, traj_q = [], []
        for j in range(committed):
            traj_s.append(env.state.copy())
            traj_q.append(env.joint_state())
            env.step(chunks[:, j])
        score = env.success() * 10.0 + env.potential()
        best = score.reshape(K, n).argmax(0)  # (n,)
        pick = best * n + np.arange(n)
        for j in range(committed):
            rec["sim_states"].append(traj_s[j][pick])
            rec["states"].append(traj_q[j][pick])
            rec["actions"].append(chunks[pick, j])
        env.state[:] = np.tile(env.state[pick], (K, 1))
        env.batch.forward()
    succ = env.success()[:n]
    for k in rec:
        parts[k].append(np.stack(rec[k], 1))
    parts["success"].append(succ)
    print(
        json.dumps(
            {
                "batch": b,
                "success": float(succ.mean()),
                "wall_s": time.perf_counter() - t0,
            }
        ),
        flush=True,
    )
data = {k: np.concatenate(v)[: a.episodes] for k, v in parts.items()}
np.savez_compressed(a.out / "search_rollouts.npz", **data)
res = {
    "k": K,
    "episodes": int(len(data["success"])),
    "success": float(data["success"].mean()),
}
(a.out / "search.json").write_text(json.dumps(res))
print(json.dumps(res))
