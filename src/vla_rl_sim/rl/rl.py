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
"""Rollouts, evaluation, and DSRL (SAC over the frozen VLA's initial flow noise) on batched MuJoCo.

One RL decision = one ExecutePolicy chunk request: render cameras, encode, pick the noise, decode a
50-step chunk, execute committed_action_steps of it, re-plan. Reward 1 the first time the stack
succeeds (episode ends there in training); evaluation scores success at the end of the full
episode, as the Runtime's scoring Objective does.
"""

import argparse
import json
import os
import time
from pathlib import Path

import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F

import stack_env as se
from vla import ACTION_DIM, SmolVLA

DEV = "cuda"


# ---------------------------------------------------------------- noise policies
class Actor(nn.Module):
    """Tanh-squashed Gaussian over the steered noise block (chunk x ACTION_DIM)."""

    def __init__(self, obs_dim, act_dim, bound, hidden=512):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(obs_dim, hidden),
            nn.LayerNorm(hidden),
            nn.ReLU(),
            nn.Linear(hidden, hidden),
            nn.ReLU(),
        )
        self.mu, self.log_std = nn.Linear(hidden, act_dim), nn.Linear(hidden, act_dim)
        nn.init.zeros_(self.mu.weight)
        nn.init.zeros_(self.mu.bias)
        nn.init.zeros_(self.log_std.weight)
        nn.init.zeros_(self.log_std.bias)
        self.bound = bound

    def forward(self, o, deterministic=False):
        h = self.net(o)
        mu, log_std = self.mu(h), self.log_std(h).clamp(-5, 1)
        u = mu if deterministic else mu + log_std.exp() * torch.randn_like(mu)
        a = torch.tanh(u)
        logp = (
            -0.5 * ((u - mu) / log_std.exp()) ** 2 - log_std - 0.5 * np.log(2 * np.pi)
        ).sum(-1)
        logp = logp - (2 * (np.log(2) - u - F.softplus(-2 * u))).sum(-1)
        return a * self.bound, logp


class Critic(nn.Module):
    def __init__(self, obs_dim, act_dim, hidden=512):
        super().__init__()

        def q():
            return nn.Sequential(
                nn.Linear(obs_dim + act_dim, hidden),
                nn.LayerNorm(hidden),
                nn.ReLU(),
                nn.Linear(hidden, hidden),
                nn.ReLU(),
                nn.Linear(hidden, 1),
            )

        self.q1, self.q2 = q(), q()

    def forward(self, o, a):
        x = torch.cat([o, a], -1)
        return self.q1(x).squeeze(-1), self.q2(x).squeeze(-1)


class ObsNorm:
    """Running mean/std over the RL observation (pooled VLA prefix + joint state)."""

    def __init__(self, dim):
        self.n, self.mean, self.m2 = (
            0,
            torch.zeros(dim, device=DEV),
            torch.ones(dim, device=DEV),
        )

    def update(self, x):
        for row in x:
            self.n += 1
            d = row - self.mean
            self.mean += d / self.n
            self.m2 += d * (row - self.mean)

    def __call__(self, x):
        return (x - self.mean) / (self.m2 / max(self.n - 1, 1)).sqrt().clamp_min(1e-3)


def to_noise(steer: torch.Tensor, vla: SmolVLA, mode: str) -> torch.Tensor:
    """Embed a (B, chunk*ACTION_DIM) steering block into the sampler's (B, chunk, max_dim) noise.
    Padded action dims get standard normal noise (what the base policy saw in training) unless
    mode == 'zero_pad'."""
    b = steer.shape[0]
    full = (
        torch.randn(vla.noise_shape(b), device=steer.device)
        if mode != "zero_pad"
        else torch.zeros(vla.noise_shape(b), device=steer.device)
    )
    full[:, :, :ACTION_DIM] = steer.view(b, vla.chunk, ACTION_DIM)
    return full


# ---------------------------------------------------------------- rollouts
def run_episodes(
    env, vla, rng, noise_fn, train_terminate: bool, record=None, shaping=0.0
):
    """Run one batch of env.n episodes. noise_fn(obs_tensor) -> (noise or None, steer or None).
    Returns per-decision records and episode outcomes."""
    spec = env.spec
    env.reset(rng)
    n, committed = env.n, spec.committed
    decisions = int(np.ceil(spec.episode_steps / committed))
    alive = np.ones(n, bool)
    first_success = np.full(n, -1)
    recs = []
    t_render = t_vla = t_sim = 0.0
    for k in range(decisions):
        t0 = time.perf_counter()
        phi0 = env.potential() if shaping else None
        imgs = env.render()
        st = env.joint_state()
        t1 = time.perf_counter()
        ctx = vla.encode(imgs, st, spec.prompt)
        obs = torch.cat([ctx["feat"], torch.as_tensor(st, device=DEV)], -1)
        noise, steer = noise_fn(obs)
        chunk = vla.decode(ctx, noise)
        t2 = time.perf_counter()
        for j in range(committed):
            if record is not None:
                record["sim_states"].append(env.state.copy())
                record["states"].append(st if j == 0 else env.joint_state())
                record["actions"].append(chunk[:, j].copy())
            env.step(chunk[:, j])
            s = env.success()
            newly = s & (first_success < 0)
            first_success[newly] = k * committed + j
        t3 = time.perf_counter()
        t_render, t_vla, t_sim = t_render + t1 - t0, t_vla + t2 - t1, t_sim + t3 - t2
        r = ((first_success >= 0) & (first_success >= k * committed)).astype(np.float32)
        if shaping:
            r = r + shaping * (env.gamma_shaping * env.potential() - phi0).astype(
                np.float32
            )
        done = (
            (first_success >= 0) if train_terminate else np.full(n, k == decisions - 1)
        )
        done = done | (k == decisions - 1)
        recs.append(
            {
                "obs": obs,
                "steer": steer,
                "r": r,
                "done": done.copy(),
                "alive": alive.copy(),
            }
        )
        if train_terminate:
            alive = alive & ~(first_success >= 0)
    final_success = env.success()
    timing = {"render_s": t_render, "vla_s": t_vla, "sim_s": t_sim}
    return recs, final_success, first_success, timing


RUNTIME_MUJOCO = os.environ.get(
    "RUNTIME_MUJOCO_PATH", ""
)  # a --target install of mujoco==3.6.0, the Runtime's engine


def make_env(num_envs, engine):
    if engine == "mjbatch":
        return se.StackEnv(num_envs)
    from plain_env import RemoteEnv

    if not RUNTIME_MUJOCO:
        raise SystemExit(
            "--engines plain36 needs RUNTIME_MUJOCO_PATH: a directory holding mujoco==3.6.0 (pip install --target <dir> mujoco==3.6.0)."
        )
    return RemoteEnv(num_envs, se.Spec.load(), str(RUNTIME_MUJOCO))


def evaluate(vla, noise_fn, episodes, num_envs, seed, label, engine="mjbatch"):
    env = make_env(num_envs, engine)
    rng = np.random.default_rng(seed)
    succ, first, timings = [], [], []
    for _ in range(int(np.ceil(episodes / num_envs))):
        _, fs, f1, tm = run_episodes(env, vla, rng, noise_fn, train_terminate=False)
        succ.append(fs)
        first.append(f1)
        timings.append(tm)
    succ, first = np.concatenate(succ)[:episodes], np.concatenate(first)[:episodes]
    p = succ.mean()
    ci = 1.96 * np.sqrt(p * (1 - p) / len(succ))
    t = {k: float(sum(x[k] for x in timings)) for k in timings[0]}
    res = {
        "label": label,
        "engine": engine,
        "episodes": int(len(succ)),
        "success": float(p),
        "ci95": float(ci),
        "mean_first_success_step": (
            float(first[first >= 0].mean()) if (first >= 0).any() else None
        ),
        "timing": t,
    }
    print(json.dumps(res), flush=True)
    return res


def collect(vla, noise_fn, episodes, num_envs, seed, out: Path):
    """Roll out a (steered) policy and save the demos-format dataset sft.py trains on: per-step
    integration states (images re-rendered on demand), joint states, commanded targets, success.
    """
    env = se.StackEnv(num_envs)
    rng = np.random.default_rng(seed)
    parts = {k: [] for k in ("sim_states", "states", "actions", "success")}
    for _ in range(int(np.ceil(episodes / num_envs))):
        rec = {k: [] for k in ("sim_states", "states", "actions")}
        _, fs, _, _ = run_episodes(
            env, vla, rng, noise_fn, train_terminate=False, record=rec
        )
        for k in rec:
            parts[k].append(np.stack(rec[k], axis=1))
        parts["success"].append(fs)
    data = {k: np.concatenate(v)[:episodes] for k, v in parts.items()}
    np.savez_compressed(out, **data)
    print(
        f"collected {len(data['success'])} episodes, success {data['success'].mean():.3f} -> {out}",
        flush=True,
    )
    return data


def load_actor(vla, path):
    ck = torch.load(path, map_location=DEV)
    a = ck["args"]
    actor = Actor(vla.feature_dim + ACTION_DIM, vla.chunk * ACTION_DIM, a["bound"]).to(
        DEV
    )
    actor.load_state_dict(ck["actor"])
    norm = ObsNorm(vla.feature_dim + ACTION_DIM)
    norm.mean, norm.m2, norm.n = ck["norm_mean"], ck["norm_m2"], ck["norm_n"]

    def steered(obs, deterministic=False):
        with torch.no_grad():
            s, _ = actor(norm(obs), deterministic=deterministic)
        return to_noise(s, vla, a["pad_mode"]), s

    return steered


# ---------------------------------------------------------------- DSRL
def dsrl(vla, args):
    env = se.StackEnv(args.num_envs)
    env.gamma_shaping = args.gamma  # F = gamma * phi(s') - phi(s) (Ng et al. 1999)
    obs_dim = vla.feature_dim + ACTION_DIM
    act_dim = vla.chunk * ACTION_DIM
    actor = Actor(obs_dim, act_dim, args.bound).to(DEV)
    critic, target = Critic(obs_dim, act_dim).to(DEV), Critic(obs_dim, act_dim).to(DEV)
    target.load_state_dict(critic.state_dict())
    log_alpha = torch.tensor(np.log(args.alpha), device=DEV, requires_grad=True)
    opt_a = torch.optim.Adam(actor.parameters(), lr=args.lr)
    opt_c = torch.optim.Adam(critic.parameters(), lr=args.lr)
    opt_alpha = torch.optim.Adam([log_alpha], lr=args.lr)
    target_entropy = -args.entropy_scale * act_dim
    norm = ObsNorm(obs_dim)
    buf = {k: [] for k in ("o", "a", "r", "o2", "d")}
    rng = np.random.default_rng(args.seed)
    history = []

    def policy(obs):
        norm.update(obs)
        with torch.no_grad():
            a, _ = actor(norm(obs))
        return to_noise(a, vla, args.pad_mode), a

    t0 = time.perf_counter()
    for rnd in range(args.rounds):
        recs, fs, f1, tm = run_episodes(
            env, vla, rng, policy, train_terminate=True, shaping=args.shaping
        )
        # transitions where the env was alive at decision start
        for k, rec in enumerate(recs):
            live = torch.as_tensor(rec["alive"], device=DEV)
            if not live.any():
                continue
            nxt = recs[k + 1]["obs"] if k + 1 < len(recs) else rec["obs"]
            buf["o"].append(rec["obs"][live])
            buf["a"].append(rec["steer"][live])
            buf["r"].append(torch.as_tensor(rec["r"], device=DEV)[live])
            buf["o2"].append(nxt[live])
            buf["d"].append(
                torch.as_tensor(rec["done"], device=DEV, dtype=torch.float32)[live]
            )
        O, A, R, O2, D = (torch.cat(buf[k]) for k in ("o", "a", "r", "o2", "d"))
        size = len(O)
        stats = []
        for _ in range(args.updates_per_round):
            i = torch.randint(size, (args.batch,), device=DEV)
            o, a, r, o2, d = norm(O[i]), A[i], R[i], norm(O2[i]), D[i]
            alpha = log_alpha.exp().detach()
            with torch.no_grad():
                a2, logp2 = actor(o2)
                tq = torch.min(*target(o2, a2)) - alpha * logp2
                y = r + args.gamma * (1 - d) * tq
            q1, q2 = critic(o, a)
            lc = F.mse_loss(q1, y) + F.mse_loss(q2, y)
            opt_c.zero_grad()
            lc.backward()
            opt_c.step()
            an, logp = actor(o)
            la = (alpha * logp - torch.min(*critic(o, an))).mean()
            opt_a.zero_grad()
            la.backward()
            opt_a.step()
            lal = -(log_alpha * (logp.detach() + target_entropy)).mean()
            opt_alpha.zero_grad()
            lal.backward()
            opt_alpha.step()
            with torch.no_grad():
                for p, tp in zip(critic.parameters(), target.parameters(), strict=True):
                    tp.mul_(1 - args.tau).add_(args.tau * p)
            stats.append((lc.item(), la.item(), alpha.item(), q1.mean().item()))
        s = np.mean(stats, 0)
        h = {
            "round": rnd + 1,
            "episodes": (rnd + 1) * env.n,
            "decisions": size,
            "train_success": float((f1 >= 0).mean()),
            "critic_loss": s[0],
            "actor_loss": s[1],
            "alpha": s[2],
            "q": s[3],
            "wall_s": time.perf_counter() - t0,
            **tm,
        }
        history.append(h)
        print(json.dumps(h), flush=True)
    return actor, norm, history


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--checkpoint", required=True)
    p.add_argument(
        "--mode", choices=["eval", "dsrl", "collect", "eval_actor"], default="eval"
    )
    p.add_argument("--actor", type=Path, help="dsrl_actor.pt for collect / eval_actor")
    p.add_argument("--collect-episodes", type=int, default=256)
    p.add_argument(
        "--engines",
        default="mjbatch,plain36",
        help="evaluation physics: mjbatch (MuJoCo 3.14) and/or plain36 (the Runtime's 3.6.0)",
    )
    p.add_argument("--num-envs", type=int, default=32)
    p.add_argument("--eval-episodes", type=int, default=192)
    p.add_argument("--eval-seed", type=int, default=12345)
    p.add_argument("--rounds", type=int, default=40)
    p.add_argument("--updates-per-round", type=int, default=200)
    p.add_argument("--batch", type=int, default=256)
    p.add_argument("--lr", type=float, default=3e-4)
    p.add_argument("--gamma", type=float, default=0.97)
    p.add_argument("--tau", type=float, default=0.005)
    p.add_argument(
        "--alpha",
        type=float,
        default=1e-4,
        help="initial entropy weight; with a 400-dim noise action the bonus alpha*|log pi| must stay well below the reward of 1",
    )
    p.add_argument("--entropy-scale", type=float, default=0.5)
    p.add_argument("--bound", type=float, default=1.5)
    p.add_argument("--pad-mode", choices=["gauss_pad", "zero_pad"], default="gauss_pad")
    p.add_argument(
        "--shaping",
        type=float,
        default=0.0,
        help="weight of potential-based progress shaping; 0 is the sparse success reward",
    )
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--out", type=Path, required=True)
    a = p.parse_args()
    a.out.mkdir(parents=True, exist_ok=True)
    torch.manual_seed(a.seed)
    spec = se.Spec.load()
    vla = SmolVLA(a.checkpoint, spec.image_names, 480)
    results = {}
    if a.mode == "collect":
        fn = load_actor(vla, a.actor) if a.actor else (lambda o: (None, None))
        collect(
            vla,
            fn,
            a.collect_episodes,
            a.num_envs,
            a.seed + 1000,
            a.out / "rollouts.npz",
        )
        return
    if a.mode == "eval_actor":
        steered = load_actor(vla, a.actor)
        for eng in a.engines.split(","):
            results[f"dsrl_stochastic_{eng}"] = evaluate(
                vla,
                lambda o: steered(o, False),
                a.eval_episodes,
                a.num_envs,
                a.eval_seed,
                "dsrl stochastic",
                eng,
            )
            results[f"dsrl_deterministic_{eng}"] = evaluate(
                vla,
                lambda o: steered(o, True),
                a.eval_episodes,
                a.num_envs,
                a.eval_seed,
                "dsrl deterministic",
                eng,
            )
    elif a.mode == "eval":
        for eng in a.engines.split(","):
            results[f"base_{eng}"] = evaluate(
                vla,
                lambda o: (None, None),
                a.eval_episodes,
                a.num_envs,
                a.eval_seed,
                "base gaussian noise",
                eng,
            )
    else:
        actor, norm, hist = dsrl(vla, a)
        torch.save(
            {
                "actor": actor.state_dict(),
                "norm_mean": norm.mean,
                "norm_m2": norm.m2,
                "norm_n": norm.n,
                "args": vars(a),
            },
            a.out / "dsrl_actor.pt",
        )
        results["history"] = hist

        def steered(obs, deterministic):
            with torch.no_grad():
                s, _ = actor(norm(obs), deterministic=deterministic)
            return to_noise(s, vla, a.pad_mode), s

        for eng in a.engines.split(","):
            results[f"dsrl_stochastic_{eng}"] = evaluate(
                vla,
                lambda o: steered(o, False),
                a.eval_episodes,
                a.num_envs,
                a.eval_seed,
                "dsrl stochastic",
                eng,
            )
            results[f"dsrl_deterministic_{eng}"] = evaluate(
                vla,
                lambda o: steered(o, True),
                a.eval_episodes,
                a.num_envs,
                a.eval_seed,
                "dsrl deterministic",
                eng,
            )
    (a.out / f"results_{a.mode}.json").write_text(
        json.dumps(results, indent=1, default=str)
    )


if __name__ == "__main__":
    main()
