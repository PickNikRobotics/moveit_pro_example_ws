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
"""Flow-Noise PPO for SmolVLA (after piRL, Chen et al. 2025): the flow sampler becomes a stochastic policy by adding
learned Gaussian noise at each denoising step, so a chunk's log-probability is the sum of the per-step Gaussian
transitions. PPO trains the action expert and the noise scales on sparse stacking success in mjbatch; the VLM prefix
stays frozen. Noise and log-probabilities cover only the executed chunk steps and the 8 real action dimensions, since
the rest never reaches the robot. An asymmetric critic reads simulator state (joints, cube positions), which the
deployed policy never needs. The output is an ordinary LeRobot checkpoint, served unchanged.
"""

import argparse
import json
import time
from pathlib import Path

import numpy as np
import torch

import stack_env as se
from rl import evaluate
from vla import ACTION_DIM, SmolVLA

DEV = "cuda"


class Critic(torch.nn.Module):
    def __init__(self, dim: int):
        super().__init__()
        self.net = torch.nn.Sequential(
            torch.nn.Linear(dim, 256),
            torch.nn.SiLU(),
            torch.nn.Linear(256, 256),
            torch.nn.SiLU(),
            torch.nn.Linear(256, 1),
        )

    def forward(self, x):
        return self.net(x).squeeze(-1)


def privileged(env, k: int, decisions: int) -> np.ndarray:
    """Critic input: joint state, the three cube positions and the decision index as a fraction of the episode."""
    parts = [env.joint_state()] + [env.cube_pos(c) for c in ("red", "green", "blue")]
    parts.append(np.full((env.n, 1), k / decisions))
    return np.concatenate(parts, axis=1).astype(np.float32)


class NoisyFlow:
    """SmolVLA's Euler flow sampler with learned per-step Gaussian noise on the executed entries."""

    def __init__(
        self, vla: SmolVLA, committed: int, steps: int, sigma0: float, fp32: bool
    ):
        self.vla, self.m, self.fp32, self.committed = (
            vla,
            vla.policy.model,
            fp32,
            committed,
        )
        self.steps = steps
        self.dt = -1.0 / steps
        mask = torch.zeros(vla.chunk, vla.max_action_dim, device=DEV)
        mask[:committed, :ACTION_DIM] = 1
        self.mask = mask
        self.log_sigma = torch.nn.Parameter(
            torch.full((steps, ACTION_DIM), float(np.log(sigma0)), device=DEV)
        )

    def sigma(self) -> torch.Tensor:
        """(steps, chunk, max_action_dim): the noise scale on executed entries, 1 elsewhere (no noise added there)."""
        s = torch.ones(self.steps, self.vla.chunk, self.vla.max_action_dim, device=DEV)
        s[:, :, :ACTION_DIM] = self.log_sigma.clamp(np.log(0.005), np.log(0.5)).exp()[
            :, None, :
        ]
        return s

    def velocity(self, ctx, x, i):
        t = torch.full(
            (x.shape[0],), 1.0 + i * self.dt, dtype=torch.float32, device=DEV
        )
        # A chunk's log-probability sums over a thousand Gaussian terms, so rounding differences between the rollout
        # and the update pass add up; fp32 keeps the recomputed log-probability equal to the recorded one.
        with torch.autocast("cuda", dtype=torch.bfloat16, enabled=not self.fp32):
            v = self.m.denoise_step(
                prefix_pad_masks=ctx["pad"],
                past_key_values=ctx["kv"],
                x_t=x,
                timestep=t,
            )
        return v.float()

    def log_prob(self, mean, x_next, i):
        """(B, committed): the log-density of each executed chunk step, summed over its action dimensions."""
        sig = self.sigma()[i]
        lp = -0.5 * ((x_next - mean) / sig) ** 2 - sig.log() - 0.5 * np.log(2 * np.pi)
        return (lp * self.mask).sum(2)[:, : self.committed]

    @torch.no_grad()
    def sample(self, ctx, stochastic: bool = True):
        """Returns the chain (steps + 1, B, chunk, max_action_dim) on the CPU, its log-probability and the chunk."""
        b = ctx["b"]
        x = self.m.sample_noise((b, self.vla.chunk, self.vla.max_action_dim), DEV)
        chain, logp = [x], torch.zeros(b, self.committed, device=DEV)
        sig = self.sigma()
        for i in range(self.steps):
            mean = x + self.dt * self.velocity(ctx, x, i)
            x = (
                mean + sig[i] * torch.randn_like(mean) * self.mask
                if stochastic
                else mean
            )
            if stochastic:
                logp += self.log_prob(mean, x, i)
            chain.append(x)
        act = self.vla.post(x[:, :, :ACTION_DIM].float()).float().cpu().numpy()
        return torch.stack(chain).cpu(), logp.cpu(), act

    def chain_log_prob(self, ctx, chain):
        """(B, committed) log-probabilities of a stored chain (steps + 1, B, ...) under the current weights: each executed
        chunk step is one PPO token with its own ratio, as LLM PPO clips per token. One ratio over the whole chunk
        (about 1,600 Gaussian terms) moves too far for any useful step size."""
        chain = chain.to(DEV)
        lp = 0
        for i in range(self.steps):
            mean = chain[i] + self.dt * self.velocity(ctx, chain[i], i)
            lp = lp + self.log_prob(mean, chain[i + 1], i)
        return lp


def collect(env, vla, flow, critic, rng, spec):
    """One batch of env.n episodes; returns per-decision tensors (decisions, n, ...) and the episodes' success."""
    env.reset(rng)
    n, committed = env.n, spec.committed
    decisions = int(np.ceil(spec.episode_steps / committed))
    first = np.full(n, -1)
    alive = np.ones(n, bool)
    out = {
        k: []
        for k in ("rows", "state", "priv", "chain", "logp", "value", "reward", "alive")
    }
    for k in range(decisions):
        rows, st, priv = (
            env.state.copy(),
            env.joint_state(),
            privileged(env, k, decisions),
        )
        ctx = vla.encode(env.render(), st, spec.prompt)
        chain, logp, chunk = flow.sample(ctx)
        with torch.no_grad():
            value = critic(torch.as_tensor(priv, device=DEV)).cpu()
        was_alive = alive.copy()
        for j in range(committed):
            env.step(chunk[:, j])
            s = env.success()
            first[s & (first < 0)] = k * committed + j
        reward = ((first >= 0) & (first >= k * committed)).astype(np.float32)
        alive = alive & ~(first >= 0)
        for key, val in (
            ("rows", rows),
            ("state", st),
            ("priv", priv),
            ("chain", chain.transpose(0, 1)),
            ("logp", logp),
            ("value", value),
            ("reward", reward),
            ("alive", was_alive),
        ):
            out[key].append(val)
    return out, first >= 0


def advantages(reward, value, alive, gamma, lam):
    """GAE over decisions; an episode ends at its success decision or at the last decision (value 0 after)."""
    d, n = reward.shape
    adv = np.zeros((d, n), np.float32)
    last = np.zeros(n, np.float32)
    for k in reversed(range(d)):
        nxt_alive = alive[k + 1] if k + 1 < d else np.zeros(n, bool)
        nxt_value = value[k + 1] if k + 1 < d else np.zeros(n, np.float32)
        delta = reward[k] + gamma * nxt_value * nxt_alive - value[k]
        last = delta + gamma * lam * nxt_alive * last
        adv[k] = last
    return adv, adv + value


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--checkpoint", required=True, help="SFT base this pipeline saved")
    p.add_argument("--out", type=Path, required=True)
    p.add_argument("--iterations", type=int, default=40)
    p.add_argument(
        "--batches-per-iter",
        type=int,
        default=2,
        help="rollout batches of --num-envs episodes per update",
    )
    p.add_argument("--num-envs", type=int, default=32)
    p.add_argument("--denoise-steps", type=int, default=10)
    p.add_argument(
        "--sigma0",
        type=float,
        default=0.02,
        help="initial noise scale (normalized action units)",
    )
    p.add_argument("--lr", type=float, default=5e-6, help="action expert")
    p.add_argument("--lr-aux", type=float, default=1e-3, help="noise scales and critic")
    p.add_argument("--epochs", type=int, default=2)
    p.add_argument("--minibatch", type=int, default=32)
    p.add_argument("--clip", type=float, default=0.2)
    p.add_argument("--target-kl", type=float, default=0.05)
    p.add_argument("--gamma", type=float, default=0.99)
    p.add_argument("--lam", type=float, default=0.95)
    p.add_argument("--eval-every", type=int, default=5)
    p.add_argument("--eval-episodes", type=int, default=96)
    p.add_argument("--eval-seed", type=int, default=12345)
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--flow-dtype", choices=["fp32", "bf16"], default="fp32")
    p.add_argument(
        "--check-ratio",
        action="store_true",
        help="collect one batch, report recorded vs recomputed log-probabilities and noisy success, and exit",
    )
    a = p.parse_args()
    a.out.mkdir(parents=True, exist_ok=True)

    torch.manual_seed(a.seed)
    rng = np.random.default_rng(a.seed + 777)
    spec = se.Spec.load()
    env = se.StackEnv(a.num_envs)
    vla = SmolVLA(a.checkpoint, spec.image_names, env.image_size)
    vla.policy.model.config.num_steps = a.denoise_steps
    flow = NoisyFlow(
        vla, spec.committed, a.denoise_steps, a.sigma0, a.flow_dtype == "fp32"
    )
    critic = Critic(8 + 9 + 1).to(DEV)
    expert = [q for q in vla.policy.parameters() if q.requires_grad]
    opt = torch.optim.AdamW(
        [
            {"params": expert, "lr": a.lr},
            {"params": [flow.log_sigma] + list(critic.parameters()), "lr": a.lr_aux},
        ],
        weight_decay=0.0,
    )
    log = (a.out / "ppo_log.jsonl").open("a")

    def record(entry):
        print(json.dumps(entry), flush=True)
        log.write(json.dumps(entry) + "\n")
        log.flush()

    def run_eval(it):
        vla.policy.eval()
        res = evaluate(
            vla,
            lambda obs: (None, None),
            a.eval_episodes,
            a.num_envs,
            a.eval_seed,
            f"ppo iter {it}",
        )
        record({"iter": it, "eval_success": res["success"], "eval_ci95": res["ci95"]})
        return res["success"]

    if a.check_ratio:
        vla.policy.eval()
        out, s = collect(env, vla, flow, critic, rng, spec)
        rows, st = np.stack(out["rows"]), np.stack(out["state"])
        chain, old = torch.stack(out["chain"]), torch.stack(out["logp"])
        diffs = []
        for k in range(len(rows)):
            for s0 in range(0, env.n, a.minibatch):
                sl = slice(s0, s0 + a.minibatch)
                ctx = vla.encode(env.render_states(rows[k, sl]), st[k, sl], spec.prompt)
                with torch.no_grad():
                    new = flow.chain_log_prob(ctx, chain[k, sl].transpose(0, 1)).cpu()
                diffs.append((new - old[k, sl]).abs())
        d = torch.cat(diffs)
        d = d.sum(1)
        record(
            {
                "check": a.flow_dtype,
                "sigma0": a.sigma0,
                "noisy_success": float(s.mean()),
                "abs_dlogp_mean": float(d.mean()),
                "abs_dlogp_max": float(d.max()),
            }
        )
        return

    best = run_eval(0)
    for it in range(1, a.iterations + 1):
        t0 = time.perf_counter()
        vla.policy.eval()
        parts, succ = [], []
        for _ in range(a.batches_per_iter):
            out, s = collect(env, vla, flow, critic, rng, spec)
            parts.append(out)
            succ.append(s)
        cat = {
            k: (
                np.concatenate([np.stack(x[k], 0) for x in parts], 1)
                if k not in ("chain", "logp", "value")
                else torch.cat([torch.stack(x[k], 0) for x in parts], 1)
            )
            for k in parts[0]
        }
        adv, ret = advantages(
            cat["reward"], cat["value"].numpy(), cat["alive"], a.gamma, a.lam
        )
        idx = np.argwhere(cat["alive"])  # (decision, env) pairs that acted
        adv_t = adv[idx[:, 0], idx[:, 1]]
        adv_t = (adv_t - adv_t.mean()) / (adv_t.std() + 1e-8)
        t1 = time.perf_counter()

        # Fixed minibatches of one size, and "old" log-probabilities recomputed through the same path the update
        # uses: the prefix and expert kernels round differently by batch shape, so the rollout's own log-probabilities
        # would start PPO's ratio away from 1.
        order = rng.permutation(len(idx))
        batches = [
            order[s0 : s0 + a.minibatch]
            for s0 in range(0, len(order) - a.minibatch + 1, a.minibatch)
        ]

        def forward(mb, grad):
            dk, ne = idx[mb, 0], idx[mb, 1]
            ctx = vla.encode(
                env.render_states(cat["rows"][dk, ne]),
                cat["state"][dk, ne],
                spec.prompt,
            )
            with torch.set_grad_enabled(grad):
                return (
                    flow.chain_log_prob(ctx, cat["chain"][dk, ne].transpose(0, 1)),
                    dk,
                    ne,
                )

        old_logp = []
        for mb in batches:
            lp, _, _ = forward(mb, False)
            old_logp.append(lp.detach())
        stats = {"kl": [], "clipfrac": [], "vloss": [], "first_ratio_dev": None}
        stop = False
        for _ in range(a.epochs):
            for b in rng.permutation(len(batches)):
                mb = batches[b]
                logp, dk, ne = forward(mb, True)
                old = old_logp[b]
                if stats["first_ratio_dev"] is None:
                    stats["first_ratio_dev"] = float((logp.detach() - old).abs().max())
                ratio = (logp - old).exp()
                A = torch.as_tensor(adv_t[mb], device=DEV)[:, None]
                pg = -torch.min(
                    ratio * A, ratio.clamp(1 - a.clip, 1 + a.clip) * A
                ).mean()
                v = critic(torch.as_tensor(cat["priv"][dk, ne], device=DEV))
                vloss = ((v - torch.as_tensor(ret[dk, ne], device=DEV)) ** 2).mean()
                opt.zero_grad(set_to_none=True)
                (pg + 0.5 * vloss).backward()
                torch.nn.utils.clip_grad_norm_(expert, 1.0)
                opt.step()
                with torch.no_grad():
                    kl = (old - logp).mean().item()
                stats["kl"].append(kl)
                stats["clipfrac"].append(
                    ((ratio - 1).abs() > a.clip).float().mean().item()
                )
                stats["vloss"].append(vloss.item())
                if kl > a.target_kl:
                    stop = True
                    break
            if stop:
                break
        record(
            {
                "iter": it,
                "train_success": float(np.concatenate(succ).mean()),
                "decisions": int(len(idx)),
                "kl": float(np.mean(stats["kl"])),
                "clipfrac": float(np.mean(stats["clipfrac"])),
                "vloss": float(np.mean(stats["vloss"])),
                "sigma": float(flow.sigma()[:, 0, :ACTION_DIM].mean().detach()),
                "first_ratio_dev": stats["first_ratio_dev"],
                "early_stop": stop,
                "rollout_s": round(t1 - t0, 1),
                "update_s": round(time.perf_counter() - t1, 1),
            }
        )
        if it % a.eval_every == 0 or it == a.iterations:
            rate = run_eval(it)
            if rate > best:
                best = rate
                vla.save(a.out / "best")
                torch.save(
                    {
                        "log_sigma": flow.log_sigma.detach().cpu(),
                        "critic": critic.state_dict(),
                    },
                    a.out / "best" / "ppo_aux.pt",
                )
    vla.save(a.out / "last")


if __name__ == "__main__":
    main()
