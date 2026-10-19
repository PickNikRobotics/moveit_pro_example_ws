# Base-policy and PPO experiments on AMD Strix Halo (October 2026)

Results: [RESULTS.md](RESULTS.md). PPO run 1 log: `ppo_run1.jsonl`.

## Environment

- `Dockerfile` adds the ROCm 7.2.2 torchvision, LeRobot 0.6.0 with SmolVLA, and mjbatch 0.1.4 to `sim-renderer-strix-nyx:gs0.1.3-rocm7.2.2`: the image built from sim-renderer-research's `docker/nyx/Dockerfile` (branch `strix/resident-renderer`, ROCm torch 2.10 for gfx1151), tagged with that name.
- `run.sh "<command>"` runs a command in that image with the GPU, this source tree mounted read-only, and `$MJBATCH_RL_DIR/exp19` (default `~/.cache/mjbatch_rl/exp19`) as the working directory. `$RL` is the prototype directory inside the container.
- Run one GPU job at a time on gfx1151: an evaluation started during training crashed it with a GPU memory access fault.

```bash
docker build -t vla-base-exp:latest src/vla_rl_sim/rl/experiment
# The Runtime's MuJoCo, for --engines plain36
docker run --rm -u "$(id -u):$(id -g)" -v ~/.cache/mjbatch_rl/sidedeps36:/out --entrypoint pip vla-base-exp:latest \
  install --target /out --no-deps mujoco==3.6.0
R=src/vla_rl_sim/rl/experiment/run.sh
C=$HOME/.cache/mjbatch_rl   # holds smolvla_base, as in the prototype README
$R 'python3 $RL/scripted.py --n 1024 --seed 100 --out demos_1024.npz'
$R "python3 \$RL/sft.py --demos demos_1024.npz --num-demos 869 --base $C/smolvla_base --steps 20000 --batch 32 \
    --lr 1e-4 --warmup 500 --save-every 5000 --out $C/exp19/sft_869"
$R "python3 \$RL/diag.py <checkpoint>"   # reads demos_256.npz: symlink it to demos_1024.npz
$R "python3 \$RL/rl.py --mode eval --checkpoint <checkpoint> --engines mjbatch,plain36 --out <dir>"
$R "python3 \$RL/ppo_flow.py --checkpoint <base> --out <dir> --iterations 30 --num-envs 16 --minibatch 16 \
    --batches-per-iter 4 --lr 1e-6 --target-kl 0.1 --eval-episodes 96 --eval-every 5"
```

The 20,000-step base was trained as 10,000 steps of that schedule (stopped at 11,175), then resumed from step 10,000 for 5,000 steps at lr 4e-5, then from step 15,000 for 5,000 steps at lr 2e-5 (`sft.py --resume`, which restarts the optimizer and schedule). Episode-length and commit-length variants were evaluated by pointing `EXAMPLE_WS_SRC` at a copy of the source tree whose `rl_post_training.yaml` sets `episode_steps` or `committed_action_steps`.

## Next steps

1. **Measure headroom on the 20,000-step base.** Run best-of-8 noise search (`search.py`) on the 192 evaluation trials. Search well above 45% means RL has behavior to recover; near 45% means more data or SFT is the better lever.
2. **PPO run 2, built around run 1's failure.** In run 1 about a third of per-step ratios clipped every iteration at a mean KL of about 0.01: updates were dominated by noise. Change:
   - noise and log-probability on the final 1-2 denoising steps only;
   - σ around 0.1 there;
   - train only the action output projection and timestep MLP (a few million parameters) instead of the 100M-parameter expert.
3. **If PPO still does not move,** try DSRL on this base (published gains start from about 20% base success), or a residual specialist distilled back into the checkpoint.
4. **Reproduce SmolVLA on LIBERO `libero_object`** (community reproductions about 89%, with `--policy.n_action_steps=10`) as a known-good check of the training and evaluation stack.
