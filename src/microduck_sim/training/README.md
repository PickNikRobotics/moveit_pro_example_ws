# Framework AMD training proof

The isolated `microduck-rl-rocm` container completed `Mjlab-Velocity-Flat-MicroDuck` with 64 environments for five PPO iterations on the Radeon 8060S. It collected 7,680 steps, saved `model_4.pt`, and exported a policy with `[1, 61]` observations and `[1, 14]` actions. ONNX Runtime accepted the exported model and returned finite output. This is an execution smoke test; five iterations do not produce a usable locomotion policy.

## Revisions

| Component | Verified revision |
| --- | --- |
| MoveIt Pro / ROCm image recipe | PR #21563, `f921f903478` |
| Container image | `mp-pr21563-inference_server-rocm7.2.2:latest`, ID `sha256:d68004607f5695b3bd635c81bbb079c5a3e359987ce394a76422ae74547cdfca` |
| PyTorch | `2.10.0+rocm7.2.2.lw.git23d69b29` package metadata |
| ROCm Warp | `58146889520006b5e56cad1b08f1c4fe22bb2eea` |
| MuJoCo Warp | `9edb8e09a6ea859a99232e154899ac0198910216` (3.8.1), plus `mujoco-warp-rocm.patch` |
| mjlab | `19d6c06c17572dafcb31ad5110a902006b58d944` (1.3.0), plus `mjlab-rocm.patch` |
| MuJoCo | `3.10.0` |
| Microduck RL | `cb70b792312d559a4da09064d92009079671815f` |
| BAM | `62bd8ce12154340be97e06f7f41a0ca8f116d967` |
| rsl-rl-lib / TensorDict | `5.0.1` / `0.10.0` |

The image's PyTorch venv is `/opt/venv`. The separate `/opt/microduck/rlvenv` includes `/opt/venv/lib/python3.12/site-packages` through `rocm-base.pth`, preserving the ROCm torch wheel. Installing Microduck's default torch pin would replace this stack with a different build. Install the checked-out mjlab and Microduck packages with `pip install --no-deps -e`, then resolve remaining dependencies with a constraint on the exact installed torch metadata version. `/home/noah/microduck-draft/rl-python-freeze.txt` records the installed local packages.

The existing ROCm Warp is mounted read-only at `/warp`. Task-owned sources are mounted from `/home/noah/microduck-draft` to `/opt/microduck`. The container runs as UID/GID 1000, with `/dev/kfd`, `/dev/dri`, and render group 990. Logging is local (`WANDB_MODE=disabled`, TensorBoard logger).

## Dependency changes

Apply each patch with `git apply` in the corresponding pinned worktree. Both upstream projects use Apache 2.0; retain their license files.

- mjlab: use eager GPU execution on HIP. Its CUDA-driver eligibility check accesses a removed Warp namespace and does not establish HIP graph support. CUDA behavior remains unchanged.
- MuJoCo Warp: disable unsupported conditional graph nodes on HIP, following the existing ROCm port. Also supply the world-frame identity matrix in `_frame_axis`'s fallback, matching current upstream; the older function otherwise refers to an undefined local variable when compiled by the newer Warp.

The newer MuJoCo Warp 3.12 ROCm worktree cannot directly replace 3.8.1: mjlab 1.3 sets `ls_parallel`, which 3.9.1 removed. Keep these revisions together. No PPO, reward, observation, actuator, or export logic was substituted.

## Repeat the smoke test

On the Framework, with the prepared container running:

```bash
docker exec -u 1000:1000 \
  -e PYTHONPATH=/warp:/opt/microduck/mujoco_warp-microduck \
  microduck-rl-rocm bash -c '
    cd /opt/microduck/microduck_rl-rocm
    /opt/microduck/rlvenv/bin/train Mjlab-Velocity-Flat-MicroDuck \
      --env.scene.num-envs 64 --agent.max-iterations 5 --agent.logger tensorboard
  '
```

The verified run is `logs/rsl_rl/velocity/2026-09-28_21-25-23_velocity/` inside the Microduck RL worktree. Its log is `/home/noah/microduck-draft/rl-smoke.log`. Training reported no NaN terminations. Expected warnings include unsupported HIPRTC precompiled headers and small matrix hipBLASLt calls falling back to hipBLAS. Eager execution is functional but is not a performance-optimized training configuration.

The Runtime deliberately continues to use the downloaded pretrained policy. To deploy a trained replacement, use upstream's normalizer-preserving ONNX export, verify its 61/14 contract, and run the physical integration tests before selecting it with `MICRODUCK_POLICY`.
