# RL post-training prototype (vla_rl_sim)

Prototype, not product code. It post-trains a SmolVLA policy for the vla_sim cube-stacking cell with RL in batched MuJoCo, outside the Runtime, and hands the Runtime a checkpoint its existing inference server and `ExecutePolicy` run unchanged. Not built or installed by CMake.

## Pipeline

1. `scripted.py`: privileged-state IK demos in mjbatch (states only; images are re-rendered on demand).
2. `sft.py`: fine-tune `lerobot/smolvla_base` on the demos (the "base" policy a customer would have).
3. `rl.py --mode dsrl`: DSRL. The VLA stays frozen; SAC learns the initial flow-matching noise (50 x 8) from the pooled VLA prefix embedding plus joint state, with a sparse success reward.
4. `rl.py --mode collect --actor ...` + `sft.py --resume ...`: distill the steered policy's successful rollouts back into the VLA weights.
5. `rl.py --mode eval --engines mjbatch,plain36`: score a checkpoint on fixed trials under MuJoCo 3.14 (mjbatch) and 3.6.0 (the Runtime's engine, via `plain_env.py` in a worker process).
6. `serve_check.py`: load a checkpoint with `vla_sim/docker/vla_inference_server.py` and request one chunk.

Every task parameter comes from `../config/rl_post_training.yaml`; `../test/test_rl_spec_matches_runtime.py` keeps that file in step with the Objective that runs the policy.

## Environment

Python 3.12, `mjbatch==0.1.4` (pins `mujoco==3.14.0`), `lerobot[smolvla]==0.6.0`, PyTorch with GPU support. On AMD Strix Halo (gfx1151) use AMD's gfx1151 wheels (`--index-url https://rocm.nightlies.amd.com/v2/gfx1151/`); `vla.py` enables AOTriton flash attention and disables MIOpen convolutions there. For the 3.6.0 evaluation engine, install `mujoco==3.6.0` into a separate directory (`--target sidedeps36`).
