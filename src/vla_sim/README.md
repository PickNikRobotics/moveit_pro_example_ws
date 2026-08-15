# vla_sim

A MoveIt Pro MuJoCo simulation of a Kinova Gen3 arm stacking colored cubes on
command, driven by a vision-language-action policy. The `Stack Cubes with the
VLA Policy` objective runs the policy, which is served over HTTP by the
`inference_server` container built from [`docker/`](docker/), where the setup
and serving instructions live.

## Hardware requirements

A supported NVIDIA CUDA or AMD ROCm GPU is recommended. No accelerator setting
is needed: MoveIt Pro detects supported host hardware, selects the matching
torch distribution, and exposes the GPU to the inference server. An explicit
`MOVEIT_TARGET` remains authoritative; an explicit empty value forces CPU.

The policy still names its device `cuda` on AMD because that is PyTorch's
supported ROCm API; `GET /health` reports `accelerator: rocm` so the backend
remains unambiguous. The stack also runs without a GPU, but the default pi0.5
checkpoint may be too slow on CPU. A smaller model such as SmolVLA is a better
fit there.

For detailed documentation see: [MoveIt Pro Documentation](https://docs.picknik.ai/)
