# vla_sim

A MoveIt Pro MuJoCo simulation of a Kinova Gen3 arm stacking colored cubes on
command, driven by a vision-language-action policy. The `Stack Cubes with the
VLA Policy` objective runs the policy, which is served over HTTP by the
`inference_server` container built from [`docker/`](docker/), where the setup
and serving instructions live.

## Hardware requirements

An NVIDIA GPU is recommended. When one is present, MoveIt Pro makes it
available to the inference server automatically. The stack still runs without
one, but inference moves to the CPU, where the default pi0.5 checkpoint might
be too slow to run at all. A smaller model such as SmolVLA might be the better
fit there. AMD GPUs are not passed through yet, so those machines run inference
on the CPU as well.

For detailed documentation see: [MoveIt Pro Documentation](https://docs.picknik.ai/)
