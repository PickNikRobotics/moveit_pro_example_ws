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

## Collecting demonstrations

The same scene also records the demonstrations a replacement policy is trained
on, one Pro dataset per prompt.

`Run Cube-Stack Oracle` performs one stack from a scripted oracle, with no
recording, which is the quickest way to see whether a change to the scene or the
planner still produces a clean demonstration. `Collect Cube-Stack Demonstration`
records one. The six `Record Cube-Stack <held> On <target>` objectives each
sweep the 60 training layouts drawn for their prompt, producing one dataset per
prompt.

[`description/mujoco/keyframes.xml`](description/mujoco/keyframes.xml) holds the
layouts: 360 `train_*` and 150 `eval_*`.

For detailed documentation see: [MoveIt Pro Documentation](https://docs.picknik.ai/)
