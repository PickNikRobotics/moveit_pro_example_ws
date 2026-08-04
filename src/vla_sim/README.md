# vla_sim

A MoveIt Pro MuJoCo simulation of a Kinova Gen3 arm stacking colored cubes on
command, driven by a vision-language-action policy. The `Stack Cubes with the
VLA Policy` objective runs the policy, which is served over HTTP by the
`inference_server` container built from [`docker/`](docker/), where the setup
and serving instructions live.

## In-process pi0.5 with RTC

In the Desktop App, open `Stack Cubes with the VLA Policy`, set `use_in_process_policy` to `true`, and set `model_bundle_manifest` to the absolute path of the exported pi0.5 bundle's `model.yaml` inside the Runtime. The loader accepts `~` and `$HOME` expansion. Use the bundle trained for this scene's joint and camera ordering and compiled for the Runtime's device and LibTorch version. The bundle and its weights are supplied separately.

`LoadPi05Policy` loads the bundle before motion and shares its handle with `ExecutePolicy`. RTC remains enabled, using the Objective's explicit guidance width. Leave `use_in_process_policy=false` to use `/get_action_chunk`. Stop and relaunch the Objective after changing either selection input. The other cube-stacking Objectives continue to use their service-backed handles.

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
