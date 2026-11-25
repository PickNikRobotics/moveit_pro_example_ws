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

The same scene records the demonstrations a replacement policy is trained on, with Trainer's recording Behaviors (`RecordEpisode`, `SaveEpisode`, `StopRecording`, `ConvertDataset`) and one dataset per prompt.

`Run Cube-Stack Oracle` performs one stack with the scripted oracle and records nothing, which is the quickest check that a scene or planner change still produces a clean demonstration. `Collect Cube-Stack Demonstration` records one stack from the `eval_0` layout into `cubestack_oracle` and converts it; if the oracle does not finish, it discards the attempt and fails. Each of the six `Record Cube-Stack <held> On <target>` Objectives sweeps the 60 training layouts drawn for its prompt into `cubestack_<held>_on_<target>`, one recording session per layout, discarding any layout the oracle cannot finish without stopping the sweep, then converts the dataset once. Conversion writes `<dataset>-lerobot` next to the recording, labels each episode with its prompt (`stack the <held> cube on the <target> cube`), and sets the robot type to `kinova_gen3`, as in the published cube-stacking dataset.

Recording uses the `vla_sim` Training Config, which MoveIt Pro creates from the running robot the first time an Objective names it:

- Cameras: `/overview_camera/color`, `/scene_camera/color` and `/wrist_camera/color`, recorded as `observation.images.overview`, `.scene` and `.wrist` at 480x480.
- FPS: the slowest camera's measured rate. The cameras render at 10 fps; confirm the config's **Default FPS** is `10` before the first recording, since a dataset keeps the fps it was created with.
- `observation.state`: every joint in `/joint_states`, including the Robotiq linkage joints, so it is wider than the 8 joints the policy commands.
- `action`: `/joint_commands` from [`script/joint_command_bridge.py`](script/joint_command_bridge.py): `joint_1`..`joint_7` and `robotiq_85_left_knuckle_joint`.

[`description/mujoco/keyframes.xml`](description/mujoco/keyframes.xml) holds the layouts: 360 `train_*` and 150 `eval_*`, each followed by a comment naming the prompt it was drawn for.

For detailed documentation see: [MoveIt Pro Documentation](https://docs.picknik.ai/)
