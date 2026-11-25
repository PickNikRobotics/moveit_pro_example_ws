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

The published cube-stacking dataset keeps only those 8 joints in `observation.state`. [`script/select_state_joints.py`](script/select_state_joints.py) writes a copy of a converted dataset whose state holds just the `action` joints, in `action` order, with `meta/info.json`, `meta/stats.json` and the per-episode statistics narrowed to match; videos are hard-linked, not re-encoded. Pass `--joints` to keep a different set. It refuses to overwrite an existing destination:

```bash
ros2 run vla_sim select_state_joints.py ~/.local/share/moveit_pro/trainer/recordings/cubestack_oracle-lerobot ~/datasets/cubestack_oracle-lerobot-8
```

To merge the copy with the published dataset through LeRobot's `aggregate_datasets`, add `--like <published-dataset-dir>`. LeRobot only merges datasets whose feature metadata is identical, and a converted dataset differs in two ways: Trainer adds `fps` to the per-frame index features, and its videos have no encoder `info` block. `--like` first checks that both datasets have the same fps, robot type, feature keys, and per-feature dtype, shape and names, and that every mp4 matches the reference's codec, pixel format, fps and resolution, which it reads with PyAV. It then takes the reference's feature metadata. Encoder settings are written as null because an mp4 does not record them. It also computes any quantile statistics (`q01`..`q99`) the reference has and the copy lacks, so the merged dataset keeps them. On any other difference it exits and names it.

[`description/mujoco/keyframes.xml`](description/mujoco/keyframes.xml) holds the layouts: 360 `train_*` and 150 `eval_*`, each followed by a comment naming the prompt it was drawn for.

For more data than 60 episodes per prompt, each of the six `Record Generated Cube-Stack <held> On <target>` Objectives records 300 more layouts for its prompt into `cubestack_gen_<held>_on_<target>`, through `Record Cube-Stack Layouts`, which records any semicolon-separated list of keyframes with a given prompt, cube frames, and dataset. The layouts are the `gen_<held>_on_<target>_*` keys of [`description/mujoco/keyframes_generated.xml`](description/mujoco/keyframes_generated.xml), drawn by [`script/generate_cube_stack_layouts.py`](script/generate_cube_stack_layouts.py) from the evaluation trials' area, spacing, and yaw range. No layout has every cube within 2 cm and 0.1 rad of the same cube in an `eval_*` or `train_*` key, an evaluation trial 0 to 299 at seed 0, or another generated layout. The script writes the keyframes and the six Objectives together, so rerun it rather than editing either; `--layouts-out` also writes the layouts as JSON lines, keyed by keyframe name:

```bash
python3 src/vla_sim/script/generate_cube_stack_layouts.py --count 300 --seed 1 --layouts-out gen_layouts.jsonl
```

For detailed documentation see: [MoveIt Pro Documentation](https://docs.picknik.ai/)
