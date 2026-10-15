# Microduck in the hangar

Draft simulation configuration using Microduck's real ONNX locomotion policy and MuJoCo/BAM actuator physics. MoveIt Pro displays measured joint states, the hangar, and live overview and head cameras. The eight favorite Objectives provide forward walking, left/right turning, head positioning, standing, and reset.

The policy owns all 14 actuators. There is no trajectory or jog controller. Use the supplied Objectives; Cartesian jogging, planned joint trajectories, navigation, and physical hardware are outside this draft. The jaw is welded closed in the upstream walking model and is displayed at zero. Six passive joints report the floating body's measured pose to the Desktop App.

Walking and turning Objectives refresh velocity commands for three seconds, then request standing. Canceling an Objective stops command refresh; a 300 ms watchdog requests zero velocity. A fall makes the readiness check fail. Run **Reset Microduck** or use **Reset Simulation** in Settings to restore the initial pose. Head-position commands persist until another head command or reset.

Backward velocity produced less than 4 mm of backward progress in five simulated seconds with either tested policy, so no backward Objective is exposed. The pretrained policy is used for the interactive demo; the five-iteration AMD training result only proves the training/export path and is not a trained replacement.

## Dependencies and build

Use the paired MoveIt Pro draft, stacked on PR #21563, which includes the empty-jog launch fix. Import the description into the example workspace:

```bash
vcs import src < src/microduck_sim/dependencies.repos
```

Keep the policy source and model outside this package. The verified source is `pollen-robotics/microduck_rl` at `cb70b792312d559a4da09064d92009079671815f`. The Runtime's Python 3.12 environment uses system ROS packages plus MuJoCo `3.10.0`, ONNX Runtime `1.24.4`, and BAM at `62bd8ce12154340be97e06f7f41a0ca8f116d967`. Install those into a venv with `--system-site-packages`; install BAM with `--no-deps` because this path uses its actuator model rather than its hardware messaging stack. The MoveIt Pro development image supplies NumPy, SciPy, and ROS dependencies.

Download `alpha_walking.onnx` from `pollen-robotics/microduck-policies` at Hugging Face revision `1b56c396825c052a4e26e95cf2b8d8298af9e9b4`. Its SHA-256 is `e36332d383997d51401897734cd3e79cf5038406feddb18b4d57ecfb141daa6c`. This policy expects 61 observations and returns 14 actions. Its embedded normalizer is preserved. Do not substitute a policy with a different observation or action contract.

Inside the isolated development container, source `/etc/skel/.moveit-pro-bashrc`, install user-workspace dependencies with `install_ros_dependencies.bash`, and build:

```bash
cd /home/noah/user_ws
colcon build --packages-up-to microduck_sim
source install/setup.bash
export MOVEIT_CONFIG_PACKAGE=microduck_sim
export MICRODUCK_PYTHON=/opt/microduck/venv/bin/python
export MICRODUCK_RL_ROOT=/opt/microduck/microduck_rl-rocm
export MICRODUCK_POLICY=/opt/microduck/policies/alpha_walking.onnx
export MUJOCO_GL=egl
agent_robot.app
```

Paths above match the Framework draft mount. Set them to your mounted source, venv, and model elsewhere. Missing dependencies, assets, incompatible model dimensions, and invalid numerical state fail explicitly. Camera transforms are sampled from MuJoCo's last physics evaluation, at most one 5 ms integration step behind the measured joint positions.

The head camera publishes `/microduck/head/image_raw` and `/microduck/head/camera_info`, with its measured optical pose in TF. Enable **Camera Frusta** in 3D Visualizer Settings to see the live image at the head. The overview feed remains at `/microduck/overview/image_raw`. The composed scene orients the head camera forward through the lens, with an upright image.

On the prepared Framework container, use `docker exec -u noah` to retain the `video` and `render` supplementary groups. Using `-u 1000:1000` drops those groups and forces slow software camera rendering. The verified live Runtime published 453 joint-state samples and 82 frames from each camera during the motion/watchdog check, moved 29.0 cm forward, and drifted 0.2 mm after stopping.

The hangar's visual floor is retained, but its coplanar collision mesh is disabled in the composed scene; a single plane supplies foot contacts. Double floor contacts suppressed the learned gait. The camera's near plane is reduced because MuJoCo scales it by the entire hangar's extent.

## Validation

Run the physical integration tests with the same environment variables and ROS setup as the Runtime:

```bash
"$MICRODUCK_PYTHON" -m pytest src/microduck_sim/test/test_policy_sim.py -q
```

The eleven tests check real forward/turn displacement, balance, stopping, head movement, reset, malformed commands, floor contacts, camera clipping, head-camera orientation and attachment, and independent URDF versus MuJoCo forward kinematics. No state is scripted to make these pass. The selected policy moved forward 49.7 cm over five simulated seconds at a 0.3 m/s command. Three repeated 3.6-second bursts after 30 seconds of standing each moved 34–35 cm. The newer `velstand.onnx` moved 9.4 cm in the first test and only 1.3 cm in the third repeated burst, so this draft selects `alpha_walking.onnx`. Command velocity is a policy input, not a guarantee of measured speed.

For the opt-in live check, start this simulation Runtime, then run `"$MICRODUCK_PYTHON" src/microduck_sim/test/verify_live_runtime.py` in its ROS environment. It resets the robot, sends a forward command, checks that the watchdog stops it without an explicit stop command, verifies camera and joint-state publication, and resets again. It refuses to run unless `MOVEIT_CONFIG_PACKAGE=microduck_sim`.

## AMD training

The Framework draft uses the ROCm 7.2.2 inference image from PR #21563, PyTorch `2.10.0+rocm7.2.2`, and the existing ROCm Warp worktree at `58146889520006b5e56cad1b08f1c4fe22bb2eea`. `training/README.md` records the exact training command and the two dependency patches needed by Microduck's pinned mjlab environment. Runtime inference uses CPU ONNX/MuJoCo; training physics and PPO use the Radeon 8060S.

## Upstream assets

Description source is Apache 2.0. Microduck RL identifies its 3D model files as CC BY-SA-NC. Fetch those assets separately for this local evaluation; this draft does not vendor them into the product or establish permission to redistribute them commercially. Preserve upstream license files. See the upstream repositories for applicable terms.
