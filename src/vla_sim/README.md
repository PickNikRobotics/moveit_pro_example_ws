# vla_sim

A MoveIt Pro MuJoCo simulation of a Kinova Gen3 arm stacking colored cubes on
command, driven by a vision-language-action policy. The `Stack Cubes with the
VLA Policy` objective runs the policy, which is served over HTTP by the
`inference_server` container built from [`docker/`](docker/), where the setup
and serving instructions live.

## Collecting training data

`Pick the Red Block` reads the cube's pose from the simulation, then approaches,
grasps, and lifts it. `Collect the Red Block Pick` wraps that Objective in
Trainer's recording Behaviors and converts the dataset to LeRobot v3.0. Nothing
checks that the cube came with the gripper.

`script/joint_command_bridge.py` publishes `/joint_commands` from the trajectory
controller's setpoint and the latched gripper command. That is the topic
conversion labels each frame's `action` from.

## Hardware requirements

An NVIDIA GPU is recommended. When one is present, MoveIt Pro makes it
available to the inference server automatically. The stack still runs without
one, but inference moves to the CPU, where the default pi0.5 checkpoint might
be too slow to run at all. A smaller model such as SmolVLA might be the better
fit there. AMD GPUs are not passed through yet, so those machines run inference
on the CPU as well.

For detailed documentation see: [MoveIt Pro Documentation](https://docs.picknik.ai/)
