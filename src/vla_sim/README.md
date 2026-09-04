# vla_sim

A MoveIt Pro MuJoCo simulation of a Kinova Gen3 arm stacking colored cubes on
command, driven by a vision-language-action policy. The `Stack Cubes with the
VLA Policy` objective runs the policy, which is served over HTTP by the
`inference_server` container built from [`docker/`](docker/), where the setup
and serving instructions live.

## Collecting training data

`Pick the Red Block` is a scripted demonstration: it reads the cube's pose from
the simulation, then approaches, grasps, and lifts it. `Collect the Red Block
Pick` wraps that Objective in Trainer's recording Behaviors, keeping the episode
only if the pick succeeds and converting the dataset to LeRobot v3.0.

Conversion labels `action` from a `sensor_msgs/JointState` command topic, and
nothing outside of teleoperation publishes one. `script/joint_command_bridge.py`
fills that gap, republishing the trajectory controller's setpoint and the latched
gripper command as `/joint_commands`; the gripper Objectives latch their command
through a parameter, since a `GripperCommand` goal is not observable on any
topic. Without the bridge the dataset converts only with **Action source: Next
state**, which labels each frame with the next frame's measured position rather
than with what the Objective commanded.

## Hardware requirements

An NVIDIA GPU is recommended. When one is present, MoveIt Pro makes it
available to the inference server automatically. The stack still runs without
one, but inference moves to the CPU, where the default pi0.5 checkpoint might
be too slow to run at all. A smaller model such as SmolVLA might be the better
fit there. AMD GPUs are not passed through yet, so those machines run inference
on the CPU as well.

For detailed documentation see: [MoveIt Pro Documentation](https://docs.picknik.ai/)
