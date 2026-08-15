# SO-101 with MoveIt Pro

This workspace runs one SO-101 follower in MuJoCo or connects MoveIt Pro to
the physical robot's Pi-hosted `ros2_control` stack. The Runtime and VLA
inference server run together on the Linux workstation; the Pi remains the
only process that owns the servo bus.

See [Architecture](ROS2_ZENOH_ARCHITECTURE.md) for the host boundary and ROS
contract.

## Start on the Framework Desktop

The launcher detects the local Tailscale address for WebRTC, downloads and
verifies the matching ROS2DDS bridge when needed, and checks the Pi with a
typed `ListControllers` request before starting the MoveIt Pro Agent:

```bash
./scripts/run-so101.bash
```

The default Pi hostname is `so101-pi.tail337068.ts.net`. Override it without
editing the workspace:

```bash
./scripts/run-so101.bash --robot-host 100.x.y.z
```

The launch fails closed if the Pi cannot answer with an active joint-state
broadcaster, trajectory controller, and gripper controller. It does not start
the Pi motion profile, enable torque, or command motion.

Run the MuJoCo configuration without the Pi:

```bash
./scripts/run-so101.bash --simulation
```

## VLA inference

No policy is selected by default. This is intentional: a checkpoint determines
physical commands and must be compatible with the real robot's coordinate and
camera contract. Select one for the current run with:

```bash
./scripts/run-so101.bash --checkpoint organization/model
```

For a private or gated model, authenticate once with `hf auth login`. The
launcher honors an existing `HF_TOKEN`; otherwise it reads `hf auth token`
without printing or persisting the token itself. You can also set the
checkpoint in
`src/so101_moveit_config/config/vla_serving.yaml`. A local checkpoint mounted
under `src/vla_sim/models/` is addressed as `/models/<directory>`.

Inference starts automatically when a checkpoint is selected. MoveIt Pro's
detected product target chooses ROCm on the Framework Desktop, CUDA on a
supported NVIDIA system, and CPU otherwise. Force the sidecar on or off with
`--with-inference` and `--without-inference`.

A live SO-101 checkpoint must use:

- state and action order `Rotation_R`, `Pitch_R`, `Elbow_R`, `Wrist_Pitch_R`,
  `Wrist_Roll_R`, `Jaw_R`;
- radians for revolute joints and meters for `Jaw_R`;
- the physical overhead and wrist camera observations expected by the
  Objective; and
- a known training frame rate, with the Objective's `dt` set to its inverse.

Simulation checkpoints trained with degree-space actions or a different camera
set are not safe substitutes. The inference server validates dimensions and
camera names, but it cannot infer a checkpoint's physical coordinate meaning.

MoveIt Pro 10.0's `ExecutePolicy` uses the
`joint_trajectory_admittance_controller` action for trajectory chunk
stitching. The launcher permits normal planning and teleoperation with the
Pi's standard trajectory controller, but refuses a requested inference launch
until JTAC is installed in the Pi sidecar.

## Cameras

The physical cameras keep their existing MediaMTX RTSP streams. For a camera
pane, the launcher maps the stable ROS topic name directly to its RTSP source
through MoveIt Pro's built-in WebRTC passthrough. This avoids decoding frames
to ROS and re-encoding them merely for display.

The same topics remain available as `sensor_msgs/Image` for policy inference,
Trainer, and ROS recording:

- `/so101/cameras/overhead/image_raw`
- `/so101/cameras/wrist/image_raw`

The camera bridge opens an RTSP decoder only while a ROS image subscriber
exists. Simulation publishes the same topic names from MuJoCo and does not use
RTSP passthrough.

## Safe bring-up order

MoveIt Pro's Stop control is a cooperative software stop, not a safety-rated
emergency stop. Keep physical power isolation accessible and clear the robot's
workspace before any live test.

1. Deploy and select the Pi's MoveIt motion profile without activating it from
   this workspace.
2. Run the launcher and verify its read-only controller preflight succeeds.
3. Confirm joint states and both camera panes without commanding motion.
4. Validate simulation policy behavior before selecting a live checkpoint.
5. Only after explicit authorization, test bounded gripper, waypoint, jog, and
   policy motions in that order.

Logs for the workstation ROS2DDS client are retained under
`${XDG_STATE_HOME:-$HOME/.local/state}/so101-moveit-pro/`.
