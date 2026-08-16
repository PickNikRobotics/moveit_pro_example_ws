# SO-101 architecture

## Host ownership

```text
Framework Desktop                                      Raspberry Pi

MoveIt Pro Desktop App
          |
MoveIt Pro Runtime ── built-in WHEP/WebRTC
  ├─ planning and Objectives             RTSP 8554 <── MediaMTX cameras
  ├─ on-demand RTSP-to-ROS camera bridge
  ├─ GetActionChunk HTTP adapter
  └─ local CycloneDDS
          |
  ROS2DDS client ═══════ Zenoh/TCP 7448 ═══════ ROS2DDS router
                                                    |
                                              local CycloneDDS
                                                    |
                                              controller_manager
                                                ├─ state broadcaster
                                                ├─ JTC + gripper
                                                ├─ JTAC for VLA
                                                └─ VFC/JVC for jog
                                                    |
                                           Feetech hardware interface
                                                    |
                                               servo serial bus

ROCm inference server
  └─ local HTTP 8973 ───────────────> GetActionChunk adapter
```

The Pi owns the hardware interface, controller manager, and every controller
that claims a physical command interface. The hardware config therefore sets
`launch_control_node: false` and lists every Pi controller under
`controllers_not_managed`. The Framework Desktop owns planning, user
interaction, camera adaptation, and policy inference.

MoveIt Pro's drivers container remains present as a lifecycle dependency. Its
hardware launch file monitors the Pi's read-only joint-state heartbeat and
fails if valid state stops arriving. It does not receive the serial device,
publish commands, or start another controller manager.

`robot_state_publisher` still runs with MoveIt Pro because the filtered Pi
bridge does not export `/robot_description`. The Pi's local publisher remains
inside its isolated DDS graph.

## ROS control contract

| Endpoint | Owner | Purpose |
| --- | --- | --- |
| `/joint_states` | Pi | Measured six-joint state |
| `/controller_manager/activity` | Pi | Durable typed readiness and controller inventory |
| `/controller_manager/list_controllers` | Pi | On-demand controller inventory |
| `/controller_manager/switch_controller` | Pi | JTC/JTAC/VFC/JVC handoff |
| `/joint_trajectory_controller/follow_joint_trajectory` | Pi | Planned waypoint and teleoperation motion |
| `/joint_trajectory_admittance_controller/follow_joint_trajectory` | Pi | VLA chunk stitching through `FollowJointTrajectoryWithAdmittance` |
| `/gripper_controller/gripper_cmd` | Pi | Physical jaw command |
| `/velocity_force_controller/*` | Pi | Pose Jog commands and services |
| `/joint_velocity_controller/*` | Pi | Joint Jog commands and services |
| `/get_action_chunk` | Framework Desktop | ExecutePolicy-to-inference adapter |

The bridge filters are directional. The Pi exports state plus service and
action servers; the workstation exports commands plus service and action
clients. Neither side imports and exports the same server endpoint, preventing
a bridge-created proxy from reflecting a goal back into itself.

## Video paths

One camera has two consumers but does not need one expensive path:

- Camera panes use `MOVEIT_WEBRTC_PASSTHROUGH` to relay the existing RTSP
  stream through MoveIt Pro's MediaMTX/WHEP server.
- Policy inference, Trainer, and recording subscribe to the stable ROS image
  topic. That demand starts the RTSP decoder in `so101_camera_bridge`; it stops
  after the last subscriber leaves.

The Desktop App therefore sees the same ROS topic name regardless of whether
its pixels arrive from MuJoCo or the physical RTSP relay.

## Startup and failure behavior

The workstation launcher gives its ROS2DDS client and the Runtime the same
loopback-only CycloneDDS profile. Zenoh is the only network transport between
hosts. The bridge is pinned to the Jazzy wire contract, and the launcher reads
the Pi's transient-local `ControllerManagerActivity` message from an ephemeral
Runtime container. The MoveIt Pro Runtime is not started until the required
controllers report their expected active states. Endpoint discovery alone is
not treated as readiness.

Inference adds one more gate: the selected checkpoint must be non-empty and
the Pi must expose JTAC. A missing Hugging Face token, incompatible checkpoint,
or unavailable accelerator leaves the inference server in an explicit error
health state; it does not substitute fake actions or silently fall back from an
explicit accelerator request.

No workstation startup step activates the Pi motion profile, enables servo
torque, or sends a trajectory. Those operations remain explicit robot-side and
operator-authorized actions.

## Pi sidecar requirement for VLA

The corresponding `so101-robot-ops` deployment must:

1. copy the arm64 `joint_trajectory_admittance_controller` prefix from the
   licensed MoveIt Pro Runtime into the Pi image;
2. declare JTAC with `planning_group_name: manipulator`, the SO-101 tool
   frames, five stop accelerations, and no force/torque sensor because this
   robot has none;
3. load JTAC inactive while leaving JTC active at startup; and
4. export
   `/joint_trajectory_admittance_controller/follow_joint_trajectory` as an
   action server in the Pi ROS2DDS filter.

The workstation filter in this workspace imports the matching action client.
Until a Pi deployment satisfies the contract, the launcher reports
planning-only readiness and refuses `--with-inference`.
