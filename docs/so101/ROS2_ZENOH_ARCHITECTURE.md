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

`robot_state_publisher` still runs with MoveIt Pro because the filtered Pi
bridge does not export `/robot_description`. The Pi's local publisher remains
inside its isolated DDS graph.

## ROS control contract

| Endpoint | Owner | Purpose |
| --- | --- | --- |
| `/joint_states` | Pi | Measured six-joint state |
| `/controller_manager/list_controllers` | Pi | Typed readiness and controller inventory |
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

The workstation launcher starts only its local ROS2DDS client, then calls the
Pi's typed `ListControllers` service from an ephemeral Runtime container. The
MoveIt Pro Agent is not started until the required controllers answer in their
expected active states. Endpoint discovery alone is not treated as readiness.

Inference adds one more gate: the selected checkpoint must be non-empty and
the Pi must expose JTAC. A missing Hugging Face token, incompatible checkpoint,
or unavailable accelerator leaves the inference server in an explicit error
health state; it does not substitute fake actions or silently fall back from an
explicit accelerator request.

No workstation startup step activates the Pi motion profile, enables servo
torque, or sends a trajectory. Those operations remain explicit robot-side and
operator-authorized actions.

## Pi sidecar requirement for VLA

The current `so101-robot-ops` sidecar provides JTC, gripper, VFC, and JVC but
does not yet stage or spawn JTAC. Its companion change must:

1. copy the arm64 `joint_trajectory_admittance_controller` prefix from the
   licensed MoveIt Pro Runtime into the Pi image;
2. declare JTAC with `planning_group_name: manipulator`, the SO-101 tool
   frames, five stop accelerations, and `ft_sensor_name: ""` because this robot
   has no force/torque sensor;
3. load JTAC inactive while leaving JTC active at startup; and
4. export
   `/joint_trajectory_admittance_controller/follow_joint_trajectory` as an
   action server in the Pi ROS2DDS filter.

The workstation filter in this workspace already imports the matching action
client. Until the Pi change is deployed, the launcher reports planning-only
readiness and refuses `--with-inference`.
