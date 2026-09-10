# so101_sim

A MoveIt Pro configuration for a LeRobot SO-101 follower arm, fitted with the
XLeRobot soft fin-ray gripper, on **mock hardware**. It brings up a digital twin of the arm in the MoveIt Pro UI, driven
by a joint source outside the Runtime. There is no MuJoCo model and no physics.

Run it:

```bash
moveit_pro run --config so101_sim
```

Then run the **Mirror SO101 Follower** Objective: the twin starts moving.
`script/so101_arm_bridge.py` runs in `--fake` mode and publishes a slow sine on
the `joint_trajectory_controller` topic, which `mock_components/GenericSystem`
echoes back as `/joint_states`. Planning, teleoperation and the waypoint
Objectives work against that same twin.

Mirroring is off until that Objective asks for it, and stops when the Objective
is stopped. The trajectory controller has one owner at a time: a stream of topic
messages restarts its trajectory on every tick, so a plan's goal would be
accepted and then hang forever, or abort on a path tolerance the moving twin
violated. The Objective keeps mirroring alive by ticking the bridge's
`~/mirror` `Trigger` service in a loop; one second without a tick and the bridge
goes quiet. That heartbeat gate is the primary guard: **stop the Mirror
Objective before planning or executing a motion.** The bridge also skips a
publish while it believes a `follow_joint_trajectory` goal is live, but that is
best-effort only — the flag is set from `GoalStatusArray` messages, so a goal
started while the Mirror Objective is still publishing can lose the race with a
20 ms bridge tick.

## Execution to hardware is out of scope

**This config cannot move a physical SO-101.** MoveIt Pro here owns nothing but
mock hardware; planning and executing drives the mock, and no serial port is
opened anywhere in the Runtime. The `real` branch of
`description/so101.urdf.xacro` is an empty stub until the Feetech STS3215 bus
interface lands.

When a live joint source is added (phase two), it publishes into the same
`joint_trajectory_controller`, so the same one-owner rule applies: stop the
**Mirror SO101 Follower** Objective before pressing Plan.

## What is here

| Path | What it is |
|---|---|
| `description/so101.urdf.xacro` | The arm, with a `hardware_interface: mock \| real` switch. The arm meshes are the upstream LeRobot description and the gripper meshes are XLeRobot's soft fin-ray parts; both are recorded in `description/assets/NOTICE.md`. |
| `config/control/so101.ros2_control.yaml` | `joint_state_broadcaster`, a `joint_trajectory_controller` over all six joints (gripper included), and the two teleop jog controllers — `joint_velocity_controller` and `velocity_force_controller` — over the five arm joints. |
| `config/moveit/` | SRDF, joint limits, IK (`PoseIKPlugin`, `optimize_distance` — the SO-101 is 5-DOF and cannot hit arbitrary 6-DOF poses), and the jog configs. |
| `script/so101_arm_bridge.py` | The joint source. `--fake` publishes a sine; `--real` is a phase-two stub. |
| `objectives/` | `Mirror SO101 Follower`, `Move SO101 to Waypoint`, `Close Gripper`, `Open Gripper`, and a `Teleoperate` override that points the core teleop tree at this config's `joint_trajectory_controller` (there is no admittance controller here). |

There is no `GripperActionController`. It would claim the gripper joint's
position command interface, and `ros2_control` would then refuse the trajectory
controller's claim on the same interface, locking the joint source out of the
jaw. `Close Gripper` and `Open Gripper` move the gripper joint group through the
trajectory controller instead. Both Objectives must exist under exactly those
names or the teleoperation gripper controls silently do nothing.

Jogging works. `config/moveit/{pose,joint}_jog.yaml` name a
`velocity_force_controller` and a `joint_velocity_controller`; both are declared
in `config/control/so101.ros2_control.yaml` over the five arm joints and listed
under `controllers_inactive_at_startup`, so MoveIt Pro activates one for a jog
and switches back to the trajectory controller for a plan. Both command the
joints' `position` interface, the same one the trajectory controller claims, so
`ros2_control` refuses to run a jog controller and the trajectory controller at
once — the switch is exclusive by construction, not by convention.

## Safe bring-up order

Lifted from the SO-101 fork PR's own bring-up notes, and still worth following
the day a real arm is attached. MoveIt Pro's Stop control is a cooperative
software stop, not a safety-rated emergency stop. Keep physical power isolation
accessible and clear the robot's workspace before any live test.

1. Bring up the config with the arm unpowered and confirm the twin appears and
   moves under the fake source.
2. Power the arm and confirm joint states without commanding motion.
3. Confirm the camera panes, when cameras are added, without commanding motion.
4. Only after explicit authorization, test bounded gripper, waypoint and jog
   motions in that order.

If a waypoint produces clicking, stop the attempt and inspect the physical joint
and its tracking error. Do not raise path tolerances to make a stalled joint look
like a success.

## Waypoints are not taught poses

`waypoints/so101_waypoints.yaml` holds poses picked to be reachable and visible
in simulation. None of them has been validated against a physical arm; re-teach
them on the bench before trusting any of them.

## Later phases

- **Phase two** — the real Feetech bus: leader and follower over USB through
  LeRobot's `SO101Follower`, behind the same bridge node. Needs udev rules for
  stable `/dev/so101_{leader,follower}` names and a `pip install` line in the
  workspace `Dockerfile`. The per-joint `joint_signs` / `joint_offsets_deg`
  parameters the bridge already declares are the calibration knobs for it.
- **Phase three** — the wrist and top USB cameras. `usb_cam` is already an
  `exec_depend` and is installed in the image, but nothing launches it yet.
- **Phase four** — Trainer recording of demonstrations. Note that a named
  training config is not a workspace file: the Trainer stores them as JSON under
  its own data directory, so `RecordEpisode(config_name="so101_sim")` fails with
  `Training config 'so101_sim' not found` until one is created for this
  deployment through the Trainer UI or its REST API.
