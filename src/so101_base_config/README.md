# so101_base_config

A MoveIt Pro configuration for a LeRobot SO-101 follower arm, fitted with the
XLeRobot soft fin-ray gripper. It owns the robot description, the
`ros2_control` hardware interfaces (**mock** by default, and the **physical
arm** over the Feetech STS3215 serial bus when `hardware_interface` is set to
`real`), and the MoveIt configuration. There is no MuJoCo model and no physics
either way.

This is the package a real-arm deployment points MoveIt Pro at. For a
ready-to-run mock overlay with a digital-twin wiggle-test bridge, see
[`so101_sim`](../so101_sim/README.md), which inherits this package via
`based_on_package` and only forces `hardware_interface: "mock"`.

Run it directly (mock hardware, no bridge — the twin sits still until you
command a plan or teleop through it):

```bash
moveit_pro run --config so101_base_config
```

## Real hardware: the Feetech STS3215 bus

`hardware_interface:=real` swaps `mock_components/GenericSystem` for
`feetech_ros2_driver/FeetechHardwareInterface`, which talks to the six STS3215
servos on one serial bus. Everything above the hardware interface is unchanged:
the same `joint_trajectory_controller` executes planned trajectories and the
gripper Objectives, the same `joint_velocity_controller` and
`velocity_force_controller` serve joint and pose jog, and `/joint_states`
becomes the real arm's encoders. Both jog controllers command `position`
(`config/control/so101.ros2_control.yaml`), which is the one command interface
the driver exports, so teleoperation reaches the servos on this branch too —
nothing about jogging is mock-only. As on mock, only one of them owns the arm
at a time: MoveIt Pro deactivates `joint_trajectory_controller` to activate a
jog controller, because `ros2_control` will not let two controllers claim the
same position command interface.

The driver is vendored. `src/external_dependencies/feetech_ros2_driver` is the
exact source of the `ros-jazzy-feetech-ros2-driver` 0.2.2 binary (the `0.2.2`
tag, pinned in its `UPSTREAM.yaml`) plus one change: servo torque follows the
`ros2_control` lifecycle. Upstream 0.2.2 only ever turns torque *off* —
`on_init` for joints without a command interface, `on_deactivate` for all of
them — and nothing turns it on, so whether the arm held anything depended on
the torque state the servos happened to power up or be left in, and a
trajectory sent to torque-off servos "executed" without moving them. The
vendored copy adds `on_configure`, which writes `TORQUE_ENABLE=0` to every servo
on the bus, and has `on_activate` write `TORQUE_ENABLE=1` to the joints that
declare a command interface once it has seeded the command from the present
position. `on_deactivate` is untouched. The effect is: the arm is limp from
power-on through configure, rigid while the hardware component is active, and
limp again only when the component is deactivated. `moveit_pro down` does not
deactivate it — it stops the drivers container without a `ros2_control`
shutdown, so `on_deactivate` never runs and the servos keep whatever torque
state they had: **after a stop the arm stays rigid.** To make it limp,
deactivate the component first
(`ros2 control set_hardware_component_state so101 inactive`) or cut bus power.
The diff against the tag is the files under `modified_paths` in
`UPSTREAM.yaml`.

Upstream `main` is deliberately not tracked: it moves the joint configuration to
YAML, changes what `offset` means, and has no release. Do not bump the vendored
copy to it without redoing the calibration recipe below — `main` ignores
`offset`, hardcodes the zero at tick 2048, and expects `homing_offset` instead,
so the recipe would leave every commanded position off by `homing_offset`
ticks.

Because the package is built in the workspace, `rosdep install --ignore-src`
no longer installs the apt deb, and `colcon-defaults.yaml` lists
`feetech_ros2_driver` under `allow-overriding` so an image that already carries
the deb builds the overlay instead of refusing. The plugin the controller
manager loads must be the workspace copy — the torque lifecycle exists nowhere
else. From inside the container, with the workspace sourced:

```bash
ros2 pkg prefix feetech_ros2_driver   # $USER_WS/install/feetech_ros2_driver, not /opt/ros/jazzy
lib="$(ros2 pkg prefix feetech_ros2_driver)/lib/libfeetech_ros2_driver.so"
strings "$lib" | grep -Fq FeetechHardwareInterface12on_configure &&
  strings "$lib" | grep -Fq FeetechHardwareInterface10set_torque &&
  strings "$lib" | grep -Fq "does not specify an offset parameter" &&
  ! strings "$lib" | grep -Fq homing_offset &&
  echo "vendored driver" || echo "WRONG DRIVER"
```

Every line must hold for the check to pass: the vendored copy defines both
mangled `FeetechHardwareInterface::...` symbols, keeps the 0.2.2 `offset`
message, and has no `homing_offset`. The apt 0.2.2 binary fails the first two
(it carries only the base `LifecycleNodeInterface::on_configure` vtable
reference, so a bare `grep on_configure` would pass on both — hence the mangled
names); a build cut from `main` fails the last. `test/test_hardware_plugin.py`
asserts only that the prefix resolves outside `/opt/ros`, at `colcon test`.

### What the URDF exposes

| xacro arg | Default | What it is |
|---|---|---|
| `hardware_interface` | `mock` | `mock` or `real`. |
| `usb_port` | `/dev/so101_follower` | Follower bus. Keep it a udev symlink; `/dev/ttyACM*` renumbers. |
| `calibration_file` | `config/so101_follower_calibration.yaml` | Per-joint servo `id` and zero `offset`. Read only on `real`. Point it at a copy to run a second arm. |

The baud rate is deliberately not an argument: the driver hardwires 1 Mbaud,
which is the SO-101 factory setting.

### Known gaps in the driver

Real limits of `feetech_ros2_driver` 0.2.2, not of this config:

- **No effort feedback.** The driver exports position and velocity state only,
  and `ros2_control` refuses to load a hardware component whose URDF declares
  a state interface the driver did not export — the controller manager logs
  `Discrepancy between robot description file (urdf) and actually exported HW
  interfaces.` and the component never exists. So the URDF declares `effort`
  on the mock branch only, and `joint_state_broadcaster` does not ask for it on
  either. `/joint_states` now carries a NaN effort array on both branches —
  mock used to report a constant 0.0 there, which was no more informative.
  Servo Present Load is readable over the bus; exporting it is an upstream
  change.
- **Velocity is unsigned.** The driver reads Present Speed as a plain word and
  never decodes the direction bit, so on `real` a joint moving negative appears
  in `/joint_states` as roughly +50.27 rad/s. Positions are unaffected. A
  settled servo reports 0, so this mostly bites at the goal instant — which is
  why `constraints.stopped_velocity_tolerance` is `0.0` in
  `config/control/so101.ros2_control.yaml`.
- **No per-joint sign.** `offset` shifts a joint's zero but cannot invert it.
  If a joint reads backwards on the bench, that is an upstream gap — do not try
  to paper over it with an offset.
- **A bad port aborts the process.** `SerialPort::close()` catches
  `std::runtime_error`, but LibSerial's `NotOpen` derives from
  `std::logic_error`, so a failed open escapes the destructor and terminates
  `ros2_control_node` instead of returning a clean init error. The log line
  before the crash is the useful one:
  `FeetechHardware::on_init -> Open [/dev/so101_follower]: ...`. Check the
  symlink and your `dialout` membership before reading the stack trace. An
  invalid or absent `usb_port` therefore terminates `ros2_control_node` outright
  rather than failing initialization cleanly, and will keep doing so until the
  driver handles the failed-open path upstream.
- **Upstream `on_activate` could seed zeros — fixed in the vendored copy.**
  Upstream calls `read()` and discards the return, then assigns
  `hw_positions_ = state_hw_positions_`. `state_hw_positions_` is resized to
  `0.0`, and `read()` returns ERROR early without touching it when `sync_read`
  fails, so a failed read at activation would have seeded a zero command and
  the first `write()` would have sent tick 2048 to every servo at the driver's
  hardcoded speed 2400. The vendored `on_activate` returns ERROR on a failed
  read, before seeding the command or enabling torque; the component stays
  inactive and limp, and the controller manager logs the failed activation.

### Bench procedure

Do this once per arm, with the workspace already built
(`moveit_pro build all`).

**1. udev.** Plug in one adapter at a time and read its serial number:

```bash
udevadm info -a -n /dev/ttyACM0 | grep -m1 '{serial}'
```

Put those serials into `config/udev/99-so101.rules` — the two in the file are
the reference bench's adapters — then install it:

```bash
sudo cp config/udev/99-so101.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
ls -l /dev/so101_follower /dev/so101_leader
```

You must also be in the `dialout` group (`groups | grep dialout`); the
container's user already is.

**2. Servo IDs and calibration.** With the arm on the bench and free to move:

- Set the IDs 1..6 from the base outward with the end-to-end repo's
  `scripts/so101/setup_motors.py`. That order is what
  `config/so101_follower_calibration.yaml` assumes.
- Run the LeRobot calibration. Its half-turn homing writes each motor's homing
  offset to EEPROM as `present_position - 2047`, so tick 2048 ends up meaning
  *the pose the arm was held in while homing ran* — not the URDF's zero. Hold
  every joint at its URDF-zero pose during homing, or `offset: 2048` is wrong
  by the difference. The gripper is the easy one to get wrong: its URDF zero is
  the near-closed end of travel (limits `-0.174533`..`1.74533`), not mid-travel,
  so homing with the jaw half open puts every gripper command roughly 512 ticks
  out. Leave `offset: 2048` when homing was done at URDF zero. If you skip the
  EEPROM write, put `2048 + homing_offset` in `offset` instead, per motor.
- If the per-joint check in *Safe bring-up order* step 3 shows a joint reporting
  the wrong angle, correct that motor's `offset` by the error in ticks:
  `offset += (reported - actual) * 4096 / (2 * pi)`, where `reported` is the
  joint's angle in `/joint_states` and `actual` is the pose the joint is really
  in, both in radians; round to whole ticks. Raising `offset` lowers the
  reported angle.

**3. Point the config at real hardware.**

Set `hardware_interface` to `"real"` under `urdf_params` in
`config/config.yaml`. That file is the only channel by which these xacro args
reach the description; `moveit_pro configure` writes the global CLI config
(license key, workspace, config package) and cannot set them.

The value is case-sensitive and must be exactly `mock` or `real`. Anything else
— `Real`, a typo, a stray quote — matches neither `xacro:if` branch, so
`<ros2_control>` expands with no `<hardware>` block at all and no plugin loads.
That produces no parse error; it surfaces later as controllers that never claim
an interface. (A loud xacro failure on an unrecognised value is a sensible
follow-up.) So confirm in two separate steps, from inside the container with the
workspace sourced so `$(find so101_base_config)` resolves.

First read back the literal string that will be used:

```bash
grep -n 'hardware_interface:' config/config.yaml
```

Then expand the description with that same value. This proves the branch
expands, not that `config.yaml` is right — the previous line is what checks
that:

```bash
xacro description/so101.urdf.xacro hardware_interface:=real | grep '<plugin>'
```

`real` prints `feetech_ros2_driver/FeetechHardwareInterface`; `mock` prints
`mock_components/GenericSystem`; anything else prints nothing.

**4. First power-on.** Follow *Safe bring-up order* below, and know what the
driver does at bring-up before you close the loop:

- **Torque follows the lifecycle.** The arm is limp until the controller
  manager activates the hardware component, rigid while it is active, and limp
  again only when the component is deactivated. `on_configure` writes
  `TORQUE_ENABLE=0` to all six servos whatever state they powered up in,
  `on_activate` writes `TORQUE_ENABLE=1` to the six commanded joints, and
  `on_deactivate` writes `TORQUE_ENABLE=0` again. So **rest the arm on the bench
  before `moveit_pro run`**: nothing holds it up until activation. Do not count
  on the servos' power-up torque state either way. No power cycle is needed
  between runs — the next activation re-enables torque.
- **`moveit_pro down` leaves the arm rigid.** Stopping the instance kills the
  drivers container without running `on_deactivate`, so the servos keep the
  torque they had. Do not expect the arm to go limp when the instance stops.
  Before you stop, either deactivate the component
  (`ros2 control set_hardware_component_state so101 inactive`, which does turn
  torque off) or be ready to cut bus power. A restart without a power cycle
  is fine: the next activation reads the held pose and continues from it.
- `on_activate` reads present position and seeds the command from it *before*
  enabling torque, so there is no jump when the controller starts. If that
  read fails, activation fails and torque stays off (upstream would have
  seeded zeros instead; see *Known gaps in the driver*). Torque is enabled one
  servo at a time and each servo's status packet is checked, so a servo that
  does not answer or reports a fault also fails the activation, with the servo
  id in the log. Still confirm `/joint_states` matches the arm's physical pose
  before you command anything.
- The first activation is the one that proves the lifecycle on the bench.
  Watch for: limp before `moveit_pro run`; the arm going rigid at the pose it
  is resting in when the controller manager logs the `so101` component
  reaching `active` (before any controller is spawned), with no motion at the
  transition; still rigid after `moveit_pro down`; limp only after
  `ros2 control set_hardware_component_state so101 inactive` or bus power off.

**Power the servo bus before you start the instance.** The unpowered-first
smoke test is a `mock` procedure only; on `real` it cannot work. The CH343
adapter is USB-powered, so the port opens, but `on_init` then times out on all
six `read_model_number()` calls and returns `CallbackReturn::ERROR`:
`controller_manager` leaves the `so101` hardware component uninitialized, no
interfaces are claimed, and both spawners fail. Powering the arm afterwards
runs no code — `on_init` is re-entered only on a new latched
`/robot_description`, so you must restart the whole instance.

**Confirm the component loaded before commanding anything.** Without an arm,
the real branch has only been proven this far: the workspace builds with the
driver, the mock path is unchanged and runs as before, the real xacro branch
expands cleanly with the driver plugin present, and the controller manager
loads it as far as the serial-port open failure. `on_init` runs before
interface export and before `ros2_control` validates the URDF against what the
driver exported, so nothing past the port open — interface export, that
validation, activation, motion — has been exercised. First powered bring-up is
that check. In the controller manager log, confirm the `so101` component
loaded and both `joint_state_broadcaster` and `joint_trajectory_controller`
reached `active` with no spawner failure, and that this line is absent:

```
Discrepancy between robot description file (urdf) and actually exported HW interfaces.
```

If it appears, the URDF is declaring a state interface the driver does not
export (see *Known gaps*), the component was discarded, and no controller has
anything to claim.

So: with the arm powered, the instance started and the component confirmed,
check `/joint_states` against the arm's actual pose — commanding nothing. Only
then run one small waypoint.

> **The `so101_sim` overlay's `Mirror SO101 Follower` Objective is the wiggle
> test on real hardware — run it deliberately.** It moves every powered joint
> about 0.1 rad either side of the pose the arm is standing in, which is the
> point: it is how you confirm each servo is alive, responds, and turns the way
> the URDF says. Have the power switch in reach the first time, check the arm
> is clear of obstacles and of itself, and confirm `/joint_states` matches the
> real pose (bench step 4) before starting it — the wiggle is centred on what
> `/joint_states` reports, so a calibration error puts the centre somewhere
> other than where the arm actually is. Stop the Objective before planning or
> executing a motion. This config does not include the wiggle-test bridge
> itself; run `so101_sim` (with `hardware_interface` overridden back to
> `"real"` in its `config.yaml`, or from a config that inherits this one and
> adds the bridge) to get it.

**5. Falling back to mock.** Set `hardware_interface` back to `mock`. Nothing
else changes — same controllers, same Objectives, same waypoints — and no
serial port is opened.

### The leader arm is not wired up yet

The leader is a second, torque-off bus at `/dev/so101_leader`. The driver
already has the mechanism: `on_init` actively disables torque for joints that
declare no `<command_interface>`, and the vendored `on_activate` enables torque
only for joints that declare one, so a state-only `<ros2_control>` block reads a
limp arm without fighting it and stays limp through activation. The shape that
fits here is:

- a second `<ros2_control name="so101_leader" type="system">` in this
  package's URDF, same plugin, `usb_port` `/dev/so101_leader`, six `leader_*`
  joints with state interfaces only;
- a second `joint_state_broadcaster` for it with `use_local_topics: true`, so
  the leader's joints publish on the controller's own topic instead of
  polluting `/joint_states` with names the robot model does not have;
- `so101_sim`'s `Mirror SO101 Follower` reading that topic in place of the fake
  sine, keeping the same heartbeat gate.

It is not implemented here because a leader needs a new controller in
`config/control/so101.ros2_control.yaml`, a new entry in `config/config.yaml`'s
startup list, and a rewrite of `so101_sim`'s `script/so101_arm_bridge.py` to
read a topic instead of generating a sine.

## What is here

| Path | What it is |
|---|---|
| `description/so101.urdf.xacro` | The arm, with a `hardware_interface: mock \| real` switch. The arm meshes are the upstream LeRobot description and the gripper meshes are XLeRobot's soft fin-ray parts; both are recorded in `description/assets/NOTICE.md`. |
| `config/control/so101.ros2_control.yaml` | `joint_state_broadcaster`, one `joint_trajectory_controller` over all six joints (gripper included), and the two teleop jog controllers — `joint_velocity_controller` and `velocity_force_controller` — over the five arm joints. The trajectory controller commands position and reads position/velocity, the set both hardware interfaces offer. |
| `config/moveit/` | SRDF, joint limits, IK (`PoseIKPlugin`, `optimize_distance` — the SO-101 is 5-DOF and cannot hit arbitrary 6-DOF poses), and the jog configs. |
| `config/so101_follower_calibration.yaml` | Per-joint servo `id` and zero `offset`, read only when `hardware_interface:=real`. |
| `config/udev/99-so101.rules` | Stable `/dev/so101_{leader,follower}` symlinks for the two CH343 adapters. |
| `../external_dependencies/feetech_ros2_driver/` | The `real` branch's hardware interface: upstream 0.2.2 plus the torque lifecycle, see *Real hardware* above. |
| `objectives/` | `Move SO101 to Waypoint`, `Close Gripper`, `Open Gripper`, and a `Teleoperate` override that points the core teleop tree at this config's `joint_trajectory_controller` (there is no admittance controller here). The wiggle-test `Mirror SO101 Follower` Objective and its bridge live in the `so101_sim` overlay, not here. |

There is no `GripperActionController`. It would claim the gripper joint's
position command interface, and `ros2_control` would then refuse the trajectory
controller's claim on the same interface. `Close Gripper` and `Open Gripper`
move the gripper joint group through the trajectory controller instead. Both
Objectives must exist under exactly those names or the teleoperation gripper
controls silently do nothing.

Jogging works. `config/moveit/{pose,joint}_jog.yaml` name a
`velocity_force_controller` and a `joint_velocity_controller`; both are declared
in `config/control/so101.ros2_control.yaml` over the five arm joints and listed
under `controllers_inactive_at_startup`, so MoveIt Pro activates one for a jog
and switches back to the trajectory controller for a plan. Both command the
joints' `position` interface, the same one the trajectory controller claims, so
`ros2_control` refuses to run a jog controller and the trajectory controller at
once — the switch is exclusive by construction, not by convention.

## Safe bring-up order

Lifted from the SO-101 fork PR's own bring-up notes; follow it every time a
real arm is attached. MoveIt Pro's Stop control is a cooperative
software stop, not a safety-rated emergency stop. Keep physical power isolation
accessible and clear the robot's workspace before any live test.

1. On `so101_sim` (mock), with the arm unpowered, confirm the twin appears and
   moves under the fake source.
2. Rest the arm on the bench, power it, then switch to `real` and restart the
   instance. `real` needs the bus live before startup; an instance that starts
   against an unpowered arm leaves the hardware component uninitialized and
   cannot recover without a restart. The arm stays limp until the hardware
   component activates, so it must be somewhere it can rest. It does not go
   limp when the instance stops: deactivate the component or cut bus power
   for that.
3. Compare `/joint_states` against each joint's actual pose, one joint at a
   time — not a whole-arm glance — and confirm the gripper before you ever run
   `Close Gripper`. A wrong `offset` raises no error anywhere; the arm simply
   goes to the wrong pose, and for the gripper that is the jaw driving past its
   mechanical stop at the driver's hardcoded speed 2400. The correction is in
   bench step 2.
4. Confirm the camera panes, when cameras are added, without commanding motion.
5. Only after explicit authorization, test bounded gripper, waypoint and jog
   motions in that order.

If a waypoint produces clicking, stop the attempt and inspect the physical joint
and its tracking error. Do not raise path tolerances to make a stalled joint look
like a success.

## Waypoints are not taught poses

`waypoints/so101_waypoints.yaml` holds poses picked to be reachable and visible
in simulation. None of them has been validated against a physical arm; re-teach
them on the bench before trusting any of them.

## Later phases

- **Phase three** — the wrist and top USB cameras. `usb_cam` is already an
  `exec_depend` and is installed in the image, but nothing launches it yet.
- **Phase four** — Trainer recording of demonstrations. Note that a named
  training config is not a workspace file: the Trainer stores them as JSON under
  its own data directory, so `RecordEpisode(config_name="so101_sim")` fails with
  `Training config 'so101_sim' not found` until one is created for this
  deployment through the Trainer UI or its REST API.
