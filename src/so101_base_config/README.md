# so101_base_config

A MoveIt Pro configuration for a LeRobot SO-101 follower arm, fitted with the
XLeRobot soft fin-ray gripper. It owns the robot description, the
`ros2_control` hardware interfaces (**mock** by default, and the **physical
arm** over the Feetech STS3215 serial bus when `hardware_interface` is set to
`real`), and the MoveIt configuration. There is no MuJoCo model and no physics
either way.

This is the package a real-arm deployment points MoveIt Pro at. It also owns
the wiggle-test bridge and its `Mirror SO101 Follower` Objective (see
*What is here* below) — the same joint-by-joint bring-up check on mock and on
the real follower. For a ready-to-run mock overlay that forces
`hardware_interface: "mock"` and otherwise changes nothing, see
[`so101_sim`](../so101_sim/README.md).

Run it directly:

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
- **`on_deactivate` tolerates a faulted servo's torque-off acknowledgement.**
  A servo in overload/overheat/etc. still answers a torque-off write with a
  status packet, just one whose working-status byte is nonzero;
  `read_response` treats that as an `is_fault` error, and `set_torque` in
  `feetech_ros2_driver.cpp` logs it as a warning (not an error) specifically
  for a torque-off ack, since torque was, in fact, turned off. A missing or
  malformed reply still fails the transition, and torque-on still fails on
  any fault (a faulted servo should not be told to hold). No unit test
  covers this — the driver's protocol layer has no test harness — so verify
  it on the bench the next time a servo faults mid-session: `ros2 control
  set_hardware_component_state so101 inactive` should succeed and leave the
  arm limp even while a servo is reporting overload.
- **The adapter can drop the very first packet after the port opens.** A
  no-reply/bad-packet failure on a register write is retried up to 3 attempts
  total with a short pause; a fault reply is never retried, since the servo
  did answer. Seen both here (`on_init`'s first `p_cofficient` write timing
  out with no reply, aborting hardware init) and in LeRobot's own calibration
  CLI on the same bench.

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

**2. Servo IDs and calibration.** With the arm on the bench, supported (torque
drops during calibration) and free to move:

- Set the IDs 1..6 from the base outward with LeRobot's motor-setup tooling
  (`lerobot setup-motors`). That order is what
  `config/so101_follower_calibration.yaml` assumes.
- Run the LeRobot calibration — `script/calibrate_so101.py` wraps both the
  follower's and the leader's end to end and is safe to use offline; see
  *Calibrating a new arm* below for what it does and what to pre-install
  before travelling. Whichever way you run it, its half-turn homing writes each motor's homing
  offset to EEPROM as `present_position - 2047`, so tick 2048 ends up meaning
  *the pose the arm was held in while homing ran* — not the URDF's zero. Hold
  every joint at its URDF-zero pose during homing, or `offset: 2048` is wrong
  by the difference. The gripper is the easy one to get wrong: its URDF zero is
  the near-closed end of travel (limits `-0.174533`..`1.74533`), not mid-travel,
  so homing with the jaw half open puts every gripper command roughly 512 ticks
  out. Leave `offset: 2048` when homing was done at URDF zero. If you skip the
  EEPROM write, put `2048 + homing_offset` in `offset` instead, per motor.
- If the `Verify Calibration` Objective (*Safe bring-up order* step 6) reports
  a joint off by more than a tick or two, correct that motor's `offset` by the
  reported error: `offset += error_ticks`, rounded to a whole tick. Raising
  `offset` lowers the reported angle. The same formula, spelled out:
  `error_ticks = reported_rad * 4096 / (2 * pi)`, where `reported_rad` is the
  joint's angle in `/joint_states` while it is held at URDF zero.

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
- **Deactivating only the component, with `joint_trajectory_controller` left
  active, does not relax the arm the way you would expect.** The controller's
  last hold target survives the component going inactive, and re-activating
  the component drives the arm straight back to that old target instead of
  wherever it was moved to by hand in between. To hand-pose the arm, deactivate
  the controller *first*, then the component:
  `ros2 control set_controller_state joint_trajectory_controller inactive` then
  `ros2 control set_hardware_component_state so101 inactive`. To lock it again,
  reverse the order - activate the component, then the controller (which
  re-seeds its hold target from the current state on activation):
  `ros2 control set_hardware_component_state so101 active` then
  `ros2 control set_controller_state joint_trajectory_controller active`.
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

> **`Mirror SO101 Follower` is the first commanded motion on real hardware —
> run it deliberately, not as an afterthought.** See *Wiggle test* under
> *What is here* below for what it does (a self-wiggle without the leader, or
> leader-driven mirroring with it) and why it is safe on a powered arm. Have
> the power switch in reach the first time, check the arm is clear of
> obstacles and of itself, and confirm `/joint_states` matches the real pose
> (`Verify Calibration`, below) before starting it — both the wiggle's centre
> and the mirrored pose are anchored to what `/joint_states` reports, so a
> calibration error puts either one somewhere other than where the arm
> actually is. Stop the Objective before planning or executing a motion.

**5. Falling back to mock.** Set `hardware_interface` back to `mock`. Nothing
else changes — same controllers, same Objectives, same waypoints — and no
serial port is opened.

## Cameras

The wrist and scene USB cameras run unconditionally from
`launch/so101_drivers.launch.py` — the *drivers* container's launch file
(`config.yaml`'s `hardware.additional_driver_launch_file`), not the
Agent/runtime one, because only the drivers container bind-mounts the host's
`/dev`; the runtime container never sees a `/dev/v4l/by-id/*` path, only
whatever `/dev/videoN` numbering it happens to enumerate on its own, which is
exactly the unstable numbering the paragraph below rules out. On mock and on
real hardware alike, each is a `usb_cam_node_exe` node in its own topic
namespace, 640x480 at 30 fps: the wrist camera (Innomaker U20CAM) on
`/wrist_camera/image_raw` + `/wrist_camera/camera_info`, the scene camera
(Logitech C920) on `/scene_camera/image_raw` + `/scene_camera/camera_info`.
The app's camera panes and the data-collection tab pick up any
`sensor_msgs/Image` topic on the graph automatically; nothing else needs
registering once the node is running.

Device paths are by-id (`/dev/v4l/by-id/...`), never `/dev/videoN` — that
numbering is not stable across replug or reboot. Find a new camera's own path
with:

```bash
ls -l /dev/v4l/by-id/
udevadm info --query=all --name=/dev/video0 | grep ID_SERIAL
```

and override the checked-in defaults with the `wrist_camera_device` /
`scene_camera_device` launch arguments if a bench's cameras differ.
`so101_drivers.launch.py` resolves that by-id path to its real `/dev/videoN`
before handing it to usb_cam — `usb_cam` 0.8.1 cannot follow the by-id
symlink itself, since it is relative (`../../video4`) and usb_cam's own
resolution turns that into `/dev/../../video4`.

Neither camera has a URDF frame or a static transform to the robot
(maintainer's call) — each publishes under its own free-floating `frame_id`
(`wrist_camera` / `scene_camera`). On a machine without these two devices
plugged in — a developer laptop running `so101_sim`, say — the two
`usb_cam_node_exe` nodes fail to open their device and exit; the rest of the
instance is unaffected.

**Brightness is set explicitly, and the two cameras do not share a scale.**
The Innomaker U20CAM's (wrist) `brightness` control is signed, `-64..64`;
the Logitech C920's (scene) is unsigned, `0..255`. Both cameras power on at
the numeric value `50`, which is opposite ends of usable on their two
scales — on the wrist camera that is nearly full-bright (blown out), on the
scene camera it is nearly black. `so101_drivers.launch.py` sets
`wrist_camera_brightness` (default `0`, this scale's midpoint) and
`scene_camera_brightness` (default `128`, this scale's midpoint) as launch
arguments passed straight through to each `usb_cam_node_exe`'s `brightness`
parameter, and both cameras run with `autoexposure` on. Check or tune a
camera live with the vendor's own controls (`v4l2-utils`, install if not
already present):

```bash
v4l2-ctl --device=/dev/v4l/by-id/<camera's-by-id-path> --list-ctrls | grep brightness
v4l2-ctl --device=/dev/v4l/by-id/<camera's-by-id-path> --set-ctrl=brightness=<value>
```

A swapped camera (a different unit, even the same model) needs this
rechecked — `--list-ctrls` also prints the control's actual min/max, so a
default from a different unit's scale is easy to catch before it ships.

### Leader-driven mirroring on real hardware

The leader is a second, torque-off bus, `/dev/so101_leader`, that never goes
through `ros2_control` — `feetech_ros2_driver` only ever talks to the
follower. Instead, `script/so101_arm_bridge.py` opens the leader port itself
and reads it directly: the runtime image has neither `pyserial` nor
`scservo_sdk`, so it hand-rolls the Feetech `READ_DATA` packet over the raw
tty (present-position register, address 56, 2 bytes) rather than pulling in a
bus SDK. Like the cameras above, this `so101_arm_bridge` instance runs from
`launch/so101_drivers.launch.py` in the *drivers* container — `/dev/so101_leader`
is a udev symlink to a host device, and only the drivers container's `/dev`
bind-mount can see it; the runtime container cannot open it by that name at
all. `launch/so101_arm_bridge.launch.py` picks the bridge's mode from
`hardware_interface` with no hand edit — `--real` when it is `"real"`,
`--fake` otherwise — so once real hardware is configured, `Mirror SO101
Follower` mirrors the leader's pose onto the follower instead of running the
self-wiggle; see *Wiggle test* below for what changes. That `auto` resolution
is `so101_arm_bridge`'s own launch argument default, not a hardwired rule — a
second, always-`--fake` instance runs alongside it under a `source=fake`
override (this one back in `runtime.launch.xml`, since it needs no host
device), backing the separate `Wiggle SO101 Follower` Objective.

Both arms were LeRobot-calibrated with the same URDF-zero homing pose, so tick
2048 is URDF zero on both and every non-gripper joint uses the same formula as
the follower driver's own calibration (`rad = (tick - 2048) * 2*pi/4096`,
`so101_arm_bridge.py`'s `LEADER_TICK_ZERO`/`LEADER_RAD_PER_TICK`). The leader
trigger and the follower gripper are different mechanisms with different
travel, so the gripper is scaled by fraction of trigger travel onto the URDF
gripper range instead (`LEADER_TRIGGER_TICKS_CLOSED`/`_OPEN`,
`leader_trigger_to_gripper_radians`). A bad reply from a servo — timeout, wrong
id, a checksum mismatch, or a bus I/O error such as an unplugged port — skips
that whole sample rather than publish a guess; the bridge logs it at most once
a second and keeps gliding toward the last known good pose (`poll_leader`,
`leader_poll_rate_hz`, default 25 Hz). That leniency is bounded: a bench
incident where a leader servo tripped an overload mid-motion showed that
"keep the last good pose" is not enough on its own, since the follower kept
creeping toward that now-stale target instead of stopping. Once
`leader_timeout_s` (0.5 s by default) passes with no good read, the bridge
stops publishing altogether — the controller holds wherever it is — logs once,
and resumes automatically the next time a read succeeds.

Real-mode motion glides rather than jumps: on the bench, commanding the
follower straight to a freshly-read leader pose moved far too fast, so the
commanded target instead moves toward the leader's pose by at most
`real_slew_rate_rad_s` (4.0 rad/s by default) every publish tick, converging
over a few tenths of a second rather than snapping. The leader's `wrist_roll`
also reads about 96° (1.68 rad) off from the follower's own homing on the
bench arms; `leader_offset_rad` (radians, one per joint, default `0.0` except
`wrist_roll`'s `1.68`) is added to the leader reading before clamping to
correct for it. The proper fix is re-homing that leader servo (*Calibrating a
new arm* above); once it reads true, set this parameter's `wrist_roll` entry
back to `0.0`.

**The leader must be plugged in and powered before the instance starts, the
same rule as the follower's own bus.** `so101_arm_bridge` opens
`leader_port` once at startup; if it cannot (leader unplugged, wrong port,
no `dialout`), the node logs the error and exits non-zero rather than
silently doing nothing — `Mirror SO101 Follower`'s service call then fails
visibly instead of hanging.

**The leader board runs on 5 V — never power it from the 12 V brick.** Its
1/345 gear-ratio `shoulder_lift` servo has an 8.0 V max and faults on 12 V.
LeRobot does not surface that as an over-voltage error: it reports any
error-status reply from a faulted servo as "motor not found", which reads like
a wiring or port problem rather than the wrong power supply. Tell the two
supplies apart by the label on the brick, not by looks.

**The leader cannot hold its own weight against gravity on 5 V.** Bench
testing tripped an overload fault on `shoulder_lift` the moment torque was
enabled with the arm in a gravity-loaded pose — the same 8.0 V-max servo, now
failing to hold rather than failing to move. In normal operation the leader's
torque stays off and the operator holds the arm by hand; never enable torque
on the leader while it is unsupported in the air.

**Telling 7.4 V servos from 12 V servos:** every STS3215 in the kit carries a
printed label with its rated voltage (`7.4V` or `12V`) and its internal gear
ratio (`1/345`, `1/191`, `1/147`). That label is the only reliable way to tell
the two sets apart — most 7.4 V units accept a 12 V bus electrically, and only
the 1/345 unit faults, so a mixed-up arm can look fine right up to the joint
that does not. The leader arm is the 7.4 V set: `shoulder_pan` 1/191,
`shoulder_lift` 1/345, `elbow_flex` 1/191, `wrist_flex` 1/147, `wrist_roll`
1/147, `gripper` 1/147. The follower arm is the 12 V set, 1/345 on every
joint. (Source: the LeRobot SO-101 assembly page,
<https://huggingface.co/docs/lerobot/so101>.)

### Tuning the mirror

Four `--real`-mode node parameters, each also a launch argument of the same
name on `so101_drivers.launch.py` (forwarded to `so101_arm_bridge.launch.py`,
same shape as the camera arguments above):

| Parameter | Default | What it does |
| --- | --- | --- |
| `real_slew_rate_rad_s` | `4.0` rad/s | How fast the commanded target may glide toward the leader's pose each publish tick, see *Leader-driven mirroring on real hardware* above. |
| `leader_wrist_roll_offset_rad` | `1.68` rad | The bench `wrist_roll` homing correction, one entry of the `leader_offset_rad` node parameter, at `wrist_roll`'s fixed position; every other joint's offset stays `0.0`. Re-homing that leader servo (*Calibrating a new arm* below) is the proper fix; once it reads true, this should go back to `0.0`. |
| `leader_timeout_s` | `0.5` s | How long the leader bus may go without a good read before the follower stops moving, see *Leader-driven mirroring on real hardware* above. |
| `leader_port` | `/dev/so101_leader` | The leader's Feetech bus device, a udev symlink; see `config/udev/99-so101.rules`. |

**How to actually change one, primary path: edit `config/so101_drivers.yaml`
and restart the instance.** That file holds every `so101_drivers.launch.py`
knob (the four above, plus `wrist_camera_device`, `scene_camera_device`,
`wrist_camera_brightness`, `scene_camera_brightness`) with a comment on each;
the launch file reads it for its `DeclareLaunchArgument` defaults. `config/`
is installed via colcon's symlink-install (`colcon-defaults.yaml`), so a
restart is enough to pick up an edit, no rebuild needed.

**Bench path: override a launch argument directly on the command line**, for
a one-off change without touching the YAML (inside the drivers container,
where the leader and camera devices are reachable):

```bash
ros2 launch so101_base_config so101_drivers.launch.py \
  real_slew_rate_rad_s:=6.0 leader_timeout_s:=1.0
```

A `moveit_pro run` instance includes `additional_driver_launch_file`
(`so101_drivers.launch.py`) with no launch arguments of its own -
`moveit_studio_agent`'s launch description does not forward any, so this
override applies only when the file is run this way, standalone, not when a
full instance includes it.

### Calibrating a new arm

`script/calibrate_so101.py` wraps LeRobot's own
`lerobot.scripts.lerobot_calibrate` for both the follower and the leader, meant
to work with no network access on site. It installs nothing itself — build the
venv it expects before travelling:

```bash
python3 -m venv ~/lerobot-venv
~/lerobot-venv/bin/pip install 'lerobot[feetech] @ git+https://github.com/huggingface/lerobot@a656a982afe4132eb48a729f06118c115631ac02'
```

That git ref — which reports itself as lerobot 0.5.2 — is what the bench was
verified against; it is not a PyPI release. The released 0.5.1 lacks the
feetech motor-position overflow fix (huggingface/lerobot#3373, merged
2026-04-13), so its wheels leave STS3215 motors in multi-turn mode and the
calibration homing offsets come out inconsistent. A released tag past that ref
(0.6.x) should carry the fix but was not verified on this bench.

Then, with that venv's `python3` on `PATH` (or invoked directly), calibrate one
arm at a time — support the arm on something first, calibration drops servo
torque:

```bash
python3 script/calibrate_so101.py follower   # default port /dev/so101_follower
python3 script/calibrate_so101.py leader     # default port /dev/so101_leader
```

Each run prints the homing pose to hold before it execs LeRobot's own
interactive `lerobot_calibrate` — answer LeRobot's own prompts as they appear;
this script only wraps it, it does not automate them. The homing pose
overrides LeRobot's own suggestion for the gripper: hold every joint at
mid-range, wrist roll neutral (LeRobot's range-of-motion sweep does not move
it, so there is nothing further to check there), and the gripper
**NEAR-CLOSED** — not the "roughly half-open" LeRobot itself prompts for. This
URDF's gripper zero is the near-closed end of travel; homing with the jaw half
open puts every gripper command about 512 ticks off (the same gotcha as bench
step 2 above).

After the run, the script checks the calibration JSON landed under
`~/.cache/huggingface/lerobot/calibration/robots/so_follower/<id>.json` (or
`.../teleoperators/so_leader/<id>.json` for the leader) and fails loudly if it
did not. If a file for that id already exists, LeRobot's own tool asks
"use existing calibration?" before it starts — answer `c` to recalibrate from
scratch rather than reuse it.

The Feetech bus needs the operator in the `dialout` group; the script checks
`groups` and warns if not (`sudo usermod -aG dialout $USER`, then log out and
back in). If `lerobot` is not importable, the script fails with a message
naming the venv command above rather than trying to `pip install` anything
itself.

The bench's WCH CH343 adapters can drop the first reply to a bus WRITE (the
same failure mode as the driver's own first-packet retry, see *Known gaps in
the driver* above), which surfaces as `lerobot_calibrate` failing to connect
with `Incorrect status packet`. By default `script/calibrate_so101.py` runs
`lerobot_calibrate` through a small shim that monkeypatches LeRobot's own
`MotorsBus._write`/`_sync_write` to floor their retry count at 5 before
calling LeRobot's unmodified CLI — nothing else about the calibration changes,
since the retry happens inside LeRobot's own bus layer. Pass
`--bus-write-retries N` to change the floor, or `--bus-write-retries 0` to run
the stock `lerobot_calibrate` invocation unwrapped.

LeRobot's calibration `connect()` writes P_Coefficient=16 into every follower
servo as a side effect of connecting, not a deliberate SO-101 tuning choice —
the STS3215 factory default is 32. At P=16, the loaded joints (especially
`shoulder_lift` and `elbow_flex`, which carry the extended arm's weight)
settle noticeably short of a commanded goal and lag a moving trajectory; see
the `trajectory`/`goal` tolerance comments in `config/control/so101.ros2_control.yaml`.
`config/so101_follower_calibration.yaml` restores the factory gain as a
documented `p_cofficient: 32` entry per joint (also accepts `i_cofficient`,
`d_cofficient` the same way): the driver writes whichever of the three are
present into the servo's **EEPROM** (register 21 for P) at every startup, so
this is a persistent hardware change, not a runtime-only one, and re-running
`lerobot_calibrate` will overwrite it back to 16 the next time you calibrate.
Re-run the calibration script and then re-apply this file (or just leave it
checked in and rebuild) to restore 32 after any recalibration. Gain writes are
EEPROM (the servo commits to flash before it acknowledges), so they take
noticeably longer to acknowledge than the SRAM writes (torque, position) the
driver otherwise sends; `on_init` only pays that cost when a joint's stored
gain does not already match the calibration file.

## What is here

| Path | What it is |
|---|---|
| `description/so101.urdf.xacro` | The arm, with a `hardware_interface: mock \| real` switch. The arm meshes are the upstream LeRobot description and the gripper meshes are XLeRobot's soft fin-ray parts; both are recorded in `description/assets/NOTICE.md`. |
| `config/control/so101.ros2_control.yaml` | `joint_state_broadcaster`, one `joint_trajectory_controller` over all six joints (gripper included), and the two teleop jog controllers — `joint_velocity_controller` and `velocity_force_controller` — over the five arm joints. The trajectory controller commands position and reads position/velocity, the set both hardware interfaces offer. |
| `config/moveit/` | SRDF, joint limits, IK (`PoseIKPlugin`, `optimize_distance` — the SO-101 is 5-DOF and cannot hit arbitrary 6-DOF poses), and the jog configs. |
| `config/so101_follower_calibration.yaml` | Per-joint servo `id`, zero `offset`, and optional PID gains (`p_cofficient` et al.), read only when `hardware_interface:=real`. |
| `config/udev/99-so101.rules` | Stable `/dev/so101_{leader,follower}` symlinks for the two CH343 adapters. |
| `../external_dependencies/feetech_ros2_driver/` | The `real` branch's hardware interface: upstream 0.2.2 plus the torque lifecycle, see *Real hardware* above. |
| `objectives/` | `Move SO101 to Waypoint`, `Close Gripper`, `Open Gripper`, a `Teleoperate` override that points the core teleop tree at this config's `joint_trajectory_controller` (there is no admittance controller here), `Mirror SO101 Follower` and `Wiggle SO101 Follower` (below), and `Verify Calibration` (below). |
| `script/so101_arm_bridge.py` | The joint source behind `Mirror SO101 Follower`: `--fake` publishes a small sine about the arm's measured pose (the wiggle test); `--real` mirrors the *leader* arm's pose onto the follower instead (`leader_port`, default `/dev/so101_leader`) — unrelated to this package's own follower connection, which always goes through `ros2_control`. See *Leader-driven mirroring on real hardware* above. |
| `launch/so101_arm_bridge.launch.py` | Picks `--fake`/`--real` from `hardware_interface` in `config.yaml` (or is forced by its own `source` launch argument) and launches the bridge under `node_name` - no hand edit needed. |
| `launch/so101_drivers.launch.py` | The drivers-container launch file (`config.yaml`'s `hardware.additional_driver_launch_file`): the leader-mirroring `so101_arm_bridge` instance and both `usb_cam` nodes - everything that opens a host device by its udev name. See *Cameras* and *Leader-driven mirroring on real hardware* above. |
| `launch/runtime.launch.xml` | The Agent/runtime-container launch file: the always-fake `so101_arm_bridge` instance backing `Wiggle SO101 Follower`, and `Verify Calibration`'s node - neither needs a host device. |
| `script/verify_calibration.py` | The node behind `Verify Calibration` (below). |
| `script/calibrate_so101.py` | Offline wrapper around LeRobot's own calibration, see *Calibrating a new arm* above. |

### Wiggle test: `Mirror SO101 Follower`

`so101_arm_bridge.py` runs unconditionally (`launch/runtime.launch.xml`
includes `launch/so101_arm_bridge.launch.py`) and publishes single-point
trajectories to `joint_trajectory_controller`. On mock, `Mirror SO101
Follower`'s `auto`-resolved instance is the **wiggle test**: each joint swings
`wiggle_amplitude_rad` (0.1 rad, about 6°, by default) either side of the pose
the arm is measured in when mirroring starts, read once from `/joint_states` —
the same joint-by-joint "is everything alive and moving the right way"
diagnostic as `lab_sim`'s. Raise `wiggle_amplitude_rad` for a livelier sim
demo; the amplitude is clamped to the URDF joint limits, so a joint already
parked on a limit is never commanded past it. On real hardware with
`hardware_interface: "real"`, that same `auto` instance instead reads the
leader's Feetech bus and mirrors its pose onto the follower — see
*Leader-driven mirroring on real hardware* above; if the leader is not
connected, the node exits rather than falling back to the wiggle (see
*Safe bring-up order* below, step 5). The follower's own `/joint_states` gate below still
applies, since mirroring should not start until the follower's own pose is
known, whichever source drives it. A second Objective, `Wiggle SO101
Follower`, ticks a second, always-`--fake` `so101_arm_bridge` instance
(`node_name`/`source` launch arguments in
`launch/so101_arm_bridge.launch.py`), so the joint-by-joint wiggle stays
runnable on real hardware — leader connected or not — even while `Mirror
SO101 Follower` is mirroring it.

Mirroring will not start until a **complete** `JointState` has arrived — one
carrying all six joints. A message naming only a subset is ignored rather than
partially applied, because a centre assembled from a partial pose would put
the missing joints at whatever the last full sample said. The sample must also
be fresh: older than `joint_states_timeout_s` (2.0 s by default) and the
`~/mirror` tick returns failure instead of centring on a stale pose, since a
stale sample means the broadcaster died or the bus went quiet.

The Objective's first step switches controllers - activating
`joint_trajectory_controller` and deactivating the two jog controllers - since
a Teleoperate jog mode can leave `joint_trajectory_controller` inactive, which
would otherwise make the bridge's publishes go nowhere. Mirroring is off until
the `Mirror SO101 Follower` Objective asks for it, and stops when the
Objective is stopped. The trajectory controller has one owner at a time: a
stream of topic messages restarts its trajectory on every tick,
so a plan's goal would be accepted and then hang forever, or abort on a path
tolerance the moving twin violated. The Objective keeps mirroring alive by
ticking the bridge's `~/mirror` `Trigger` service in a loop; one second
without a tick and the bridge goes quiet. That heartbeat gate is the primary
guard: **stop the Mirror Objective before planning or executing a motion.**
The bridge also skips a publish while it believes a `follow_joint_trajectory`
goal is live, but that is best-effort only — the flag is set from
`GoalStatusArray` messages, so a goal started while the Mirror Objective is
still publishing can lose the race with a 20 ms bridge tick.

### `Verify Calibration`

Run this after calibrating (*Calibrating a new arm* above), with the arm held
at its **URDF-zero
pose** — every joint at mid-travel; the gripper **NEAR-CLOSED**, which is URDF
zero for this gripper, not half-open. `verify_calibration.py` reads one
complete `/joint_states` message and reports each joint's Feetech `offset`
error in servo ticks to its own node log — the Objective log in the UI only
shows the preamble, so read the numbers from the runtime container:

```bash
docker logs moveit_pro_<instance>-runtime-1 2>&1 | grep -A 7 "Verify Calibration:"
```

where `<instance>` is the `--instance` name given to `moveit_pro run` (e.g.
`--instance so101` → `moveit_pro_so101-runtime-1`; `docker ps` lists the exact
name). `moveit_pro logs` opens the same output in a viewer window if a display
is handy. The Objective fails rather than reporting if no complete
`/joint_states` has arrived, or if the last one is older than
`joint_states_timeout_s` (2.0 s by default, the same gate the wiggle test
uses) — a stale sample means the broadcaster died or the bus went quiet, and
numbers from an old pose would look just like good ones. `feetech_ros2_driver`
reports
position as `(tick - offset) * 2*pi/4096`, so held at URDF zero the reported
radians *are* the offset error, and converting back to ticks
(`* 4096/(2*pi)`) gives exactly how far `config/so101_follower_calibration.yaml`
is wrong — the same formula *Bench procedure* step 2 uses for the manual
correction, printed next to the six numbers:
`offset += error_ticks` (round to a whole tick); raising `offset` lowers the
reported angle.

This also runs on mock, which is how the tick math is tested with no arm
attached: mock starts at `config/initial_positions.yaml` and reports it
verbatim, so the expected errors there are that pose's radians converted to
ticks (`test/test_verify_calibration.py`).

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

Follow this order every time a real arm is attached — the first time on a
newly assembled arm, and every time after. MoveIt Pro's Stop control is a
cooperative software stop, not a safety-rated emergency stop. Keep physical
power isolation accessible and clear the robot's workspace before any live
test.

1. **Assemble** the arm and, on mock (`so101_sim`), with no arm attached,
   confirm the twin appears and moves under `Mirror SO101 Follower`'s fake
   source — this is the same Objective the real arm uses later, so this step
   also proves the Objective itself works before hardware is in the loop.
2. **Install the udev rule** (bench step 1) so `/dev/so101_{follower,leader}`
   exist and are stable, then confirm `groups | grep dialout`. The calibration
   script's default ports are those symlinks, so this comes first.
3. **Calibrate**, one arm at a time, with `script/calibrate_so101.py`
   (*Calibrating a new arm* above). Support the arm on the bench first —
   calibration drops servo torque.
4. **Point the config at real hardware**: set `hardware_interface: "real"`
   under `urdf_params` in `config/config.yaml` (bench step 3), then run a
   uniquely-named `moveit_pro build all` — see the workspace docs on running
   from a worktree if more than one lane is building at once.
5. **First power-on.** Rest the arm on the bench, **power the servo bus before
   starting the instance** (an instance that starts against an unpowered bus
   leaves the hardware component uninitialized and cannot recover without a
   restart — bench step 4), **and have the leader plugged in and powered too**
   (its own 5 V supply, never the 12 V brick) — `so101_arm_bridge` opens
   `leader_port` at startup on `hardware_interface: "real"` and exits if it
   cannot, taking `Mirror SO101 Follower` down with it. Then start the
   instance. The arm is limp until the hardware component activates and stays
   rigid after that even once `moveit_pro down` stops the instance —
   `on_deactivate` never runs on a plain stop, so deactivate the component
   (`ros2 control set_hardware_component_state so101 inactive`) or cut bus
   power to make it limp again.
6. **Compare `/joint_states` against the real pose with `Verify Calibration`
   before commanding anything.** The six numbers land in the runtime
   container's log, not the Objective log in the UI:
   `docker logs moveit_pro_<instance>-runtime-1 2>&1 | grep -A 7 "Verify Calibration:"`
   (see *`Verify Calibration`* above). Read them one joint at a time — not a
   whole-arm glance — and confirm the gripper before you ever run
   `Close Gripper`. A wrong `offset` raises no error anywhere; the arm simply
   goes to the wrong pose, and for the gripper that is the jaw driving past its
   mechanical stop at the driver's hardcoded speed 2400. If a joint is off,
   correct it per bench step 2 and re-run `Verify Calibration` before
   proceeding.
7. **Run `Mirror SO101 Follower`.** With the leader plugged in, this mirrors
   the leader's pose onto the follower — move the leader gently and confirm
   every follower joint (including the gripper) turns the way the URDF says
   before planning or executing anything. Stop the Objective when done.
8. Confirm the wrist and scene camera panes show a live image, without
   commanding motion (*Cameras* above).
9. Only after explicit authorization, test bounded gripper, waypoint and jog
   motions in that order.

If a waypoint produces clicking, stop the attempt and inspect the physical joint
and its tracking error. Do not raise path tolerances to make a stalled joint look
like a success.

Uncalibrated STS3215 servos in position mode will not cross the 4095/0 tick
boundary — commanding a joint whose true path crosses that wrap point instead
drives it the long way around, or stalls it. This is why the calibration and
`Verify Calibration` steps above come before any commanded motion, not just
before planning: the boundary check requires the joint to already read the
right angle.

## Waypoints are not taught poses

`waypoints/so101_waypoints.yaml` holds poses picked to be reachable and visible
in simulation. None of them has been validated against a physical arm; re-teach
them on the bench before trusting any of them.

## Later phases

- **Phase four** — Trainer recording of demonstrations. Note that a named
  training config is not a workspace file: the Trainer stores them as JSON under
  its own data directory, so `RecordEpisode(config_name="so101_sim")` fails with
  `Training config 'so101_sim' not found` until one is created for this
  deployment through the Trainer UI or its REST API.
