# reBot bench base configuration

The shared configuration for the Seeed reBot Arm 102 / **B601-RS** owns the
URDF/Xacro, SRDF, meshes, controllers, Objectives, waypoints and optional real
wrist D435 and scene OAK-D cameras. It follows the workspace's SO-101
base/sim layout. For simulation select [`rebot_bench_sim`](../rebot_bench_sim/README.md):

```bash
moveit_pro run --config-package rebot_bench_sim
```

The arm defaults to `hardware_interface: mock`, using only
`mock_components/GenericSystem`. Setting it to `real` selects the RobStride
`ros2_control` driver and drives the seven motors over SocketCAN - see
[Real hardware](#real-hardware). `mujoco` is `rebot_bench_sim`'s selector: this
description then emits no `ros2_control` block and no colour optical frames,
which the sim's MuJoCo hardware supplies. Any other value fails Xacro expansion
rather than producing a description with no hardware plugin.
Cameras in this base package default on: each attached supported camera starts
and a missing one is skipped (see [Optional real cameras](#optional-real-cameras)).
The sim is a MuJoCo twin of the bench: it renders both cameras and starts no
camera drivers.
The `world_to_base_xyz`/`_rpy` and `scene_camera_xyz`/`_rpy` xacro arguments
place `base_link` and `scene_camera_link` in `world`. `config/config.yaml` sets
them to this bench's calibration (see [the stack recipe](#sam3-blue-on-red-stack));
only the xacro fallback, when they are unset, has both coincide with `world`.
`wrist_camera_mount_xyz`/`_rpy` set the `d435_mount_joint` origin on
`gripper_end`; unset, it is the nominal mount, which the real configuration
keeps. The sim places all of them from its bench layout file instead, over the
inherited calibration.

## Try it

Build this workspace once with the normal MoveIt Pro build/install workflow.

First-time setup on your reBot, in `config/config.yaml`:

1. Find the Star Arm's serial port: run `ls -l /dev/serial/by-id/` with the
   Star Arm unplugged, plug in only the Star Arm, and run it again. The new
   entry is its port. The default `rebot_leader_port` matches the stock CH340
   adapter (`usb-1a86_USB_Serial-if00-port0`).
2. Set `ros_global_params.rebot_leader_port` to that path and
   `ros_global_params.rebot_leader_enabled: true`. The same setting works with
   mock, real, and the `rebot_bench_sim` overlay.
3. For the real follower, set `hardware_interface: "real"` under
   `hardware.robot_description.urdf_params` and check `can_interface`; see
   [Real hardware](#real-hardware). The default `"mock"` needs neither.
4. Cameras are optional and configured independently in `config/cameras.yaml`;
   see [Optional real cameras](#optional-real-cameras).
5. On another installation, re-measure the bench placements; see
   [Bench calibration](#bench-calibration).

Then start the Desktop App and backend:

```bash
moveit_pro_desktop
moveit_pro run -c rebot_bench_base_config
```

In the Desktop App, connect to this backend and run **Mirror reBot Leader**.
The backend starts and stops its bridge and single USB reader automatically.
The default arm selector remains `hardware_interface: "mock"`; for the MuJoCo
twin, use `moveit_pro run -c rebot_bench_sim`. Real startup engages torque by
default and must begin at HOME; see [Real hardware](#real-hardware).


Run **Move reBot to Waypoint** with `Raised`, then `Zeroed Rest`. Both are saved
waypoints and SRDF named states; startup is `Zeroed Rest` (all seven motor
coordinates zero). The raised arm pose is `[0, 0.65, 0.85, 0.3, 0, 0]` radians.
`Open Gripper` and `Close Gripper` override the standard teleoperation Objectives,
using `Gripper Open` and `Gripper Closed`. One trajectory controller,
MoveIt Pro's `joint_trajectory_admittance_controller`, owns all seven joints through
the `manipulator_and_gripper` group and accepts partial goals for the arm and gripper
groups. It has no force/torque sensor configured, so it runs without admittance.

For proportional gripper control, open **Teleoperation → Joint → Teleoperation
Settings**, select **gripper**, and start **Teleoperate**. Wait for the controller
to activate, then hold J7's increment/decrement buttons; release to stop at an
intermediate opening. The group contains one commanded joint, bounded to
0–310°; the finger sliders remain dependent mimics. Select **manipulator** to
return to arm joint jogging. Pose jogging always uses the six-joint arm group.
Use one connected teleoperation UI at a time. Two UIs selecting different groups
can publish conflicting feedback, repeatedly switch controllers, and make jog
motion crawl or stop. Close the other UI and reload the remaining one before
checking controller readiness again.

Open/Close Gripper remain endpoint shortcuts. This configuration adds bounded
velocity jogging; exact-angle slider and interactive-marker execution are not
covered by this validation.

### FashionStar leader

The YAML setting `ros_global_params.rebot_leader_publish_rate` defaults to
100 Hz, matching the driver, and accepts 20–100 Hz. Change it before startup
at HOME; the backend passes it to the bridge's `publish_rate` ROS parameter.
Each tick checks the latest target and collision path; there is no interpolation,
velocity cap, acceleration cap, or position filter. Audible motor noise at this
rate needs hardware verification.

Jaw stiffness `jaw_kp` (12 Nm/rad) and damping `jaw_kd` (0.5 Nm·s/rad) are
Xacro arguments, set like `arm_control_mode` under `urdf_params` in
`config/config.yaml` (see [Motors, limits and gains](#motors-limits-and-gains)).
`config/rebot_leader_tuning.yaml` exposes the jaw motor torque limit (1 Nm) and
J4 motor speed limit (33 rad/s, matching the other RS00 wrist motors). The jaw is an
impedance position servo, not contact detection or force control. The firmware torque-limit register requests a motor torque bound; transient
feedback can exceed it. It is not a calibrated fingertip-force limit. Lower torque reduces grip;
more damping reduces fast-open bounce. Keep stiffness responsive. This
configuration ships `arm_control_mode: "position_csp"`, so J1–J6 ignore their GPIO
kp/kd settings and each tracks at up to its stored speed limit (0x7017); J4's
factory 1 rad/s limit caused lag even though the bridge was uncapped.
Changing J4's speed limit changes physical response and requires commissioning.

Inspect the exact proposed writes without contacting hardware:

```bash
ros2 run rebot_bench_base_config rebot_leader_tune.py
```

After explicit physical commissioning approval, at healthy HOME with every motor
OFF, add `--apply`. It sends volatile settings through the owning driver's service;
it never sends a save/store, changes mode, enables a joint, or moves the robot.
The service ACK confirms transmission, not firmware acceptance: independently read
back motor registers 0x700B (J7) and 0x7017 (J4) before enabling. Nothing may enable or command the arm until both writes and readbacks finish. On any failure,
leave the arm OFF; partial writes are possible. Power-cycle retention of these
settings has not been verified; re-read the motor registers before each session,
including after a power cycle. This tool is
never called implicitly by launch or the Objective. Reapply only by explicit
command after a driver restart. The MuJoCo jaw actuator mirrors kp=12, but keeps
kd=0.05 and the RS00's ±14 Nm range: at kd 0.5 a twin running below real time lags Close Gripper past
the then-configured 0.15 rad path tolerance (now 0.16 rad).
It does not model firmware CSP limits or the physical jaw servo; its path proof cannot validate grip force,
bounce, or motor sound. Those require a separately approved hardware test.


The workspace installs the FashionStar SDK used by Seeed’s Star Arm 102 leader
into the Python used by ROS; [dependency provenance](docs/leader-dependencies.md)
records the pinned SDK and license. The existing bench calibration for
`rebot_arm_102_leader` is installed at `config/leader_calibration/rebot_arm_102_leader.json`.
The YAML `rebot_leader_calibration_file` resolves relative to this package's share
folder (or accepts a workspace path). Keep your existing calibrated file and
set this value and `rebot_leader_port` for another device; startup never
calibrates or rewrites it. The file cannot prove the servos' stored origins, so
it does not gate following: the engage check below does. With the leader
disabled, no USB reader or leader bridge starts. Real thermal recovery starts
independently.

Wait for `Native collision scene ready`, then run **Mirror reBot Leader**.
The Objective requires fresh measured HOME (every arm joint within 0.08 rad of
zero), explicitly prepares a new session, and switches the seven position
interfaces to `leader_position_controller`. The bridge seeds its reference from
measured positions and holds that fixed HOME reference until following engages.
Following engages only if the mapped leader reads within 0.1 rad of the
measured follower on arm joints joint1-joint6, and within 0.3 rad on gripper
joint7. The gripper limit allows about 2.4 mm of travel per finger and about
2.9 deg of Star Arm trigger travel through its ×6 mapping. Otherwise it refuses,
reports each joint by its operator name and number, its offset in degrees and radians,
and which way to move that Star Arm joint to match reBot. The Objective shows
this reason in its UI failure message, including the arm's 5.7 deg / 0.1 rad
limit and the gripper's 17.2 deg / 0.3 rad limit in reBot joint units. Gripper
corrections are expressed in Star Arm trigger degrees. It requests
**Return reBot Leader Home**; align the leader and run the Objective
again. There is no catch-up, ramp-in, or re-synchronisation during following.
Once engaged, it forwards fresh leader positions directly, without software
velocity or acceleration caps. Existing motor limits remain unchanged.

Native MoveIt Pro FCL checks each target and the measured-to-target joint path
at at most 0.005 rad intervals on joint1-joint6 and 0.05 rad on joint7, whose
0.008 m/rad mimic moves each finger 0.4 mm per sample, using the robot's collision meshes, SRDF allowed
collision matrix, and monitored planning-scene objects. Extra padding is zero,
matching this configuration's waypoint Objectives. An unsupported octomap
refuses following. These sampled straight paths do not model the motor's
transient response or provide continuous collision checking.

The mapping is absolute, with no additional zero offsets:

| Leader coordinate | Follower coordinate | Direction / scale |
| --- | --- | --- |
| shoulder_pan | joint1 | +1 |
| shoulder_lift | joint2 | +1 |
| elbow_flex | joint3 | −1 |
| wrist_flex | joint4 | −1 |
| wrist_yaw | joint5 | −1 |
| wrist_roll | joint6 | +1 |
| gripper | joint7 | +6 |

The stored-origin calibrated leader angles are unwrapped, mapped to motor degrees, bounded
to the approved motor ranges, and converted to radians. Raw SDK reads avoid
the plugin's monitor deadband and stale-pose fallback. An incomplete or failed
read is rejected; an old pose is never replayed as a fresh sample. The backend-owned reader sends JSON lines to its bridge over a private pipe.
Neither process writes stored follower parameters or recalibrates the leader.

Leader targets retain the approved full ranges; J2 and J3 leader commands have
a zero lower bound at HOME. Measured-position holds use the same command
bounds, so noisy feedback cannot create a negative HOME-side reference.

The reader starts only with the backend and never reopens its port. If it exits
(Star Arm unplugged or unpowered at startup, wrong `rebot_leader_port`, or a
rejected calibration file), Mirror refuses with that cause. If the Star Arm USB
is unplugged while it runs, every later read fails, even with the cable back,
and Mirror refuses with the reader's read error. Either way, reconnect the Star
Arm and, once reBot is at HOME, restart the backend.

Stopping the Objective, calling `/rebot_leader_bridge/clutch`, losing the USB
stream, or receiving stale/invalid data latches following off and requests
**Return reBot Leader Home**. So does a prepared session that has not started
following within 3 s. On the real RobStride model, so does any
tick whose Robstride status is 0.5 s or more old, has a non-zero `comm_state` or
any fault bit (the driver sets both on a reported motor fault), or does not list
exactly seven motors, all enabled: the check runs before each target is sent, and
healthy or reconnected feedback afterwards does not re-engage following. A motor
that stops answering on the CAN bus changes none of these fields, so the bridge
does not detect it; the driver's `freeze_on_joint_loss` freezes the arm instead,
and the HOME return cannot move it until separate hardware and driver recovery.
Stopping the backend mid-session also makes the bridge request
**Return reBot Leader Home**; it then ignores further stop signals until that
return finishes, for up to 120 s. The Objective server shuts down at the same
time, so do not rely on it: return reBot HOME before stopping the backend (see
[Start, home and stop](#start-home-and-stop)). The HOME return centres wrist yaw before moving to
`Zeroed Rest`, at 0.05 rad/s maximum, preserving the jaw. An arm within the same
0.08 rad of HOME that preparation accepts is left where it is. Otherwise both
legs are planned with `PlanToJointGoalFromBoundedStart`, which is `PlanToJointGoal`
(collision-checked and free to detour) from the measured arm clamped into its
joint bounds, and anchored to that clamped command. If that planner refuses the
rest leg, **reBot Move to Arm Waypoint** falls back to a point-to-point move from
the clamped command, collision-checked as planned and again after anchoring. The
yaw leg has no fallback: a refused yaw plan fails the return. A joint sagged past a limit therefore starts the return at the
limit: planned from the measurement, the leg would start past it, and a sag deep
enough to reach self-collision (joint3 at about −0.08 rad) would be refused. HOME recovery restores the normal
`joint_trajectory_admittance_controller` and retains motor torque, leaving the arm
ready for other Objectives at HOME. Normal stop/cancel never disables motors.
Driver-owned thermal detection and backend recovery run automatically; see
[Thermal protection](#thermal-protection).
A failed HOME return is
reported and requires operator recovery. Only starting the Objective again at
measured HOME re-engages; reconnecting USB never does, and after a Star Arm
USB unplug the backend must be restarted first.

The bridge identifies mock, real, or MuJoCo from the robot description. Real
sessions require fresh healthy status from exactly seven enabled motors; there
is no second mode-enable switch. Startup torque remains true by default. Normal
Stop retains torque after HOME. Disable or change hardware lifecycle only at
proven physical and measured HOME. A motor fault requires separate recovery
before further following. The Objective runs until stopped and has no software
speed guard; fast leader changes can command fast motion.

The dedicated gripper JointVelocityController uses position commands with
3.0 rad/s velocity and 6.0 rad/s² acceleration limits. It starts inactive and
participates in managed controller switching with the seven-joint trajectory
controller. Disjoint arm controllers may remain active during gripper jogging;
only one active controller claims each command interface.

The package's **Teleoperate** Objective sets additional collision padding to
zero, matching the waypoint Objectives, and routes its trajectory execution
through `joint_trajectory_admittance_controller`. Mesh collision checking stays enabled.
The inherited 10 mm padding reports false collisions between nearby links on
this compact model and prevents Cartesian jogging even at the saved raised
pose. This is a mock-model policy, not a validated hardware clearance setting.

The base owns the robot configuration and standard runtime-services launch.
The sim overlay inherits it through `based_on_package`, following SO-101 conventions.

## Source and license

The original URDF and STL meshes come from
[Seeed's B601-RS mechanical description](https://github.com/Seeed-Projects/reBot-DevArm/tree/58f9e433deec2ba118c5c22d60c07b9897e92acc/Rebot_Arm_description/RS),
commit `58f9e433deec2ba118c5c22d60c07b9897e92acc`, under **CERN-OHL-W-2.0**.
The modified URDF retains that license. Configuration and tests are BSD-3-Clause.
[description/NOTICE.md](description/NOTICE.md) records retained assets,
modifications, attribution, source locations and the full license location.

## Printed visuals

Nine supplied DAE assets replace the corresponding printed trim and add the
D435 wrist bracket. DAE coordinates are metres, mesh scale is `1 1 1`, and the
purple/white materials are embedded (no URDF material override). Metal/motor
visuals, collision geometry and inertias are unchanged. Internal black plastic
is retained in triangle subsets after removing the superseded side panels and
badges; the original upstream files remain available for provenance.

| Assets | Visual owner | Source-to-link xyz (m) | Source-to-link rpy (rad) |
| --- | --- | --- | --- |
| 1-DOWN-DL/DR, 1-COVER-upper | link2 | -0.020, 0.145, -0.02625 | 0, 0, π |
| 1-UP-DL/DR | link3 | 0.216, 0.145, -0.02625 | 0, 0, π |
| 1-COVER-lower | link3 | 0.020, 0.145, -0.02625 | π, 0, 0 |
| 1-STOPPER-1 | link5 | 0, 0, 0.048 | -π/2, 0, π |
| 1-RAIL-BASE-2 | gripper_end | -0.25730933, 0, -0.1445 | π, 0, -π/2 |
| D435_mount | d435_mount_link | 0, 0, 0 | 0, 0, 0 |

Trim transforms match the retained filler contact surfaces and cover mounting
frames. STOPPER and RAIL-BASE fit the original green visual surfaces within
0.060 mm and 0.107 mm, respectively (different tessellations). They retain
those original physical link owners: the rail support is fixed, not a moving
finger. The bracket preserves the upstream reference's 20.4 mm clamp gap and
3.3 mm bolt holes, supporting the published wrist bracket visual transform.
The link3 panels are swapped left/right by a 180° turn about the common
assembly Y axis through `(0.098, 0.218746, 0)` m. This puts MoveIt Pro on the
opposite face and preserves readable lettering; rotating each panel in place
would invert its lettering and misalign its asymmetric seating details. The
lower cover keeps its original mounting transform.

The D435 body is the **Apache-2.0** mesh from
[realsense2_description](https://github.com/realsenseai/realsense-ros/tree/9215f26e8348ad5922a608b77882a9bfe05940f0/realsense2_description),
revision `9215f26e8348ad5922a608b77882a9bfe05940f0`, decimated from 15.8 MB to
1.7 MB (same origin, scale and material; surface within 0.75 mm of the original).
It uses Seeed's published D435 assembly pose on the purple bracket. The
following fixed transforms attach the existing camera frames to the body:

| Parent → child | xyz (m) | rpy (rad) |
| --- | --- | --- |
| gripper_end → d435_mount_link | -0.1201, 0.0003, 0.045 | 1.5827, 0.0024, 1.5515 |
| d435_mount_link → d435_bottom_screw_frame | -0.0003, 0.0445, -0.0095 | 0.0126, -1.0489, -1.5981 |
| d435_bottom_screw_frame → d435_link | 0.0106, 0.0175, 0.0125 | 0, 0, 0 |
| d435_link → d435_color_optical_frame | 0, 0.015, 0 | -π/2, 0, -π/2 |

The mesh's visual origin relative to `d435_link` is `(0.0043, -0.0175, 0)` m
with RPY `(π/2, 0, π/2)`, as in Intel's description. These are nominal CAD
assembly and RGB optical offsets, not a measured camera-to-arm calibration.
The body and bracket are visual-only; arm collisions and inertias are unchanged.

The two STEP-derived parts come from [Seeed's printed-part sources](https://github.com/Seeed-Projects/reBot-DevArm/tree/171ed82cb9a4a037a6beb7d1e034352baa604b63/hardware/reBot_B601_RS/3D_Printed_Parts)
and are redistributed under **CERN-OHL-W-2.0**, retaining attribution and the
license in `description/LICENSE`. The six redesigned trims and supplied bracket
are hardware-design derivatives under the same license. See
[description/NOTICE.md](description/NOTICE.md) and the
[asset manifest](description/assets/printed/manifest.json) for source and
modification details. Brand artwork identifies the visuals, without implying
endorsement or granting trademark rights.

## Model preview

These are offline renders of the package visual geometry at its named poses,
not MoveIt Pro UI screenshots or evidence of trajectory execution.

| Zeroed Rest | Raised |
| --- | --- |
| ![Zeroed rest](docs/zeroed-rest.png) | ![Raised](docs/raised.png) |

## Coordinates and limits

`joint1` through `joint6` preserve Seeed's URDF zero, origin and axis. Their
coordinate mapping is **q_URDF = q_motor**, in radians, with zero offset.
This agrees with Seeed's
[RS configuration](https://github.com/Seeed-Projects/reBotArm_control_py/blob/6415d43130d1e143c70dc106096a857ac5556f81/config/rebotarm_rs.yaml)
and controller, which pair motor IDs 1–6 with URDF `joint1`–`joint6` directly.
The negative axes on joints 1, 3, 4, 5 and 6 already encode their directions;
do not negate those coordinates a second time.

The [LeRobot follower configuration](https://github.com/Seeed-Projects/lerobot-robot-seeed-b601/blob/0da1a1a5575905d77823227003ffee7672059780/lerobot_robot_seeed_b601/config_seeed_b601_rs_follower.py)
uses `joint_directions` to map **leader actions to motor targets**. Those signs
and the gripper factor 6 are not the motor-to-URDF mapping. The optional leader
reader uses that external plugin; follower control stays in the native ROS controllers.

| Motor / coordinate | Meaning | Approved motor range (degrees) | Stored motor speed (rad/s) |
| --- | --- | --- | --- |
| J1 / `joint1` | Pan | −145 … 145 | 10 |
| J2 / `joint2` | Shoulder lift | −1.146 … 203 | 10 |
| J3 / `joint3` | Elbow | −0.573 … 239 | 10 |
| J4 / `joint4` | Wrist flex | −106 … 86 | 1 |
| J5 / `joint5` | Wrist yaw | −101 … 108 | 33 |
| J6 / `joint6` | Wrist roll | −175 … 175 | 33 |
| J7 / `joint7` | Gripper motor, positive opens | 0 … 310 | 33 |

These approved motor-frame ranges replace upstream's model limits; some are
wider and some narrower.
The exact J2/J3 lower bounds are −0.02/−0.01 rad; the degree values above are
rounded. J7 and both dependent jaw sliders retain zero lower bounds.
[Lower-limit commissioning](docs/lower-limit-commissioning.md) records the
observations and uncertainty behind these home margins.
**Planning limits are 0.6 rad/s and 1.2 rad/s² on joint1-joint6**, twice the measured
teleop limits (0.3 rad/s, 0.6 rad/s²), **and 3.0 rad/s and 6.0 rad/s² on the gripper
(joint7)**, its measured teleop limits, all below the stored motor speeds. The jaw is not
doubled: at 6.0 rad/s it lagged its command by about 0.151 rad. A later
measured jaw lag of 0.1514 rad also exceeded the former 0.15 rad allowance.
`default_path_tolerance` is now 0.16 rad for all seven joints, including the arm;
the jaw planning speed remains 3.0 rad/s. Parameter readback confirmed 0.16 rad,
but subsequent physical execution at that setting has not been verified.
The arm and the twin use the same planning limits. Change those limits in `config/moveit/joint_limits.yaml` and keep the URDF `velocity` limits in
`description/rebot.urdf.xacro` equal (the finger mimics at 0.008 times joint7's); the jog
controllers in `config/control/rebot.ros2_control.yaml` keep the measured teleop limits,
and its trajectory controller brakes a canceled goal at those limits too
(`stop_accelerations`, 0.6 rad/s² on the arm), not the faster planning ones. Size the
bench speed guard from the planned peak (see [Speed settings](#sam3-blue-on-red-stack)).
**Close Gripper**, which **Pick Blue Cube** runs first, plans at the full limits; the
other waypoint Objectives default to 50% scaling (0.3 rad/s, 0.6 rad/s²).

J7 is an explicitly bounded revolute coordinate (0 … 5.410520681182 rad) on a
geometry-free link. The original fingers remain prismatic joints, both mimicking
J7 at **0.008 m/rad**. This is derived from the
[Seeed BOM's module-1, 16-tooth pinion](https://github.com/Seeed-Projects/reBot-DevArm/blob/58f9e433deec2ba118c5c22d60c07b9897e92acc/hardware/reBot_B601_RS/README.md):
pitch radius = module × teeth / 2 = 8 mm. The sliders retain their opposed
upstream axes. At 310° each travels 43.284 mm, for 86.568 mm total additional
opening. This transmission is a model derivation, not a measured jaw calibration.

The SRDF permits link2/link5 contact in the folded arm, alongside the existing
adjacent assembly and opposing gripper-jaw exclusions. All other movable-link
pairs retain collision checks. In particular, `gripper_end` is rigidly attached to
`link6` across J6 from `link5`; their meshes have a 9.0 mm axial separating gap
that J6 rotation cannot close. Default 10 mm planning padding otherwise makes
this mount report a collision. The coupled jaws approach one another at closure
and separate on opening. Tests re-enable both pairs for unpadded mesh checks
along the saved motions. The package Objectives, including the leader
Objectives, explicitly use **zero additional link padding** with these SRDF
exclusions. The stack Objectives have run on the real arm with zero padding,
with the scene camera and base pose calibrated with ChArUco
(see [bench calibration](#bench-calibration)), but before link2/link5 contact
was allowed (see [SAM3 blue-on-red stack](#sam3-blue-on-red-stack)). Remaining pairs retain the
original collision meshes. No extra clearance margin is claimed. The table
surface is protected by the bench box that **reBot Stack Scene** adds to the
planning scene, with its top at the mat, not by padding. The wrist camera mount
is still provisional.

## Optional real cameras

Cameras default **on** and are independently optional. The backend checks USB
descriptors and skips each missing or unsupported camera with a warning; neither
camera is required for backend startup, the leader, or Mirror. Out of the box,
the wrist driver supports an Intel RealSense D435 and the scene driver a Luxonis
OAK-D. The arm's `hardware_interface` selection is independent of the cameras.
These camera settings apply only when launching the base package; the sim overlay
does not invoke this camera launch file.
The persistent backend launch reads [config/cameras.yaml](config/cameras.yaml),
so the normal two-command startup also starts attached supported cameras. Both
serial selectors are blank by default and use the first matching device. Set
`wrist_serial_no` or `scene_serial_no` to select a particular supported camera.
Set `enable_wrist_camera: false` or `enable_scene_camera: false` independently,
or `enable_cameras: false` to disable both. If the D435 disconnects mid-run, the
backend stops its driver rather than let librealsense retry and log without end;
reconnect it and restart the backend to use it again.

Other camera models need their own ROS driver; configure it to publish `Image`
and `CameraInfo` topics in the backend's ROS domain, select its image topic in
the Desktop App, and disable the corresponding built-in camera. Do not start a
second driver for a device already owned by another publisher.
For a cameras-only ROS launch in a sourced workspace:

```bash
ros2 launch rebot_bench_base_config cameras.launch.py enable_cameras:=true
```

Every setting in the YAML is also a launch argument, for example
`image_width:=640 wrist_image_height:=480 scene_image_height:=400 framerate:=30`.
Stop this standalone launch
before enabling cameras in MoveIt Pro; each device must have one driver owner.

| Role | Device | RGB image | Camera info | Optical frame |
| --- | --- | --- | --- | --- |
| Wrist | Intel RealSense D435 (`8086:0b07`) | `/wrist_mounted_camera/color/image_raw` | `/wrist_mounted_camera/color/camera_info` | `d435_color_optical_frame` |
| Scene | Luxonis OAK-D-PRO-W-97 | `/scene_camera/color/image_raw` | `/scene_camera/color/camera_info` | `scene_camera_color_optical_frame` |

The wrist defaults to **640×480** and the scene to **640×400**, both at 30 fps.
The wrist is RGB only, depth off; the scene camera also runs aligned depth (see
below). The heights differ on purpose; see the scene camera's field
of view below. These stable
`sensor_msgs/Image` and `CameraInfo` topics support the UI and later recording
for policies such as pi0.5 (which resizes images to 224×224). Image and calibration topics follow the sibling configs' naming convention.
Recording needs both RGB topics at about 30 Hz, and the episode cycle's stack
check needs `/scene_camera/depth/color/points`; check all three with
`ros2 topic hz` before a run.
The wrist image encoding is `rgb8`; the scene encoding is `bgr8`, matching
the DepthAI v2 MJPEG decoder's OpenCV output. Recording consumers should honor
`Image.encoding` (for example, request `rgb8` from cv_bridge) when converting
either stream to an RGB tensor.

Both cameras intentionally use **USB 2 (480 Mbps)** for long cables. The D435
uses its supported 640×480/30 color mode, with depth, infrared, and IMU disabled.
The OAK uses the standard Jazzy `depthai_ros_driver` (DepthAI v2): CAM_A's OV9782
color sensor, USB speed `HIGH`, and **on-device MJPEG**
(quality 95) before USB transfer. It takes the **ISP output** at 640×400 rather than
the video path's crop, then encodes; the driver decodes to `bgr8` ROS Images on the
host. The BNO086 IMU and IR illumination are disabled.

`scene_aligned_depth: true` (the default) also runs CAM_B/C's OV9282 stereo pair
as an RGBD pipeline, with depth aligned to CAM_A at the colour stream's 640×400:
the depth image on `/scene_camera/depth/image_rect_raw` (with
`/scene_camera/depth/camera_info`), in `scene_camera_color_optical_frame`, so it
should line up pixel for pixel with the colour image. A `depth_image_proc`
`point_cloud_xyz_node` turns it into `/scene_camera/depth/color/points`, which
**Stack and Unstack Cycle** uses to confirm each stack. `scene_depth_scale`
defaults to 1.0, which publishes the depth unchanged. Any other value multiplies
every depth (the `scale_depth.py` node, between the depth image and the point
cloud) before the cloud is built. Measure it for your own camera: find the
factor along the camera rays that puts the empty mat at z=0, then check a known
height. The original bench's OAK read 4% long; measured on 2026-10-13 in the
calibrated `world` frame, 0.96 moved its mat from a median of -24 mm to 0 mm
and a 100 mm blue-on-red stack's top from 80 mm to 100 mm. The colour stream's
settings are the same either way. To run colour only, set
`scene_aligned_depth: false` in the YAML or pass `scene_aligned_depth:=false`;
the OAK then runs its RGB-only pipeline with the stereo pair off, and the cycle
cannot confirm a stack. Change `scene_mjpeg_quality` in the same YAML to adjust compression.
ISP luma/chroma denoising and sharpening are disabled to preserve detail.
Allow a few seconds after startup for the scene camera's image to settle before
recording; the first frames can look heavily processed.
Resolution changes must be supported by both devices and the OAK video encoder
(including its width alignment).

### Scene camera field of view: state the ISP ratio, or lose most of it

Measured on the bench, not inferred from the datasheet. The OAK's sensor is
1280×800 and sees 97.1° horizontally. Which part of that reaches ROS depends
entirely on how the 640-wide output is produced:

| Scene output path | Sensor region used | Horizontal FOV | Published `fx` vs truth |
| --- | --- | --- | --- |
| **ISP output, explicit 1/2 (default)** | **all of it, offset (0,0)** | **97.1°** | **correct** |
| ISP output, driver's default ratio | central 960×600 | 80.7° | low by ×1.333 |
| video path crop at 640×480 | central 640×480 | 59.1° | low by ×2 |

The ratio is the whole point. `color.i_set_isp_scale` alone lets the ISP pick
2/3 and then centre-crop to the requested size, so `color.i_isp_num: 1` and
`color.i_isp_den: 2` are set explicitly: the full sensor then maps to 640×400 at
offset (0, 0), confirmed by image registration against a native 1280×800
capture, at 30.0 Hz on the same on-device MJPEG path.

Because that is a true downscale, the driver's width-proportional intrinsics are
right: published `K` and `P` come out as exactly the native values halved, `P`
is `[K | 0]`, `R` is identity, and `D` is unchanged from native. **No
`CameraInfo` correction is needed anywhere**, which is why none is shipped.

The wrist D435's intrinsics are likewise self-consistent (`fx` 607.84 at
640×480, a 55.5° view, which is what a 4:3 crop of its 69.4° 16:9 color sensor
gives).

Intrinsics being correct does not make a hand-eye result correct: where each
camera *is* is a separate calibration, covered [below](#bench-calibration).

### Achieved frame rate depends on the DDS transport

A raw 640×480 `rgb8` stream is ~0.92 MB per message and ~27 MB/s at 30 fps, which
is large enough that the transport, not the camera, decides the delivered rate.
Measured: under Fast DDS with built-in defaults the wrist stream delivered well
under 30 Hz with losses, and raising its shared-memory segment to 8 MB alone did
not fix it; under CycloneDDS with MoveIt Pro's generated configuration the same
stream ran at 29.979 Hz with no loss. That configuration sets
`SocketReceiveBufferSize min="10MB"`, which is what the raw stream needs, and
CycloneDDS is the deployment's default middleware. If you run these cameras
under a different transport, or with hand-written DDS settings, **measure the
delivered rate at the subscriber** rather than trusting the configured 30 fps —
a silently degraded rate is the failure mode that bites a recording run.
`docs/camera-acceptance.json` records its rates under `rmw_fastrtps_cpp` with an
8 MB segment, and its scene entry was captured at the superseded 640×480 crop,
so its scene intrinsics do not describe the current default.

Install the declared ROS dependencies in the runtime environment:

```bash
sudo apt install ros-jazzy-realsense2-camera ros-jazzy-depthai-ros-driver \
  ros-jazzy-depth-image-proc ros-jazzy-compressed-image-transport
```

The launch uses the drivers process, which has access to devices. Containers
need **all** of the D435's `/dev/video*` nodes, including its depth interface
even though depth streaming is off: librealsense uses that interface to discover
the device's base stream. Check the nodes' USB parent in `/sys/class/video4linux/`;
the `/dev/v4l/by-id/` symlinks alone may omit some interfaces. USB-bus access
must also allow device re-enumeration because the OAK boots over USB.
These are camera dependencies only.

**The scene camera is calibrated for this bench; the wrist camera is not.**
`config/config.yaml` places `scene_camera_link` from the 2026-10-11 ChArUco
calibration (see [bench calibration](#bench-calibration)), with the standard
ROS optical-axis rotation; the xacro alone leaves it at `world`. The wrist D435
body and optical frame follow the nominal assembly described above, not a
measured mount: the provisional calibrated mount in `rebot_bench_sim`'s
`config/bench_layout.yaml` is not applied on the real arm. Camera-driver TF publication stays disabled so the robot
description owns these frames. Re-measure the scene camera after moving it, and
measure the wrist mount before wrist-camera perception or image-based motion.
Neither camera has collision geometry yet.

### Camera validation

The imported camera implementation was previously tested with both cameras
connected at 480 Mbps; a simultaneous 65-second subscriber
check measured **30.06 Hz wrist** and **30.00 Hz scene**, both 640×480 with
matching `CameraInfo` dimensions and the optical frame IDs above. Tested with
Jazzy `realsense2_camera` 4.58.1 / librealsense 2.58.1 and
`depthai_ros_driver` 2.12.2 / DepthAI 2.31.1. The default camera-off launch
exited successfully without starting either driver. `robot_state_publisher`
loaded the URDF and published the expected fixed camera transforms.
See [camera acceptance results](docs/camera-acceptance.json). These checks
predate the scene camera's aligned depth, which none of them exercised.

The isolated test container used an 8 MiB Fast DDS shared-memory transport
segment. Its default segment dropped full image messages at the subscriber
while both `CameraInfo` streams still arrived at 30 Hz. When checking rates,
measure the image topics themselves and size the ROS transport buffers for
640×480×3-byte images; a camera's configured fps alone is not a delivery check.
All camera drivers were stopped after testing.

## Real hardware

> The arm has **no brakes**. Torque may only be released at the home (zero)
> pose; releasing it anywhere else drops the arm onto whatever is below it.
> Everything in this section follows from that one fact.

### Selecting it

Set the arm selector in `config/config.yaml`:

```yaml
    urdf_params:
      - hardware_interface: "real"
      - can_interface: "can0"
      - torque_enable: "true"
```

`torque_enable: "false"` brings the arm up readable but limp: all seven joints
publish state and can be moved by hand, and nothing is commanded. Use it for the
first session on an arm whose zero, signs and wiring have not been confirmed
through this config, then switch to `"true"`. `rebot_bench_sim` pins
`hardware_interface: "mujoco"` and is unaffected by any of this.

### Host CAN bring-up

The motors run at 1 Mbps. Bring the interface up on the **host**, before
starting MoveIt Pro, and again after replugging the CAN adapter:

```bash
sudo ip link set can0 down 2>/dev/null || true
sudo ip link set can0 type can bitrate 1000000 restart-ms 100
sudo ip link set can0 up
sudo ip link set can0 txqueuelen 1000
ip -details -statistics link show can0
```

`txqueuelen` is not optional. Activation sends a burst of several frames per
motor and the kernel default of 10 drops most of it. The driver ships
`scripts/setup_can.sh`, which does the same thing and tolerates adapters without
bus-off auto-recovery.

### Start the Desktop App and backend

With the arm physically at HOME, power and CAN ready, and existing calibration
available, use the same two commands as mock:

```bash
moveit_pro_desktop
moveit_pro run -c rebot_bench_base_config
```

Then run **Mirror reBot Leader** in the Desktop App. Enable the leader with the
same `rebot_leader_enabled` YAML setting; the backend owns its full lifecycle.

**No `--instance`.** SocketCAN interfaces are network-namespace scoped, and
`can0` is not a `/dev` device, so bind-mounting `/dev` does not help. An
instanced deployment gets its own network namespace in which host `can0` does
not exist, and the driver then fails to open the bus. The default deployment
keeps `network_mode: host`, where `can0` is visible. One consequence: a real
deployment cannot share a host with an instanced deployment of the same name.

### Start, home and stop

1. **Check the pose before starting anything.** Activation sends a Stop frame to
   every motor as a position probe, and that Stop also clears torque left over
   from a previous run - so bringing the stack up is itself a torque-off event
   for all seven motors. Starting with the arm away from home releases it where
   it stands. Confirm the arm is at its home pose and the gripper is closed
   first.
2. Start the deployment. `on_activate()` opens the bus, selects each motor's run
   mode, probes every position with a Stop frame, preloads each motor's target
   from that measurement, and only then engages torque. It refuses to activate at
   all if any joint reports no position, rather than enabling torque against an
   unknown target.
3. Run Objectives. Every test should **end by returning to home under power**.
4. Stop the deployment only with the arm at home.

`moveit_pro down` tears a deployment down. `docker rm -f` on the containers does
not: compose brings them straight back.

### Stopping behaviour

Three distinct cases, all deliberate:

| Event | What the motors do |
|---|---|
| Clean stop (`on_deactivate`) | **Hold** the pose. `hold_torque_on_deactivate: true`. |
| Hard kill of the process | Hold. The destructor closes the buses without sending Disable. |
| One joint lost or faulted | **Freeze**: the lost joint is left limp, every reachable joint keeps torque and holds where it was, and controller commands are ignored. |

The hold through a clean stop is only real because `motor_can_timeout_ms` is
`0`. Deactivation closes the bus, so a motor whose own `CAN_TIMEOUT` watchdog is
armed releases torque that long afterwards no matter what
`hold_torque_on_deactivate` says. The two parameters are one decision; `on_init`
warns when they disagree. Both are set in the real branch of
`description/rebot.urdf.xacro`.

The arm stays **rigid** after the stack stops, until motor power is cut. There is
no software path back to limp once the component is gone, which is why the pose
at shutdown has to be home. Set `torque_enable: "false"` and restart to get a
limp arm back.

The freeze is latched. A recovered joint stays limp and control is not handed
back until a fresh activation: a rebooted motor's position may be a whole turn
off, and the controllers' setpoints have moved on. `~/reboot_robstride` clears a
latched fault but does not by itself restore the joint.

### Thermal protection

`hardware.robot_description.urdf_params` sets `thermal_cutoff_c: "100.0"` and
`thermal_clear_c: "75.0"` (degrees C). The clear threshold must be finite,
nonnegative and strictly below the enabled cutoff. The driver defaults
`thermal_clear_c` to 75 C when omitted; an enabled cutoff of 75 C or lower
therefore requires an explicit lower clear threshold, otherwise initialization
is refused.
The RobStride driver detects any motor at or above that temperature and latches
`/robstride_hardware_interface/thermal_tripped`. Already-enabled motors keep
torque for recovery; automatic re-enable and further torque-enable requests are
blocked while latched. Initial hot feedback prevents startup enable.

The backend's `rebot_thermal_guard` cancels the current Objective, requests
**Return reBot Leader Home**, waits for successful completion and fresh measured
stationary HOME for 0.5 s, then calls the owning driver's `set_torque` service
with false. The leader bridge latches off without sending a competing HOME goal.
No separate guard command is needed, including when the leader is disabled.

The split is deliberate. The driver owns the YAML threshold, detection, the
latch and refusing re-enable. The HOME-then-OFF response stays in this backend
node because the driver cannot plan a HOME motion and the brakeless arm must
never lose torque away from HOME. An arm already fully OFF when the latch trips
(hot at startup) stays OFF and is reported as such, without waiting for the
Objective server.

If cancellation, HOME, hardware health, or fresh HOME verification fails, or a
step exceeds its own time bound (120 s for native HOME, a few seconds for each
other step), the system keeps the latch and reports intervention required; it never disables
away from HOME. Reachable joints retain torque under the existing loss/fault
policy. Existing collision checks and motor fault handling remain active.
After successful HOME-then-OFF recovery, the guard continues checking fresh
stationary measured HOME and healthy all-seven OFF feedback. While those hold,
it publishes a `~/thermal_rest_verified` attestation to the owning driver once
per second. Its `std_msgs/msg/UInt64` payload is the measured HOME sample's
same-host steady-clock timestamp; delayed attestations expire after 0.1 s. The
driver clears only if torque remains requested OFF and every configured motor
reports RESET/OFF, no current fault, and a finite nonnegative temperature strictly
below `thermal_clear_c`. Each motor needs complete Type-2 feedback less than 0.5 s old;
missing, stale, future-dated or invalid temperature samples cannot clear the
latch. Historical fault words alone do not block cooling-stage attestation or
clear; earlier HOME/OFF recovery stages retain their strict fault checks.
Fault-only traffic does not refresh a temperature sample. The driver's
`thermal_tripped` publication then becomes false, and the leader bridge clears
its thermal flag while preserving its stopped state. The rest
attestation belongs to the recovery owner; the generic driver cannot verify
HOME itself, so do not publish it as a manual bypass of HOME verification.

Cooling and clearing never enable torque or restart an Objective. Wait for the
clear confirmation at HOME, use the supported HOME enable procedure
(`~/set_torque`, `data: true`), and explicitly start the Objective again. For
**Mirror reBot Leader**, match the Star Arm to reBot and start Mirror again at
HOME; its existing preparation, enabled-motor and alignment checks still apply.
A failed HOME never requests clearing and keeps its existing latch and torque
retention policy. A hot startup already fully OFF stays OFF with its latch
retained; it has not completed HOME recovery. Recover and restart at confirmed
HOME only through the supported intervention procedure. Software recovery
cannot guarantee HOME after motor or communication failure and is not a
safety-rated thermal shutdown.

### Motors, limits and gains

See [Motor firmware settings](docs/firmware-settings.md) for the per-joint
commissioned register limits, zero alignment, write/save procedure and the
vendor CLI read shutdown hazard.

One motor per joint on one bus, ids 1-7 matching `joint1`-`joint7`, host
`master_id` 253:

| Joint | Motor id | Model | kp | kd |
|---|---|---|---|---|
| joint1 (base yaw) | 1 | RS06 | 50.0 | 3.0 |
| joint2 (shoulder lift) | 2 | RS06 | 150.0 | 10.0 |
| joint3 (elbow) | 3 | RS06 | 150.0 | 10.0 |
| joint4 (wrist tilt) | 4 | RS00 | 50.0 | 5.0 |
| joint5 (wrist yaw) | 5 | RS00 | 50.0 | 4.0 |
| joint6 (wrist roll) | 6 | RS00 | 50.0 | 4.0 |
| joint7 (gripper) | 7 | RS00 | 12.0 | 0.5 |

`actuator_type` is not cosmetic: it selects the motor's CAN quantization ranges,
so an RS00 declared where an RS06 is fitted silently rescales every position,
velocity and effort.

joint1-joint6 run in the mode set by the `arm_control_mode` entry under
`urdf_params` in `config/config.yaml`: `position_csp`, which it ships, or `motion`
(impedance, the Xacro argument's default when no value is passed). Impedance computes `kp * error + kd * velocity error` and the
driver sends no gravity feedforward, so a loaded joint settles short by load
torque / kp - joint4 stopped about 50 mrad short lifting 0.3 rad. `position_csp`
uses the motor's own position loop, which settles on target, capped by the speed
limit stored in each motor (`0x7017`). Switching needs a restart of both the
runtime and the drivers process, at home. The gripper always uses motion mode,
and kp/kd act only in motion mode. The arm values are the vendor's tuned follower
values for this arm. The jaw's are the `jaw_kp`/`jaw_kd` Xacro arguments, set
under `urdf_params` like `arm_control_mode`; they default to the vendor's kp with
damping raised from 0.05 to 0.5 to reduce fast-open bounce.

The URDF is already expressed in the **motor frame** with the measured limits in
radians, so no `direction` override is used: joint frame equals motor frame. The
limits in [Coordinates and limits](#coordinates-and-limits) are therefore the
numbers that bound the real arm - the driver clamps position commands to them and
wraps feedback into a one-turn window centred on them. Every joint spans less
than a full turn, which is what makes a motor power cycle read the same angle as
before it. That recovers whole turns only: a mechanical zero the motor forgets
shifts the reading by an arbitrary amount and the wrapped value then looks
plausible and is wrong. Confirm the zero is stored in the motors before relying
on it.

### Controllers

Unchanged from mock, and that is the point: every controller in
`config/control/rebot.ros2_control.yaml` already commands `position` only, which
is what both the CSP and motion modes are driven through. One
`joint_trajectory_admittance_controller` owns all seven joints and accepts partial
goals, so the arm and gripper planning groups both work through it. The joints a partial
goal leaves out are commanded to their measured positions when the goal arrives. A goal
whose first point is more than `default_path_tolerance` (0.16 rad, every joint) from the
measured position is rejected before anything moves. The 0.16 rad path tolerance
accommodates the measured 0.1514 rad jaw lag; the 0.1 rad goal tolerance
accommodates motion mode's gravity sag (up to 0.088 rad measured).

The real branch declares `position` as the **only** command interface per joint,
where mock also declares `velocity`. The driver exports whatever the description
declares, so a stray `velocity` declaration would be claimed by a controller and
then never written - a joint that accepts commands and does not move, with
nothing in the log to explain why.

Motor health (`id`, `enabled`, `run_state`, `fault_bits`) is published on the
hardware component's own `~/robstride_state` topic every read cycle. Its services
live under the component's node, `robstride_hardware_interface`: `~/set_torque`,
`~/set_zero_robstride`, `~/get_data_from_robstride`, `~/set_data_to_robstride`,
`~/reboot_robstride`. None of them overrides whichever controller currently
claims the command interface - an active `joint_trajectory_admittance_controller` keeps
writing its held setpoint regardless, so deactivate it first. `~/set_zero_robstride`
deserves particular care: it moves nothing itself, but it redefines where zero is,
and a controller still chasing its old setpoint number will then drive the motor
to match.

### Waypoints and Objectives

The saved waypoints, SRDF named states and Objectives are the same on real
hardware as on mock, including **Move reBot to Waypoint**, **Open Gripper** and
**Close Gripper**.

Two Objectives split planning from execution, so a trajectory can be inspected
before the arm moves:

- **Plan reBot Joint Move** plans from the current state to a named waypoint
  (`waypoint_name`) or, when that is empty, to `target_positions`, plus the exact
  reverse, and saves both as YAML without moving anything. At the default
  `velocity_scale_factor` 0.075 every joint is capped at 0.045 rad/s.
- **Execute reBot Joint Trajectory** loads a saved trajectory, validates it
  against the current planning scene and executes it unchanged. Run it from the pose
  it was planned from. Before every execution, compare the measured joint positions
  (`/joint_states`) with the trajectory's first point and execute only if every joint
  is within 0.005 rad. The Objective does not make this comparison itself, and the
  controller's own start check only rejects an arm more than 0.16 rad (its path
  tolerance) from the first point; it does not interpolate to that point, so a
  smaller offset is commanded as a step.

`target_positions` is a vector port, and Objective parameter overrides do not
support vectors, so per-run targets go through a named waypoint. Two small
waypoints near home exist for this: `rebot_small_a` and `rebot_small_b`. A
target exactly on a limit fails planning on the profile's numerical undershoot
(`Joint position out of bounds`). They were saved when joint2 and joint3 had a
lower limit of exactly 0.0, their home value, so their joint3 target sits 2 mrad
inside it (0.002). The lower limits are now −0.02 rad (joint2) and −0.01 rad
(joint3); see [Lower-limit commissioning](docs/lower-limit-commissioning.md).

Two bench-specific rules the model cannot express:

- **joint5 needs joint4 raised first.** From the home pose, joint5 reaches a
  mechanical stopper that the URDF limits do not describe; raising joint4 by
  about 0.3 rad clears it, and the return has to unwind in the reverse order.
- **Check the wrist camera's cable slack before moving joint6.** Wrist roll has
  the largest travel of any joint and will wind a camera cable.

### Cameras

The optional wrist D435 and scene OAK-D streams described in
[Optional real cameras](#optional-real-cameras) work the same way with a
real arm: `enable_cameras: true`, RGB at 30 fps (wrist 640x480, scene 640x400).
By default the scene camera also runs depth aligned to its colour image and
publishes its point cloud (`scene_aligned_depth: true`). Bench collection used
aligned depth and the measured scale correction described above. This revision
has source and package-test validation; the historical bench runs do not
validate later source changes.

**Only the scene camera is calibrated, for this bench; the real arm keeps the
nominal, UNCALIBRATED wrist camera mount** (see [bench calibration](#bench-calibration)). The wrist
camera has no depth output configured. Anything that needs metric accuracy from
the wrist camera needs an extrinsic calibration first.

### What has and has not been verified

Verified offline: the three vendored driver packages build; the driver's
hold-torque regression test passes against a fake motor on a virtual CAN
interface; and the Xacro selector matrix behaves as described above.

Verified on the bench arm through this configuration, non-instanced on `can0`:
all seven motors answer with matching signs and stored zero; enable at home and
disable at home; a first move of every joint 1-6, 0.05 rad out and back at about
0.05 rad/s; joint4 raised 0.3 rad in CSP, then joint5 moved and both unwound;
and Plan reBot Joint Move / Execute reBot Joint Trajectory running a three-joint
move and both small waypoints out and back, with every run inside 0.07 rad of
its target and 0.10 rad/s. Faster moves and large waypoints such as Raised have
not been run on the arm; for the stack Objectives' gripper and leg speeds on the
arm, see [SAM3 blue-on-red stack](#sam3-blue-on-red-stack).

An earlier revision of the [FashionStar leader](#fashionstar-leader) bridge
replayed one recorded 455 s leader-position stream on the arm: no fault frames
were reported, the largest arm-joint RMS tracking error was 0.01754 rad, the
return to HOME succeeded, and all seven motors were confirmed off afterward.
That was recorded input, not live operator following. The repeated-stop-signal
handling, the bounds-clamped HOME return, the twin jaw settings, the per-tick
Robstride health stop and joint7's 0.05 rad collision-path sampling added after
it were checked only in the twin and unit tests: this configuration as shipped has not
been rerun on the arm, and live leader following in `position_csp` mode has not
been verified.

Known limitations:

- The wrist camera mount is nominal; the provisional calibrated mount is not
  applied on the arm (see [Bench calibration](#bench-calibration)).
- At home joint7 reads about -0.044 rad, below its 0 lower limit, and the UI
  warns about it.
- After a disable at home, a joint can settle a few mrad away from the
  trajectory controller's held reference. A trajectory sent with that
  difference starts with a step, so check the reference against the measured
  position before the next enable.

## Hardware follow-up

- Verify the model zero and signs against the installation's motor calibration
  before trusting commanded positions.
- Measure the installed gripper's motor-angle-to-jaw-travel relation and closed
  offset to confirm the BOM-derived 8 mm/rad transmission.
- Apply a wrist camera calibration on the arm (the provisional six-pose solve in
  `rebot_bench_sim`'s `config/bench_layout.yaml` is not applied here), and add
  the D435/bracket collision geometry.
- Validate the gripper kp/kd values and the per-joint velocity limits on the bench.

## Validation

After the base/sim split, both packages built and their model, hardware-selector
and configuration-inheritance tests passed. The sim overlay executed Raised,
Open Gripper, Close Gripper and Zeroed Rest with zero final joint error; see
[split-package runtime results](docs/sim-split-acceptance.json). No camera drivers
were started for this verification. The earlier acceptance files below predate
the split and describe the single-package implementation.

Verified locally: package build, model loading, measured position bounds, J7
finger coupling, collision checks along the saved waypoint paths, formatting,
and initialization/activation of the mock hardware and trajectory controller.
**Runtime planning and execution passed** on a MoveIt Pro main source build:
`Raised`, `Open Gripper`, `Close Gripper`, then `Zeroed Rest` all returned success,
with final commanded-joint errors below 0.01 rad (reported as zero by the mock).
[Recorded action results and joint states](docs/runtime-acceptance.json) provide
the execution evidence. No physical arm hardware was used for those tests.

Joint and Pose jogging also pass on the same mock runtime. Selecting `gripper`
in the Joint tab exposes J7 alone; its bounded sweep and intermediate releases
showed zero arm drift and zero drift after stopping. Predictive stopping kept
J7 inside both limits (about 0.034 and 5.375 rad). Switching back to arm Joint
jogging, Cartesian jogging and trajectory execution succeeded. See
[recorded teleoperation results](docs/teleop-acceptance.json).

`colcon test --packages-select rebot_bench_base_config` loads the installed URDF/SRDF
through MoveIt Pro, verifies motor limits and the J7 mimic behavior, and checks
saved waypoints and interpolated motions with the real collision checker. It
also verifies that `Raised` lifts the tool more than 20 cm above `Zeroed Rest`.
A geometric regression reads the named Gripper Closed/Open waypoints and checks
that opposing finger-face separation increases monotonically from approximately
0.146 mm to 86.715 mm, including the dependent mimics.

After launching this config in an isolated ROS graph, run this command in a
ROS terminal inside the same MoveIt Pro instance with its workspace sourced.
The acceptance client executes both arm waypoints and opens/closes the gripper:

```bash
ros2 run rebot_bench_base_config verify_mock_execution.py
```

It verifies the robot description is this mock model before submitting goals,
waits up to 60 seconds for the planning-scene service, and checks action results
plus final joint states.

For a repeatable gripper jog check, leave Teleoperate running in **Joint** mode
with **gripper** selected and **Jog Collision Checking** enabled. Do not operate
other controls while running:

```bash
ros2 run rebot_bench_base_config verify_mock_gripper_jog.py
```

The mock-only client sends commands through `/joint_jog/gripper`, checks J7's
exclusive position-interface ownership, exercises both bounds and intermediate
hold/release stops, and checks that the arm stays still. It finishes near the
closed limit. This is a manual runtime acceptance check, not a hardware test or
an unattended CI test. Afterward, switch to manipulator Joint jogging and Pose
jogging, verify movement and release-to-stop, then stop Teleoperate and rerun
`verify_mock_execution.py` to check the return to trajectory execution.

## SAM3 blue-on-red stack

**Stack Blue on Red** is a durable Objective that locates both 50 mm cubes,
picks blue, places it on red, opens the gripper, retreats and returns home. **Pick Blue Cube**
runs just the location, reach check, survey and pick and leaves blue held above
its starting location.
**Locate reBot Cubes** is the perception-only entry point. Run **Clear reBot
Stack Scene** to remove geometry left by a previous attempt before resetting
or moving the cubes externally.

The scene camera must publish RGB and CameraInfo on
`/scene_camera/color/{image_raw,camera_info}` with calibrated optical TF into
`world`; see [bench calibration](#bench-calibration) below. World is the printed red/green/blue frame, z=0 at the mat top.
**Stack Blue on Red** uses RGB-only perception. **Unstack Blue from Red**,
**Stack and Unstack Cycle**, and stacked **Recover Cube from Wall** also require
the scene camera's aligned point cloud, `/scene_camera/depth/color/points`,
to distinguish a stacked blue cube from one on the mat (see
[the cycle](#unstack-and-the-stackunstack-episode-cycle)). SAM3 text prompts segment the red and blue cubes; the shared
`FitCubeToMask` Behavior fits their full silhouettes to upright 50 mm cubes on
that plane, rejects invalid or ambiguous fits, and returns their top centres.
See [behavior contracts](../rebot_bench_behaviors/README.md) for the fit limits.
The scene-camera/robot-base calibration must describe the current physical
installation; installing these Objectives does not calibrate a camera.

The tree uses `gripper_end` as the fingertip plane, with tool +X out of the
fingers. Pick hover is 75 mm above world z=0, followed by a straight descent to
35 mm. Closure is 45 mm (joint7=2.8125 rad); opening is 84 mm (5.25 rad). The
place hover is 110 mm with 25 degrees of tilt toward the base to reach the far
cubes; an IK-checked vertical fallback avoids arm collisions near the wall,
whose planning box stands 5 mm proud of the real face so no plan ends grazing it. Release uses 5 degrees of tilt and an 87.0 mm fingertip height that rests the tilted cube on red before opening. The short
place descent and the retreat to the place hover are planned Cartesian paths. The pick and
place legs hold the preceding jaw command, the grasp's 2.8125 rad while holding,
and validate the seven-joint trajectory against the scene. On the bench the cube
stops the jaw about 0.05 rad wider (measured about 2.861 rad), within the
0.10 rad stale-start guard below, so these legs keep commanding 2.8125 rad and
keep the squeeze. Every leg then starts
from the controller's previous command rather than the measured state:
`AnchorTrajectoryToCommand` shifts the validated trajectory onto the last
commanded endpoint, holds every joint it leaves out at its command, and logs any
start it replaces with measurement, which it does for a missing endpoint or one
more than 0.10 rad from measured state. After a leg aborts mid-execution its
endpoint is not committed, so the next leg starts from the last committed
command, up to 0.10 rad from where the arm stopped, and from the measured state
beyond that. The waypoint legs (the survey and every
return home) leave joint7 out, so it holds its command, or its measured position
when that command is stale, without moving. A gripper move plans the jaw alone
from its measured position and the anchor keeps that start, so a retry after a
canceled opening really reopens it. Only successful execution commits the
exact planned endpoint for the next leg, and `RecordRebotTrajectory` saves each
trajectory before it is sent. These numbers are shared by hardware and
simulation.
The waypoint planner starts from the measured arm clamped into its joint
bounds, and the pose planner from the measured state. A waypoint or pose leg the
planner refuses falls back to a point-to-point move from the commanded state,
collision-checked as planned and again after anchoring for both fallback paths.
Both generate only the six arm joints and fade the arm-start correction to zero
at the target. Waypoint anchoring adds the held jaw separately, so a stale jaw
command stays at its measured position throughout the move. Before the SRDF
allowed link2/link5 contact, the bench arm's sagged home (joint4 near
-0.017 rad) failed the planner's start check as a link2-link5 self-collision,
so every bench leg out of home took this fallback; that is how the stack ran on
the real arm. With the exclusion those legs plan natively, which has not been
run on the arm. In the twin, whose home measures exactly zero, the planner runs.
**Pick Blue Cube** first locates the cubes, adds them to the planning scene and
checks reach (see **Failures** below), all before any motion. It then runs
**Close Gripper**, planned natively from measured state, so the commanded home it then sets for all seven joints holds for the jaw
even after an aborted run left the jaw command stale. The real jaw rests closed, so
it then opens it at home with a
gripper-only move, before any arm motion, then reads the measured joint7 and
requires at least 5.1 rad, leaving a 0.15 rad gap to the 5.25 rad open
command, within the controller's 0.16 rad start tolerance, so the arm legs
that hold the open command are accepted. The controller can report a canceled opening as success, so a short
opening is retried once and then fails the Objective with the measured value;
the survey never starts on a closed jaw. Both openings, at home and at the
release, are this one check, **reBot Open Jaw**. After the release, the tree
retreats and returns home (`Zeroed Rest`); **Stack Blue on Red** then closes the
empty jaw there with **Close Gripper**, so a successful stack ends with all seven joints at home. If the release opening fails or stays
short after its retry, the tree still attempts the detach, the planned retreat
and the return home, and then reports failure; it does not treat a release
timeout or a canceled opening as a successful stack. That retreat holds joint7
at its measured position rather than the open command, so the controller
accepts a goal that starts where the jaw stalled.

**Failures.** Once the cubes are located, and before any jaw or arm motion,
**reBot Check Stack Reach** solves IK for the pick hover, the pick, the place
hover and the place. If one has no solution the Objective fails without moving,
with a message naming the cube and where it sits, such as `blue cube at x=0.277
y=0.253 is outside the arm's reach`. Any failed leg after that, an IK or planning failure
included, sends **Stack Blue on Red** home (`Zeroed Rest`) holding the jaw at
its command, then closes the jaw with **Close Gripper** (**reBot Recover Home**),
and the Objective still reports failure. That return home starts like any leg
after an abort: from the last committed command, up to 0.10 rad from where the
arm stopped, or from the measured state beyond that. The jaw counts as holding
the cube from just before the grasp closes until the release opening is
confirmed. A failure in that window ends at home with the jaw held at its last
command, and **Close Gripper** does not run: open if the grasp never moved, the
45 mm grasp width (2.8125 rad) once it closed, since the cube's stall stays
within the 0.10 rad stale-start guard, or where it stalled after a failed
release opening. Only a failure of that return home
leaves the arm where it stopped, and an empty jaw closes even then, as it must
after a leg that failed at home, where the one-point plan home is refused.

The pick, stack and unstack pre-motion checks refuse planned joint2 positions
above 2.4 rad. A fresh blue-cube hover needed about 2.36 rad; the former 2.2 rad
bound had also refused a reachable cube at 2.214 rad. Change `maximum` in
`objectives/rebot_check_pick_reach.xml`, `objectives/rebot_check_stack_reach.xml`,
`objectives/rebot_check_source_heat_reach.xml` and
`objectives/rebot_draw_place_spot.xml` together. **Unstack Blue from Red** checks
its approach hover, grasp and both lifts through the shared helpers, so it
holds a stack to the same bound Stack Blue on Red placed it under. The drawn
spot's release and next pick also use 2.4 rad. Physical execution at this bound
has not been verified.

Joint2's heat per leg grows with reach, and the measured joint2 runs 0.07-0.16 rad
past the plan under load. Historically, a leg planned to 2.29 rad from a 30 C
start reached the then-configured 55 C heat stop. The geometric reach bound
does not measure temperatures; the current 100 C thermal trip and 75 C reset
threshold, driver thermal latch and backend recovery remain unchanged.
The operator still supervises heat as described under **Running the real arm**.
After reaching home, recovery clears the carried cube's planning-scene
attachment. A failed home return still reports failure.

**Speed settings.** [`objectives/rebot_stack_motion.yaml`](objectives/rebot_stack_motion.yaml)
holds one velocity and one acceleration scale factor per kind of leg, as
fractions of the limits in `config/moveit/joint_limits.yaml`: `joint` (the
survey, the pick and place approaches and the return home), `cartesian` (the
pick descent and lift, the place descent and both release retreats) and
`gripper` (the opening at home, the grasp and the release). The two jaw closings
at home, first and last, are the native **Close Gripper**, which plans to the closed
limit at its own full scaling. **Pick Blue Cube**,
**Stack Blue on Red**, **Unstack Blue from Red** and **Prepare reBot Stack** load it at the start of every
run through **reBot Stack Motion** and the `LoadMotionScales` Behavior, and
every leg plans at its kind's factors. To change a speed, edit the file and
rebuild `rebot_bench_base_config` (a symlink-install build reads the source
file directly); a missing entry or a factor outside (0, 1] fails the run before
any motion.

Every factor ships at 1.0, the full limits, on the arm and in simulation. Size
the bench's joint speed guard from the planned peak before running the arm:
measured peaks run about 1.8 times the planned peak on the arm joints and 1.3 to
2 times on the gripper, so at 1.0 (0.6 rad/s planned on joints 1-6) an arm guard
needs about 1.1 rad/s, and the gripper's 3.0 rad/s about 4 to 6 rad/s. To keep the
0.17 rad/s guard instead, lower `joint` and `cartesian` to 0.15 or less
(0.15 x 0.6 rad/s x 1.8 = 0.16 rad/s), and size the
gripper factor against its guard the same way; an earlier hardware run had a
gripper opening faster than 0.1 canceled, which **reBot Open Jaw** detects and
fails on. Lowering a factor lowers every leg of its kind. The 2026-10-12 trip
was a jump at the start of a leg planned from the sagged measured state, which
does not scale with planning speed; anchoring each leg to the previous command,
above, removes it. `test/test_stack_objectives.cpp` runs the shipped trees with
the shipped file and fails if a leg misses its kind's setting, including the
recovery retreat and the return home. `rebot_bench_sim`
overrides **reBot Stack Motion** with its own settings file, so the simulator
stays at full speed whatever the arm's values; see its README.

Deploy into a MoveIt Pro user workspace with the runtime's ROS and proprietary
overlay already sourced. The SAM3 Behaviors come from `moveit_pro_sam3`, which
the workspace keeps optional (it is only built once the SAM License is
accepted), so it is not a declared dependency: confirm it is already installed,
or add it to the `colcon build` line.

```sh
colcon build --packages-up-to rebot_bench_behaviors rebot_bench_base_config
source install/setup.bash
```

Restart the backend to load the new behavior library and configuration. A
user-workspace colcon build is sufficient; these Objectives do not depend on
Direct Motion or a modified MoveIt Pro source build. On a MAIN-based bench
runtime, invoke the ordinary MCP tool:

```json
{"name":"run_objective","arguments":{"objective_name":"Stack Blue on Red"}}
```

### Unstack and the stack/unstack episode cycle

**Unstack Blue from Red** reverses the stack. From home, before any motion, it
locates blue with SAM3 as a cube resting on another cube (`Locate reBot Cube`
with `support_height` 0.05 and `top_face_fallback`, since against the mat SAM3
can segment the stacked cube's top face alone) and places red directly under it,
since blue hides red's top face. It plans the grasp as Stack Blue on Red's own place descent
reversed, **reBot Plan Place** for red: its place hover, then straight down to
its release pose. Both must have collision-free IK, or the Objective fails
without moving, with a message such as `blue cube at x=0.287 y=0.148 is outside
the arm's reach`. It then draws a random spot on the mat for blue and keeps it
only if both setting blue down there and the next stack's pick from there
(**reBot Check Pick Reach**) have collision-free IK, redrawing up to 20 times.
The next stack's place on red is the grasp just checked. A rejected draw names
no cube, since none is out of reach; if no draw qualifies, the Objective fails
without moving with `No reachable free spot for blue after 20 draws.` Only then does the arm move: close and open the jaw at
home, grasp blue (**reBot Grasp Blue**), lift it 30 mm straight up without turning the
wrist, so it clears red and the elbow stays off the wall (where that pose is out
of reach, near the base, or plans joint2 past its 2.4 rad bound, it lifts instead
to a place hover 40 mm above the approach one, held to the same bound; both are
reach-checked before motion), carry it at about that height to above the spot, then set it down and release it there (**reBot Place Blue**, the stack's
place on a different support), retreat, go home and close the jaw. A failure
after motion starts runs **reBot Recover Home**, the stack's failure path: home,
then the jaw closes unless it still holds the cube, and the Objective still
fails. Speeds come from `rebot_stack_motion.yaml` like the stack's.

The unstack's settings are in [`objectives/rebot_draw_place_spot.xml`](objectives/rebot_draw_place_spot.xml)
unless noted:

| Setting | Where | Value |
|---|---|---|
| Spot region | `DrawPlaceSpot` `x_range`, `y_range`: the cube centre's range in world | x 0.35-0.46 m, y 0.02-0.20 m: the base side of the proven set (the base is at x 0.726), every spot within 0.42 m of it; the region is unchanged and release and next-pick reach are checked at 2.4 rad |
| Heat-reach bound | `maximum` in the three `rebot_check_*_reach.xml` helpers and `rebot_draw_place_spot.xml`: pick, place, unstack grasp and both lifts, spot release and next pick are checked before motion (an invalid spot is redrawn) | 2.4 rad; a fresh blue-cube hover needed about 2.36 rad |
| Clearance from red | `DrawPlaceSpot` `clearance`: smallest centre-to-centre distance | 0.11 m |
| Draws before failing | `RetryUntilSuccessful` `num_attempts` | 20 |
| Release aim offset | `translation_xyz` of `release_spot` in [`objectives/rebot_draw_place_spot.xml`](objectives/rebot_draw_place_spot.xml): released 5 degrees tilted, blue pivots flat and lands farther from the base | 5 mm toward the base (-x in the spot's axes, whose yaw follows the base) |
| Carry height | `raised_spot` in `rebot_draw_place_spot.xml`: the carry ends 50 mm above the spot's place hover and only then descends, so it does not dip past red | 0.05 m |
| Lift off red | `translation_xyz` of `lift` in `unstack_blue_from_red.xml`: the matched release pose of [`objectives/rebot_plan_place.xml`](objectives/rebot_plan_place.xml) raised 30 mm. It is written out, not derived, so change its z with the release | `-0.000871557;0;0.067045702`: 30 mm above the release |
| Raised-hover lift | `translation_xyz` of `raised_red_top` in `unstack_blue_from_red.xml`: where the straight lift is out of reach, near the base, or past the 2.4 rad heat-reach bound, blue lifts to a place hover chosen this far above red's top instead, held to the same bound | 0.04 m |

The region is the part of the mat the scene camera sees and the stack's proven
placements span: at least 60 mm in from the mat's edges and 100 mm from the
foam. The clearance keeps the open jaw around either cube clear of the other.
Edit the values and run again; Objective XML is read at each run. The drawn spot
is published as a `place spot` marker.

**Stack and Unstack Cycle** collects training episodes with MoveIt Pro's own
recorder. It runs Stack Blue on Red, then Unstack Blue from Red, `num_cycles`
times (default 5). Each leg is one recorded episode in the dataset
`dataset_name`, labelled with its language prompt, `stack the blue on the red`
or `unstack the blue from the red`: `RecordEpisode` starts it with the prompt as
its task (retried for up to 10 s while the previous episode is still being
written), and `SaveEpisode` keeps it once the scene camera confirms the leg
from home; `StopRecording` closes the session 1 s later, once the saved episode
has finished updating. After a stack, `IsCubeStacked` requires three things of
blue: it fits a cube resting on another cube, its located centre lies within
12 mm horizontally of red's centre as Stack Blue on Red located it before moving,
and it stands that high in the scene camera's aligned depth. For the last, the
cycle keeps the points of `/scene_camera/depth/color/points` inside the mask
blue was fitted to (`GetMasks3DFromMasks2D`, `GetPointCloudFromMask3D`),
transforms them to `world`, and requires their upper-quartile height above the
mat to be at least 75 mm. The scene camera sees blue's top face and the side
facing it, so that height sits near its top: about 95 mm stacked, at most 50 mm
on the mat. Not the median: on the bench about 15% of the mask's depth pixels,
at its edge, land on the mat behind the cube, which pulled a real stack's median
to 65 mm while its upper quartile stayed at 94 mm. Colour
alone cannot make this call: a lone blue on the mat just behind red, as the
camera sees it, shows only its top face over red, which fits a stacked cube's
top face within a few millimetres of red's centre. After an unstack, both cubes
must fit whole cubes apart on the mat. Before saving, the blue mask's aligned
depth must also have an upper-quartile height of at least 25 mm and below 75 mm,
confirming a cube on the mat rather than one still stacked. The check uses the
same extracted cloud for both bounds and refuses missing, empty or invalid
depth before inverting the upper-bound condition; a missed physical grasp
cannot be accepted on RGB alone. See
[`objectives/rebot_check_unstacked_blue.xml`](objectives/rebot_check_unstacked_blue.xml).
A failed motion attempts home recovery,
closing an empty jaw while preserving a possibly held cube. Recovery can itself
fail. A failed leg or confirmation stops recording without saving, which
discards that leg's episode,
and fails. Change `num_cycles`, `dataset_name`, `training_config` or
`recorder_start_s` (the wait between starting a recording and moving) as
Objective inputs, or their defaults in
[`objectives/stack_and_unstack_cycle.xml`](objectives/stack_and_unstack_cycle.xml).
The stack check's settings are `IsCubeStacked`'s in
[`objectives/rebot_check_stacked_blue.xml`](objectives/rebot_check_stacked_blue.xml): `max_offset`
0.012 m, the horizontal tolerance, which must stay above the stack's own
placement error of about 5 mm or good stacks are discarded; `min_height`
0.075 m, half way between a cube on the mat's top (50 mm) and a stacked cube's
(100 mm), which leaves 25 mm for depth error either way; and `height_quantile`
0.75, the share of blue's depth points below the height compared, which must
stay above the share of edge pixels on the mat.
`test/test_fit_cube.cpp` in `rebot_bench_behaviors` checks both against the bench
camera's pose and intrinsics. The depth comes from the scene camera's
`scene_aligned_depth` setting ([Optional real cameras](#optional-real-cameras));
without it the first stack check fails and the cycle stops. Standalone unstack
and stacked wall recovery use the same depth gate before scene setup or motion,
so a lone blue on the mat cannot pass as a stacked cube from RGB alone.

Each leg is its own Objective, **Recorded Stack Leg** and **Recorded Unstack
Leg**, with the same inputs; the cycle repeats the pair. On the real arm the
cycle cannot run at full speed without cooling: one leg takes joint2 from about
40 °C to 50-53 °C, so the second leg of a pair would pass 55 °C. It has no
temperature gate of its own. Collect real episodes one leg per start instead,
alternating the two leg Objectives under the bench temperature monitor, which
enables only when every motor is below 45 °C and disables at home after each
leg. If any joint reaches 55 °C, send the arm home before continuing. **Stop** does not send the arm
home, so cancel a cycle through the bench run monitor, whose heat cancel returns
the arm home, rather than with **Stop**.

The recorded topics come from the named training config
[`config/training/rebot_bench_jpeg.json`](config/training/rebot_bench_jpeg.json):
state `/joint_states`, action
`/joint_trajectory_admittance_controller/controller_state` (the recorder takes
one action topic and `/joint_commands` carries only teleoperation), and both
cameras' JPEG `image_raw/compressed` topics at 30 Hz, about an eighth of the
raw images' size. Its camera keys are explicit, `observation.images.scene` and
`observation.images.wrist_mounted`: keys derived from the topic names would both
be `observation.images.compressed`, and the recorder refuses the collision. The
cycle therefore passes only `config_name` to `RecordEpisode`, never
`camera_topics`. Install the config once per runtime, either with
`POST /train/configs` or by copying the file into the Trainer's
`configs/` directory (`~/.local/share/moveit_pro/trainer/configs/` in the
runtime container).

Hardware recording requires the `compressed_image_transport` plugin declared
by this package: the camera drivers use it to publish their JPEG transport.
Check both `image_raw/compressed` topics before starting a recording; raw RGB
streams alone do not establish that the JPEG inputs are available.

### Bench calibration

`config/config.yaml` places `base_link` and `scene_camera_link` in the printed
frame with the 2026-10-11 ChArUco calibration of this bench:
`world_to_base_xyz`/`_rpy` `0.7259 0.0216 -0.003` / `0 0 3.1380` and
`scene_camera_xyz`/`_rpy` `-0.075 0.069 0.565` / `0.169 0.975 0.246`. The base's
x, y and yaw come from the hand-eye solve, through the provisional calibrated
wrist camera mount in `rebot_bench_sim`'s `config/bench_layout.yaml`. This
configuration does not apply that mount and keeps the nominal one; applying it
on the arm is follow-up work. The base plate is bolted level, so
roll and pitch are 0; it sits on the bench top under the 3 mm mat, so z is
-0.003, as in `rebot_bench_sim`'s layout. The camera pose is its board pose. Both, and the hard-coded bench and wall boxes in **reBot Stack Scene**,
are specific to this installation. After moving the arm, the scene camera or
the printed frame, or on another bench, re-measure: lay the ChArUco board on the
mat, solve the scene camera's board pose and the arm's base pose through the
wrist camera into the printed frame, and override the four `urdf_params`
entries, here or in a config with `based_on_package: rebot_bench_base_config`.
The identity placement the xacro falls back to when they are unset is not a
calibration: the scene camera then sits at `world`, and `FitCubeToMask` rejects
every view with "The calibrated RGB camera must look down at the mat".

The real configuration uses only `rebot_bench_base_config` and
`rebot_bench_behaviors`. Do not select `rebot_bench_sim` on hardware. Simulation
validation may use its explicitly marked grasp proxy and therefore establishes
perception/planning/Objective execution, not physical grip fidelity or calibrated
hardware placement accuracy. The hardware run is a separate validation.

## Running the real arm

How to run **Stack Blue on Red** on the bench arm. The arm has no brakes: torque
may only be released at home ([Real hardware](#real-hardware)).

**Before starting**

1. Robot power on and `can0` up at 1 Mbps ([Host CAN bring-up](#host-can-bring-up)).
2. The arm at home: joint1-joint6 each within 0.080 rad of zero (joint7 is the
   jaw and is not part of that test), every motor below 45 °C, and no motor or
   communication fault on `~/robstride_state`.
3. Both 50 mm cubes apart on the mat inside the calibrated camera area
   ([Bench calibration](#bench-calibration)), and nobody inside the arm's reach.

**Deployment.** Select the real arm ([Selecting it](#selecting-it)) and run
without `--instance` so the drivers container shares the host network and sees
`can0` ([Start the Desktop App and backend](#start-the-desktop-app-and-backend)). SAM3
perception needs CUDA: give the runtime container NVIDIA GPU access, or
**Locate reBot Cubes** fails loading `libcuda.so.1`. Bring up both cameras
([Optional real cameras](#optional-real-cameras)) and check each colour stream
publishes at about 30 Hz before a run. With `torque_enable: "false"`, enable all
seven motors at home with `~/set_torque` (`std_srvs/srv/SetBool`, `data: true`);
the service cannot enable the jaw alone.

**Run it.** In the MoveIt Pro UI, select **Objectives → Stack Blue on Red** and
press **Run** once, then watch the arm and the motor temperatures. A good run
closes the jaw at home, opens it, surveys, picks blue, places it on red, opens,
retreats, returns to **Zeroed Rest** and closes the jaw. Check physically that
blue is on red; a successful result alone does not prove the cube stayed in the
fingers. Disable torque only once the arm is back home within 0.080 rad, and
confirm all seven motors report disabled.

**Heat.** Keep runs short and supervise temperatures throughout. Return home
before any motor reaches about 55 °C; if the arm is already on its way home,
let that leg finish. Cool at home with torque disabled until every motor is
below about 45 °C before enabling again.

**If something goes wrong**

- **Stop** cancels the Objective. It does not release torque and does not send
  the arm home; a failed leg does (see **Failures** above). Keep clear and leave torque on away from home.
- Do not restart the driver, take the deployment down or cut power while the
  arm is raised. Return it home with a planned, collision-checked move, then
  disable at home.
- Do not clear faults, reboot or re-enable motors automatically after an
  encoder, undervoltage or CAN fault; find the cause first. A second fault on
  the same joint in one session ends the session.
- A refused jaw opening can come from a stale starting command or tracking lag.
  Keep the **Close Gripper** that **Pick Blue Cube** runs first to refresh the
  starting command. The 0.16 rad path tolerance allows the measured 0.1514 rad
  jaw lag; it does not replace source-start freshness checks.

**Settings.** Change these at home with torque disabled, and restart the
runtime after a YAML change; Objective XML is reread on every run.

| Setting | Where | Value |
|---|---|---|
| Arm velocity, acceleration | `config/moveit/joint_limits.yaml`, joint1-joint6 | 0.6 rad/s, 1.2 rad/s² |
| Jaw velocity, acceleration | same file, joint7 | 3.0 rad/s, 6.0 rad/s² |
| Finger mimic velocity | `description/rebot.urdf.xacro`, `gripper_joint1`/`2` | 0.024 m/s (0.008 × joint7) |
| Leg speed scales | `objectives/rebot_stack_motion.yaml` | 1.0 for every kind |
| Pick hold depth | `objectives/rebot_check_pick_reach.xml`, `TransformPose` to `{pick}` | `0;0;-0.015`: fingertips 35 mm above the mat |
| Matched release | `objectives/rebot_plan_place.xml`, `TransformPose` to `{place}` | `-0.000871557;0;0.037045702` at 5° tilt |
| Joint2 pre-motion reach | Three `rebot_check_*_reach.xml` helpers and `rebot_draw_place_spot.xml` | 2.4 rad; fresh blue-cube hover needed about 2.36 rad; former 2.2 rad refused a reachable cube at 2.214 rad |
| Path tolerance | `config/control/rebot.ros2_control.yaml`, `default_path_tolerance` | 0.16 rad, every joint |
| Speed guard | the bench's own joint speed monitor | planned peak × 1.8 on the arm (1.08 rad/s at 0.6), 1.3-2 × on the jaw |

Pick depth and release move together: the release keeps the same cube contact
only for the 35 mm grasp, so changing one alone can drive the cube into red.
**Unstack Blue from Red** grasps blue at that release pose and lifts it to the
same pose raised 30 mm, written out in `objectives/unstack_blue_from_red.xml`
(the `lift` row in the unstack's settings above); change its z with the release.

### Recover Cube from Wall

**Recover Cube from Wall** is an experimental recovery Objective kept separate
from the episode cycle. It approaches a cube near the foam wall with a low,
tilted grasp, carries it into the stacking area, and attempts to return home.
Its motion has not been validated on this revision or approved for real-arm
use. The simulation grasp proxy cannot establish that a tilted grasp releases
blue from a supporting red cube without carrying both.

The inputs in [`objectives/recover_cube_from_wall.xml`](objectives/recover_cube_from_wall.xml)
select `prompt` (`red cube`) and `other_prompt` (`blue cube`), then set the
target top-face centre `target_xyz` (`0.37;0.14;0.05` world metres) and
`target_xyzw` (`0;0;0.9872;0.1593`, its x axis away from the base). Change the
orientation with the target position. `stacked=true` locates blue on red instead
of using the prompts, and automatically selects `rebot_red` as the collision
support. `support_object` applies only when `stacked=false` (its default
`rebot_bench` describes a cube on the mat). Reach checks precede motion, and a
failed motion attempts the shared home recovery. A possibly held cube keeps
its jaw command; home recovery itself can fail.
