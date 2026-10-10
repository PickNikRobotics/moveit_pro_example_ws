# reBot bench base configuration

The shared configuration for the Seeed reBot Arm 102 / **B601-RS** owns the
URDF/Xacro, SRDF, meshes, controllers, Objectives, waypoints and optional real
wrist D435 and scene OAK-D RGB cameras. It follows the workspace's SO-101
base/sim layout. For simulation select [`rebot_bench_sim`](../rebot_bench_sim/README.md):

```bash
moveit_pro run --config-package rebot_bench_sim
```

The arm defaults to `hardware_interface: mock`, using only
`mock_components/GenericSystem`. Setting it to `real` selects the RobStride
`ros2_control` driver and drives the seven motors over SocketCAN - see
[Real hardware](#real-hardware). Any other value fails Xacro expansion rather
than producing a description with no hardware plugin.
Cameras in this base package default off and may be enabled separately. The sim
overlay forces mock arm hardware and replaces the camera launch hook with an
empty launch: no physical cameras and no simulated imagery. Its `_sim` suffix
means ros2_control mock execution, not physics simulation.

## Try it

Run **Move reBot to Waypoint** with `Raised`, then `Zeroed Rest`. Both are saved
waypoints and SRDF named states; startup is `Zeroed Rest` (all seven motor
coordinates zero). The raised arm pose is `[0, 0.65, 0.85, 0.3, 0, 0]` radians.
`Open Gripper` and `Close Gripper` override the standard teleoperation Objectives,
using `Gripper Open` and `Gripper Closed`. One trajectory controller owns all
seven joints and accepts partial goals for the arm and gripper groups.

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

The dedicated gripper JointVelocityController uses position commands with
3.0 rad/s velocity and 6.0 rad/s² acceleration limits. It starts inactive and
participates in managed controller switching with the seven-joint trajectory
controller. Disjoint arm controllers may remain active during gripper jogging;
only one active controller claims each command interface.

The package's **Teleoperate** Objective sets additional collision padding to
zero, matching the waypoint Objectives, and routes its trajectory execution
through `joint_trajectory_controller` (this config has no admittance
controller). Mesh collision checking stays enabled.
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

The D435 body is the unchanged **Apache-2.0** mesh from
[realsense2_description](https://github.com/realsenseai/realsense-ros/tree/9215f26e8348ad5922a608b77882a9bfe05940f0/realsense2_description),
revision `9215f26e8348ad5922a608b77882a9bfe05940f0`.
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
and the gripper factor 6 are not the motor-to-URDF mapping. No LeRobot code or
runtime dependency is included.

| Motor / coordinate | Meaning | Approved motor range (degrees) | Stored motor speed (rad/s) |
| --- | --- | --- | --- |
| J1 / `joint1` | Pan | −145 … 145 | 10 |
| J2 / `joint2` | Shoulder lift | 0 … 203 | 10 |
| J3 / `joint3` | Elbow | 0 … 239 | 10 |
| J4 / `joint4` | Wrist flex | −106 … 86 | 1 |
| J5 / `joint5` | Wrist yaw | −101 … 108 | 33 |
| J6 / `joint6` | Wrist roll | −175 … 175 | 33 |
| J7 / `joint7` | Gripper motor, positive opens | 0 … 310 | 33 |

These approved motor-frame ranges replace upstream's model limits; some are
wider and some narrower.
**Planning limits are 0.3 rad/s and 0.6 rad/s² on joint1-joint6 and 3.0 rad/s and
6.0 rad/s² on the gripper (joint7)**, below the stored motor speeds. Change them in
`config/moveit/joint_limits.yaml`; joint7's URDF `velocity` limit and the gripper jog
controller in `config/control/rebot.ros2_control.yaml` carry the same value. The waypoint Objectives default to 50% scaling
(0.15 rad/s, 0.3 rad/s²). These are mock planning choices, not hardware tuning.

J7 is an explicitly bounded revolute coordinate (0 … 5.410520681182 rad) on a
geometry-free link. The original fingers remain prismatic joints, both mimicking
J7 at **0.008 m/rad**. This is derived from the
[Seeed BOM's module-1, 16-tooth pinion](https://github.com/Seeed-Projects/reBot-DevArm/blob/58f9e433deec2ba118c5c22d60c07b9897e92acc/hardware/reBot_B601_RS/README.md):
pitch radius = module × teeth / 2 = 8 mm. The sliders retain their opposed
upstream axes. At 310° each travels 43.284 mm, for 86.568 mm total additional
opening. This transmission is a model derivation, not a measured jaw calibration.

The SRDF excludes adjacent assembly pairs and the opposing gripper jaws from
self-collision checking. In particular, `gripper_end` is rigidly attached to
`link6` across J6 from `link5`; their meshes have a 9.0 mm axial separating gap
that J6 rotation cannot close. Default 10 mm planning padding otherwise makes
this mount report a collision. The coupled jaws approach one another at closure
and separate on opening. Tests re-enable both pairs for unpadded mesh checks
along the saved motions. The four package Objectives explicitly use **zero
additional link padding** while retaining mesh collision checks. Both 10 mm and
1 mm padding reject movable-link pairs in the tightly folded zeroed-rest pose;
those pairs remain checked with the original collision meshes. No extra
clearance margin is claimed. This is a mock-only planning choice and must be
reconsidered with calibrated geometry before hardware use.

## Optional real RGB cameras

Cameras default **off**, so the package starts without either device plugged in.
The arm currently uses `mock_components/GenericSystem`, including when cameras are on.
These camera settings apply only when launching the base package; the sim overlay
does not invoke this camera launch file.
Edit [config/cameras.yaml](config/cameras.yaml), set `enable_cameras: true`, and
restart the MoveIt Pro instance. The persistent driver launch reads that file.
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

The wrist defaults to **640×480** and the scene to **640×400**, both at 30 fps,
RGB only, depth off. The heights differ on purpose; see the scene camera's field
of view below. These stable
`sensor_msgs/Image` and `CameraInfo` topics support the UI and later recording
for policies such as pi0.5 (which resizes images to 224×224). Image and calibration topics follow the sibling configs' naming convention.
No registered depth pair or point cloud is advertised.
The wrist image encoding is `rgb8`; the scene encoding is `bgr8`, matching
the DepthAI v2 MJPEG decoder's OpenCV output. Recording consumers should honor
`Image.encoding` (for example, request `rgb8` from cv_bridge) when converting
either stream to an RGB tensor.

Both cameras intentionally use **USB 2 (480 Mbps)** for long cables. The D435
uses its supported 640×480/30 color mode, with depth, infrared, and IMU disabled.
The OAK uses the standard Jazzy `depthai_ros_driver` (DepthAI v2): CAM_A's OV9782
color sensor, an RGB-only pipeline, USB speed `HIGH`, and **on-device MJPEG**
(quality 95) before USB transfer. It takes the **ISP output** at 640×400 rather than
the video path's crop, then encodes; the driver decodes to `bgr8` ROS Images on the
host. CAM_B/C's OV9282 stereo pair, the BNO086 IMU and IR illumination
are disabled. Change `scene_mjpeg_quality` in the same YAML to adjust compression.
ISP luma/chroma denoising and sharpening are disabled to preserve detail.
Allow a few seconds after startup for the scene camera's image to settle before
recording; the first frames can look heavily processed.
The configured scene serial selects the installed OAK; `wrist_serial_no` may be
set when more than one D435 is connected. Resolution changes must be supported
by both devices and the OAK video encoder (including its width alignment).

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

What is still **not** calibrated is where either camera *is*: the mount
transforms are nominal, taken from the assembly drawings and measured by eye.
Intrinsics being correct does not make a hand-eye result correct.

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
sudo apt install ros-jazzy-realsense2-camera ros-jazzy-depthai-ros-driver
```

The launch uses the drivers process, which has access to devices. Containers
need **all** of the D435's `/dev/video*` nodes, including its depth interface
even though depth streaming is off: librealsense uses that interface to discover
the device's base stream. Check the nodes' USB parent in `/sys/class/video4linux/`;
the `/dev/v4l/by-id/` symlinks alone may omit some interfaces. USB-bus access
must also allow device re-enumeration because the OAK boots over USB.
These are camera dependencies only.

**Camera transforms are not calibrated.** The wrist D435 body and optical
frame now follow the nominal assembly described above. `scene_camera_link`
still coincides with `world`, with the standard ROS optical-axis rotation.
Camera-driver TF publication stays disabled so the robot description owns
these frames. Measure both camera-to-robot transforms before spatial
perception or image-based motion. Neither camera has collision geometry yet.

### Camera validation

The imported camera implementation was previously tested with both cameras
connected at 480 Mbps; a simultaneous 65-second subscriber
check measured **30.06 Hz wrist** and **30.00 Hz scene**, both 640×480 with
matching `CameraInfo` dimensions and the optical frame IDs above. Tested with
Jazzy `realsense2_camera` 4.58.1 / librealsense 2.58.1 and
`depthai_ros_driver` 2.12.2 / DepthAI 2.31.1. The default camera-off launch
exited successfully without starting either driver. `robot_state_publisher`
loaded the URDF and published the expected fixed camera transforms.
See [camera acceptance results](docs/camera-acceptance.json).

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
`hardware_interface: "mock"` and is unaffected by any of this.

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

### Run it without `--instance`

```bash
moveit_pro run --config-package rebot_bench_base_config
```

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

### Motors, limits and gains

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
| joint7 (gripper) | 7 | RS00 | 12.0 | 0.05 |

`actuator_type` is not cosmetic: it selects the motor's CAN quantization ranges,
so an RS00 declared where an RS06 is fitted silently rescales every position,
velocity and effort.

joint1-joint6 run in the mode set by the `arm_control_mode` entry under
`urdf_params` in `config/config.yaml`: `motion` (impedance, the default) or
`position_csp`. Impedance computes `kp * error + kd * velocity error` and the
driver sends no gravity feedforward, so a loaded joint settles short by load
torque / kp - joint4 stopped about 50 mrad short lifting 0.3 rad. `position_csp`
uses the motor's own position loop, which settles on target, capped by the speed
limit stored in each motor (`0x7017`). Switching needs a restart of both the
runtime and the drivers process, at home. The gripper always uses motion mode,
and kp/kd act only in motion mode; the values are the vendor's tuned follower
values for this arm.

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
`joint_trajectory_controller` owns all seven joints and accepts partial goals, so
the arm and gripper planning groups both work through it.

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
claims the command interface - an active `joint_trajectory_controller` keeps
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
  `velocity_scale_factor` 0.15 every joint is capped at 0.045 rad/s.
- **Execute reBot Joint Trajectory** loads a saved trajectory, validates it
  against the current planning scene and executes it unchanged.

`target_positions` is a vector port, and Objective parameter overrides do not
support vectors, so per-run targets go through a named waypoint. Two small
waypoints near home exist for this: `rebot_small_a` and `rebot_small_b`. joint2
and joint3 have a lower limit of exactly 0.0, which is also their home value, and
a target exactly on a limit fails planning on the profile's numerical undershoot
(`Joint position out of bounds`). Targets for those joints therefore sit a few
mrad inside the limit (0.002 for joint3).

Two bench-specific rules the model cannot express:

- **joint5 needs joint4 raised first.** From the home pose, joint5 reaches a
  mechanical stopper that the URDF limits do not describe; raising joint4 by
  about 0.3 rad clears it, and the return has to unwind in the reverse order.
- **Check the wrist camera's cable slack before moving joint6.** Wrist roll has
  the largest travel of any joint and will wind a camera cable.

### Cameras

The optional wrist D435 and scene OAK-D streams described in
[Optional real RGB cameras](#optional-real-rgb-cameras) work the same way with a
real arm: `enable_cameras: true`, RGB at 30 fps (wrist 640x480, scene 640x400),
depth off. Both streams have been seen live in the MoveIt Pro UI alongside the
real arm.

**The camera mounts are measured by eye and the transforms in the URDF are
nominal. They are UNCALIBRATED.** No calibrated perception and no depth output is
claimed or configured. Anything that needs metric accuracy from either camera
needs an extrinsic calibration first.

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
its target and 0.10 rad/s. Faster moves, large waypoints such as Raised, the
gripper and teleoperation have not been run on the arm.

Known limitations:

- The collision model refuses joint4 at -0.05 rad near home (`link2` - `link5`),
  although the real arm moves there cleanly. The SRDF is unchanged.
- The camera mounts are uncalibrated (see [Cameras](#cameras)).
- At home joint7 reads about -0.044 rad, below its 0 lower limit, and the UI
  warns about it.
- `assets/realsense/d435.dae` is 15.8 MB, above the UI's 10 MB recommendation.
- After a disable at home, a joint can settle a few mrad away from the
  trajectory controller's held reference. A trajectory sent with that
  difference starts with a step, so check the reference against the measured
  position before the next enable.

## Hardware follow-up

- Verify the model zero and signs against the installation's motor calibration
  before trusting commanded positions.
- Measure the installed gripper's motor-angle-to-jaw-travel relation and closed
  offset to confirm the BOM-derived 8 mm/rad transmission.
- Measure the actual D435/bracket transform, add its collision geometry, and
  calibrate the wrist and scene-camera optical extrinsics.
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
