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
[Real hardware](#real-hardware). `mujoco` is `rebot_bench_sim`'s selector: this
description then emits no `ros2_control` block and no colour optical frames,
which the sim's MuJoCo hardware supplies. Any other value fails Xacro expansion
rather than producing a description with no hardware plugin.
Cameras in this base package default off and may be enabled separately. The sim
is a MuJoCo twin of the bench: it renders both cameras and starts no camera
drivers.
The `world_to_base_xyz`/`_rpy` and `scene_camera_xyz`/`_rpy` xacro arguments
place `base_link` and `scene_camera_link` in `world`. `config/config.yaml` sets
them to this bench's calibration (see [the stack recipe](#sam3-blue-on-red-stack));
only the xacro fallback, when they are unset, has both coincide with `world`.
`wrist_camera_mount_xyz`/`_rpy` set the `d435_mount_joint` origin on
`gripper_end`; unset, it is the nominal mount, which the real configuration
keeps. The sim places all of them from its bench layout file instead, over the
inherited calibration.

## Try it

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
and the gripper factor 6 are not the motor-to-URDF mapping. No LeRobot code or
runtime dependency is included.

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
observations, uncertainty and deployment cleanup for these home margins.
**Planning limits are 0.6 rad/s and 1.2 rad/s² on joint1-joint6**, twice the measured
teleop limits (0.3 rad/s, 0.6 rad/s²), **and 3.0 rad/s and 6.0 rad/s² on the gripper
(joint7)**, its measured teleop limits, all below the stored motor speeds. The jaw is not
doubled: at 6.0 rad/s it lagged its command by 0.151 rad, past the trajectory controller's
0.15 rad `default_path_tolerance`, and that tolerance is one value for all seven joints, so
raising it would also relax the arm. The arm and the twin use the same values. Change them
in `config/moveit/joint_limits.yaml` and keep the URDF `velocity` limits in
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
sudo apt install ros-jazzy-realsense2-camera ros-jazzy-depthai-ros-driver
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
`joint_trajectory_admittance_controller` owns all seven joints and accepts partial
goals, so the arm and gripper planning groups both work through it. The joints a partial
goal leaves out are commanded to their measured positions when the goal arrives. A goal
whose first point is more than `default_path_tolerance` (0.15 rad, every joint) from the
measured position is rejected before anything moves. Both that and the 0.1 rad goal
tolerance are sized for motion mode's gravity sag (up to 0.088 rad measured).

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
  controller's own start check only rejects an arm more than 0.15 rad (its path
  tolerance) from the first point; it does not interpolate to that point, so a
  smaller offset is commanded as a step.

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

**Only the scene camera is calibrated, for this bench; the real arm keeps the
nominal, UNCALIBRATED wrist camera mount** (see [bench calibration](#bench-calibration)). No depth
output is configured. Anything that needs metric accuracy from the wrist camera
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
its target and 0.10 rad/s. Faster moves, large waypoints such as Raised and
teleoperation have not been run on the arm; for the stack Objectives' gripper
and leg speeds on the arm, see [SAM3 blue-on-red stack](#sam3-blue-on-red-stack).

Known limitations:

- The collision model refuses joint4 at -0.05 rad near home (`link2` - `link5`),
  although the real arm moves there cleanly. The SRDF is unchanged.
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
`world`; see [bench calibration](#bench-calibration) below. World is the printed red/green/blue frame, z=0 at the mat top. No scene
depth is needed. SAM3 text prompts segment the red and blue cubes; the shared
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
**Pick Blue Cube** first locates the cubes, adds them to the planning scene and
checks reach (see **Failures** below), all before any motion. It then runs
**Close Gripper**, planned natively from measured state, so the commanded home it then sets for all seven joints holds for the jaw
even after an aborted run left the jaw command stale. The real jaw rests closed, so
it then opens it at home with a
gripper-only move, before any arm motion, then reads the measured joint7 and
requires at least 5.1 rad: the 5.25 rad open command less the controller's
0.15 rad start tolerance, so the arm legs that hold the open command are
accepted. The controller can report a canceled opening as success, so a short
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

**Speed settings.** [`objectives/rebot_stack_motion.yaml`](objectives/rebot_stack_motion.yaml)
holds one velocity and one acceleration scale factor per kind of leg, as
fractions of the limits in `config/moveit/joint_limits.yaml`: `joint` (the
survey, the pick and place approaches and the return home), `cartesian` (the
pick descent and lift, the place descent and both release retreats) and
`gripper` (the opening at home, the grasp and the release). The two jaw closings
at home, first and last, are the native **Close Gripper**, which plans to the closed
limit at its own full scaling. **Pick Blue Cube**,
**Stack Blue on Red** and **Prepare reBot Stack** load it at the start of every
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
`can0` ([Run it without `--instance`](#run-it-without---instance)). SAM3
perception needs CUDA: give the runtime container NVIDIA GPU access, or
**Locate reBot Cubes** fails loading `libcuda.so.1`. Bring up both RGB cameras
([Optional real RGB cameras](#optional-real-rgb-cameras)) and check each
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
- A refused jaw opening usually means a stale starting command, not a tight
  tolerance. Keep the **Close Gripper** that **Pick Blue Cube** runs first, and
  do not widen path tolerances to hide it.

**Settings.** Change these at home with torque disabled, and restart the
runtime after a YAML change; Objective XML is reread on every run.

| Setting | Where | Value |
|---|---|---|
| Arm velocity, acceleration | `config/moveit/joint_limits.yaml`, joint1-joint6 | 0.6 rad/s, 1.2 rad/s² |
| Jaw velocity, acceleration | same file, joint7 | 3.0 rad/s, 6.0 rad/s² |
| Finger mimic velocity | `description/rebot.urdf.xacro`, `gripper_joint1`/`2` | 0.024 m/s (0.008 × joint7) |
| Leg speed scales | `objectives/rebot_stack_motion.yaml` | 1.0 for every kind |
| Pick hold depth | `objectives/rebot_check_stack_reach.xml`, `TransformPose` to `{pick}` | `0;0;-0.015`: fingertips 35 mm above the mat |
| Matched release | `objectives/rebot_check_stack_reach.xml`, `TransformPose` to `{place}` | `-0.000871557;0;0.037045702` at 5° tilt |
| Path tolerance | `config/control/rebot.ros2_control.yaml`, `default_path_tolerance` | 0.15 rad, every joint |
| Speed guard | the bench's own joint speed monitor | planned peak × 1.8 on the arm (1.08 rad/s at 0.6), 1.3-2 × on the jaw |

Pick depth and release move together: the release keeps the same cube contact
only for the 35 mm grasp, so changing one alone can drive the cube into red.
