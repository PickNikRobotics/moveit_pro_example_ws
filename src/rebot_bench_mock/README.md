# reBot bench — mock hardware

A standalone MoveIt Pro configuration for the Seeed reBot Arm 102 / **B601-RS**
(RobStride variant): six arm joints, the J7 gripper motor coordinate, and
optional real wrist Intel RealSense D435 and fixed Luxonis OAK-D RGB cameras. Select `rebot_bench_mock` in
MoveIt Pro, or run:

```bash
moveit_pro run --config-package rebot_bench_mock
```

This package targets the workspace's `main` configuration API. Its `_mock`
suffix denotes ros2_control mock hardware, not MuJoCo physics. The only hardware
plugin is `mock_components/GenericSystem`; there is no real-hardware option,
CAN/serial connection or RobStride driver. Camera drivers are opt-in and
do not change the arm backend. Mock execution verifies kinematics and control plumbing, not
loads, contact dynamics, or hardware safety.

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
0.3 rad/s velocity and 0.6 rad/s² acceleration limits. It starts inactive and
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

The structure follows the workspace's SO-101 waypoint/Objectives conventions
and standard runtime-services launch. It has no config parent, so it does not
inherit another robot's hardware dependencies.

## Source and license

The URDF and STL meshes come from
[Seeed's B601-RS mechanical description](https://github.com/Seeed-Projects/reBot-DevArm/tree/58f9e433deec2ba118c5c22d60c07b9897e92acc/Rebot_Arm_description/RS),
commit `58f9e433deec2ba118c5c22d60c07b9897e92acc`, under **CERN-OHL-W-2.0**.
The modified URDF retains that license. Configuration and tests are BSD-3-Clause.
[description/NOTICE.md](description/NOTICE.md) records retained assets,
modifications, attribution, source locations and the full license location.

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
**Planning limits are 0.3 rad/s and 0.6 rad/s² on every commanded joint**, well
below the stored motor speeds. The waypoint Objectives default to 50% scaling
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
The arm always uses `mock_components/GenericSystem`, including when cameras are on.
Edit [config/cameras.yaml](config/cameras.yaml), set `enable_cameras: true`, and
restart the MoveIt Pro instance. The persistent driver launch reads that file.
For a cameras-only ROS launch in a sourced workspace:

```bash
ros2 launch rebot_bench_mock cameras.launch.py enable_cameras:=true
```

Every setting in the YAML is also a launch argument, for example
`image_width:=640 image_height:=480 framerate:=30`. Stop this standalone launch
before enabling cameras in MoveIt Pro; each device must have one driver owner.

| Role | Device | RGB image | Camera info | Optical frame |
| --- | --- | --- | --- | --- |
| Wrist | Intel RealSense D435 (`8086:0b07`) | `/wrist_mounted_camera/color/image_raw` | `/wrist_mounted_camera/color/camera_info` | `d435_color_optical_frame` |
| Scene | Luxonis OAK-D-PRO-W-97 | `/scene_camera/color/image_raw` | `/scene_camera/color/camera_info` | `scene_camera_color_optical_frame` |

Both default to **640×480 at 30 fps, RGB only, depth off**. These stable
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
(quality 95) before USB transfer. Its 1280×800 sensor video output is center-cropped
to the configured dimensions, then encoded; the driver decodes to `bgr8` ROS Images
on the host. CAM_B/C's OV9282 stereo pair, the BNO086 IMU and IR illumination
are disabled. Change `scene_mjpeg_quality` in the same YAML to adjust compression.
ISP luma/chroma denoising and sharpening are disabled to preserve detail.
Allow a few seconds after startup for the scene camera's image to settle before
recording; the first frames can look heavily processed.
The configured scene serial selects the installed OAK; `wrist_serial_no` may be
set when more than one D435 is connected. Resolution changes must be supported
by both devices and the OAK video encoder (including its width alignment).

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

**Both mounting transforms are uncalibrated placeholders.** `d435_mount_link`
and `d435_link` coincide with `gripper_end`; `scene_camera_link` coincides with
`world`. Their optical frames use the ROS optical-axis rotation. The robot state
publisher owns these transforms; driver TF publication is disabled to avoid
conflicting publishers. Device intrinsics in `CameraInfo` do not calibrate the
camera-to-robot transform. Images are usable for recording, but these poses must
be measured before spatial perception or image-based motion. Neither camera has
collision geometry yet.

### Camera validation

With both cameras connected at 480 Mbps, a simultaneous 65-second subscriber
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

## Hardware follow-up

Before a separate hardware integration:

- Verify the model zero and signs against that installation's motor calibration;
  the mapping here follows the published RS model and SDK, not a hardware test.
- Measure the installed gripper's motor-angle-to-jaw-travel relation and closed
  offset to confirm the BOM-derived 8 mm/rad transmission.
- Measure the actual D435/bracket transform, add its collision geometry, calibrate
  the wrist and scene-camera optical extrinsics.
- Add a hardware driver separately, with its own lifecycle, gravity-load handling,
  command ownership, stopping behavior and independently validated motion limits.
  These mock-only results do not validate any of those behaviors.

## Validation

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

`colcon test --packages-select rebot_bench_mock` loads the installed URDF/SRDF
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
ros2 run rebot_bench_mock verify_mock_execution.py
```

It verifies the robot description is this mock model before submitting goals,
waits up to 60 seconds for the planning-scene service, and checks action results
plus final joint states.

For a repeatable gripper jog check, leave Teleoperate running in **Joint** mode
with **gripper** selected and **Jog Collision Checking** enabled. Do not operate
other controls while running:

```bash
ros2 run rebot_bench_mock verify_mock_gripper_jog.py
```

The mock-only client sends commands through `/joint_jog/gripper`, checks J7's
exclusive position-interface ownership, exercises both bounds and intermediate
hold/release stops, and checks that the arm stays still. It finishes near the
closed limit. This is a manual runtime acceptance check, not a hardware test or
an unattended CI test. Afterward, switch to manipulator Joint jogging and Pose
jogging, verify movement and release-to-stop, then stop Teleoperate and rerun
`verify_mock_execution.py` to check the return to trajectory execution.
