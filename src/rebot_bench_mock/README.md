# reBot bench — mock hardware

A standalone MoveIt Pro configuration for the Seeed reBot Arm 102 / **B601-RS**
(RobStride variant): six arm joints, the J7 gripper motor coordinate, and a
wrist Intel RealSense D435 frame placeholder. Select `rebot_bench_mock` in
MoveIt Pro, or run:

```bash
moveit_pro run --config-package rebot_bench_mock
```

This package targets the workspace's `main` configuration API. Its `_mock`
suffix denotes ros2_control mock hardware, not MuJoCo physics. The only hardware
plugin is `mock_components/GenericSystem`; there is no real-hardware option,
CAN/serial connection, RobStride driver, camera driver, or physical-device
launch file. Mock execution verifies kinematics and control plumbing, not
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
along the saved motions. The three package Objectives explicitly use **zero
additional link padding** while retaining mesh collision checks. Both 10 mm and
1 mm padding reject movable-link pairs in the tightly folded zeroed-rest pose;
those pairs remain checked with the original collision meshes. No extra
clearance margin is claimed. This is a mock-only planning choice and must be
reconsidered with calibrated geometry before hardware use.

## D435 and hardware follow-up

`d435_mount_link` and `d435_link` currently coincide with `gripper_end`.
`d435_color_optical_frame` supplies the conventional ROS optical-axis rotation.
They are **uncalibrated placeholders**, with no geometry, imagery or camera node.
They do not claim the pose or collision envelope of a physical camera/bracket.

Before a separate hardware integration:

- Verify the model zero and signs against that installation's motor calibration;
  the mapping here follows the published RS model and SDK, not a hardware test.
- Measure the installed gripper's motor-angle-to-jaw-travel relation and closed
  offset to confirm the BOM-derived 8 mm/rad transmission.
- Measure the actual D435/bracket transform, add its collision geometry, calibrate
  the optical extrinsics, and provide a camera driver in the hardware workspace.
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
the execution evidence. No physical hardware was used.

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
