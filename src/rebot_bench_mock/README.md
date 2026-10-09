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

These specified position ranges replace upstream's narrower model limits.
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
finger coupling, collision checks along the saved waypoint paths, Python lint,
and initialization/activation of the mock hardware and trajectory controller.
**Runtime planning and execution in MoveIt Pro remain unverified**: the runtime
stopped on a license/product mismatch on the validation host. A maintainer with
a matching license must complete the acceptance sequence below before treating
this draft as ready. No physical hardware was used.

`colcon test --packages-select rebot_bench_mock` loads the installed URDF/SRDF
through MoveIt Pro, verifies motor limits and the J7 mimic behavior, and checks
saved waypoints and interpolated motions with the real collision checker. It
also verifies that `Raised` lifts the tool more than 20 cm above `Zeroed Rest`.

After launching this config in an isolated ROS graph, run this command in a
ROS terminal inside the same MoveIt Pro instance with its workspace sourced.
The acceptance client executes both arm waypoints and opens/closes the gripper:

```bash
ros2 run rebot_bench_mock verify_mock_execution.py
```

It verifies the robot description is this mock model before submitting goals,
and checks action results plus final joint states.
