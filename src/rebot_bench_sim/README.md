# reBot bench simulation

A standalone MuJoCo twin of the reBot bench. MoveIt Pro drives the reBot arm
and gripper through `picknik_mujoco_ros`, the same way `lab_sim` drives its UR.
The description assets, SRDF, planning and controller config, waypoints and
pick/stack Objectives are local. No motor, Orbbec or DepthAI drivers or
leader teleoperation scripts are required. The twin holds the arm, the wrist
Orbbec Gemini 305 and scene OAK-D Pro W 97 camera frames, the blue mat, the
white wall, a red and a blue 50 mm cube, and an axes triad at the world origin.

[Historical D435-era render sheet](docs/render_sheet.png); the current wrist
camera uses the frame-only stock Gemini 305 attachment described below.

```bash
moveit_pro run --config-package rebot_bench_sim --instance rebot-sim --headless
```

The package loads the core and MuJoCo Behaviors, **Reset MuJoCo Sim**, and
**Move reBot to Pose**, a Cartesian move of the gripper to a pose in
`base_link` (by default pointing straight down above the cubes). **Move reBot to
Waypoint** is the joint move. Run **Plan reBot Joint Move** from **Raised**. From
the Zeroed Rest start its planned profile dipped just below joint2's and joint3's
former 0 rad lower limits; that start has not been rechecked against the current
−0.02 and −0.01 rad limits.

## The bench layout file

Every world placement is in [`config/bench_layout.yaml`](config/bench_layout.yaml).
The build copies it unchanged to `description/bench_layout.yaml` and expands
the MuJoCo scene and arm (`description/scene.xml.xacro`,
`description/rebot_arm.xml.xacro`) from that
snapshot into the installed `description/scene.xml` and `rebot_arm.xml`. The
robot description (`description/rebot_bench_sim.urdf.xacro`, through the local
geometry description's `world_to_base_*`, `scene_camera_*` and `wrist_camera_color_optical_*`
xacro arguments) reads the same snapshot, so MoveIt and MuJoCo always see the
layout of the same build.

The world frame is the 3D-printed TF frame object on the mat: +X along its red
axis, +Y along its green axis, +Z (blue) up, z = 0 on the mat's top surface.
Lengths are metres, angles radians, `rpy` is URDF roll-pitch-yaw.

| Entry | Places | Notes |
| --- | --- | --- |
| `robot_base` | `base_link` (the arm's mounting face) | `xyz`, `rpy` |
| `scene_camera` | `scene_camera_link` (x along the view axis, z up) | the colour optical frame is published by a MuJoCo site |
| `wrist_camera` | `wrist_mounted_camera_color_optical_frame` in `gripper_end` | nominal stock Gemini 305 optical pose, no camera housing |
| `mat` | the centre of the mat's top surface | `size` is the full `[x, y, thickness]`; the bench top sits at the mat's underside |
| `wall` | the wall box centre | `size` is the full `[x, y, z]` extent |
| `cubes` | each cube's centre, plus a yaw | `size` (edge) and `mass` apply to both |
| `keyframe` | the `default` keyframe, the start state and the **Reset MuJoCo Sim** target | joint1-joint6, then the gripper motor joint7 |

To change one, edit the file, then rebuild the package before running
(`colcon build --packages-select rebot_bench_sim`, or `moveit_pro build`).
Restarting `moveit_pro run` without a rebuild keeps the previous layout. A
camera or robot-base calibration lands here: express world placements in the
printed frame, or the wrist optical pose in `gripper_end`, and replace the
entry. Cube start poses are world (printed-frame) coordinates like every other placement, not mat-grid coordinates: the cubes are
free bodies that calibration measures directly, and editing `mat`'s `xyz` or
`rpy` does not move them.

**`robot_base` and `scene_camera` retain their CALIBRATED placements**, from a
ChArUco board on the mat. The base's x, y and yaw were solved through the former
D435's provisional hand-eye result, good to about 2 cm; roll, pitch and height
remain level on the bench plane. **`wrist_camera` is NOMINAL**, the stock Gemini
305 optical transform on `gripper_end`, not a transferred D435 calibration.
This package does not configure physical hardware. **Every other value is a
first guess, marked APPROXIMATE**, from bench facts and the real scene
camera's frame rather than measurement. The mat runs along world X, away from
the robot, with the foam wall along its +Y edge; the printed frame lies near
the mat's aisle edge, its red axis pointing back toward the robot. The cubes
are at grid (309, 273) mm (blue) and (212, 302) mm (red), where grid +X runs
across the mat toward the aisle and grid +Y along it away from the robot; the
file gives the grid-to-world mapping and the reasoning behind each entry.

## Cameras

| Camera | MuJoCo camera | Image frame | Topics |
| --- | --- | --- | --- |
| Wrist Gemini 305 colour | `wrist_mounted_camera_color` | `wrist_mounted_camera_color_optical_frame` | `/wrist_mounted_camera/color/{image_raw,camera_info}` |
| Scene OAK-D Pro W 97 | `scene_camera_color` | `scene_camera_color_optical_frame` | `/scene_camera/color/{image_raw,camera_info}`, `/scene_camera/depth/image_rect_raw`, `/scene_camera/depth/color/points` |

The colour topics and frames are the real drivers'. The scene depth image and
point cloud are the real scene camera's `scene_aligned_depth` topics, aligned to
its colour image, from which the episode cycle confirms each stack. The wrist
is RGB-only, matching the stock Gemini driver (no depth, IR or point cloud).
Images render at a shared 30 Hz for validation video; stock Gemini hardware uses
1280x800 MJPG at 10 Hz. This simulator exposes no per-camera rate, so the twin
retains its global recording rate rather than adding a custom throttle. Point
clouds publish at 5 Hz (`point_cloud_publish_rate` in
`description/rebot_bench_sim.urdf.xacro`).
The image frame is the MuJoCo camera's name plus `_optical_frame`, so the
cameras are named after the real frames, and MuJoCo publishes those two frames
from the camera sites: the driver-free geometry leaves them out, which keeps
one TF parent per frame.

The wrist uses the [stock reBot Gemini 305 optical mount](https://github.com/PickNikRobotics/moveit_pro_rebot_ws/blob/main/src/rebot_b601_rs_hw/description/rebot.urdf.xacro):
`gripper_end` to colour optical xyz `-0.111 0.009 0.109` m, rpy
`-2.109 0.003 -1.579` rad. It is a rounded CAD/optical-convention composition,
not a hand-eye calibration. The [Orbbec public specification](https://www.orbbec.com/products/stereo-vision-camera/gemini-305/)
gives nominal vertical FOV 68 degrees (±3 degrees); `fovy=68` produces nominal
pinhole fx/fy 593.024 px at 1280x800, principal point (639.5, 399.5), without lens
distortion. **These are not a physical unit's factory intrinsics.** The stock
hardware driver derives `CameraInfo` from the attached device; calibrated
physical deployments must use that data and their measured mount.

`picknik_mujoco_ros` requires every camera to match one offscreen buffer, now
1280x800. The scene camera retains its pose, frames, topics and 97.1-degree
horizontal view, with fx/fy 565.229 px (twice its measured 640-wide focal length)
and `fovy=70.572362571`. This uses the OAK colour stream's native aspect ratio
instead of the former 640x480 buffer's extra 40 rows above and below. The
`opengl`, `nyx` and `both` renderer modes use the same model and topic contract.

The real drivers publish JPEG `color/image_raw/compressed` topics through
the declared `compressed_image_transport` plugin. MuJoCo publishes raw ROS
images without image transport, so `scripts/sim_compressed_cameras.py`, started
by `launch/sim_drivers.launch.py`, encodes both colour streams onto the same
topics with the same `format`. Installing the plugin does not add compressed
publishers to MuJoCo's raw image publishers. The native JPEG training config
records these topics.

## Model notes

- The arm is `description/rebot_arm.xml.xacro` and `description/rebot_assets.xml`,
  transcribed from the local, driver-free `rebot.urdf.xacro`. Meshes use this
  package's `package://` URIs. The purple printed covers are COLLADA, which
  MuJoCo cannot load, so
  `description/assets/` holds STL conversions in the same frame and units, one
  file per embedded material (purple print, white lettering); see
  [`description/assets/NOTICE.md`](description/assets/NOTICE.md).
- One position actuator per motor joint, with the real driver's motion-mode
  kp/kd and effort limits, except the jaw. The twin's joint7 actuator uses kp 12,
  kv 0.05 and its class's default -14 to 14 Nm force range (see
  `rebot_assets.xml`); the real jaw runs kd 0.5 with a 1 Nm motor torque limit.
  The twin does not model that limit, so it does not validate the real grip
  force. The jaws follow joint7 through equality constraints, the
  URDF mimic. The links are gravity-compensated: a real-arm sag out of the
  folded rest pose can put the arm in MoveIt self-collision.
- Robot geoms collide with the scene but not with each other, since the folded
  rest pose is closer than MuJoCo's convex hulls allow. MoveIt still checks
  self-collision on the real meshes.
- `scene.xml.xacro` enables ten NoSlip postprocessing iterations. MuJoCo's
  soft friction constraints otherwise let a resting, off-center stack creep
  sideways under load even with zero initial velocity. This changes the solver,
  not the cubes' friction, contact softness, placement tolerances or grasp proxy.
  See [MuJoCo's NoSlip option](https://mujoco.readthedocs.io/en/stable/XMLreference.html#option-noslip-iterations).
  The released-stack replay in `test/test_scene_model.py` exercises the complete
  compiled scene for 300 simulated seconds and requires under 0.5 mm cube drift
  and under 5 mm horizontal stack error throughout the sampled trajectory.
- The stack Objectives add the bench, wall, and perceived cubes to MoveIt's
  planning scene. Other Objectives do not add them automatically.

## Tests

- `test/test_scene_model.py` loads the generated scene and checks every MuJoCo
  body and both optical frames against the URDF at three joint configurations,
  the stock nominal wrist pose, 1280x800 intrinsics and RGB-only mode, renderer
  remap parity, frames and topics, the cubes settling on the mat, the
  arm holding its keyframe, the axes triad and the keyframe.
- `test/test_config.py` loads the standalone configuration through MoveIt
  Pro's loader and checks local resources, driver-free dependencies, controller
  declarations and the sim-only launch, with and without the grasp proxy.
- `test/test_model.py`, `test/test_joint_limits.cpp`, `test/test_jaw_legs.cpp`
  and `test/test_stack_objectives.cpp` retain the model/FCL, limits and executable
  pick/stack control-flow regressions.
- `test/test_sim_grasp_proxy.py` drives the grasp proxy through its topics: it
  welds only at finger contact and releases after a simulation reset.
- `test/test_sim_compressed_cameras.py` checks a compressed frame decodes back
  to the raw one, header and format included.
- `test/objectives_integration_test.py` runs runnable Objectives against the
  twin, as `lab_sim` does, after checking controller readiness, except the SAM3
  locate, prepare, pick, stack, unstack, cycle, recording and wall-recovery entry
  points, which the CI image cannot run; see [Unstack and cycle twin runs](#unstack-and-cycle-twin-runs).
  Between tests it clears perceived scene geometry and resets the keyframe.

## SAM3 pick and stack

**Locate reBot Cubes**, **Pick Blue Cube**, **Stack Blue on Red**, **Unstack
Blue from Red** and **Stack and Unstack Cycle** live in this package's
`objectives/` directory and use `rebot_bench_behaviors`. They locate cubes from
calibrated scene RGB and the known mat plane, with no simulation state in
perception. Locate, pick and standalone stack use RGB only. Unstack, the cycle,
and stacked wall recovery also require `/scene_camera/depth/color/points`.

### Build and run

Use a MoveIt Pro workspace with ROS Jazzy and the proprietary overlay sourced:

```sh
colcon build --packages-up-to rebot_bench_sim
source install/setup.bash
colcon test --packages-select rebot_bench_behaviors rebot_bench_sim
colcon test-result --all --verbose
```

SAM3 is optional and not a declared dependency: its model files are built only
after accepting the SAM License through the installed MoveIt Pro configuration
workflow. For perception Objectives, confirm `moveit_pro_sam3` is installed or
build that package separately after accepting its license. Restart the backend
after building. No custom runtime or Direct Motion tools are needed.

Start the named simulation instance above, connect the Desktop App to its
printed endpoint, and run **Locate reBot Cubes**, **Pick Blue Cube** or **Stack
Blue on Red** from Objectives. The supported `run_objective` client can run the
same Objective without the UI:

```json
{"name":"run_objective","arguments":{"objective_name":"Stack Blue on Red"}}
```

A pick leaves blue held above its starting point. A stack releases blue onto
red, retreats and returns to **Zeroed Rest** with the jaw closed. Run **Clear
reBot Stack Scene** before **Reset MuJoCo Sim** after a pick or failed attempt;
reset restores the layout's initial cube poses, not MoveIt's added geometry.
If a pick is still held, run **Open Gripper** before resetting: the simulator's
proxy latch can otherwise carry the held cube through a teleport before its
release command arrives. Confirm cube start poses before a subsequent run.
Stop only this instance with `moveit_pro down --instance rebot-sim`.

The release is planned with a 5-degree tilt; both approach routes and the pick
are collision-checked before motion. All reach helpers keep the existing
2.4 rad joint2 bound. Missing perception or an unreachable pose fails before
motion; post-motion failures attempt home recovery while preserving a possibly
held cube. The explicit grasp-proxy limitation below still applies.

### Planning speed

[`objectives/rebot_stack_motion.yaml`](objectives/rebot_stack_motion.yaml) holds
velocity and acceleration factors for joint, Cartesian and gripper legs.
**reBot Stack Motion** loads it before any motion; each factor defaults to 1.0,
the full limits in `config/moveit/joint_limits.yaml`. Edit that file and rebuild
to change the twin's speed. Missing factors or values outside (0, 1] refuse the
run. Close Gripper uses its own full scaling.

### Recorded episodes

The cycle repeats **Recorded Stack Leg** and **Recorded Unstack Leg**, saving
each episode only after the scene camera confirms the result. It uses the named
training config [`config/training/rebot_bench_jpeg.json`](config/training/rebot_bench_jpeg.json):
`/joint_states`, `/joint_trajectory_admittance_controller/controller_state`, and
both JPEG camera topics at 30 Hz. Install that config once through Trainer's
`POST /train/configs` endpoint or its `configs/` directory. Set `num_cycles`,
`dataset_name`, `training_config` and `recorder_start_s` as Objective inputs.
A failed leg discards its episode and stops the cycle.

### Gemini 305 execution check

Built `rebot_bench_sim` and `rebot_bench_behaviors` against an existing MoveIt
Pro runtime with MuJoCo 3.13.0. The package tests passed:

```sh
colcon test --packages-select rebot_bench_behaviors rebot_bench_sim
colcon test-result --test-result-base build/rebot_bench_behaviors/test_results --all --verbose
colcon test-result --test-result-base build/rebot_bench_sim/test_results --all --verbose
```

Package-scoped result summaries
reported 292 tests, zero failures/errors and 69 expected Objective skips.
The camera contracts and Nyx remap drift test passed, as did native-resolution
cube fitting and depth-based stack confirmation without tolerance changes.

In a named OpenGL simulation instance, wrist RGB, CameraInfo and decoded JPEG
were 1280x800 on `wrist_mounted_camera_color_optical_frame`, with fx/fy
593.024 px. A 15-second probe received 450 advancing wrist frames at 30.00 Hz;
scene RGB/depth were also 1280x800 and scene clouds arrived at 5.00 Hz.

At blue XY (0.360, 0.060) m and red XY (0.400, 0.200) m, **Locate reBot Cubes**,
**Pick Blue Cube** and **Stack Blue on Red** succeeded through `/do_objective`
in 78.9, 105.8 and 150.4 seconds. SAM3 used CPU inference in this runtime.
After release, simulator qpos truth showed 2.318 mm horizontal stack error,
49.892 mm centre-height difference, under 0.029 degrees tilt and under
0.001 mm motion over the final two seconds. The arm returned home with the
jaw closed. The **SIM-ONLY contact-triggered grasp proxy was enabled**; this
checks perception and Objective execution, not physical grip fidelity.
The default layout was restored after validation.

### Earlier standalone execution check

Before the Gemini 305 change, the standalone package was built and tested with
MuJoCo 3.13.0 on an existing MoveIt Pro runtime. **Locate reBot Cubes** and
**Prepare reBot Stack** succeeded at the default placement. The default far pick was refused before motion by
the unchanged 2.4 rad joint2 reach bound (the selected red-cube plan needed
2.487 rad). This cleanup does not widen that bound or change the default layout.

For the documented mid-reach placement (blue XY 0.360, 0.060 m; red XY 0.400,
0.200 m), the same **Pick Blue Cube** and **Stack Blue on Red** completed through
`/do_objective` in 100.7 s and 118.6 s. After release, simulator truth showed
1.872 mm horizontal stack error, 49.918 mm centre-height difference, under
0.043 degrees tilt and under 0.001 mm motion over the final two seconds. The
arm returned home with the jaw closed. The **SIM-ONLY contact-triggered grasp
proxy was enabled**; this proves perception and Objective execution, not
physical gripper fidelity. The default layout was restored afterwards.

### Simulation grasp proxy and physical fidelity gap

`sim_grasp_proxy: true` in `bench_layout.yaml` explicitly enables a **SIM-ONLY**
contact-triggered weld. A twin-only node observes joint7's ordinary controller
reference and the simulator's own state on `/mujoco/qpos_stamped`. It commands
the weld only while joint7 is commanded at or below 2.90 rad **and** it finds the
left finger in contact with the blue cube in that state; MoveIt Pro's MuJoCo
suction interface then welds them at their contact-time relative pose. Opening
above 2.90 rad releases it. The node retains the hold between trajectories,
because JTAC does not publish controller state while idle, and commands release
when it starts and after **Reset MuJoCo Sim** (simulation time going back), since
the suction interface's latch survives a keyframe reset and clears only on a
release command. That reset deactivates every command controller, the proxy's
included, so until they are reactivated the latch can still hold a cube that
was grasped at reset time welded where the keyframe put it; the arm is held still
throughout, and the release reaches MuJoCo within one 20 ms publish period of
reactivation. No Objective calls or knows about the proxy, and it never snaps a
cube to a target pose.

This is an execution/perception test aid, **not a validated physical gripper**.
The unassisted 35 g cube slips in the current contact model at the real 45 mm
closure. A bounded stiff-coupling trial produced 7.0–7.3 N per pad but 1.49 mm
penetration and 7.48 mm lateral drift during a 50 mm lift. The physical regression
`test_45_mm_grip_physically_lifts_a_cube_50_mm` never enables the weld. It is a
strict expected failure before MuJoCo 3.13.0 and must lift the cube on 3.13.0 and
later; that single lift does not validate sustained grasps or the full pick/stack
contact model. Fixing contact fidelity is follow-up work. Set
`sim_grasp_proxy: false` and rebuild to run the twin without assistance.

### Unstack and cycle twin runs

Run by hand in the twin at full planning speed, with the grasp proxy, and
checked from MuJoCo truth. No artifacts from these runs are in the repository.

Historical results on `b727505da1369a0f73ac6e2e9163d690e93ed8b5`, each
from a cold twin restart. These runs predate the current reach settings,
recovery changes and rebase; they do not validate this revision. The rebase
was reviewed and tested without starting a twin or moving the real arm:

- **Unstack Blue from Red, five placements.** Blue stacked on red, with red at
  (0.288, 0.148), (0.400, 0.200), (0.360, 0.060), (0.400, 0.260) and
  (0.310, 0.020). Every run left blue upright on the mat within 3.15 mm of its
  drawn spot, moved red at most 0.077 mm, and ended home with the jaw closed; a
  306 s hold afterwards drifted 0.000 mm. 5 of 5 passed.
- **Stack and Unstack Cycle, five cycles back to back.** Every leg checked from
  truth: each stack left blue on red within 2.82 mm, each unstack left blue
  upright within 2.67 mm of its drawn spot, red moved at most 0.078 mm per leg,
  and the arm was home with the jaw closed between legs; a 306 s hold afterwards
  drifted 0.000 mm. It recorded 10 episodes, 5 per prompt, each with both camera
  keys, at 29.99-30.01 Hz.
- **Blue on the mat just behind red.** Red at (0.343, 0.145), a lone blue at
  (0.40, 0.15), with the cycle's stack check run directly after locating red:
  `IsCubeStacked` rejected it (`median height of 51 mm, below 75 mm`), no episode
  was saved, and the arm did not move.
- **Forced out of reach.** With the stack at x=0.12 y=0.15, Unstack Blue from
  Red refused before any motion with `blue cube at x=0.120 y=0.150 is outside
  the arm's reach`, and the arm stayed home with the jaw closed. With blue at
  x=0.12 y=0.10, the cycle refused before motion and saved no episode.

`scripts/record_stack_evidence.py`, below, scores Stack Blue on Red only;
recording unstack and cycle truth with it is follow-up work.

### Recording validation evidence

Run the recorder **before** starting the Objective and wait for its `READY` line:

```sh
ros2 run rebot_bench_sim record_stack_evidence.py /tmp/stack-run \
  --video --duration 300 --stop-file /tmp/stack-run.stop
```

`--video` pipes frames to an `ffmpeg` CLI with libx264, which the MoveIt Pro image
does not ship: `apt-get install ffmpeg` in the container first. It is not a rosdep
because the `ffmpeg` key also installs FFmpeg -dev packages that uninstall the
image's OpenCV and `cv_bridge`.

It saves each live camera frame to a continuous 30 fps H.264 video, raw simulator
qpos samples, the model schema hash, layout, and before/after camera frames.
Touch the stop file after the released cubes have settled; a bounded duration
also guarantees shutdown. `result.json` reports whether the final two seconds
keep blue within 5 mm horizontally and vertically of a 50 mm stack, both cubes
within 5 degrees of upright, and motion under 1 mm. It explicitly labels
proxy-assisted runs. A successful Objective result alone does not satisfy this
check. Rebuild after changing cube placements and restart the twin before the
next recording.

[Five-placement validation and labelled render pairs](docs/stack_validation/README.md).
