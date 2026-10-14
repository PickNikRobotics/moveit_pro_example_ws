# reBot bench simulation

A MuJoCo twin of the reBot bench, layered over
[`rebot_bench_base_config`](../rebot_bench_base_config/README.md): MoveIt Pro
drives the reBot arm and gripper in MuJoCo through `picknik_mujoco_ros`, the
same way `lab_sim` drives its UR. The twin holds the arm, the wrist D435 and
the scene OAK-D Pro W 97 cameras, the blue mat, the white wall, a red and a
blue 50 mm cube, and an axes triad at the world origin.

![Scene camera, wrist camera and third-person renders at the default keyframe](docs/render_sheet.png)

```bash
moveit_pro run --config-package rebot_bench_sim
```

The controllers, joint limits, SRDF, waypoints and Objectives are the base
config's. On top of them this package adds the MuJoCo behaviors, **Reset MuJoCo
Sim**, and **Move reBot to Pose**, a Cartesian move of the gripper to a pose in
`base_link` (by default pointing straight down above the cubes). **Move reBot to
Waypoint** is the joint move. Run **Plan reBot Joint Move** from **Raised**: from
the Zeroed Rest start, joint2 and joint3 sit on their 0 rad lower limits and the
planned profile dips just below them.

## The bench layout file

Every world placement is in [`config/bench_layout.yaml`](config/bench_layout.yaml).
The build installs a snapshot of it as `description/bench_layout.yaml`, adding
the wrist-camera mount joint derived from `wrist_camera`
(`scripts/layout_snapshot.py`), and expands the MuJoCo scene and arm
(`description/scene.xml.xacro`, `description/rebot_arm.xml.xacro`) from that
snapshot into the installed `description/scene.xml` and `rebot_arm.xml`. The
robot description (`description/rebot_bench_sim.urdf.xacro`, through the base
description's `world_to_base_*`, `scene_camera_*` and `wrist_camera_mount_*`
xacro arguments) reads the same snapshot, so MoveIt and MuJoCo always see the
layout of the same build.

The world frame is the 3D-printed TF frame object on the mat: +X along its red
axis, +Y along its green axis, +Z (blue) up, z = 0 on the mat's top surface.
Lengths are metres, angles radians, `rpy` is URDF roll-pitch-yaw.

| Entry | Places | Notes |
| --- | --- | --- |
| `robot_base` | `base_link` (the arm's mounting face) | `xyz`, `rpy` |
| `scene_camera` | `scene_camera_link` (x along the view axis, z up) | the colour optical frame follows as in the base config |
| `wrist_camera` | `d435_link` in `link6` (a hand-eye result) | the printed bracket moves with it |
| `mat` | the centre of the mat's top surface | `size` is the full `[x, y, thickness]`; the bench top sits at the mat's underside |
| `wall` | the wall box centre | `size` is the full `[x, y, z]` extent |
| `cubes` | each cube's centre, plus a yaw | `size` (edge) and `mass` apply to both |
| `keyframe` | the `default` keyframe, the start state and the **Reset MuJoCo Sim** target | joint1-joint6, then the gripper motor joint7 |

To change one, edit the file, then rebuild the package before running
(`colcon build --packages-select rebot_bench_sim`, or `moveit_pro build`).
Restarting `moveit_pro run` without a rebuild keeps the previous layout. A
camera or robot-base calibration lands here: express the measured pose in the
printed frame and replace the entry. Cube start poses are world (printed-frame)
coordinates like every other placement, not mat-grid coordinates: the cubes are
free bodies that calibration measures directly, and editing `mat`'s `xyz` or
`rpy` does not move them.

**`robot_base`, `scene_camera` and `wrist_camera` are calibrated, marked
CALIBRATED**, from a ChArUco board on the mat. The wrist camera mount is a
provisional hand-eye solve over six arm poses, good to about 2 cm; the robot
base's x, y and yaw are solved through it, and its roll, pitch and height are
held level on the bench plane. The real configuration uses the same base pose
but keeps the nominal wrist camera mount; applying this one on the arm is
follow-up work. **Every other value
is a first guess, marked APPROXIMATE**, from bench facts and the real scene
camera's frame rather than measurement. The mat runs along world X, away from
the robot, with the foam wall along its +Y edge; the printed frame lies near
the mat's aisle edge, its red axis pointing back toward the robot. The cubes
are at grid (309, 273) mm (blue) and (212, 302) mm (red), where grid +X runs
across the mat toward the aisle and grid +Y along it away from the robot; the
file gives the grid-to-world mapping and the reasoning behind each entry.

## Cameras

| Camera | MuJoCo camera | Image frame | Topics |
| --- | --- | --- | --- |
| Wrist D435 colour | `d435_color` | `d435_color_optical_frame` | `/wrist_mounted_camera/color/{image_raw,camera_info}`, `/wrist_mounted_camera/depth/image_rect_raw` |
| Scene OAK-D Pro W 97 | `scene_camera_color` | `scene_camera_color_optical_frame` | `/scene_camera/color/{image_raw,camera_info}`, `/scene_camera/depth/image_rect_raw` |

The colour topics and frames are the real drivers'; the scene depth is extra
(the real scene camera is RGB-only); the real wrist D435 provides aligned depth. Both render at 30 Hz, without point clouds, for continuous validation video.
The image frame is the MuJoCo camera's name plus `_optical_frame`, so the
cameras are named after the real frames, and MuJoCo publishes those two frames
from the camera sites: with `hardware_interface: mujoco` the base description
leaves them out, which keeps one TF parent per frame.

Intrinsics follow the base config's measured values. The wrist camera's
`fovy` 43.09 deg gives its fy of 607.8 px at 640x480. The scene camera's
`fovy` 80.69 deg gives fx 282.5 px, the OAK's full 97.1 deg across 640 px.
`picknik_mujoco_ros` renders every camera into one offscreen buffer, so both
cameras are 640x480. The real scene stream is 640x400, the same view without
the top and bottom 40 rows.

## Model notes

- The arm is `description/rebot_arm.xml.xacro` and `description/rebot_assets.xml`,
  transcribed from `rebot.urdf.xacro`. Meshes are the base package's STL files
  through `package://` URIs. The purple printed covers, the wrist bracket and
  the D435 body are COLLADA there, which MuJoCo cannot load, so
  `description/assets/` holds STL conversions in the same frame and units, one
  file per embedded material (purple print, white lettering); see
  [`description/assets/NOTICE.md`](description/assets/NOTICE.md).
- One position actuator per motor joint, with the real driver's motion-mode
  kp/kd and effort limits. The jaws follow joint7 through equality constraints,
  the URDF mimic. The links are gravity-compensated: a real-arm sag out of the
  folded rest pose puts link2 and link5 in MoveIt collision.
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
  the camera intrinsics, frames and topics, the cubes settling on the mat, the
  arm holding its keyframe, the axes triad and the keyframe.
- `test/test_config_inheritance.py` checks the merged configuration through
  MoveIt Pro's loader, including that the layout, not the base config's bench
  calibration, places the twin's robot description.
- `test/test_sim_grasp_proxy.py` drives the grasp proxy through its topics: it
  welds only at finger contact and releases after a simulation reset.
- `test/objectives_integration_test.py` runs every runnable Objective against
  the twin, as `lab_sim` does, except the stateful stack entry points, which
  the evidence runs below validate.

## SAM3 pick and stack

The shared **Locate reBot Cubes**, **Pick Blue Cube**, and **Stack Blue on Red**
Objectives live in `rebot_bench_base_config`; see its
[run recipe](../rebot_bench_base_config/README.md#sam3-blue-on-red-stack).
They use calibrated scene RGB and the known mat plane, with no scene depth or
simulation state in perception. The same XML, waypoints, and
`rebot_bench_behaviors` plugin are used on hardware.

### Planning speed

Speed is a separate setting from hardware. The base config's
`objectives/rebot_stack_motion.yaml` holds the arm's per-leg scale factors,
which have to fit under the bench's joint speed guard (its README says how to
size one against the other). Simulation has no guard, so this package ships its
own [`objectives/rebot_stack_motion.xml`](objectives/rebot_stack_motion.xml),
which shadows the base **reBot Stack Motion** (when two Objective libraries hold
the same file name, the later one wins, and this package's library is listed
last) and loads [`objectives/rebot_stack_motion.yaml`](objectives/rebot_stack_motion.yaml)
from this package instead: 1.0 velocity and acceleration for the joint,
Cartesian and gripper legs, the full limits in the base config's
`config/moveit/joint_limits.yaml`, whatever the arm's values. To run the twin
at another speed, edit that file and rebuild `rebot_bench_sim`; the arm's
values are unaffected. `test/test_config_inheritance.py` checks that the
override wins.

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
cube to a target pose. The real configuration neither launches the proxy nor
loads its controller or model.

This is an execution/perception test aid, **not a validated physical gripper**.
The unassisted 35 g cube slips in the current contact model at the real 45 mm
closure. A bounded stiff-coupling trial produced 7.0–7.3 N per pad but 1.49 mm
penetration and 7.48 mm lateral drift during a 50 mm lift. The physical regression
`test_45_mm_grip_physically_lifts_a_cube_50_mm` remains a strict expected failure;
it never enables the weld. Fixing contact fidelity is follow-up work. Set
`sim_grasp_proxy: false` and rebuild to run the twin without assistance.

### Recording validation evidence

Run the recorder **before** starting the Objective and wait for its `READY` line:

```sh
ros2 run rebot_bench_sim record_stack_evidence.py /tmp/stack-run \
  --video --duration 300 --stop-file /tmp/stack-run.stop
```

`--video` pipes frames to an `ffmpeg` CLI with libx264, which the MoveIt Pro image
does not ship: `apt-get install ffmpeg` in the container first. It is not a rosdep
because the `ffmpeg` key also installs FFmpeg -dev packages that uninstall the
image's OpenCV, `cv_bridge`, and `realsense2_camera`.

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
