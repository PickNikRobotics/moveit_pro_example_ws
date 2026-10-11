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
The build installs a copy of it as `description/bench_layout.yaml` and expands
the MuJoCo scene (`description/scene.xml.xacro`) from that copy into the
installed `description/scene.xml`. The robot description
(`description/rebot_bench_sim.urdf.xacro`, through the base description's
`world_to_base_*` and `scene_camera_*` xacro arguments) reads the same installed
copy, so MoveIt and MuJoCo always see the layout of the same build.

The world frame is the 3D-printed TF frame object on the mat: +X along its red
axis, +Y along its green axis, +Z (blue) up, z = 0 on the mat's top surface.
Lengths are metres, angles radians, `rpy` is URDF roll-pitch-yaw.

| Entry | Places | Notes |
| --- | --- | --- |
| `robot_base` | `base_link` (the arm's mounting face) | `xyz`, `rpy` |
| `scene_camera` | `scene_camera_link` (x along the view axis, z up) | the colour optical frame follows as in the base config |
| `mat` | the mat's grid frame: grid origin on the top surface, +X toward the aisle, +Y away from the robot | `size` is `[x, y, thickness]`; the bench top sits at the mat's underside |
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

**Every value in the shipped file is a first guess, marked APPROXIMATE**, from
bench facts rather than measurement: the mat in front of the robot, the cubes at
grid (309, 273) mm (blue) and (212, 302) mm (red), the printed frame front-left
on the mat beyond the cubes, the scene camera about 0.74 m from the mat. The
file notes the reasoning behind each.

## Cameras

| Camera | MuJoCo camera | Image frame | Topics |
| --- | --- | --- | --- |
| Wrist D435 colour | `d435_color` | `d435_color_optical_frame` | `/wrist_mounted_camera/color/{image_raw,camera_info}`, `/wrist_mounted_camera/depth/image_rect_raw` |
| Scene OAK-D Pro W 97 | `scene_camera_color` | `scene_camera_color_optical_frame` | `/scene_camera/color/{image_raw,camera_info}`, `/scene_camera/depth/image_rect_raw` |

The colour topics and frames are the real drivers'; the depth images are extra
(the real config runs colour only). Both render at 10 Hz, without point clouds.
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

- The arm is `description/rebot_arm.xml` and `description/rebot_assets.xml`,
  transcribed from `rebot.urdf.xacro`. Meshes are the base package's STL files
  through `package://` URIs. The printed covers, wrist bracket and D435 body are
  COLLADA, which MuJoCo cannot load; they are left out and the D435 is a box.
- One position actuator per motor joint, with the real driver's motion-mode
  kp/kd and effort limits. The jaws follow joint7 through equality constraints,
  the URDF mimic. The links are gravity-compensated: a real-arm sag out of the
  folded rest pose puts link2 and link5 in MoveIt collision.
- Robot geoms collide with the scene but not with each other, since the folded
  rest pose is closer than MuJoCo's convex hulls allow. MoveIt still checks
  self-collision on the real meshes.
- MoveIt's planning scene does not contain the mat, wall or cubes.

## Tests

- `test/test_scene_model.py` loads the generated scene and checks every MuJoCo
  body and both optical frames against the URDF at three joint configurations,
  the camera intrinsics, frames and topics, the cubes settling on the mat, the
  arm holding its keyframe, the axes triad and the keyframe.
- `test/test_config_inheritance.py` checks the merged configuration through
  MoveIt Pro's loader.
- `test/objectives_integration_test.py` runs every runnable Objective against
  the twin, as `lab_sim` does.
