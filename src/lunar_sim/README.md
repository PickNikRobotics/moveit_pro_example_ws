# lunar_sim

A MoveIt Pro configuration for a Clearpath Husky A300 running under MuJoCo physics on a
procedurally cratered lunar regolith heightfield. A world-fixed camera on a visible mast
overlooks the demo route, and front and rear image-based lidars ride on the rover.
The base uses open-loop `/cmd_vel` commands from the `Dead Reckon Square` objective or
the Desktop App's Pose tab. See [Teleoperation](#teleoperation). There is no Nav2 stack.

The base spawns at `husky_scene.xml`'s `default` keyframe rather than the world origin
(`config.yaml`'s `mujoco_keyframe`); that keyframe's own comment records the pose and why it was
picked. `Dead Reckon Square` is open-loop and never resets itself, so after a run - or any drift -
the stock `Reset MuJoCo Sim` objective (from `moveit_pro_objectives`' `objectives/mujoco` library,
loaded in `config.yaml`; it deactivates the controllers around the keyframe reset so
`platform_velocity_controller` never sees the wheels teleport) puts the base back on that start
pose.

The robot description composes the real A300 platform body from
[`clearpath_platform_description`](../external_dependencies/clearpath_common/clearpath_platform_description)
(vendored from [`clearpathrobotics/clearpath_common@jazzy`](https://github.com/clearpathrobotics/clearpath_common),
BSD-licensed) with a `picknik_mujoco_ros/MujocoSystem` ros2_control block
(`description/husky_a300_mujoco.xacro`, physics model in `description/husky_scene.xml` /
`husky_a300.xml`) and a stock
[`diff_drive_controller`](https://control.ros.org/jazzy/doc/ros2_controllers/diff_drive_controller/doc/userdoc.html)
named `platform_velocity_controller`, matching the real robot's controller naming
(`wheel_separation: 0.562`, `wheels_per_side: 2`) but with `wheel_radius` and
`wheel_separation_multiplier` calibrated against MuJoCo rather than carried over from the real
robot. `/cmd_vel` (`geometry_msgs/TwistStamped`) and `/odom` (`nav_msgs/Odometry`) are remapped to
those plain top-level topic names from `platform_velocity_controller`'s own namespaced topics,
unchanged from the earlier mock-hardware setup. `open_loop` is now `false`: MuJoCo reports real
per-wheel position/velocity state from physics, so `/odom` reflects that feedback instead of
integrating the commanded velocity.

**Skid-steer calibration:** the real robot's `wheel_separation_multiplier: 1.75` compensates for
pavement scrub friction during a skid-steer turn; it does not carry over to MuJoCo's
regolith-plane contact model, which has different (measured: higher) turning resistance. Using
1.75 unchanged under real closed-loop feedback (`open_loop: false`) makes commanded and true
motion diverge - `/odom` reports the *commanded* twist, not the chassis's *true* motion, since
both are computed from the same wheel encoders via the same (wrong) parameters and round-trip by
construction; only the MuJoCo chassis pose itself (freejoint `xpos`/`xquat` read from MuJoCo
directly - not `/odom`, and not TF either, since MuJoCo's only world edge is the identity
`mj_world -> odom`, so TF carries the same wheel odometry, see the xacro's TF-ownership comment)
exposes the error. Measured directly: a 3 s,
0.5235988 rad/s (30 deg/s) commanded turn achieved only ~59-61 deg of true chassis rotation with
`wheel_separation_multiplier: 1.75`. Recalibrated against that ground truth (see
`husky_a300.ros2_control.yaml`'s comment for the method):
`wheel_radius: 0.1645` (was 0.1625, straight-line rolling resistance) and
`wheel_separation_multiplier: 2.57` (was 1.75, in-place-turn scrub resistance) bring a single
corner to ~90 deg (measured 89.9 deg) and hold across a full 4-corner Dead Reckon Square (each
corner within ~2-5 deg of 90 deg; the turn is intrinsically stick-slip-sensitive to the
multiplier at the ~0.1% level, so a few degrees of residual error per corner is the practical
floor of this contact model, not an unconverged fit).

**Calibration recheck on the cratered heightfield (layer 2):** the values above were calibrated on
the flat plane; re-running the same Dead Reckon Square (RECORDED STATE - chassis freejoint
`xpos`/`xquat`, not `/odom`, for the reason above) on the cratered terrain with `wheel_radius`/
`wheel_separation_multiplier` left unchanged shows a real, measurable regression: per-corner turns
of 83.4, 111.5, 86.4, 115.4 deg (flat-ground baseline: each within ~2-5 deg of 90) and a 1.071 m
closure error over the full square (flat-ground baseline: near-zero). This is expected - the
terrain now has real slope and rim geometry under the wheels, unlike the flat plane's uniform
contact - and is shipped as-is rather than retuned: retuning against one seed's specific crater
layout would overfit that layout rather than the contact model. See `validate_and_render.py` to
reproduce.

The four `outdoor` wheel joints (`front_left_wheel_joint` etc., named to match
`clearpath_platform_description/urdf/a300/drivetrain/wheels/outdoor.urdf.xacro`) each carry a
MuJoCo velocity actuator (`kv=500`) on a hinge with `armature=1.0`; wheel collision is a primitive
cylinder (matching upstream's own URDF collision choice), with the vendored `outdoor_{left,right}.stl`
as a non-colliding visual mesh. The chassis is a free-floating body (MuJoCo `freejoint`) resting on
those four wheels under gravity - there is no virtual planar rail like hangar_sim's mecanum base,
since a plain 4-wheel skid-steer base doesn't need one. `description/assets/{chassis_collision,
outdoor_left,outdoor_right}.stl` are byte-identical copies of the vendored meshes (MuJoCo mesh
paths don't survive a colcon install split across package share directories, so they're copied
into this package rather than referenced cross-package - see hangar_sim's own `description/assets`
for the same pattern).

The ground plane's color map (`description/assets/lunar_regolith_untiled.png`, built by
`description/generate_ground_colormap.py`) covers the whole 20x20 m plane once
(`texrepeat="0.05 0.05"` - with `texuniform="true"` that value is repeats per metre, so 1/20 m
draws the image exactly once) at real lunar photo detail with no repeating tile anywhere - built
from nine real lunar surface photographs rather than a terrestrial stand-in or a single repeating tile:
**NASA Apollo 15/17 Hasselblad frames**, including AS15-86-11671 (Apollo 15, Station 7, Spur
Crater, EVA-2 - the "Genesis Rock" in-situ documentation photo). Public domain, NASA/JSC;
archive.org collection `johnsonspacecentermediaarchive`. All nine cropped source frames and their
full exclusion/rejection notes (near-duplicate content, boulder-dominated frames, hardware/people
in frame) are committed under `description/assets/ground_colormap_sources/` and
`description/assets/ground_colormap_provenance.txt`.

Earlier attempts used terrestrial CC0 photo scans (ambientCG/Poly Haven gravel, sand, and
desiccated-mudflat sets) at the right tile-scale detail but didn't read as lunar on inspection -
they're photographs of Earth ground, not the Moon. A single repeating Apollo tile (this ground's
first iteration, `texrepeat="0.3 0.3"`) fixed the "reads as lunar" problem but showed an obvious
~3.3 m repeat grid; nine distinct frames, randomly placed/rotated/mirrored across the whole plane,
fixes that without needing more than nine real photographs.

`generate_ground_colormap.py` is a reproducible, seeded, committed build step (unlike the earlier
single-frame version, since none of its choices are by-eye): it crops-and-cleans each committed
source frame once (removing Hasselblad reseau-plate fiducial lines, flat-fielding, desaturating to
a neutral ~0.49 grey albedo, then equalizing every frame's contrast to one reference frame's own -
process_tile alone matches every frame's mean but not its dynamic range, and the nine source photos
vary a lot there, which read as a light/dark patchwork at the frame pitch once placed), then
randomly places/rotates/mirrors those frames across a grid covering the whole plane - each
placement also jittered off its nominal grid position so there's no regular spacing for the eye to
lock onto - with feathered (blended) edges between placements, and finally bakes in sparse 10-45cm
soft craterlets and 3-12cm rocks shaded consistent with `husky_scene.xml`'s sun direction (~74
degree elevation - verified against the chassis's own cast shadow in a rendered frame). Re-run it
with `--plane-m`/`--px-per-m` to regenerate at a different plane size or resolution (default: 20m
plane, ~2.4mm/px, matching the ground plane's current size). That is finer than the fixed
`scene_camera` can show (roughly 9 mm per rendered pixel at its range); the detail is there for the
planned robot-mounted cameras, which will look at the ground from wheel height for visual-odometry
evaluation against the simulation's ground truth - if the clone weight of the 81 MB asset matters
more than that, re-run the generator with a lower `--px-per-m` and replace the asset.
`verify_ground_colormap.py` checks the output has no periodic repeat (FFT autocorrelation +
template matching, adapted from the same check used on `generate_terrain.py`'s heightfield); a
downsample-to-256px std, printed by the generator itself, checks there's no residual low-frequency
patchwork from mismatched per-frame exposure either.

## Ground terrain

The ground is a procedurally generated heightfield (`description/assets/lunar_hfield.png`, 1000x1000
px over the 20x20 m plane, ~2 cm cells) plus a scattered rock library, both built by
`description/generate_terrain.py` - an offline, seeded script (numpy/scipy/PIL only) whose output
is committed as ordinary assets; nothing is generated at sim load time. Re-run it (`--seed`
controls the layout) to regenerate; see the script's own docstring for the follow-up steps
(prettier on the two `*_generated.xml` includes, and copying its printed `<hfield>`/ground `<geom>`
size/pos values into `husky_scene.xml` if `elevation_z`/`z_min` change).

Craters are sampled from a truncated power-law size-frequency distribution (600 craters, diameters
0.05-2 m) stamped as a paraboloid bowl plus a raised, cosine-falloff rim, each referenced to its
own local pre-existing terrain height so later craters correctly cut across earlier rims instead of
summing unboundedly. A few octaves of Gaussian-blurred noise add gentle low-frequency undulation.
200 rocks (a 6-shape procedural mesh library, 8 discrete size buckets so many instances share a
handful of loaded meshes rather than each getting a unique one) are scattered across the plane,
excluded from a keep-out box around the Dead Reckon Square's footprint (`x` -1..3 m, `y` -1..3 m)
so the calibration recheck below isolates the heightfield's own effect rather than a wheel snagging
a rock.

VRAM: not directly measured on the target RTX 4060 8 GB (this repo's dev hardware differs and the
measurement tooling available couldn't attribute per-container GPU memory here) - accepted on an
analytical estimate instead. Two items dominate: the existing `shadowsize="8192"` shadow map
(~268 MB), and the 8241x8241 RGB ground colour map (~204 MB as held in `mjModel`, ~200-270 MB of
VRAM once uploaded, the upper end if the driver builds a full mipmap chain). The heightfield mesh
(~1M vertices) and the 48 small rock meshes add well under 100 MB combined. That puts the total
around 550-640 MB - still inside the 1 GB budget, but the colour map has spent most of the headroom
the earlier flat-plane scene had, so a second asset of this size would need the budget revisited.

## Camera mast and lidar sensors

The fixed `scene_camera` keeps its original position `-1 -5 5`, orientation and field of view.
A 4.9 m post, ground base, bracket and camera housing now make its support visible.
The post sits behind the optical center, clear of the overview image.

MujocoSystem publishes these topics at a configured 10 Hz:

| Sensor | Topics | Frame |
| --- | --- | --- |
| Scene camera | `/scene_camera/color`, `/scene_camera/depth`, `/scene_camera/camera_info` | `scene_camera_optical_frame` |
| Front lidar | `/lidar_front/points` | `lidar_front_optical_frame` |
| Rear lidar | `/lidar_rear/points` | `lidar_rear_optical_frame` |

Live checks on the shared development host measured about 7.2 Hz front lidar, 6.0 Hz
rear lidar and 5.6 Hz scene images. The configured rate is a ceiling, not a guaranteed
throughput. Both lidar clouds contain finite returns within the configured range.
At the starting pose, the rear scanner sees mostly open sky, so its cloud is sparse.

The lidar positions follow the vendored Clearpath A300 accessory mounts:

- Front: `enclosure_front_lidar_mount` in
  [`amp_enclosure.urdf.xacro`](../external_dependencies/clearpath_common/clearpath_platform_description/urdf/a300/attachments/amp_enclosure.urdf.xacro),
  at chassis coordinates `0.4 0 0.2708593`. A bracket offsets the scan center 50 mm forward
  and 50 mm up to `0.45 0 0.3208593`, looking along rover +X.
- Rear: `sensor_arch_lidar_mount` in
  [`amp_sensor_arch.urdf.xacro`](../external_dependencies/clearpath_common/clearpath_platform_description/urdf/a300/attachments/amp_sensor_arch.urdf.xacro).
  The arch attaches to `enclosure_antenna_mount`. The composed mount position is
  `-0.2728 0 0.7100193`; the scan center hangs 60 mm below it at
  `-0.2728 0 0.6500193`, looking along rover -X.

Each scanner uses a MuJoCo depth camera with `user="2 270 0.05 25"`, a 270-degree sweep
and range limits of 0.05 to 25 m. The renderer's near clip (`znear` times the model
extent, about 0.28 m) is the effective minimum range: anything closer, including the
housing caps, is not rendered. `fovy="70"` tiles the sweep into three renders within
the scene's 1280x720 offscreen buffer. `resolution="811 3"` selects 811 horizontal beams
and three vertical rows, but the upstream projection places the outer rows at plus and
minus 35 degrees, on the render image boundary, and drops them. Each cloud is therefore
effectively one horizontal scan line at the mount height. The camera's optical site flips
local Y and Z into ROS optical coordinates. The chassis and arch cause real
self-occlusion beyond the near clip. The housings have no collision or added mass,
preserving the existing lumped chassis dynamics.

This requires image-based lidar support in `picknik_mujoco_ros`, introduced by MoveIt Pro
PR 22320. The main runtime image's installed 10.2.0 package provides `THREE_D_LIDAR = 2`.
See the [MuJoCo configuration guide](https://docs.picknik.ai/how_to/configuration_tutorials/create_robot_sim_config/migrate_to_mujoco_config/)
for the camera user fields and `point_cloud_publish_rate`.

## Route boulders

Four large boulders surround the demo route, in addition to the 200 small scattered rocks.
They reuse four committed procedural rock meshes at larger scales and the existing Apollo
ground material. The lower part of each mesh is buried in the sampled terrain height.
See `description/boulders_assets.xml`, `description/boulders_geoms.xml` and
[`boulders_provenance.txt`](description/assets/boulders_provenance.txt) for scales,
placements and how the assets were made. Nothing is generated at runtime.

`description/validate_and_render.py` checks the compiled camera/site transforms, the
lidar tiling derived from the compiled camera parameters, and the front central render's
axial depth. Radial ranges are checked separately in live point clouds. It also records chassis
positions and turns during the demo square and rejects any boulder contact. The before/after square results match:
0.6290 m closure error and turns of 112.42, 89.72, 94.06 and 41.22 degrees on the current
terrain. These are an unchanged open-loop baseline, not a claim of accurate square tracking.

The live `Dead Reckon Square` objective also completed all 80 forward commands and 40
turn commands, with 7.370 m of wheel-odometry travel and 6.365 rad of accumulated yaw.
A separate Pose Jog check through `Request Teleoperation` forwarded 40 commands at
0.2 m/s, recorded 0.905 m of odometry displacement, and published a zero twist on
completion. These odometry checks verify the command paths; the chassis freejoint
comparison above checks physical route regression.

![Labelled views of the camera mast, lidar mounts and four route boulders](description/assets/sensor_sheet.png)

## Roadmap

This is layer 2 (procedural crater heightfield + scattered rocks) of a lunar-environment stack,
built on layer 1's MuJoCo migration, with [moon base structures](#moon-base) beyond the demo route.
Nav2 is also a later layer: this configuration intentionally ships no navigation stack yet.

## Moon base

Five static bodies sit beyond the Dead Reckon Square's far corner: a cylindrical
habitat with a south-facing door, a small lander on a circular landing pad, a
tilted solar array, and an antenna mast. The habitat is centred at world x = 0,
y = 6.8 m; the array is west of it and the lander is east. The base's collision
geometry is also its visible geometry, all in rendered group 1, for future lidar
and camera localization landmarks and navigation obstacles.

The structures rest on the committed cratered terrain, with buried supports.
They leave at least 2 m of clearance around the route's far corner. A future
navigation demo can drive from the default start at -0.30, 0.50 m toward the
habitat door, stopping near 0, 3.8 m with heading +90 degrees. This package still
has no Nav2 stack.

The two include fragments are `description/moon_base_assets.xml` and
`description/moon_base_geoms.xml`. All new geometry uses simple MuJoCo shapes and
untextured materials with approximate albedo. See
[moon base provenance and placement](description/moon_base_provenance.md) for
coordinates, ground-height sampling, and the material limitations.

[Validation results and labelled renders](docs/moon_base_validation.md) record the
unchanged square closure and turn measurements, clearance checks, and sensor rays.
`description/validate_moon_base.py` reproduces those checks and the render sheet
after running the existing `validate_and_render.py`.

## Frames

The ground-contact frame is **`footprint`**, not `base_footprint`. `platform_velocity_controller`
publishes `odom` -> `footprint` (`odom_frame_id: odom`, `base_frame_id: footprint`), and anything
consuming a REP-105 ground frame - TF consumers, Nav2, the controller's own `base_frame_id` - must
use `footprint`.

The vendored description also publishes a `base_footprint` link, but upstream's
`base_footprint_joint` places it 0.30 m above the ground plane rather than on it, so it is unused
here. The correction lives in `description/husky_a300_mujoco.xacro`, which adds `footprint` as
`base_link`'s parent at the FK-measured offset (0.13597 m below `base_link`) while keeping the
vendored package byte-identical to upstream. The correction can't reuse the name `base_footprint`
- URDF requires unique link names, and `check_urdf` rejects the duplicate - hence the
otherwise-unconventional `footprint` name.

The 3D Visualizer's fixed frame is a different thing and is `odom`: with no localization the only
frame above `odom` is MuJoCo's `mj_world`, published as identity so the world-fixed
`scene_camera_optical_frame` resolves in `odom`, and `config/frontend_settings.yaml` sets
`referenceFrame: odom` to override the frontend's `world` default.

## Teleoperation

The Pose Jog and Joint Jog UI panels are wired to the `base` group to avoid a MoveIt Pro launch
crash on an empty jog config. Joint Jog stays unusable on this armless base (no per-joint velocity
controller), but Pose Jog drives the Husky base: the Pose tab's jog pad, a connected gamepad, and
the Quest headset all publish a `VelocityForceCommand` on `/pose_jog/base`, and the
`TeleoperateBase` Behavior (`lunar_sim_behaviors`) forwards `linear.x`/`angular.z` from that stream
into a `TwistStamped` on `/cmd_vel` for `platform_velocity_controller`.

The Pose tab's controls are labeled for an end-effector, since that is what the panel is designed
for; on this base the labels map to driving as follows:

- Jog pad: `X +` drives forward, `X -` reverses; `Roll +` turns left, `Roll -` turns right. The
  `Y`, `Z`, `Pitch`, and `Yaw` buttons do nothing.
- Gamepad: the left stick left/right drives forward/reverse; the bumpers turn left/right. The right
  stick and triggers do nothing.

The Joint, IMarker, and Waypoints tabs do nothing on this base - they operate on wheel joints or
wheel-link poses, which have no meaningful teleoperation behavior for a differential-drive base.

Speed is capped in two places: the Pose tab's speed slider (0.25 m/s max by default, since this
config has no Cartesian velocity limit parameter for the slider to read), and the
`TeleoperateBase` Behavior's own clamp ports (0.8 m/s linear, 2.0 rad/s angular, matching
`platform_velocity_controller`'s limits).

For detailed documentation see: [MoveIt Pro Documentation](https://docs.picknik.ai/)
