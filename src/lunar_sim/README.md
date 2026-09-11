# lunar_sim

A MoveIt Pro configuration for a Clearpath Husky A300 running under MuJoCo physics on a
procedurally cratered lunar regolith heightfield. The only sensor is `husky_scene.xml`'s
mast-mounted, world-fixed `scene_camera`, an overview of the start area and demo route published
as an image stream at the xacro's `render_publish_rate` of 10 Hz; the robot itself carries none
(see the roadmap below). No Nav2 stack either - the only way to drive the base is the
`Dead Reckon Square` objective's open-loop `/cmd_vel` commands.

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

## Roadmap

This is layer 2 (procedural crater heightfield + scattered rocks) of a lunar-environment stack,
built on layer 1's MuJoCo migration. Moon-base structures are a later layer, not started here.
Nav2 is also a later layer: this configuration intentionally ships no navigation stack yet.

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
