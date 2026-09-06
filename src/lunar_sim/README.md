# lunar_sim

A MoveIt Pro configuration for a Clearpath Husky A300 running under MuJoCo physics on a flat
lunar regolith plane: no sensors, no terrain features yet (see the roadmap below). No Nav2 stack
either - the only way to drive the base is the `Dead Reckon Square` objective's open-loop
`/cmd_vel` commands.

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
construction; only the MuJoCo chassis pose itself (freejoint `xpos`/`xquat`, not `/odom`) exposes
the error. Measured directly: a 3 s, 0.5235988 rad/s (30 deg/s) commanded turn achieved only
~59-61 deg of true chassis rotation with `wheel_separation_multiplier: 1.75`. Recalibrated against
that ground truth (see `husky_a300.ros2_control.yaml`'s comment for the method):
`wheel_radius: 0.1645` (was 0.1625, straight-line rolling resistance) and
`wheel_separation_multiplier: 2.57` (was 1.75, in-place-turn scrub resistance) bring a single
corner to ~90 deg (measured 89.9 deg) and hold across a full 4-corner Dead Reckon Square (each
corner within ~2-5 deg of 90 deg; the turn is intrinsically stick-slip-sensitive to the
multiplier at the ~0.1% level, so a few degrees of residual error per corner is the practical
floor of this contact model, not an unconverged fit).

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

The ground plane's color map (`description/assets/lunar_regolith_as15-86-11671_2k.png`) is built
from a real lunar surface photograph rather than a terrestrial stand-in: **NASA Apollo 15
Hasselblad frame AS15-86-11671** (Station 7, Spur Crater, EVA-2 - the "Genesis Rock" in-situ
documentation photo, taken with the 60mm lens at a sun elevation of 31 degrees). Public domain,
NASA/JSC; archive.org identifier
[`AS15-86-11671`](https://archive.org/details/AS15-86-11671), collection
`johnsonspacecentermediaarchive`.

Earlier attempts used terrestrial CC0 photo scans (ambientCG/Poly Haven gravel, sand, and
desiccated-mudflat sets) at the right tile-scale detail but didn't read as lunar on inspection -
they're photographs of Earth ground, not the Moon. AS15-86-11671 is a genuine ~1-2 m wide,
straight-down shot of undisturbed regolith (cropped clear of the gnomon, the rock sample, and any
footprints), so it carries real lunar grain and lighting instead of an approximation.

Processing (see `inpaint_reseau.py`/`prep_base2.py`/`stamp_craterlets.py`, kept out of this PR -
not a reproducible build step since it starts from a manually-inspected external download and
several by-eye crop/threshold choices; steps below are exact):
1. Crop a 1650x1650 square clear of the gnomon, the rock fragment, and the footprint/scuff area.
2. Remove the Hasselblad reseau-plate fiducial grid lines (thin calibration lines etched across
   the whole frame by the camera, not scene content): detect them as sharp, image-spanning spikes
   in the column/row median profile, then replace each line band with a same-width band of real
   texture shifted in from just outside it (not a blur/diffusion fill, which flattens the grain
   into a visible soft stripe).
3. Flat-field (divide out a heavily-blurred illumination estimate) and desaturate to a neutral,
   slightly warm-grey albedo (~0.49), matching every prior lunar_sim texture round.
4. Make it seamless via toroidal offset-and-blend tiling, resized to 2048px (kept high-resolution
   so ground-level close-ups still show real grain instead of blurring to mush).
5. Bake in sparse 10-45cm soft craterlets (raised rim, short cast shadow) and 3-12cm small rocks,
   both shaded consistent with `husky_scene.xml`'s sun direction (~74 degree elevation) - verified
   against the chassis's own cast shadow in a rendered frame.

## Roadmap

This is layer 1 (MuJoCo migration) of a lunar-environment stack. Crater terrain and moon-base
structures are later layers, gated on a maintainer decision on shape and not started here.

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

The 3D Visualizer's fixed frame is a different thing and is `odom`: with no localization there is
nothing above `odom` in the TF tree, so `config/frontend_settings.yaml` sets `referenceFrame: odom`
to override the frontend's `world` default.

The Pose Jog and Joint Jog UI panels are wired to the `base` group only to avoid a MoveIt Pro
launch crash on an empty jog config; neither is functionally usable on this robot (no IK-capable
group, no per-joint velocity controller).

For detailed documentation see: [MoveIt Pro Documentation](https://docs.picknik.ai/)
