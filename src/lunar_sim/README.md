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
named `platform_velocity_controller`, matching the real robot's controller naming and calibration
(`wheel_separation: 0.562`, `wheel_separation_multiplier: 1.75`, `wheel_radius: 0.1625`,
`wheels_per_side: 2`). `/cmd_vel` (`geometry_msgs/TwistStamped`) and `/odom`
(`nav_msgs/Odometry`) are remapped to those plain top-level topic names from
`platform_velocity_controller`'s own namespaced topics, unchanged from the earlier mock-hardware
setup. `open_loop` is now `false`: MuJoCo reports real per-wheel position/velocity state from
physics, so `/odom` reflects that feedback instead of integrating the commanded velocity.

**Known limitation:** `wheel_separation_multiplier: 1.75` is a real-hardware pavement-slip
calibration constant carried over unchanged for controller-config parity with the physical robot.
Under the old `open_loop: true` mock setup it only affected a command -> twist -> command round
trip and canceled out; under real closed-loop feedback it doesn't, since MuJoCo's simulated
wheel-ground contact has no reason to reproduce the same slip ratio. `/odom`'s reported angular
velocity/yaw is systematically off from the geometrically-correct value for the simulated chassis
by roughly that multiplier. Not corrected here - fixing it means picking a simulated-track-width
constant, which is a calibration decision, not a MuJoCo-migration bug.

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

The ground plane's color map (`description/assets/lunar_regolith_moon01_2k.png`) is
[Poly Haven's "Moon 01"](https://polyhaven.com/a/moon_01) diffuse map, downloaded at 4K and
resized to 2K PNG (MuJoCo's `<texture file="...">` loader only accepts PNG, not JPEG) to keep the
file small; Poly Haven textures are CC0 (public domain). Only the color map is used - MuJoCo's
`<texture type="2d">` is diffuse-only, so the normal/AO/roughness maps in the same asset are not
needed.

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
