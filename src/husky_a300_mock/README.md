# husky_a300_mock

A minimal MoveIt Pro configuration for a Clearpath Husky A300 in a blank world: no sensors, no
environment assets, mock hardware. Built for fast iteration, decoupled from a full simulated
environment.

The robot description composes the real A300 platform body from
[`clearpath_platform_description`](../external_dependencies/clearpath_platform_description)
(vendored from [`clearpathrobotics/clearpath_common@jazzy`](https://github.com/clearpathrobotics/clearpath_common),
BSD-licensed) with a `mock_components/GenericSystem` ros2_control block and a stock
[`diff_drive_controller`](https://control.ros.org/jazzy/doc/ros2_controllers/diff_drive_controller/doc/userdoc.html)
named `platform_velocity_controller`, matching the real robot's controller naming and calibration
(`wheel_separation: 0.562`, `wheel_separation_multiplier: 1.75`, `wheel_radius: 0.1625`,
`wheels_per_side: 2`). `/cmd_vel` (`geometry_msgs/TwistStamped`) and `/odom`
(`nav_msgs/Odometry`) are remapped to those plain top-level topic names from
`platform_velocity_controller`'s own namespaced topics.

## Frames

The ground-contact frame is **`footprint`**, not `base_footprint`. `platform_velocity_controller`
publishes `odom` -> `footprint` (`odom_frame_id: odom`, `base_frame_id: footprint`), and anything
consuming a REP-105 ground frame - TF consumers, Nav2, the controller's own `base_frame_id` - must
use `footprint`.

The vendored description also publishes a `base_footprint` link, but upstream's
`base_footprint_joint` places it 0.30 m above the ground plane rather than on it, so it is unused
here. The correction lives in `description/husky_a300_blank_world.xacro`, which adds `footprint` as
`base_link`'s parent at the FK-measured offset (0.13597 m below `base_link`) while keeping the
vendored package byte-identical to upstream.

The 3D Visualizer's fixed frame is a different thing and is `odom`: with no localization there is
nothing above `odom` in the TF tree, so `config/frontend_settings.yaml` sets `referenceFrame: odom`
to override the frontend's `world` default.

The Pose Jog and Joint Jog UI panels are wired to the `base` group only to avoid a MoveIt Pro
launch crash on an empty jog config; neither is functionally usable on this robot (no IK-capable
group, no per-joint velocity controller).

For detailed documentation see: [MoveIt Pro Documentation](https://docs.picknik.ai/)
