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

For detailed documentation see: [MoveIt Pro Documentation](https://docs.picknik.ai/)
