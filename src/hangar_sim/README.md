# hangar_sim

A MoveIt Pro MuJoCo simulation for PickNik's Universal Robots (UR) arms.

For detailed documentation see: [MoveIt Pro Documentation](https://docs.picknik.ai/)

## Build Map

Keep `hardware.robot_description.urdf_params`'s `slam` option in `config/config.yaml` set to `false` and localization enabled (the default). **Build Map** temporarily switches from localization to SLAM; it is unavailable with localization disabled or SLAM already running. It requires the paired MoveIt Pro Build Map changes (PR #22585), including the custom mapping pane and `SwitchLifecycleManagers`'s `reset_source` support.

Run **Build Map** from the Desktop App's mapping pane:

1. Select **Start** to pause navigation and start mapping using the front lidar.
2. Drive with **Forward**, **Reverse**, **Turn left**, and **Turn right** under **Pose** in Teleoperation, previewing the SLAM map.
3. Stop the robot, then select **Continue** to save `hangar_map.pgm` and `hangar_map.yaml` in the Runtime user workspace. **Abort** ends mapping without saving.

Localization is reset during mapping so the shipped map cannot compete with SLAM's `/map` when saving. After mapping ends, the workflow restores localization against the shipped map and refines the robot's pose; navigation resumes only if localization restoration and refinement succeed. Saving does not replace the shipped map or select the new map for navigation. A successful save alone does not establish successful recovery, and the existing scan-fit rejection remains in place.

## Base teleoperation

Open the MoveIt Pro Desktop App's Teleoperation workspace and select **Base** to drive forward, reverse, or turn. `config/moveit/base_jog.yaml` sets the speed caps; the Teleoperation Speed control scales them. BaseJog does not perform collision checking or support sideways strafing.

The Base tab drives its own controller, `platform_velocity_controller_base_jog`, which ramps commands with its acceleration limits. It claims the same wheels as the navigation and whole-body base controllers, so opening the Base tab deactivates them and only one source drives the base at a time.

Navigation uses nav2's MPPI controller with `open_loop: true` (see `params/nav2_params.yaml`), which needs `nav2_mppi_controller` 1.3.13 or newer. The workspace `Dockerfile` installs that build; older builds ignore the parameter without error and drive the base well below `vx_max`.
