# hangar_sim

A MoveIt Pro MuJoCo simulation for PickNik's Universal Robots (UR) arms.

For detailed documentation see: [MoveIt Pro Documentation](https://docs.picknik.ai/)

## Base teleoperation

Open the MoveIt Pro Desktop App's Teleoperation workspace and select **Base** to drive forward, reverse, or turn. `config/moveit/base_jog.yaml` sets the speed caps; the Teleoperation Speed control scales them. BaseJog does not perform collision checking or support sideways strafing.

The Base tab drives its own controller, `platform_velocity_controller_base_jog`, which ramps commands with its acceleration limits. It claims the same wheels as the navigation and whole-body base controllers, so opening the Base tab deactivates them and only one source drives the base at a time.

Navigation uses nav2's MPPI controller with `open_loop: true` (see `params/nav2_params.yaml`), which needs `nav2_mppi_controller` 1.3.13 or newer. The workspace `Dockerfile` installs that build; older builds ignore the parameter without error and drive the base well below `vx_max`.
