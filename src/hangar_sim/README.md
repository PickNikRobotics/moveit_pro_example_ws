# hangar_sim

A MoveIt Pro MuJoCo simulation for PickNik's Universal Robots (UR) arms.

For detailed documentation see: [MoveIt Pro Documentation](https://docs.picknik.ai/)

## The hangar in the planning scene

The robot description, `description/ur5e_ridgeback.xacro`, describes only the robot. MuJoCo simulates the hangar from `description/hangar.xml`; its planning-scene geometry is in `description/hangar.urdf`. Keeping the scene out of the robot description keeps every node that loads `robot_description` from also loading the scene's meshes.

**Add Hangar to Planning Scene** adds `description/hangar.urdf` to the planning scene as one collision object, `hangar`, at the world origin. If `hangar` is already there, it does nothing. Each `hangar_sim` Objective that moves the arm or base runs it first, and **ML Move Boxes to Loading Zone** runs it again after clearing the planning scene. A new motion Objective must do the same, or it plans through the airplane.

Core library Objectives, such as **Move to Waypoint**, do not add the hangar. Before running one at startup or after clearing the planning scene, run **Add Hangar to Planning Scene**.

## Base teleoperation

Open the MoveIt Pro Desktop App's Teleoperation workspace and select **Base** to drive forward, reverse, or turn. `config/moveit/base_jog.yaml` sets the speed caps; the Teleoperation Speed control scales them. BaseJog does not perform collision checking or support sideways strafing.

The Base tab drives its own controller, `platform_velocity_controller_base_jog`, which ramps commands with its acceleration limits. It claims the same wheels as the navigation and whole-body base controllers, so opening the Base tab deactivates them and only one source drives the base at a time.

Navigation uses nav2's MPPI controller with `open_loop: true` (see `params/nav2_params.yaml`), which needs `nav2_mppi_controller` 1.3.13 or newer. The workspace `Dockerfile` installs that build; older builds ignore the parameter without error and drive the base well below `vx_max`.
