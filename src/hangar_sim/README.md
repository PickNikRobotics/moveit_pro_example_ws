# hangar_sim

A MoveIt Pro MuJoCo simulation for PickNik's Universal Robots (UR) arms.

For detailed documentation see: [MoveIt Pro Documentation](https://docs.picknik.ai/)

## The hangar in the planning scene

The robot description, `description/ur5e_ridgeback.xacro`, describes only the robot. MuJoCo simulates the hangar from `description/hangar.xml`; its planning-scene geometry is in `description/hangar.urdf`. Keeping the scene out of the robot description keeps every node that loads `robot_description` from also loading the scene's meshes.

**Add Hangar to Planning Scene** adds `description/hangar.urdf` to the planning scene as one collision object, `hangar`, at the world origin. If `hangar` is already there, it does nothing. Each `hangar_sim` Objective that plans arm or whole-body motion runs it first, as does **Request Teleoperation**, and **ML Move Boxes to Loading Zone** runs it right after clearing the planning scene. Navigation through Nav2 does not use the planning scene. A new motion Objective must add the hangar too, or it plans through the airplane.

The hangar is not in the planning scene at startup, and **Reset Planning Scene** removes it. Either way, the next `hangar_sim` motion Objective adds it back. Before running any other Objective that plans motion, run **Add Hangar to Planning Scene**.

The hangar shows in the 3D Visualizer only as planning-scene collision geometry; its visual meshes are not loaded.

## Base teleoperation

Open the MoveIt Pro Desktop App's Teleoperation workspace and select **Base** to drive forward, reverse, or turn. `config/moveit/base_jog.yaml` sets the speed caps; the Teleoperation Speed control scales them. BaseJog does not perform collision checking or support sideways strafing.

The Base tab drives its own controller, `platform_velocity_controller_base_jog`, which ramps commands with its acceleration limits. It claims the same wheels as the navigation and whole-body base controllers, so opening the Base tab deactivates them and only one source drives the base at a time.

Navigation uses nav2's MPPI controller with `open_loop: true` (see `params/nav2_params.yaml`), which needs `nav2_mppi_controller` 1.3.13 or newer. The workspace `Dockerfile` installs that build; older builds ignore the parameter without error and drive the base well below `vx_max`.
