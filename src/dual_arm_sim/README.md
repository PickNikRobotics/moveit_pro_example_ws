# dual_arm_sim

**A MoveIt Pro MuJoCo simulation of the Franka Mobile FR3 Duo (a TMR v0.2 swerve base, a vertical spine, a head and two FR3 v2 arms with Franka Hands) working at three tables.**

The arms draw and wipe text, sort colored blocks and find objects with ML segmentation, as on a fixed dual-arm robot. The base, the spine and both arms also move together as one whole-body motion, and Nav2 drives the base.

For general documentation see [MoveIt Pro Documentation](https://docs.picknik.ai/).

## A standalone package

Each example_ws sim is an independent, standalone package: it needs no other config package. The same Mobile FR3 Duo robot files are planned for a second, supermarket sim in [#1046](https://github.com/PickNikRobotics/moveit_pro_example_ws/pull/1046), deliberately as a second standalone copy rather than a shared one, so a fix to one copy will have to be made in the other by hand.

These files, and the whole-body and navigation Objectives, are copied from the Mobile FR3 Duo configuration in PickNik's [moveit_pro_franka_ws](https://github.com/PickNikRobotics/moveit_pro_franka_ws) (`mobile_fr3_duo_mock` and `mobile_fr3_duo_sim` at `b510ad58bde5740d748a0b5e79e0c151a0309e98`) and adapted.

A file-by-file comparison against that second copy belongs here once it exists; it is left out until #1046 lands, so this page describes only what is in this branch.

The robot meshes that are byte-identical to franka_description's are not copied: `description/mujoco/mobile_fr3_duo.xml` loads them from the vendored franka_description through `package://franka_description/meshes/...` paths. Only the converted chassis mesh and the two Menagerie finger meshes are in `description/mujoco/assets/`.

## Scene

The tables, the blocks and the scene camera stand where they keep their place relative to the arm mount with the spine at 0.4 m, the spine height of the `Home` waypoint. The arm waypoints and the drawing poses are therefore the same arm motions as on a fixed dual-arm mount.

- Each table top is an obstacle in Nav2's map (`maps/scene.pgm`, made by `scripts/generate_map.py`), projected onto the ground so the base cannot drive under an overhang. The legs sit inside that projection and add nothing. `test/test_scene.py` checks the map matches the scene and that the footprint clears it at the spawn pose.
- The base spawns clear of that projection rather than at the map origin, and the two side tables moved with it so the place waypoints still sit over them. `description/mujoco/scene.xml`'s keyframe sets the spawn pose.
- That spawn pose sits near the front gap's edge: a few millimetres further forward and the Nav2 footprint covers an occupied map cell, and about two map cells further back and the drawing surface is out of reach. `test/test_scene.py` guards both edges. The drawing centre in `objectives/writing_demo.xml` sets the reach edge. Read "The spawn window" below before moving the pick table, changing the map resolution or lengthening the `Stow` waypoint.
- The three table surfaces and their legs are also links in `description/environment.xacro`, so MoveIt plans against them from start-up. The blocks are deliberately not: a block in the planning scene would make every pick a collision. Each surface link's name and origin match the same-named MuJoCo body, so the simulator reports any drift between the two files.
- At `Home` the tables box the robot in on three sides: only backing away is free. The base cannot turn there, and an arm lowered with the spine would reach a table top, so "Stow Arms for Navigation" keeps the spine at 0.6 m. The blocks are not in MoveIt's planning scene.
- The scene camera matches the 640 by 480 resolution of the base cameras, because every camera renders into one offscreen buffer.

### The spawn window

The Nav2 map projects each table's whole top onto the ground, so the base cannot plan under an
overhang. The pick table's projection reaches back past where the arms must stand to work it, and
the Nav2 footprint includes the stowed arms, so two constraints squeeze the spawn pose from
opposite sides:

- move the base **back** and it clears the map but the drawing surface goes out of reach, and
  `PlanCartesianPath` fails inside Writing Demo;
- move it **forward** and the drawing reaches but the footprint covers an occupied cell, so Nav2
  refuses to plan and "Move Base to Ready" fails with the start in collision.

The window is about two map cells wide, and the committed spawn sits near its **clearance** edge:
only single-digit millimetres of front gap, against a map cell an order of magnitude larger, with
room to spare on the reach side. The drawing centre in `objectives/writing_demo.xml` is what buys
that reach room; shifting it toward the robot widens the window, shifting it away narrows it.

Anything that moves either bound lands outside the window: moving the pick table, changing
`generate_map.py`'s `RESOLUTION`, or lengthening the `Stow` waypoint so the footprint reaches
further forward. The side tables bound neither edge.

`test/test_scene.py` guards both edges, one test each, and each failure message names the edge
crossed. **Run it after any of those changes, and do not widen the window by editing the Nav2
footprint:** the arm-width footprint is deliberate, and the narrow front gap was accepted knowingly
rather than overlooked.

## Frames and odometry

`world` -> `map` (fixed) -> `odom` (fixed) -> planar joints (`planar_x`, `planar_y`, `planar_theta`) -> `base_link`. `world` -> `mj_world` anchors the simulator's own frames, such as the scene camera. `launch/navigation.launch.py` publishes the fixed frames.

- The simulator publishes the true base pose only as a message, `/ground_truth/odom`.
- `scripts/odometry_joint_state_publisher.py` turns it into the planar joint states and `/odom`, with no drift. The planar joints MoveIt plans from are then the ones the controllers drive, and there is nothing for AMCL to correct.
- `scripts/base_twist_to_planar.py` turns Nav2's `/cmd_vel` into planar joint velocities for `base_jgvc`.

## Controllers

| Controller | Group | At startup |
| --- | --- | --- |
| `joint_trajectory_admittance_controller` | `manipulator` (spine and both arms) | active |
| `left_gripper_controller`, `right_gripper_controller` | `left_gripper`, `right_gripper` | active |
| `base_jgvc` | planar joints, for Nav2 | active |
| `left_jtac`, `right_jtac` | `left_manipulator_without_spine`, `right_manipulator_without_spine` | inactive |
| `left_jtac_w_spine`, `right_jtac_w_spine` | `left_manipulator`, `right_manipulator` | inactive |
| `left_vfc`, `right_vfc`, `left_jvc`, `right_jvc` | the arms without the spine (teleoperation) | inactive |
| `spine_jtc`, `spine_jvc` | `spine` | inactive |
| `base_jvc`, `base_vfc` | `mobile_base` (teleoperation) | inactive |
| `body_jtc` | `whole_body`, `left_manipulator_with_base`, `right_manipulator_with_base` | inactive |
| `base_pid` | planar joints, chained under `body_jtc` | inactive |

`body_jtc` runs whole-body trajectories. Its planar joints are chained through `base_pid`, a `pid_controller/PidController` that turns their position references into the planar velocity commands `base_jgvc` sends for Nav2. Both pairs claim the planar joints, so one runs at a time: the whole-body Objectives switch `base_pid` and `body_jtc` in, then `joint_trajectory_admittance_controller` and `base_jgvc` back, also when the motion fails.

## Objectives

| Objective | What it does |
| --- | --- |
| "Writing Demo", "Draw and Wipe", "Draw" | draw text on the front table with one arm and wipe it with the other |
| "Sort Blocks", "Find Green Block", "Find Red Block" | find blocks with ML segmentation and sort them onto the side tables |
| "Move Left Home", "Move Right Home" | move one arm, without the spine, to `Home` |
| "Whole Body Move to Waypoint" | move the base, the spine and both arms together to a `whole_body` waypoint (`Whole Body Demo`, `Whole Body Home`) |
| "Whole Body Reach Left" | back into open floor, then reach the left gripper sideways past the arm's reach, so the base and the spine move with it |
| "Whole Body Move to Pose", "Whole Body Move to Joint State" | the shared subtrees: IK on a `*_manipulator_with_base` group, then plan, switch, execute and switch back |
| "Move Base to Ready", "Navigate to Clicked Point" | stow the arms, then drive with Nav2 to the Base Ready pose or to a clicked point |

## Build and run

```bash
git lfs pull
moveit_pro build
moveit_pro run -c dual_arm_sim
```

## Layout

| Path | Contents |
| --- | --- |
| `config/` | MoveIt Pro configuration, ros2_control controllers, SRDF and MoveIt parameters |
| `description/` | robot xacro, MuJoCo ros2_control hardware, MuJoCo scene and robot model |
| `launch/` | the runtime entry point; frames, odometry and Nav2 |
| `maps/`, `params/` | the Nav2 map and parameters |
| `objectives/`, `waypoints/` | Objectives, drawing paths and saved waypoints |
| `scripts/` | the odometry and `/cmd_vel` bridges, the map generator, and the SVG drawing tools (see `scripts/README.md`) |
| `test/` | the scene, bridge and Objectives integration tests |
