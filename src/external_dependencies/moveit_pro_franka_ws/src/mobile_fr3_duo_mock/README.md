# mobile_fr3_duo_mock

MoveIt Pro configuration for the Franka Mobile FR3 Duo: a TMR v0.2 swerve base, a vertical spine, two FR3 v2 arms and two Franka Hands.

This configuration runs on mock hardware (`mock_components/GenericSystem`), with no physics simulation and no driver. Nav2 drives the base on a map of a room (`maps/room.yaml`).

## Frames

`world` -> `map` (fixed) -> `odom` -> planar joints (`planar_x`, `planar_y`, `planar_theta`) -> `base_link`.

The URDF starts at `odom`. `launch/navigation.launch.py` publishes `world` -> `map` and `map` -> `odom` as fixed identities, because mock hardware has no localization and no drift. The planar joints are the base pose in `odom`, and `script/joint_state_odometry.py` publishes them as `/odom` for Nav2.

## Navigation

`launch/navigation.launch.py` starts the map server and Nav2 with `params/nav2_params.yaml`: the room map only (no laser on mock hardware) and a fixed `map` -> `odom`. Nav2's `/cmd_vel` goes through `script/base_twist_to_planar.py`, which turns it into world-axis planar joint velocities for `base_jgvc`.

Every navigation Objective first runs the subtree **"Stow Arms for Navigation"** (`objectives/stow_arms_for_navigation.xml`). It moves both arms, and the spine to its lowest height, to the `Stow` waypoint. Then the base drives. The Nav2 footprint in `params/nav2_params.yaml` covers the robot in that pose, so plans keep the stowed arms clear of obstacles.

| Objective | What it does |
| --- | --- |
| "Navigate to Clicked Point" | prompts for a point in the Visualization pane, stows the arms, then drives there |
| "Move Base to Ready" | stows the arms, then drives to the map origin |
| "Stow Arms for Navigation" | the shared stow step; a child configuration calls it before its own navigation |

`mobile_fr3_duo_sim` reuses this launch file. It adds AMCL and its laser obstacle layers through `params_overlay`: mappings merge and lists replace.

## Base teleoperation

The base is holonomic (a swerve drive), so it can move sideways. The `mobile_base` planning group holds only the planar joints, and the teleop jogs it like an arm: joint jog through `base_jvc` and pose jog of `base_link` through `base_vfc` (`config/moveit/joint_jog.yaml`, `pose_jog.yaml`). All base controllers command velocity, so a jog takes the base from `base_jgvc`, and the navigation Objectives give it back.

The base also has a teleop iMarker: `base_link` is the end effector of `mobile_base` (SRDF), and PoseIK solves `mobile_base` (`config/moveit/pose_ik.yaml`). Drag it in x, y or yaw and release: the arms stow and Nav2 drives the base to the marker, within Nav2's goal tolerance. This starts at once, with no preview or approval prompt, even when the teleop asks for approval before it moves the other groups. z, roll and pitch drags have no IK solution, so the UI does not run them. This comes from `objectives/request_teleoperation.xml`, a copy of MoveIt Pro's core "Request Teleoperation" (10.1.0) whose iMarker branch sends `mobile_base` to Nav2; every other group runs the core "Move to Cartesian Pose". With `mobile_base` selected, the teleop also shows the end-effector Open and Close Gripper buttons; they run this package's gripper Objectives.

## Spine

The spine joint (`franka_spine_vertical_joint`, group `spine`) moves through:

- joint jog in the teleop, through `spine_jvc`;
- "Move Spine Absolute": an absolute height in meters, within the travel;
- "Move Spine to Pose Height": the height of a target pose, plus an optional bias, clamped to the travel.

Both Objectives run `spine_jtc` and leave the arms where they are. Their ports follow the real spine's `MoveAbsolute` action (position, velocity, acceleration, deceleration, timeout, action name), so callers stay the same on real hardware. Mock and sim ignore everything but the position. Real hardware has no ros2_control interface for the spine: it will need its own versions of these Objectives, with a Behavior that calls Franka's spine `MoveAbsolute` action. That is not part of this workspace yet.

The Behaviors `CreateSpineState` and `GetSpineStateForPoseHeight` come from `franka_behaviors`.

## Requirements

The robot description comes from the `franka_description` submodule:

```bash
git submodule update --init
```

## Build and run

```bash
moveit_pro build
moveit_pro run -c mobile_fr3_duo_mock
```

## Layout

| Path | Contents |
| --- | --- |
| `config/config.yaml` | hardware, MoveIt parameters, controllers, Objectives |
| `config/control/` | ros2_control controllers |
| `config/moveit/` | SRDF, joint limits, kinematics, jogging |
| `description/` | robot xacro and mock ros2_control hardware |
| `launch/` | runtime entry point; frames, odometry and Nav2 |
| `maps/`, `params/` | the room map and the Nav2 parameters |
| `objectives/`, `waypoints/` | Objectives (including the Request Teleoperation copy) and saved waypoints |
| `script/`, `test/` | the odometry and `/cmd_vel` bridges, and their unit tests |

## Controllers

The whole upper body (`manipulator`: spine and both arms) keeps the standard controller name. Controllers for a part of the robot use short names: `jtac` (JointTrajectoryAdmittanceController), `jtc` (JointTrajectoryController), `vfc` (VelocityForceController), `jvc` (JointVelocityController), `jgvc` (JointGroupVelocityController). `left_`/`right_` is one arm; the `_w_spine` suffix adds the spine.

| Controller | Type | Group | At startup |
| --- | --- | --- | --- |
| `joint_state_broadcaster` | `joint_state_broadcaster/JointStateBroadcaster` | all joints | active |
| `joint_trajectory_admittance_controller` | `joint_trajectory_admittance_controller/JointTrajectoryAdmittanceController` | `manipulator` | active |
| `left_gripper_controller`, `right_gripper_controller` | `position_controllers/GripperActionController` | `left_gripper`, `right_gripper` | active |
| `base_jgvc` | `velocity_controllers/JointGroupVelocityController` | planar joints, for Nav2 | active |
| `left_jtac_w_spine`, `right_jtac_w_spine` | `joint_trajectory_admittance_controller/JointTrajectoryAdmittanceController` | `left_manipulator`, `right_manipulator` | inactive |
| `left_jtac`, `right_jtac` | `joint_trajectory_admittance_controller/JointTrajectoryAdmittanceController` | `left_manipulator_without_spine`, `right_manipulator_without_spine` | inactive |
| `left_vfc`, `right_vfc` | `velocity_force_controller/VelocityForceController` | `left_manipulator_without_spine`, `right_manipulator_without_spine` | inactive |
| `left_jvc`, `right_jvc` | `joint_velocity_controller/JointVelocityController` | `left_manipulator_without_spine`, `right_manipulator_without_spine` | inactive |
| `spine_jtc` | `joint_trajectory_controller/JointTrajectoryController` | `spine` | inactive |
| `spine_jvc` | `joint_velocity_controller/JointVelocityController` | `spine` | inactive |
| `base_jvc` | `joint_velocity_controller/JointVelocityController` | `mobile_base` | inactive |
| `base_vfc` | `velocity_force_controller/VelocityForceController` | `mobile_base` | inactive |
