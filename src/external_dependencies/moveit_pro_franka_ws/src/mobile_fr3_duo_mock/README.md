# mobile_fr3_duo_mock

MoveIt Pro configuration for the Franka Mobile FR3 Duo: a TMR v0.2 swerve base, a vertical spine, two FR3 v2 arms and two Franka Hands.

This configuration runs on mock hardware (`mock_components/GenericSystem`), with no physics simulation and no driver.

## Frames

`world` -> `map` (fixed) -> `odom` -> planar joints (`planar_x`, `planar_y`, `planar_theta`) -> `base_link`.

The URDF starts at `odom`. `launch/frames.launch.py` publishes `world` -> `map` and `map` -> `odom` as fixed identities, because mock hardware has no localization. The planar joints are the base pose in `odom`; `base_jtc` moves them.

## Base teleoperation

The base is holonomic (a swerve drive), so it can move sideways. The `mobile_base` planning group holds only the planar joints, and the teleop jogs it like an arm: joint jog through `base_jvc` and pose jog of `base_link` through `base_vfc` (`config/moveit/joint_jog.yaml`, `pose_jog.yaml`).

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
| `launch/` | runtime entry point and the static `world` -> `map` -> `odom` frames |
| `objectives/`, `waypoints/` | Objectives and saved waypoints |

## Controllers

The whole upper body (`manipulator`: spine and both arms) keeps the standard controller name. Controllers for a part of the robot use short names: `jtac` (JointTrajectoryAdmittanceController), `jtc` (JointTrajectoryController), `vfc` (VelocityForceController), `jvc` (JointVelocityController). `left_`/`right_` is one arm; the `_w_spine` suffix adds the spine.

| Controller | Type | Group | At startup |
| --- | --- | --- | --- |
| `joint_state_broadcaster` | `joint_state_broadcaster/JointStateBroadcaster` | all joints | active |
| `joint_trajectory_admittance_controller` | `joint_trajectory_admittance_controller/JointTrajectoryAdmittanceController` | `manipulator` | active |
| `left_gripper_controller`, `right_gripper_controller` | `position_controllers/GripperActionController` | `left_gripper`, `right_gripper` | active |
| `base_jtc` | `joint_trajectory_controller/JointTrajectoryController` | `mobile_base` | active |
| `left_jtac_w_spine`, `right_jtac_w_spine` | `joint_trajectory_admittance_controller/JointTrajectoryAdmittanceController` | `left_manipulator`, `right_manipulator` | inactive |
| `left_jtac`, `right_jtac` | `joint_trajectory_admittance_controller/JointTrajectoryAdmittanceController` | `left_manipulator_without_spine`, `right_manipulator_without_spine` | inactive |
| `left_vfc`, `right_vfc` | `velocity_force_controller/VelocityForceController` | `left_manipulator_without_spine`, `right_manipulator_without_spine` | inactive |
| `left_jvc`, `right_jvc` | `joint_velocity_controller/JointVelocityController` | `left_manipulator_without_spine`, `right_manipulator_without_spine` | inactive |
| `spine_jtc` | `joint_trajectory_controller/JointTrajectoryController` | `spine` | inactive |
| `base_jvc` | `joint_velocity_controller/JointVelocityController` | `mobile_base` | inactive |
| `base_vfc` | `velocity_force_controller/VelocityForceController` | `mobile_base` | inactive |
