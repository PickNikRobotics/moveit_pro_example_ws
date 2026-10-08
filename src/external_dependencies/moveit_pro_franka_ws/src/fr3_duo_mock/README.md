# fr3_duo_mock

MoveIt Pro configuration for the Franka FR3 Duo without a spine: the FR3 Duo mount (v0.3) on a fixed base, two FR3 arms and two Franka Hands. There is no head.
For the FR3 Duo on the vertical spine, see `fr3_duo_spine_mock`.

This configuration runs on mock hardware (`mock_components/GenericSystem`), with no physics simulation and no driver.
The mount stands at a fixed height above `world`, with no pedestal geometry.

It is also the parent configuration for two-arm FR3 configurations without a spine, for example `fr3_duo_sim`.
The robot and SRDF are named `franka`. The joint names are the franka_description FR3 names with the arm prefix (`left_fr3_joint1` to `left_fr3_joint7`, `left_fr3_finger_joint1`, and the same with `right_`). The planning groups are `left_manipulator`, `right_manipulator` and `manipulator` (both arms). A configuration that reuses this SRDF must keep these names.

## Build and run

```bash
git submodule update --init
moveit_pro build
moveit_pro run -c fr3_duo_mock
```

## Layout

| Path | Contents |
| --- | --- |
| `config/config.yaml` | hardware, MoveIt parameters, controllers, Objectives |
| `config/control/` | ros2_control controllers |
| `config/moveit/` | SRDF, joint limits, kinematics, jogging |
| `description/` | robot xacro and mock ros2_control hardware |
| `objectives/`, `waypoints/` | gripper Objectives and saved waypoints |

## Controllers

Both arms together (`manipulator`) keep the standard controller name. Controllers for one arm use short names: `jtac` (JointTrajectoryAdmittanceController), `vfc` (VelocityForceController), `jvc` (JointVelocityController), with the prefix `left_` or `right_`.

| Controller | Type | Group | At startup |
| --- | --- | --- | --- |
| `joint_state_broadcaster` | `joint_state_broadcaster/JointStateBroadcaster` | all joints | active |
| `joint_trajectory_admittance_controller` | `joint_trajectory_admittance_controller/JointTrajectoryAdmittanceController` | `manipulator` | active |
| `left_gripper_controller`, `right_gripper_controller` | `position_controllers/GripperActionController` | `left_gripper`, `right_gripper` | active |
| `left_jtac`, `right_jtac` | `joint_trajectory_admittance_controller/JointTrajectoryAdmittanceController` | `left_manipulator`, `right_manipulator` | inactive |
| `left_vfc`, `right_vfc` | `velocity_force_controller/VelocityForceController` | `left_manipulator`, `right_manipulator` | inactive |
| `left_jvc`, `right_jvc` | `joint_velocity_controller/JointVelocityController` | `left_manipulator`, `right_manipulator` | inactive |
