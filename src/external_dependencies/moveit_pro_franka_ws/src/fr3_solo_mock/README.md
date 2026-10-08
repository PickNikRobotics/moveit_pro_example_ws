# fr3_solo_mock

MoveIt Pro configuration for one Franka Research 3 (FR3) arm with a Franka Hand on mock hardware (`mock_components/GenericSystem`): no physics simulation and no driver.

It is also the parent configuration for single-arm FR3 configurations: `fr3_solo_sim` and the example workspace's `kitchen_sim` inherit its SRDF and MoveIt parameters through `based_on_package`.

## Build and run

```bash
git submodule update --init
moveit_pro build
moveit_pro run -c fr3_solo_mock
```

## Layout

| Path | Contents |
| --- | --- |
| `config/config.yaml` | hardware, MoveIt parameters, controllers, Objectives |
| `config/control/` | ros2_control controllers |
| `config/moveit/` | SRDF, joint limits, kinematics, jogging, 3D sensors |
| `description/` | robot xacro and the mock ros2_control hardware |
| `objectives/`, `waypoints/` | gripper Objectives and saved waypoints |

## Controllers

| Controller | Type | At startup |
| --- | --- | --- |
| `joint_state_broadcaster` | `joint_state_broadcaster/JointStateBroadcaster` | active |
| `joint_trajectory_admittance_controller` | `joint_trajectory_admittance_controller/JointTrajectoryAdmittanceController` | active |
| `gripper_controller` | `position_controllers/GripperActionController` | active |
| `joint_trajectory_controller` | `joint_trajectory_controller/JointTrajectoryController` | inactive |
| `velocity_force_controller` | `velocity_force_controller/VelocityForceController` | inactive |
| `joint_velocity_controller` | `joint_velocity_controller/JointVelocityController` | inactive |

The robot and SRDF are named `franka`, and the joint names are the franka_description FR3 names (`fr3_joint1` to `fr3_joint7`, `fr3_finger_joint1`). A configuration that reuses this SRDF must keep both.
