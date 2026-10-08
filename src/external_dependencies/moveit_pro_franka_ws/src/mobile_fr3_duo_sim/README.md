# mobile_fr3_duo_sim

MoveIt Pro MuJoCo simulation configuration for the Franka Mobile FR3 Duo, with front and rear lidars, an IMU and base cameras.

The scene is a 12 m by 12 m room with a narrow passage, obstacles, a table and two scene cameras. The table stands within the arms' reach in front of the robot and is not in MoveIt's planning scene, so plan arm motions that keep clear of it. Nav2's map holds only the table's legs, not its top, so it stands where the Ready arms, a base turn at the start and the stowed robot anywhere within Nav2's goal tolerance of the map origin all keep clear of it; keep that clearance if you move it. Nav2 runs on a map of that room (`mobile_fr3_duo_mock`'s `maps/room.yaml`, made with slam_toolbox), and the "Navigate to Clicked Point" Objective drives the base there.

It inherits everything from `mobile_fr3_duo_mock` (robot description, SRDF, MoveIt parameters, Objectives, waypoints, Nav2 launch and parameters) and replaces the hardware and the controllers. On top of the mock's Nav2 it adds AMCL and the laser obstacle layers (`params/nav2_params.yaml`, layered on the mock's file).

## Frames and odometry

`world` -> `map` (fixed) -> `odom` (AMCL) -> planar joints -> `base_link`, the same chain as `mobile_fr3_duo_mock`; `world` -> `mj_world` anchors the simulator's own frames.

- The simulator publishes the true base pose only as a message, `/ground_truth/odom` in frame `mj_world`.
- `script/odometry_joint_state_publisher.py` is the only publisher of the planar joints and of `/odom`: it adds drift that grows with distance driven and angle turned, so AMCL has a real correction to make. Drift is on by default. For exact odometry, set the `odometry_drift` launch argument in `launch/additional_driver_nodes.launch.py` to `false`; it sets the bridge's `noise_enabled` parameter.
- `joint_state_broadcaster` lists every joint except the planar ones (`config/control/`), so the true planar values never reach `/joint_states`.
- `mobile_fr3_duo_mock`'s `script/base_twist_to_planar.py` turns Nav2's `/cmd_vel` into planar joint velocities for `base_jgvc`, using the true heading from `/ground_truth/odom`.

This package has no base trajectory controller. With drift on, a planned base trajectory would land off by the drift accumulated so far: MoveIt plans it in the drifted `odom` frame, and the controller drives the true joints. So the base moves through Nav2 (`base_jgvc`, active at startup; AMCL corrects `map` -> `odom`) and through teleop on the `mobile_base` group: joint jog (`base_jvc`) and pose jog of `base_link` (`base_vfc`). The jog controllers read the simulator's true planar joints, so a pose jog in `base_link` moves the base along its true body axes. "Move Base to Ready" and "Navigate to Clicked Point" come from `mobile_fr3_duo_mock`: both stow the arms through "Stow Arms for Navigation" first, and the Nav2 footprint covers the robot in that pose.

## MuJoCo xacro arguments

A child configuration can override these through `urdf_params`; the defaults are this package's values.

| Argument | Default |
| --- | --- |
| `mujoco_model` | `mjcf/scene.xml` |
| `mujoco_model_package` | `mobile_fr3_duo_sim` |
| `mujoco_keyframe` | `default` |
| `mujoco_viewer` | `false` |
| `publish_object_tf` | `true` |
| `ground_truth_odom_topic` | `/ground_truth/odom` |
| `ground_truth_odom_frame` | `mj_world` |
| `base_link_name` | `base_link` |

## Build and run

```bash
git lfs pull
git submodule update --init
moveit_pro build
moveit_pro run -c mobile_fr3_duo_sim
```

## Layout

| Path | Contents |
| --- | --- |
| `config/config.yaml` | inheritance from `mobile_fr3_duo_mock`, MuJoCo hardware parameters, controllers |
| `config/control/` | ros2_control controllers for the simulation |
| `description/` | MuJoCo ros2_control hardware macro |
| `mjcf/` | MuJoCo scene and robot model |
| `mjcf/assets/` | meshes |
| `launch/`, `script/` | lidar flattening, scan merging, the odometry bridge, and `mobile_fr3_duo_mock`'s navigation launch with AMCL |
| `test/` | tests for the odometry bridge |
| `params/` | the Nav2 layer for AMCL and the lasers, and slam_toolbox parameters |

## Third-party assets

The finger meshes `finger_0.obj` and `finger_1.obj` in `mjcf/assets/` come from [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie) (Apache-2.0, see [`LICENSE-mujoco_menagerie`](mjcf/assets/LICENSE-mujoco_menagerie)).
The other meshes are copied or converted from [franka_description](https://github.com/frankarobotics/franka_description) 2.9.0 (Apache-2.0, Copyright Franka Robotics GmbH).
