# mobile_fr3_duo_sim

MoveIt Pro MuJoCo simulation configuration for the Franka Mobile FR3 Duo, with front and rear lidars, an IMU and base cameras.

The scene is a 12 m by 12 m room with a narrow passage, obstacles, a table and two scene cameras. Nav2 runs on a map of that room (`maps/room.yaml`, made with slam_toolbox), and the "Navigate to Clicked Point" Objective drives the base there.

It inherits everything from `mobile_fr3_duo_mock` (robot description, SRDF, MoveIt parameters, Objectives and waypoints) and replaces only the hardware and the controllers.

## Frames and odometry

`world` -> `map` (fixed) -> `odom` (AMCL) -> planar joints -> `base_link`, the same chain as `mobile_fr3_duo_mock`; `world` -> `mj_world` anchors the simulator's own frames.

- The simulator publishes the true base pose only as a message, `/ground_truth/odom` in frame `mj_world`.
- `script/odometry_joint_state_publisher.py` is the only publisher of the planar joints and of `/odom`: it adds drift that grows with distance driven and angle turned, so AMCL has a real correction to make. Drift is on by default. For exact odometry, set the `odometry_drift` launch argument in `launch/additional_driver_nodes.launch.py` to `false`; it sets the bridge's `noise_enabled` parameter.
- `joint_state_broadcaster` lists every joint except the planar ones (`config/control/`), so the true planar values never reach `/joint_states`.
- `script/base_twist_to_planar.py` turns Nav2's `/cmd_vel` into planar joint velocities for `base_jgvc`, using the true heading.

This package has no base trajectory controller. With drift on, a planned base trajectory would land off by the drift accumulated so far: MoveIt plans it in the drifted `odom` frame, and the controller drives the true joints. So the base moves through Nav2 (`base_jgvc`, active at startup; AMCL corrects `map` -> `odom`) and through teleop on the `mobile_base` group: joint jog (`base_jvc`) and pose jog of `base_link` (`base_vfc`). The jog controllers read the simulator's true planar joints, so a pose jog in `base_link` moves the base along its true body axes. "Move Base to Ready" drives to the map origin with Nav2 here, replacing the mock's trajectory version. `mobile_fr3_duo_mock`, which has no drift, keeps `base_jtc`.

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
git submodule update --init
moveit_pro build
moveit_pro run -c mobile_fr3_duo_sim
```

## Layout

| Path | Contents |
| --- | --- |
| `config/config.yaml` | inheritance from `mobile_fr3_duo_mock`, MuJoCo hardware parameters, Nav2 Behaviors |
| `config/control/` | ros2_control controllers for the simulation |
| `description/` | MuJoCo ros2_control hardware macro |
| `mjcf/` | MuJoCo scene and robot model |
| `mjcf/assets/` | meshes and the conversion input URDF |
| `launch/`, `script/` | frames, lidar flattening, scan merging, the odometry bridge, the `cmd_vel` bridge, Nav2 bring-up |
| `test/` | tests for the odometry and `cmd_vel` bridges |
| `maps/`, `params/` | room map, Nav2 and slam_toolbox parameters |
| `objectives/` | Navigate to Clicked Point |

## Third-party assets

The finger meshes `finger_0.obj` and `finger_1.obj` in `mjcf/assets/` come from [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie) (Apache-2.0, see [`LICENSE-mujoco_menagerie`](mjcf/assets/LICENSE-mujoco_menagerie)).
The other meshes are copied or converted from [franka_description](https://github.com/frankarobotics/franka_description) 2.9.0 (Apache-2.0, Copyright Franka Robotics GmbH).
