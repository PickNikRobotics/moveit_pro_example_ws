# Nav2 and Whole-Body Planning in hangar_sim

hangar_sim is a UR5e on a Clearpath Ridgeback mecanum base, simulated in the MoveIt Pro Simulator. The base moves three ways, each owned by one controller at a time:

| Mode | Who commands the wheels | Activated by |
| --- | --- | --- |
| Whole-body motion (planned base + arm) | `joint_trajectory_controller`, arm and wheel joints in one trajectory | `SwitchController` in the motion Objectives |
| Navigation | Nav2 → `platform_velocity_controller_nav2` | `SwitchController` in the Nav Objectives |
| Base teleoperation (Base tab) | `BaseJog` → `platform_velocity_controller_base_jog` | `Request Teleoperation`, teleop mode 8 |

All three controllers claim the same four wheel velocity command interfaces, so the controller manager allows only one at a time and every switch deactivates the previous owner.

The design this config follows is MoveIt Pro's whole-body control design: the base is its wheels plus a `<mobile_base>` SRDF element, not a set of virtual joints.

## Robot model

The URDF (`description/ur5e_ridgeback.xacro`) is rooted at `ridgeback_base_link`. It contains no `world` link, no virtual rail joints and no hangar geometry; it describes the robot only.

The SRDF (`config/moveit/picknik_ur.srdf`) declares the base:

```xml
<mobile_base name="ridgeback" link="ridgeback_base_link" parent_frame="odom" drive="mecanum"
             wheel_radius="0.0756" half_wheelbase="0.319" half_track="0.2755">
  <wheel joint="front_left_wheel"/>
  <wheel joint="rear_left_wheel"/>
  <wheel joint="rear_right_wheel"/>
  <wheel joint="front_right_wheel"/>
</mobile_base>
```

- `wheel_radius` is the roller hull's rolling radius, the same `kinematics.wheels_radius` both mecanum controllers use; `test/test_base_geometry.py` holds all of them to the MJCF rollers.
- Each wheel's sign comes from its URDF joint axis.
- `parent_frame="odom"` makes `odom` the robot model's model frame, so MoveIt plans in `odom`. The robot's placement in `odom`, the robot pose, is not a joint value: MoveIt reads it from the `odom → ridgeback_base_link` TF.

Planning groups:

| Group | Contents | Use |
| --- | --- | --- |
| `manipulator` | arm chain from `ridgeback_base_link` to `grasp_link`, plus the four wheels | Whole-body planning and `velocity_force_controller` |
| `arm_only` | arm chain from `base_link` to `grasp_link` | Arm-only motion; the base stays still |
| `gripper`, `hand` | vacuum gripper | End effector |

A group that contains all four wheels has the base; a group with only some of them fails to load.

## Frames

```text
mj_world ── map ── odom ── ridgeback_base_link ── (robot_state_publisher: chassis, arm, sensors)
                     └── world
```

| Edge | Publisher | Notes |
| --- | --- | --- |
| `mj_world → map` | static identity | Anchors the Nav2 map to the simulated world. |
| `map → odom` | beluga AMCL (default), slam_toolbox with `slam:=true`, or a static identity when both are off | The localization correction. |
| `odom → ridgeback_base_link` | MuJoCo's odom publisher (`odom_publish_tf`) | Ground truth: `odom` coincides with `mj_world`. The only edge that places the robot. |
| `odom → world` | static identity | Alias for Objectives and the UI that still name `world`. Not a separate frame. |

`odom_planar` is off, so the odom edge carries the base's 0.0251 m ride height. The MJCF base moves only in x, y and yaw, so the pose stays planar without it. With `odom_planar` on, every link would sit 2.5 cm below where the MoveIt Pro Simulator has it; `test/test_base_geometry.py` and the integration test fail on that.

MuJoCo also publishes `/odom` (`nav_msgs/Odometry`), which Nav2's `bt_navigator` and `controller_server` consume (`params/nav2_params.yaml`). It is exact, so there is no state estimator: AMCL corrects `map → odom` against the map, and nothing corrects `odom` itself.

## Whole-body motion

Planners that move the base (`PlanCartesianPath`, the MTC Path IK stage, Pose Jog) solve in the base's motion coordinates, x, y and yaw for this mecanum base, and turn each step into wheel rotations through the base kinematics. Their output is an ordinary joint trajectory over the arm and wheel joints, relative to the current wheel positions. `ValidateTrajectory` replays the base pose from the wheel rotations and checks the whole robot, chassis included, at each replayed pose.

`joint_trajectory_controller` (`config/control/picknik_ur.ros2_control.yaml`) executes it:

- `joints`: the six arm joints and the four wheels, `command_interfaces: [velocity]` with per-joint gains, closed loop.
- Wheel joints are continuous with a 6.6 rad/s velocity limit (`ridgeback_description`), which time parameterization enforces like any joint limit.

`Move Tip Forward - Whole Body` is the minimal example: it activates `joint_trajectory_controller`, captures a wrist-camera point cloud into the octomap, plans a 0.3 m Cartesian move of the tool along its own axis with the `manipulator` group, validates it with `ValidateTrajectory` (`PlanCartesianPath` does not collision-check base motion), and executes. The planner splits the motion between base and arm, so repeated runs keep moving once the arm alone cannot reach.

Velocity streaming uses the same group: `velocity_force_controller` runs on `manipulator` and commands the wheels directly. Both `arm_only_*` controllers run on `arm_only`, since jogging one wheel would move the base uncoordinated with the others.

## Navigation

Nav2 runs in `nav2_container` from `launch/sim/navigation_launch.py`:

- Localization: beluga AMCL on `/scan_interleaved`, the front and rear filtered lidar scans relayed onto one topic, against `maps/`.
- Sensing: `script/lidar_flattener.py` turns the two simulated TIM571 point clouds into `/scan_front` and `/scan_rear`, which the laser filters, AMCL, slam_toolbox and the costmaps consume.
- Control: MPPI (`FollowPath`). The velocity smoother publishes unstamped body-frame twists on `/platform_velocity_controller_nav2/cmd_vel_unstamped`.

`platform_velocity_controller_nav2` is a `clearpath_mecanum_drive_controller/MecanumDriveController` (vendored in `src/external_dependencies`) with `body_frame_control: true`. Its odometry output is unused, since MuJoCo's `/odom` is the reference.

The Nav Objectives (`Navigate Hangar Loop`, `Navigate to Clicked Point`, `Navigate to Clicked Point with Replanning`, and `Navigate Base to Pose`, the SubTree the manipulation Objectives drive the base with) follow one pattern:

1. Check localization (`Refine Localization In Place`): run `Localize Robot` if the robot is not localized yet, seed AMCL, request in-place updates, and require each lidar to fit the map (`ScanMatchResidual`). A poor fit fails the Objective and points at `Localize Robot`. `Navigate Base to Pose` skips this step when its `refine_localization` port is false, then moves the arm to `Arm upright` with `arm_only`, since the Nav2 footprint covers the base only.
2. `SwitchController`: activate `platform_velocity_controller_nav2`, deactivate `velocity_force_controller` and `joint_trajectory_controller`.
3. Plan and follow the path (`ComputePathToPoseAction`, `FollowPathAction`; `NavigateToPoseAction` in `Navigate Base to Pose`).
4. `SwitchController`: activate `joint_trajectory_controller` again. `Navigate Hangar Loop` and `Navigate Base to Pose` also do this when navigation fails; the clicked-point Objectives only on success.

Step 4 is what lets a whole-body Objective run right after navigation. After a failed clicked-point navigation, the next whole-body Objective's own `SwitchController` hands the wheels back. Planning reads the robot pose from TF, so it starts wherever Nav2 left the base, with no state to reset.

`Localize Robot` asks the operator to place a marker on the robot. The marker works in the robot model frame, `odom`. In sim it starts at `odom → ridgeback_base_link`, which is the simulated ground truth, and `odom` coincides with `map`, so accepting it unmoved seeds AMCL with the true pose.

## Base teleoperation

`config/moveit/base_jog.yaml` points the Base tab at `platform_velocity_controller_base_jog`, a third `MecanumDriveController` instance on the same wheels with stamped commands and symmetric acceleration limits. `planning_group: ridgeback` names the `/base_jog/ridgeback` command topic; it is the `<mobile_base>` name, not an SRDF group.

## Known gaps

- **No hangar geometry in the planning scene.** The URDF holds the robot only, and the planning scene has no static model of the hangar or the aircraft. Each Objective that plans a whole-body path near the aircraft captures a wrist-camera point cloud into the octomap right before planning, so the hangar is seen only through that capture: anything outside the camera's view at that moment is invisible to the collision check.
- **AMCL sits several centimeters off the simulated truth.** The true `map → odom` is the identity, but AMCL has been measured at 5–12 cm and up to 3.4° off, including right after startup. Once that failed the localization check after whole-body motion, with the rear lidar fitting 47.7% of beams against a 50% threshold; seeding AMCL with the true pose recovered it. The cause is not yet known.

## Diagnostics

```bash
# Which controller owns the wheels.
ros2 control list_controllers

# The robot pose MoveIt plans from; z is the 0.0251 m ride height.
ros2 run tf2_ros tf2_echo odom ridgeback_base_link

# The localization correction.
ros2 run tf2_ros tf2_echo map odom
```
