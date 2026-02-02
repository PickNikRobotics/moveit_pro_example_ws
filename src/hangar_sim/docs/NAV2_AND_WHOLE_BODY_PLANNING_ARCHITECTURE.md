# Architecture for Nav2 and Whole Body Planning in Hangar Sim

## Executive Summary

This document describes the architectural implementation of mobile manipulation in the hangar_sim MoveIt Pro configuration, which integrates a UR5e manipulator mounted on a Clearpath Ridgeback mobile base. The system supports both autonomous navigation via Nav2 and coordinated whole body motion planning via MoveIt through a layered control architecture comprising:

1. Virtual joint representation of mobile base degrees of freedom in the URDF
2. Bidirectional odometry-to-joint-state conversion for state estimation
3. **Custom-modified chainable controller** with frame transformation capabilities
4. Dual platform controller configuration for mode-specific operation

The architecture enables transparent integration of mobile base motion into MoveIt's trajectory planning framework while maintaining compatibility with standard Nav2 navigation workflows.

### Critical Implementation Note

The whole body planning capability requires **custom modifications** to the `clearpath_mecanum_drive_controller` that are not present in the standard implementation. The stock controller does not support the frame transformation logic and configuration parameters necessary for world-frame velocity commands. This document describes both the architectural patterns and the specific controller modifications required for successful deployment.

---

## System Architecture Overview

```
┌─────────────────────────────────────────────────────────────────────────┐
│                          MoveIt Pro Application                          │
└─────────────────────────────────────────────────────────────────────────┘
                                    │
                    ┌───────────────┴───────────────┐
                    │                               │
                    ▼                               ▼
        ┌───────────────────┐           ┌──────────────────────┐
        │   Nav2 Behaviors  │           │  MoveIt Behaviors    │
        │   (Navigation)    │           │  (Manipulation)      │
        └───────────────────┘           └──────────────────────┘
                    │                               │
                    ▼                               ▼
        ┌───────────────────┐           ┌──────────────────────┐
        │ platform_velocity │           │ joint_trajectory     │
        │ _controller_nav2  │           │ _controller          │
        │                   │           │                      │
        │ (body_frame: true)│           │ Commands 9 joints:   │
        └───────────────────┘           │ - 6 arm joints       │
                    │                   │ - 3 virtual joints   │
                    │                   │   (via chaining)     │
                    │                   └──────────────────────┘
                    │                               │
                    └───────────┬───────────────────┘
                                │
                                ▼
                ┌───────────────────────────────────┐
                │  platform_velocity_controller     │
                │  (Chainable Controller)           │
                │  ** CUSTOM MODIFIED **            │
                │                                   │
                │  - Exports reference interfaces   │
                │  - Performs IK for mecanum drive  │
                │  - Handles frame transformations  │
                │  (body_frame_control: false)      │
                └───────────────────────────────────┘
                                │
                                ▼
                ┌───────────────────────────────────┐
                │      Mecanum Wheel Commands       │
                │  (front_left, rear_left,          │
                │   rear_right, front_right)        │
                └───────────────────────────────────┘
                                │
                                ▼
                        ┌───────────────┐
                        │ MuJoCo Sim    │
                        └───────────────┘
                                │
                                ▼
                        ┌───────────────┐
                        │  Odometry     │
                        │  Publisher    │
                        └───────────────┘
                                │
                                ▼
                ┌───────────────────────────────────┐
                │ odometry_joint_state_publisher.py│
                │                                   │
                │ Converts /odom → /joint_states   │
                │ - linear_x_joint                  │
                │ - linear_y_joint                  │
                │ - rotational_yaw_joint            │
                └───────────────────────────────────┘
                                │
                                ▼
                        ┌───────────────┐
                        │ Joint States  │
                        └───────────────┘
                                │
                                ▼
                        ┌───────────────┐
                        │  MoveIt       │
                        │  Robot State  │
                        └───────────────┘
```

---

## Custom Controller Modifications

### Overview

The whole body planning architecture described in this document requires **custom modifications** to the `clearpath_mecanum_drive_controller`. The standard Clearpath mecanum drive controller available in public repositories does not include the functionality necessary for coordinated arm-base motion planning. Attempting to implement this architecture with an unmodified controller will result in incorrect base motion or system failure.

### Location of Modified Controller

**Repository Path**: `src/external_dependencies/clearpath_mecanum_drive_controller`

This is a forked and modified version of the original Clearpath controller, maintained within the MoveIt Pro workspace to support whole body planning requirements.

### Key Modifications

The following functionality has been added to enable whole body planning:

#### 1. World-to-Body Frame Transformation

**Modified Files**: `clearpath_mecanum_drive_controller.cpp` (lines 438-444)

**Functionality Added**: Automatic transformation of world-frame velocity commands to body-frame velocities using current robot orientation.

**Standard Controller Behavior**: Expects all velocity commands in body frame; cannot accept world-frame commands from MoveIt.

**Custom Implementation**:
```cpp
if (!params_.body_frame_control && !params_.body_frame_yaw_joint.empty() &&
    state_interfaces_.size() >= 5)
{
  const double theta = state_interfaces_[4].get_value();
  const double rotated_x = std::cos(-theta)*reference_interfaces_[0]
                          - std::sin(-theta)*reference_interfaces_[1];
  const double rotated_y = std::sin(-theta)*reference_interfaces_[0]
                          + std::cos(-theta)*reference_interfaces_[1];
  reference_interfaces_[0] = rotated_x;
  reference_interfaces_[1] = rotated_y;
}
```

This code block reads the current yaw angle from a state interface and applies a 2D rotation transformation to convert world-frame velocities into the body frame required by mecanum kinematics.

#### 2. Configurable Frame Control Mode

**Modified Files**: `clearpath_mecanum_drive_controller.yaml` (parameter definitions)

**Parameters Added**:
- `body_frame_control`: Boolean flag to enable/disable automatic frame transformation
- `body_frame_yaw_joint`: Name of the joint providing yaw angle for transformation

**Standard Controller Behavior**: Only supports body-frame control mode.

**Custom Implementation**: Allows controllers to operate in world-frame mode (for whole body planning) or body-frame mode (for Nav2), selectable via parameter configuration.

#### 3. Enhanced State Interface Reading

**Modified Files**: `clearpath_mecanum_drive_controller.cpp` (state interface configuration)

**Functionality Added**: Support for reading additional state interface (yaw joint position) beyond the standard four wheel states.

**Standard Controller Behavior**: Reads only wheel joint states.

**Custom Implementation**: Configurable to read an additional state interface (`state_interfaces_[4]`) corresponding to the `body_frame_yaw_joint`, enabling the controller to access the robot's current orientation for frame transformations.

#### 4. Reference Joint Name Configuration

**Modified Files**: `clearpath_mecanum_drive_controller.yaml`

**Parameter Added**: `reference_joint_names` array with configurable joint identifiers

**Functionality**: Allows the exported reference interface names to match the virtual joint names in the URDF, ensuring semantic consistency between planning and control layers.

**Default Configuration**:
```yaml
reference_joint_names: ["linear_x_joint", "linear_y_joint", "rotational_yaw_joint"]
```

### Compatibility Considerations

**Incompatibility with Standard Controller**: The standard `clearpath_mecanum_drive_controller` from public repositories will not function correctly in this architecture because:

1. It lacks the frame transformation logic, causing incorrect base motion when receiving world-frame commands
2. It does not support the `body_frame_yaw_joint` parameter, preventing orientation-based transformations
3. The state interface configuration does not accommodate additional interfaces beyond wheel states
4. Reference joint names may not align with URDF virtual joint conventions

**Required Controller Version**: Deployments must use the modified controller located in `src/external_dependencies/clearpath_mecanum_drive_controller` of this workspace. This version includes all necessary modifications for whole body planning support.

### Verification Steps

To verify the correct controller version is in use:

1. **Parameter Validation**: Confirm the controller accepts `body_frame_control` and `body_frame_yaw_joint` parameters without error
2. **Frame Transformation Test**: Command world-frame velocities and verify base motion aligns with global coordinates, not body-relative coordinates
3. **State Interface Check**: Verify the controller successfully reads five state interfaces (four wheels + yaw joint)
4. **Reference Interface Inspection**: Confirm exported reference interfaces match configured `reference_joint_names`

### Summary of Required Modifications

| Feature | Standard Controller | Modified Controller (Required) |
|---------|--------------------|---------------------------------|
| Frame transformation | Not supported | World-to-body coordinate conversion |
| `body_frame_control` parameter | Not available | Boolean flag to enable/disable transformation |
| `body_frame_yaw_joint` parameter | Not available | Joint name for orientation state |
| State interface count | 4 (wheels only) | 5 (wheels + yaw joint) |
| Frame compatibility | Body frame only | Both world frame and body frame |
| Reference joint names | Fixed or limited | Fully configurable via parameter |
| Use case | Navigation only | Navigation + whole body planning |

### Upstream Contribution Status

These modifications represent domain-specific functionality for mobile manipulation planning and may not be suitable for upstream contribution to the standard Clearpath controller, which is designed primarily for navigation applications. The modifications introduce dependencies on MoveIt-specific planning conventions and virtual joint patterns that would increase complexity for standard navigation use cases.

### Adaptation to Other Platforms

When adapting this architecture to different mobile platforms (differential drive, omnidirectional, Ackermann steering), similar modifications will be required in the respective platform controllers:

1. **Add frame transformation logic**: Implement world-to-body coordinate conversion using current robot orientation
2. **Configure orientation state access**: Enable controller to read robot yaw angle from state interfaces
3. **Implement mode selection**: Support both body-frame (for navigation) and world-frame (for planning) command modes
4. **Parameterize configuration**: Allow runtime selection of operation mode and state interface sources

These modifications follow the same architectural pattern regardless of drive configuration, though the specific kinematic equations will vary by platform type.

---

## Component 1: Virtual Joint Representation

### Design Rationale
The mobile base's three planar degrees of freedom (translation in X and Y, rotation about Z) are represented as joints within the URDF kinematic tree. This abstraction allows MoveIt to treat the mobile base as additional articulated joints, enabling unified trajectory planning across the combined arm-base system.

### Implementation Details
**Location**: `src/hangar_sim/description/ur5e_ridgeback.xacro:73-100`

```xml
<link name="virtual_rail_link_1"/>

<joint name="linear_x_joint" type="prismatic">
  <axis xyz="1 0 0"/>
  <parent link="world" />
  <child link="virtual_rail_link_1" />
  <origin xyz="0 0 0" rpy="0 0 0" />
  <limit effort="1000.0" lower="-20.0" upper="20.0"
         velocity="0.175" acceleration="10.0"/>
  <dynamics damping="20.0" friction="500.0" />
</joint>

<link name="virtual_rail_link_2"/>

<joint name="linear_y_joint" type="prismatic">
  <axis xyz="0 1 0"/>
  <parent link="virtual_rail_link_1" />
  <child link="virtual_rail_link_2" />
  <origin xyz="0 0 0" rpy="0 0 0" />
  <limit effort="1000.0" lower="-5.0" upper="55.0"
         velocity="0.175" acceleration="10.0"/>
  <dynamics damping="20.0" friction="500.0" />
</joint>

<joint name="rotational_yaw_joint" type="continuous">
  <axis xyz="0 0 1"/>
  <parent link="virtual_rail_link_2"/>
  <child link="ridgeback_base_link"/>
  <origin rpy="0 0 0" xyz="0 0 0"/>
  <limit effort="150.0" velocity="0.5235987755982988"/>
  <dynamics damping="0" friction="0"/>
</joint>
```

### Kinematic Chain Structure
The virtual joints establish the following kinematic chain: `world` → `virtual_rail_link_1` → `virtual_rail_link_2` → `ridgeback_base_link`. This chain represents the mobile base's pose in the global reference frame through three sequential transformations corresponding to planar motion.

### Control Characteristics
These joints lack direct physical actuators. Instead, their commanded velocities are transformed into mecanum wheel commands through inverse kinematics performed by the platform velocity controller. Joint state feedback is derived from odometry integration via the odometry-to-joint-state bridge.

---

## Component 2: MoveIt Planning Group Configuration

### Purpose
The Semantic Robot Description Format (SRDF) defines planning groups that specify which joint subsets MoveIt should consider for different manipulation tasks.

### Configuration Details
**Location**: `src/hangar_sim/config/moveit/picknik_ur.srdf:12-38`

```xml
<group name="manipulator">
  <chain base_link="ridgeback_base_link" tip_link="grasp_link"/>
  <joint name="linear_x_joint"/>
  <joint name="linear_y_joint"/>
  <joint name="rotational_yaw_joint"/>
</group>

<group name="arm_only">
  <chain base_link="base_link" tip_link="grasp_link"/>
</group>

<group name="linear_actuator">
  <link name="chassis_link"/>
  <link name="riser_link"/>
  <link name="top_link"/>
  <joint name="linear_x_joint"/>
  <joint name="linear_y_joint"/>
  <joint name="rotational_yaw_joint"/>
</group>
```

### Planning Group Specifications

| Group | DOF Count | Application | Joint Composition |
|-------|-----------|-------------|-------------------|
| `manipulator` | 9 | Coordinated whole body motion | 6 UR arm joints + 3 virtual base joints |
| `arm_only` | 6 | Fixed-base manipulation | 6 UR arm joints |
| `linear_actuator` | 3 | Base-only positioning | 3 virtual base joints |

### Architectural Significance
The `manipulator` group defines a kinematic chain originating from `ridgeback_base_link`, which is positioned by the virtual joints. This configuration enables MoveIt to optimize end-effector poses through coordinated motion of both the arm and mobile base, expanding the effective workspace and potentially improving manipulability.

---

## Component 3: Odometry-to-Joint-State Conversion

### System Requirements
MoveIt requires joint state information for all joints in the planning group to maintain an accurate robot state representation. For virtual joints representing the mobile base, these states must be derived from the platform's odometry.

### Implementation
**Location**: `src/hangar_sim/script/odometry_joint_state_publisher.py`

```python
class OdometryJointStateRepublisher(Node):
    def odom_callback(self, odom_msg):
        # Extract yaw angle from quaternion orientation
        q = odom_msg.pose.pose.orientation
        rotation_yaw = atan2(
            2.0 * (q.z * q.w + q.x * q.y),
            -1.0 + 2.0 * (q.w * q.w + q.x * q.x)
        )

        # Populate joint state message
        joint_state_msg = JointState()
        joint_state_msg.name = [
            "linear_x_joint",
            "linear_y_joint",
            "rotational_yaw_joint",
        ]
        joint_state_msg.position = [
            odom_msg.pose.pose.position.x,
            odom_msg.pose.pose.position.y,
            rotation_yaw,
        ]
        self.joint_states_pub_.publish(joint_state_msg)
```

### Data Flow Architecture
```
MuJoCo Simulation → /odom (nav_msgs/Odometry) → odometry_joint_state_publisher.py
    → /joint_states (sensor_msgs/JointState) → MoveIt Robot State
```

### Node Configuration
**Location**: `src/hangar_sim/launch/sim/robot_drivers_to_persist_sim.launch.py:274-280`

```python
odom_to_joint_state_repub = Node(
    package="hangar_sim",
    executable="odometry_joint_state_publisher.py",
    name="odometry_joint_state_publisher",
    output="log",
)
```

### Functional Description
This node subscribes to odometry messages published by the mecanum drive controller and converts the pose information into joint states for the three virtual joints. The conversion extracts the X and Y positions directly and computes the yaw angle from the quaternion orientation. These joint states are then published on the `/joint_states` topic, where they are consumed by `robot_state_publisher` and MoveIt's planning scene monitor.

---

## Component 4: Chainable Controller Architecture

### Architectural Pattern
The custom-modified `clearpath_mecanum_drive_controller` implements the ROS 2 control chainable controller pattern with **additional frame transformation capabilities** not present in the standard implementation. This layered control architecture facilitates modular command transformation and enables the joint trajectory controller to command the mobile base through a standardized interface while automatically handling coordinate frame conversions.

**Important**: This component requires the modified controller described in the "Custom Controller Modifications" section. The standard Clearpath mecanum drive controller does not provide the necessary frame transformation or configuration flexibility for whole body planning.

### Base Class Implementation
**Location**: `src/external_dependencies/clearpath_mecanum_drive_controller/.../clearpath_mecanum_drive_controller.hpp:51`

```cpp
class MecanumDriveController : public controller_interface::ChainableControllerInterface
```

### Reference Interface Export

**Location**: `src/external_dependencies/clearpath_mecanum_drive_controller/.../clearpath_mecanum_drive_controller.cpp:308-322`

```cpp
std::vector<hardware_interface::CommandInterface>
MecanumDriveController::on_export_reference_interfaces()
{
  std::vector<hardware_interface::CommandInterface> reference_interfaces;
  reference_interfaces_.resize(NR_REF_ITFS, std::numeric_limits<double>::quiet_NaN());

  for (size_t i = 0; i < reference_interfaces_.size(); ++i)
  {
    reference_interfaces.push_back(
      hardware_interface::CommandInterface(
        get_node()->get_name(),
        params_.reference_joint_names[i] + "/" + hardware_interface::HW_IF_VELOCITY,
        &reference_interfaces_[i]
      )
    );
  }

  return reference_interfaces;
}
```

### Reference Joint Naming Convention

**Location**: `src/external_dependencies/clearpath_mecanum_drive_controller/.../clearpath_mecanum_drive_controller.yaml:34-42`

```yaml
reference_joint_names: {
  type: string_array,
  default_value: ["linear_x_joint", "linear_y_joint", "rotational_yaw_joint"],
  description: "The names of the reference joints to be exported for controller chaining.",
}
```

### Exported Command Interfaces

The controller exports three velocity command interfaces accessible to upstream controllers:
- `platform_velocity_controller/linear_x_joint/velocity`
- `platform_velocity_controller/linear_y_joint/velocity`
- `platform_velocity_controller/rotational_yaw_joint/velocity`

These interfaces are named to match the virtual joints defined in the URDF, creating a semantic correspondence between the planning representation and the control interface. The joint trajectory controller writes velocity commands to these interfaces, which the mecanum drive controller then transforms into wheel velocity commands through its internal inverse kinematics.

---

## Component 5: Dual Controller Configuration

### Design Rationale
The system employs two distinct instances of the mecanum drive controller to support different operational modes with varying command frame conventions and input sources. This separation provides operational flexibility while maintaining a clean interface boundary between navigation and manipulation subsystems.

### Controller Specifications

**Location**: `src/hangar_sim/config/control/picknik_ur.ros2_control.yaml`

#### Platform Velocity Controller (Whole Body Planning)

```yaml
platform_velocity_controller:
  ros__parameters:
    interface_name: velocity
    command_joint_names: ["front_left_wheel", "rear_left_wheel",
                          "rear_right_wheel", "front_right_wheel"]
    reference_joint_names: ["linear_x_joint", "linear_y_joint",
                            "rotational_yaw_joint"]
    body_frame_control: false
    body_frame_yaw_joint: "rotational_yaw_joint"
```

#### Platform Velocity Controller Nav2 (Navigation)

```yaml
platform_velocity_controller_nav2:
  ros__parameters:
    interface_name: velocity
    command_joint_names: ["front_left_wheel", "rear_left_wheel",
                          "rear_right_wheel", "front_right_wheel"]
    body_frame_control: true
```

### Configuration Comparison

| Parameter | `platform_velocity_controller` | `platform_velocity_controller_nav2` |
|-----------|-------------------------------|-------------------------------------|
| **Operational Mode** | Whole body planning | Autonomous navigation |
| **Command Source** | `joint_trajectory_controller` via reference interfaces | Nav2 via `/cmd_vel` topic |
| **Command Frame** | World frame (internally transformed) | Body frame |
| **body_frame_control** | `false` | `true` |
| **body_frame_yaw_joint** | `rotational_yaw_joint` | Not specified |
| **Exports References** | Yes (for controller chaining) | No |

### Operational Characteristics

The `platform_velocity_controller` instance operates in world frame mode with automatic body frame transformation. It receives velocity commands from the joint trajectory controller through its exported reference interfaces. These commands are expressed in the world coordinate frame, consistent with MoveIt's planning output.

The `platform_velocity_controller_nav2` instance operates in body frame mode and subscribes to velocity commands on a topic remapped to Nav2's output. This configuration aligns with Nav2's control convention of commanding velocities in the robot's body frame.

---

## Component 6: Joint Trajectory Controller Configuration

### Controller Scope
The joint trajectory controller manages the execution of trajectories planned by MoveIt for the combined arm-base system. It commands both the physical arm joints and the virtual base joints through distinct interfaces.

### Configuration Specification

**Location**: `src/hangar_sim/config/control/picknik_ur.ros2_control.yaml:177-256`

```yaml
joint_trajectory_controller:
  ros__parameters:
    joints:
      - shoulder_pan_joint
      - shoulder_lift_joint
      - elbow_joint
      - wrist_1_joint
      - wrist_2_joint
      - wrist_3_joint
      - linear_x_joint
      - linear_y_joint
      - rotational_yaw_joint

    command_joints:
      - shoulder_pan_joint
      - shoulder_lift_joint
      - elbow_joint
      - wrist_1_joint
      - wrist_2_joint
      - wrist_3_joint
      - platform_velocity_controller/linear_x_joint
      - platform_velocity_controller/linear_y_joint
      - platform_velocity_controller/rotational_yaw_joint

    command_interfaces:
      - velocity

    state_interfaces:
      - position
      - velocity
```

### Parameter Semantics

**`joints` Parameter**: Specifies the complete set of joints in the URDF for which this controller maintains state tracking and trajectory execution. This list includes all nine joints of the manipulator planning group.

**`command_joints` Parameter**: Defines the actual command interface destinations for each joint. For the arm joints, this corresponds directly to the hardware interface. For the virtual base joints, this references the chainable interfaces exported by `platform_velocity_controller`.

### Interface Mapping

| Joint | State Source | Command Destination |
|-------|-------------|---------------------|
| Arm joints (6) | Direct hardware interfaces | Direct hardware interfaces |
| `linear_x_joint` | Odometry-derived joint state | `platform_velocity_controller/linear_x_joint/velocity` |
| `linear_y_joint` | Odometry-derived joint state | `platform_velocity_controller/linear_y_joint/velocity` |
| `rotational_yaw_joint` | Odometry-derived joint state | `platform_velocity_controller/rotational_yaw_joint/velocity` |

This asymmetric interface mapping enables closed-loop trajectory tracking. State feedback arrives through the odometry bridge, while commands are issued through the chainable controller's reference interfaces. The platform velocity controller receives these commands and transforms them into wheel velocities through its internal kinematics.

---

## Component 7: Coordinate Frame Transformation

### Transformation Requirements
MoveIt generates trajectories with velocities expressed in the world coordinate frame (typically `odom` or `map`). However, mecanum drive kinematics equations require velocity inputs expressed in the robot's body frame. The **custom-modified** mecanum drive controller performs this transformation automatically based on the robot's current orientation.

**Critical Note**: This frame transformation capability is **not present in standard mobile base controllers**. It has been specifically added to the modified `clearpath_mecanum_drive_controller` to enable whole body planning. Attempting to use a standard controller will result in incorrect base motion, as world-frame velocity commands will be incorrectly interpreted as body-frame commands.

### Implementation

**Location**: `src/external_dependencies/clearpath_mecanum_drive_controller/.../clearpath_mecanum_drive_controller.cpp:438-444`

```cpp
// Frame transformation when not in body_frame_control mode
if (!params_.body_frame_control && !params_.body_frame_yaw_joint.empty() &&
    state_interfaces_.size() >= 5)
{
  // Read current yaw angle from state interface
  const double theta = state_interfaces_[4].get_value();

  // Apply rotation matrix to transform velocities from world to body frame
  const double rotated_x = std::cos(-theta)*reference_interfaces_[0]
                          - std::sin(-theta)*reference_interfaces_[1];
  const double rotated_y = std::sin(-theta)*reference_interfaces_[0]
                          + std::cos(-theta)*reference_interfaces_[1];

  reference_interfaces_[0] = rotated_x;
  reference_interfaces_[1] = rotated_y;
}
```

### Transformation Mechanism

The controller applies a 2D rotation matrix to the commanded linear velocities, using the current yaw angle obtained from the `rotational_yaw_joint` state interface. This state is populated by the odometry bridge and represents the robot's current orientation in the world frame.

The transformation converts:
- Input: Linear velocities (vx, vy) in world frame coordinates
- Output: Linear velocities (vx', vy') in body frame coordinates

The angular velocity component requires no transformation as rotation about the Z-axis is invariant between the world and body frames for planar motion.

### State Interface Organization

The platform velocity controller reads state information from five interfaces corresponding to the four wheel positions and the yaw joint:

**Location**: `src/external_dependencies/clearpath_mecanum_drive_controller/.../clearpath_mecanum_drive_controller.cpp:296-306`

```cpp
for (const auto & joint : state_joint_names_)
{
  state_interfaces_config.names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
  state_interfaces_config.names.push_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
}
```

State interface mapping:
- `state_interfaces_[0-3]`: Wheel joint positions (front_left, rear_left, rear_right, front_right)
- `state_interfaces_[4]`: Yaw joint position (from odometry bridge)

The yaw position state enables the controller to perform frame transformations without external dependencies, maintaining the frame conversion logic within the controller boundary.

---

## Component 8: Mecanum Drive Inverse Kinematics

### Kinematic Model
The mecanum drive controller implements the inverse kinematic mapping from body-frame twist commands (linear x, linear y, angular z velocities) to individual wheel angular velocities. This transformation accounts for wheel geometry, robot dimensions, and mecanum wheel characteristics.

### Implementation

**Location**: `src/external_dependencies/clearpath_mecanum_drive_controller/.../clearpath_mecanum_drive_controller.cpp:446-501`

```cpp
// Transform to center frame accounting for base_frame_offset
velocity_in_center_frame_linear_x_ =
  velocity_in_base_frame_w_r_t_center_frame_.x() +
  linear_trans_from_base_to_center.y() * reference_interfaces_[2];

velocity_in_center_frame_linear_y_ =
  velocity_in_base_frame_w_r_t_center_frame_.y() -
  linear_trans_from_base_to_center.x() * reference_interfaces_[2];

velocity_in_center_frame_angular_z_ = reference_interfaces_[2];

// Mecanum drive inverse kinematics
double w_front_left_vel =
  1.0 / wheel_radius * (vx - vy - wheelbase * wz);
double w_back_left_vel =
  1.0 / wheel_radius * (vx + vy - wheelbase * wz);
double w_back_right_vel =
  1.0 / wheel_radius * (vx - vy + wheelbase * wz);
double w_front_right_vel =
  1.0 / wheel_radius * (vx + vy + wheelbase * wz);

// Write commands to wheel interfaces
command_interfaces_[0].set_value(w_front_left_vel);
command_interfaces_[1].set_value(w_back_left_vel);
command_interfaces_[2].set_value(w_back_right_vel);
command_interfaces_[3].set_value(w_front_right_vel);
```

### Kinematic Equations

The inverse kinematics are based on the standard mecanum drive model, which relates the robot's planar twist to wheel angular velocities through the wheel radius and robot geometry. The equations account for:

- Wheel radius (conversion from linear to angular velocity)
- Wheelbase geometry (sum of distances from robot center to wheel contact points)
- Mecanum roller angle (45-degree rollers creating omnidirectional motion capability)

Each wheel's velocity is computed as a linear combination of the three commanded velocity components, with coefficients determined by the wheel's position and roller orientation.

---

## Complete Control Flow: MoveIt to Hardware

### Trajectory Execution Sequence

The following sequence describes the complete data flow from motion planning through trajectory execution to hardware actuation:

**Step 1: Motion Request**
User initiates a motion request through MoveIt Pro interface or behavior tree execution, specifying the `manipulator` planning group (9 degrees of freedom).

**Step 2: Trajectory Generation**
MoveIt generates a collision-free trajectory in the world coordinate frame. The trajectory specifies positions, velocities, and accelerations for all nine joints (six arm joints and three virtual base joints) at discrete time points.

**Step 3: Trajectory Dispatch**
The trajectory is sent to the `joint_trajectory_controller` via its FollowJointTrajectory action interface.

**Step 4: Command Computation**
The controller interpolates the trajectory and computes instantaneous velocity commands for each joint. Example world-frame commands:
- `linear_x_joint/velocity = 0.1 m/s`
- `linear_y_joint/velocity = 0.0 m/s`
- `rotational_yaw_joint/velocity = 0.0 rad/s`

**Step 5: Interface Writing**
The controller writes arm joint commands directly to hardware interfaces and writes virtual joint commands to the chainable reference interfaces:
- `platform_velocity_controller/linear_x_joint/velocity = 0.1`
- `platform_velocity_controller/linear_y_joint/velocity = 0.0`
- `platform_velocity_controller/rotational_yaw_joint/velocity = 0.0`

**Step 6: Reference Interface Reading**
The `platform_velocity_controller` reads commanded velocities from its exported reference interfaces during its update cycle.

**Step 7: Frame Transformation**
The controller reads the current yaw angle from `state_interfaces_[4]` and applies the rotation transformation to convert world-frame linear velocities to body-frame coordinates.

**Step 8: Inverse Kinematics**
The body-frame twist is transformed into four wheel angular velocities using the mecanum drive kinematic model.

**Step 9: Hardware Command**
Wheel velocity commands are written to the MuJoCo simulation interfaces, causing wheel actuation.

**Step 10: Odometry Integration**
The simulation integrates wheel velocities to compute platform motion and publishes odometry messages on `/odom`.

**Step 11: State Conversion**
The `odometry_joint_state_publisher.py` node converts odometry messages to joint state messages and publishes them on `/joint_states`.

**Step 12: State Update**
MoveIt's planning scene monitor receives the joint state updates, updating the robot state representation. The `joint_trajectory_controller` uses this feedback for closed-loop trajectory tracking and error correction.

---

## Controller State Management

### Initial Configuration

**Location**: `src/hangar_sim/config/config.yaml:94-105`

```yaml
ros2_control:
  controllers_active_at_startup:
    - "force_torque_sensor_broadcaster"
    - "joint_state_broadcaster"
    - "platform_velocity_controller"
    - "vacuum_gripper"

  controllers_inactive_at_startup:
    - "joint_trajectory_controller"
    - "servo_controller"
    - "platform_velocity_controller_nav2"
    - "velocity_force_controller"
    - "arm_only_velocity_force_controller"
```

### State Transition Model

```
┌─────────────────────────────────────────────────────────────┐
│                      System Startup                         │
│                                                             │
│  Active Controllers:                                        │
│    - platform_velocity_controller                          │
│    - joint_state_broadcaster                               │
│                                                             │
│  Inactive Controllers:                                      │
│    - joint_trajectory_controller                           │
│    - platform_velocity_controller_nav2                     │
└─────────────────────────────────────────────────────────────┘
                           │
         ┌─────────────────┴─────────────────┐
         │                                   │
         ▼                                   ▼
┌─────────────────────┐          ┌──────────────────────────┐
│  Navigation Mode    │          │  Whole Body Planning     │
│                     │          │       Mode               │
│  Active:            │          │                          │
│  - Nav2 stack       │          │  Active:                 │
│  - platform_        │          │  - joint_trajectory_     │
│    velocity_        │          │    controller            │
│    controller_nav2  │          │                          │
│                     │          │  platform_velocity_      │
│  Inactive:          │          │  controller remains      │
│  - joint_trajectory_│          │  active, receiving       │
│    controller       │          │  reference commands      │
└─────────────────────┘          └──────────────────────────┘
```

### Controller Lifecycle

The `platform_velocity_controller` remains active throughout system operation as it serves dual purposes: exporting reference interfaces for whole body planning and providing a base controller for potential direct velocity commands. Its persistent activation ensures reference interfaces remain available for controller chaining.

The trajectory and navigation controllers are activated on-demand when their respective operational modes are engaged, preventing command conflicts between different control paradigms.

---

## Nav2 Integration

### Command Routing Configuration

**Location**: `src/hangar_sim/launch/sim/robot_drivers_to_persist_sim.launch.py:81-85`

```python
remappings = [
    ("/tf", "tf"),
    ("/tf_static", "tf_static"),
    ("/cmd_vel", "/platform_velocity_controller_nav2/cmd_vel_unstamped"),
]
```

Nav2's velocity commands are remapped to the `platform_velocity_controller_nav2` instance. This controller operates with `body_frame_control: true`, matching Nav2's convention of publishing body-frame velocity commands.

### Transform Tree Configuration

**Location**: `src/hangar_sim/launch/sim/robot_drivers_to_persist_sim.launch.py:256-272`

```python
# Static transform: MuJoCo world to map frame
static_tf_world_to_map = Node(
    package="tf2_ros",
    executable="static_transform_publisher",
    arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "mj_world", "map"],
)

# Static transform: map to odometry frame
static_tf_map_to_odom = Node(
    package="tf2_ros",
    executable="static_transform_publisher",
    arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "map", "odom"],
)
```

### Frame Hierarchy

The transform tree structure for the simulation environment:

```
mj_world (MuJoCo simulation root)
  │
  ├─ [static] → map
  │    │
  │    └─ [static] → odom
  │         │
  │         └─ [robot_state_publisher] → ridgeback_base_link
  │              (via virtual joint chain)
  │
  └─ [Other scene elements]
```

In production deployments with active localization, the static `map` to `odom` transform would be replaced with a dynamic transform published by the localization system (e.g., AMCL), representing the estimated pose correction between odometry and the global map frame.

---

## Transform Tree Configuration

### Odometry Transform Publishing Configuration

**Location**: `src/hangar_sim/config/control/picknik_ur.ros2_control.yaml`

Both platform velocity controller instances include the following parameter:

```yaml
platform_velocity_controller:
  ros__parameters:
    base_frame_id: "ridgeback_base_link"
    odom_frame_id: "odom"
    enable_odom_tf: false

platform_velocity_controller_nav2:
  ros__parameters:
    base_frame_id: "ridgeback_base_link"
    odom_frame_id: "odom"
    enable_odom_tf: false
```

### Rationale for Disabled Transform Publishing

The `enable_odom_tf: false` parameter prevents the mecanum drive controllers from publishing the `odom` to `ridgeback_base_link` transform to the `/tf` topic. This configuration is necessary to avoid transform conflicts in the ROS 2 transform system.

### Transform Publisher Allocation

| Transform | Publishing Node | Mechanism |
|-----------|----------------|-----------|
| `mj_world` → `map` | `static_transform_publisher` | Launch file configuration |
| `map` → `odom` | `static_transform_publisher` | Launch file configuration (simulation only) |
| `odom` → `ridgeback_base_link` | `robot_state_publisher` | URDF virtual joint chain with joint state feedback |

### Transform Source Analysis

The `robot_state_publisher` node computes and publishes the `odom` to `ridgeback_base_link` transform based on:
1. The URDF virtual joint chain definition (`world` → `virtual_rail_link_1` → `virtual_rail_link_2` → `ridgeback_base_link`)
2. Current joint state values received on `/joint_states` (populated by the odometry bridge)

If the mecanum drive controllers were configured with `enable_odom_tf: true`, they would publish the same transform based on wheel odometry integration. This would result in multiple publishers for the same transform, violating the ROS 2 transform system's requirement for unique transform publishers.

### Consequence of Misconfiguration

Enabling odometry transform publication on the controllers would produce the following issues:

**Multiple Transform Publishers**
```
Transform odom → ridgeback_base_link published by:
1. robot_state_publisher (from virtual joint states)
2. platform_velocity_controller (from wheel odometry)
3. platform_velocity_controller_nav2 (from wheel odometry)
```

**System-Level Effects**
- TF_REPEATED_DATA warnings in system logs
- Non-deterministic transform lookups with race conditions
- Inconsistent robot state representation across nodes
- Planning failures due to unreliable transform data
- Potential robot position discontinuities in visualization

### Odometry Message Publication

The controllers continue to publish odometry data as messages despite disabled transform publishing:

| Component | Publishes Transform | Publishes Odometry Messages |
|-----------|--------------------|-----------------------------|
| `platform_velocity_controller` | No (`enable_odom_tf: false`) | Yes (to `/platform_velocity_controller/odom`) |
| `platform_velocity_controller_nav2` | No (`enable_odom_tf: false`) | Yes (to `/platform_velocity_controller_nav2/odom`) |
| `robot_state_publisher` | Yes (from URDF with joint states) | No |
| `odometry_joint_state_publisher.py` | No | No (converts messages to joint states) |

### Message vs Transform Distinction

**Odometry Messages**: Data structures of type `nav_msgs/Odometry` published on topics, containing pose and twist estimates with covariance information. These messages are consumed by:
- Nav2 for localization and path tracking
- The odometry bridge for joint state conversion
- Monitoring and diagnostic tools
- Data logging systems

**Odometry Transforms**: Geometric transformations published to the `/tf` system, representing the spatial relationship between coordinate frames. These are used by:
- Transform lookup operations
- Coordinate frame conversions
- Robot state publisher
- Planning and control algorithms requiring spatial relationships

The architecture separates these concerns by having controllers publish odometry messages while delegating transform publication to the robot state publisher, which integrates information from all joint states to maintain a consistent kinematic tree.

### Production Deployment Considerations

In real-world systems with active localization:
- Remove the static `map` → `odom` transform publisher
- Configure the localization system (AMCL, Cartographer, etc.) to publish the dynamic `map` → `odom` transform
- Maintain `enable_odom_tf: false` on platform controllers
- The `odom` → `base_link` transform continues to be published by `robot_state_publisher` via virtual joints
- Localization system consumes odometry messages to estimate drift and publish correction transforms

---

## Key Architectural Principles

### Virtual Joint Abstraction

The virtual joint representation decouples the motion planning interface from the physical actuation mechanism. MoveIt interacts with the mobile base through standard joint interfaces, without knowledge of the underlying mecanum drive kinematics. This abstraction enables the use of standard motion planning algorithms without mobile-base-specific modifications.

### Layered Control Through Chaining

The chainable controller pattern establishes a hierarchical control architecture where the joint trajectory controller operates at the trajectory execution layer, while the mecanum drive controller operates at the velocity-to-wheel-command layer. This separation of concerns improves modularity and allows independent development and testing of control layers.

### Automatic Frame Transformation

The architecture implements coordinate frame transformations within the controller layer rather than requiring planning-level frame awareness. MoveIt generates trajectories in the world frame using its standard planning algorithms, and the mecanum drive controller transparently handles the conversion to body-frame velocities required for drive kinematics. This design preserves the separation between planning and control concerns.

### Mode-Specific Controller Instances

The dual controller configuration provides specialized interfaces for navigation and manipulation modes while sharing the underlying kinematic model implementation. This approach avoids mode logic within individual controllers and enables clean transitions between operational paradigms through controller activation state changes.

### Closed-Loop State Feedback

The odometry-to-joint-state bridge completes the control loop by providing state feedback for the virtual joints. This enables:
- Accurate robot state representation for collision checking and workspace analysis
- Closed-loop trajectory tracking with error correction
- Frame transformation based on current orientation
- Consistent state estimates across planning and control subsystems

---

## Implementation Checklist

For implementing whole body planning on mobile manipulator platforms:

**Critical Prerequisite: Custom Controller**
- **Obtain or implement modified platform controller** with frame transformation support
- Verify controller supports `body_frame_control` and `body_frame_yaw_joint` parameters
- Confirm frame transformation logic is implemented (world-to-body conversion)
- Test controller can read orientation state interface beyond standard wheel states
- **Note**: Standard mecanum drive controllers will not work without these modifications

**URDF Configuration**
- Define virtual prismatic and/or revolute joints representing mobile base degrees of freedom
- Establish kinematic chain from world frame through virtual joints to mobile base link
- Specify appropriate joint limits, velocity limits, and dynamics parameters

**SRDF Planning Groups**
- Create planning group including both manipulator and virtual base joints
- Define kinematic chain with base link as child of virtual joints
- Configure alternative groups for arm-only and base-only planning as needed

**Odometry Bridge Implementation**
- Implement node to subscribe to platform odometry messages
- Extract pose information and convert to joint states for virtual joints
- Publish joint states on `/joint_states` topic with appropriate frame and timestamp
- Ensure node launches with simulation or robot driver systems

**Chainable Platform Controller**
- Implement or configure platform controller extending `ChainableControllerInterface`
- Configure `reference_joint_names` to match virtual joint identifiers
- Set `body_frame_control: false` for whole body planning mode
- Specify `body_frame_yaw_joint` parameter for frame transformation
- Configure `enable_odom_tf: false` to prevent transform conflicts
- Implement or verify inverse kinematics for specific drive configuration

**Trajectory Controller Configuration**
- Include virtual joints in `joints` parameter for state tracking
- Specify chainable reference interfaces in `command_joints` (format: `controller_name/joint_name`)
- Configure velocity command interfaces for mobile base control
- Verify controller chaining sequence in controller manager

**Navigation Controller Setup**
- Configure separate platform controller instance with `body_frame_control: true`
- Establish topic remapping from Nav2 command output to controller input
- Configure appropriate activation state (inactive at startup, activated with Nav2)

**Launch Configuration**
- Start odometry bridge node with appropriate topic remappings
- Configure static transform publishers for simulation environment
- Verify controller activation sequence and dependencies
- Establish appropriate QoS settings for real-time control topics

**Transform Tree Validation**
- Verify single publisher for each transform in the tree
- Confirm `robot_state_publisher` publishes odometry-to-base transform
- Validate frame consistency using `tf2_tools view_frames`
- Monitor for TF_REPEATED_DATA warnings during operation

---

## Troubleshooting Reference

### Common Issues and Resolutions

| Symptom | Probable Cause | Diagnostic Steps | Resolution |
|---------|---------------|------------------|------------|
| Mobile base unresponsive during whole body planning | Trajectory controller not activated | Query controller manager state: `ros2 control list_controllers` | Verify controller activates when executing trajectories; check action server connection |
| Base motion in incorrect direction | Frame transformation error | Verify yaw joint in `/joint_states`; check `body_frame_yaw_joint` parameter | Confirm `odometry_joint_state_publisher.py` is running; verify parameter configuration |
| Nav2 commands not executed | Incorrect controller active state | Check controller manager state; verify command topic remapping | Activate `platform_velocity_controller_nav2`; verify `/cmd_vel` remapping |
| Virtual joints absent from state | Odometry bridge failure | Monitor `/joint_states` for virtual joint messages; check node status | Restart `odometry_joint_state_publisher.py`; verify `/odom` topic publication |
| Planning failures for whole body group | Incorrect planning group configuration | Examine SRDF planning group definition; verify joint names | Use `manipulator` group for whole body; confirm SRDF includes virtual joints |
| TF_REPEATED_DATA warnings | Multiple transform publishers | Execute `ros2 run tf2_tools view_frames`; identify duplicate publishers | Set `enable_odom_tf: false` in both platform controllers; verify single source per transform |
| Discontinuous position in visualization | Transform tree inconsistency | Monitor `/tf` for conflicting transforms; check timing | Verify transform publication sources; ensure consistent timestamp sources |
| Reference interface commands ignored | Controller chaining misconfiguration | Verify exported reference interfaces; check command_joints parameter | Confirm chainable controller exports interfaces; validate naming in trajectory controller |
| Excessive position drift | Odometry integration error | Compare wheel odometry to ground truth; monitor joint state updates | Verify odometry message publication rate; check wheel velocity measurements |
| Base moves in robot-relative direction instead of world coordinates | Using standard controller without frame transformation | Verify controller version; check for `body_frame_yaw_joint` parameter | Replace with modified controller containing frame transformation logic; confirm `body_frame_control: false` |
| Controller fails to load with "unknown parameter" errors | Standard controller missing custom parameters | Review controller parameter definitions | Use modified controller from `src/external_dependencies/clearpath_mecanum_drive_controller` |

### Diagnostic Commands

**Controller State Inspection**
```bash
ros2 control list_controllers
ros2 control list_hardware_interfaces
```

**Transform Tree Analysis**
```bash
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo odom ridgeback_base_link
```

**Topic Monitoring**
```bash
ros2 topic echo /joint_states
ros2 topic echo /platform_velocity_controller/odom
ros2 topic hz /joint_states
```

**Interface Verification**
```bash
ros2 control list_controller_types
ros2 param get /controller_manager platform_velocity_controller
```

---

## Conclusion

This architecture achieves coordinated mobile manipulation through a modular, layered design that integrates mobile base control into MoveIt's standard planning framework. The key architectural innovations include:

**Abstraction Layer**: Virtual joints provide a uniform interface for mobile base motion, enabling MoveIt to plan for the combined arm-base system using standard algorithms without platform-specific modifications.

**Controller Chaining**: The chainable controller pattern establishes a clean separation between trajectory execution and platform-specific kinematics, improving modularity and maintainability.

**Transparent Frame Management**: Automatic coordinate transformation within the control layer eliminates the need for planning-level frame awareness, preserving the separation of concerns between planning and control.

**Bidirectional State Flow**: The odometry bridge closes the control loop by converting platform odometry into joint state feedback, enabling accurate state estimation and closed-loop trajectory tracking.

**Mode-Specific Interfaces**: Dual controller configuration provides optimized interfaces for navigation and manipulation while sharing implementation, facilitating clean mode transitions through controller state management.

This design pattern can be adapted to other mobile manipulator configurations by modifying:
- Virtual joint definitions to match the mobile base's degrees of freedom
- Platform controller kinematics to match the drive configuration (differential, omnidirectional, Ackermann, etc.)
- Odometry conversion to extract appropriate state information for the virtual joints

The architecture's modularity ensures that changes to the mobile platform do not require modifications to the motion planning subsystem, and vice versa, supporting independent evolution of system components.

### Implementation Requirements

**Critical**: Successful deployment requires a platform controller with integrated frame transformation capabilities. Standard mobile base controllers designed exclusively for navigation do not provide this functionality. Implementation teams must either:

1. Modify existing platform controllers to add world-to-body frame transformation logic
2. Develop custom controllers with built-in frame conversion capabilities
3. Utilize the modified `clearpath_mecanum_drive_controller` included in this workspace as a reference implementation

The frame transformation capability is fundamental to the architecture and cannot be omitted or worked around through alternative configuration. Controllers lacking this feature will produce incorrect base motion when commanded by MoveIt's world-frame trajectories.
