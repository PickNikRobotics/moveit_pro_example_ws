# KDL gravity compensation controller

A ros2_control **chainable** downstream controller for a URDF serial chain.
It reads measured joint positions and writes, per joint:

```text
hardware effort = upstream effort + gains[i] * gravity(q)[i] + offsets[i]
```

Gravity comes from KDL `ChainDynParam::JntToGravity`. Gains and offsets apply
only to the gravity contribution, never to upstream effort. The controller
adds gravity once per update; it does not accumulate commands or change the
driver, motor gains, torque limits, or motion policy.

## Interfaces and ownership

Plugin: `kdl_gravity_compensation_controller/GravityCompensationController`.
Requires ROS 2 Jazzy `controller_interface` **4.45.2 or newer** (shared-pointer
`on_export_reference_interfaces_list()` API). The plugin base is now
`controller_interface::ChainableControllerInterface`, rather than
`ControllerInterface`; the plugin name and gravity parameters are unchanged.

For `joints: [joint1, joint2]` and controller name
`gravity_compensation_controller`:

| Role | Interfaces, in configured joint order |
| --- | --- |
| Hardware command claims | `joint1/effort`, `joint2/effort` |
| Hardware state reads | `joint1/position`, `joint2/position` |
| Exported upstream command references | `gravity_compensation_controller/joint1/effort`, `gravity_compensation_controller/joint2/effort` |

All values are doubles, in the URDF joint frame (effort in Nm for revolute
joints, N for prismatic joints). `joints` must contain exactly the movable
joints from root to tip, in chain order. Hardware loans are mapped by name;
no hardware declaration order is assumed. No state interfaces are exported.

**Gravity is the sole hardware effort writer.** An upstream effort producer
claims the exported names, not `joint1/effort` concurrently. A position
controller can coexist only if the hardware supports simultaneous position
and effort commands. An effort-only driver needs only the downstream effort
claim. This controller does not turn a position-only driver into an effort
driver, and it does not provide position holding, damping or trajectory control.

## Standalone gravity-only setup

Add the plugin type and parameters to a controller manager's configuration:

```yaml
controller_manager:
  ros__parameters:
    update_rate: 100
    gravity_compensation_controller:
      type: kdl_gravity_compensation_controller/GravityCompensationController

gravity_compensation_controller:
  ros__parameters:
    joints: [joint1, joint2]
    root_link: base_link
    tip_link: tool0
    gravity_vector: [0.0, 0.0, -9.81]
    payload_mass: 0.0
    payload_com: [0.0, 0.0, 0.0]
    gains: [0.0, 0.0]
    offsets: [0.0, 0.0]
```

Use the actual URDF chain and declare each joint's `effort` command and
`position` state in its `<ros2_control>` block. The robot description is
provided by controller manager, not by a separate controller parameter.
Load/configure the controller **inactive** first:

```bash
ros2 run controller_manager spawner gravity_compensation_controller --inactive
```

In a private fake-hardware setup, activating only this controller exercises
standalone operation:

```bash
ros2 control switch_controllers --activate gravity_compensation_controller --strict
```

No upstream producer or input topic is needed. Standalone always uses zero
upstream effort. The example's explicit zero gains/offsets produce zero output;
nonzero gravity requires deliberate configuration and commissioning. The
library preserves the original parameter default of unit gains when `gains`
is omitted, so **do not omit gains to disable gravity**.

## Chaining an upstream effort controller

Configure the upstream controller's command interface names to use the exported
references. For example, the standard `forward_command_controller` can forward
an effort array in a fake-hardware test:

```yaml
controller_manager:
  ros__parameters:
    upstream_effort:
      type: forward_command_controller/ForwardCommandController

upstream_effort:
  ros__parameters:
    joints:
      - gravity_compensation_controller/joint1
      - gravity_compensation_controller/joint2
    interface_name: effort
```

Merge that with the gravity configuration above. Load/configure gravity first,
then the upstream controller. In a private fake-hardware setup:

```bash
ros2 run controller_manager spawner upstream_effort --inactive
ros2 control switch_controllers --activate gravity_compensation_controller upstream_effort --strict
ros2 topic pub --once /upstream_effort/commands std_msgs/msg/Float64MultiArray '{data: [0.2, -0.1]}'
```

Controller manager makes gravity's references available, enters chained mode,
loans the references to the upstream controller, and activates the requested
controllers before their first update. It schedules **upstream before gravity**
on update; activation order within a switch request is not update order.
**Producers must write every claimed reference on every gravity cycle**,
synchronously at gravity's update rate. Each update consumes the references and
clears them to NaN, so a skipped cycle contributes zero effort, not the last
value; asynchronous/multi-rate producers are not supported. A trajectory or spring-mass-damper controller must likewise
support separate command-interface names; adding another direct hardware
`effort` claimant is not chaining. No production upstream controller is added
by this package.

The forward controller is only an interface example, not a safe motion
controller: it holds its last array and provides no command timeout, damping,
position/velocity limits or collision checking. Commands above are for fake
hardware only. Use a commissioned effort producer and hardware-specific safety
procedures on a real robot.

## Lifecycle, missing data and diagnostics

- Reference handles start as NaN, the standard ros2_control "no command"
  value. Every update reads each reference once and clears it to NaN;
  activation, deactivation and chained-mode switches clear them too. A NaN
  reference (unclaimed, released, or not written this cycle) contributes zero,
  so gravity-only output continues. Standalone ignores references entirely.
- Controller manager performs mode switches while gravity is inactive. To
  return to standalone, deactivate the upstream controller through controller
  manager; to stop the chain, deactivate upstream and gravity together. It
  rejects stopping gravity alone while an upstream claimant remains active.
- Swapping producers within one switch, including to one that claims fewer
  joints, leaves no effort behind on released joints: they read NaN on the
  next update.
- Activation requires finite measured positions and complete, correctly named
  double interfaces. Missing/non-finite positions, unreadable or infinite
  references, solver errors, non-finite output (including arithmetic
  overflow), and failed command writes return `ERROR`. Invalid updates attempt to zero **all** hardware efforts and
  do not publish a successful output. A failed zero write is still an error,
  not evidence that hardware reached zero. Controller manager handles error
  deactivation of the chain.
- Deactivation and shutdown attempt to zero effort. **Zero effort is not a
  gravity-loaded arm's safe hold.** Support the load and follow the robot's
  stopping procedure before controller switches or deactivation; no ramp,
  automatic hold, thermal policy or safety-rated stop is supplied here.

`~/commanded_effort` (`sensor_msgs/msg/JointState`) reports the measured
positions used and the **summed** effort requested at a successful update, in
`joints` order. It is not a motor-torque measurement or post-driver-limit
readback. Publication is best-effort realtime and may skip samples.

### Parameters

| Parameter | Default | Applied |
| --- | --- | --- |
| `joints` | empty, must be set | configure |
| `root_link` | `base_link` | configure |
| `tip_link` | empty, must be set | configure |
| `gravity_vector` | `[0.0, 0.0, -9.81]` m/s² in root frame | configure |
| `payload_mass` | `0.0` kg; finite and nonnegative | configure |
| `payload_com` | `[0.0, 0.0, 0.0]` m in tip frame | configure |
| `gains` | one per joint; empty means all `1.0` on configure | live |
| `offsets` | one per joint; empty means all `0.0` on configure | live |

Live gains/offsets must have one finite value per joint; rejected parameter
transactions leave both the parameters and realtime trim unchanged. Other
parameters require cleanup/reconfigure to take effect. Gravity and payload
vectors must have three finite values. Payload is added to the tip's existing
inertia; do not count a payload already in the URDF twice. Use the gravity
vector for mount orientation. The serial chain does not automatically include
masses on branches outside root-to-tip.

Compared with the earlier standalone controller, unreadable position no longer
returns `OK` with the previous torque. It returns `ERROR`, as do non-finite
inputs/outputs. This is intentional failure reporting, not a new hardware
safety policy.

## Tests

With this workspace and its dependencies sourced:

```bash
colcon build --packages-select kdl_gravity_compensation_controller
source install/setup.bash
colcon test --packages-select kdl_gravity_compensation_controller
colcon test-result --verbose
```

`test_gravity_model` compares KDL with hand-derived planar-arm torques, payloads
and mount orientation. `test_gravity_compensation_controller` uses real loaned
interfaces to exercise ordering, reference resets, live trim, finite/missing
inputs and failed writes. `test_controller_chaining` loads the plugin into a
real `ControllerManager` with `mock_components/GenericSystem` and a test-only
upstream producer, on two- and six-joint test arms. It checks reference
ownership, claim conflicts, activation and update ordering, nonzero varying
inputs, partial-joint references, a six-to-three-joint producer swap, missed
writes, summed hardware output, standalone operation, lifecycle transitions and
error recovery.
No test opens a device, uses CAN, or launches a MoveIt Pro deployment. These are
software composition tests, not a physical balance or safety certification.
