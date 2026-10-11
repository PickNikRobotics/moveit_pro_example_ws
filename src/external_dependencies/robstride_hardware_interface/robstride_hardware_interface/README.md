# robstride_hardware_interface

## 1. Introduction

A [ros2_control](https://github.com/ros-controls/ros2_control) `hardware_interface::SystemInterface` plugin for [RobStride](https://www.robstride.com/) CAN actuators (RS00–RS06), built on [robstride_sdk](https://github.com/kmj060703/RobstrideSDK) and [robstride_interfaces](https://github.com/kmj060703/robstride_interfaces).

`read()` and `write()` never block on CAN I/O — they touch only the SDK's lock-free per-motor state and batch outgoing frames through `sendmmsg()`. The achievable update rate is therefore bounded by CAN bus bandwidth rather than by request/response round trips; see §10.

## 2. Prerequisites

- ROS 2 Jazzy
- A SocketCAN adapter per bus (e.g. a canable/candlelight USB-CAN), RobStride motors, and a motor power supply
- `robstride_sdk` and `robstride_interfaces` in the same workspace

## 3. Installation

```bash
cd ~/${WORKSPACE}/src
git clone https://github.com/kmj060703/RobstrideSDK.git
git clone https://github.com/kmj060703/robstride_hardware_interface.git
git clone https://github.com/kmj060703/robstride_interfaces.git

cd ~/${WORKSPACE}
colcon build --packages-select robstride_sdk robstride_interfaces robstride_hardware_interface
source install/setup.bash
```

## 4. CAN interface setup

RobStride motors run at 1 Mbps. Bring each interface up before launching:

```bash
./scripts/setup_can.sh can0                 # 1 Mbps, txqueuelen 1000
./scripts/setup_can.sh can0 1000000 1000    # explicit bitrate / txqueuelen
```

`txqueuelen` matters: activation sends a burst of several frames per motor, and the kernel default of 10 is small enough to drop most of it on a rig of any size.

Many gs_usb/candlelight adapters do not support bus-off auto-recovery (`restart-ms`); the script detects this and continues without it. On those adapters a bus-off event needs a manual `down`/`up` cycle, or the adapter re-plugged. The symptom is `SendFrames()` failing with `ENOBUFS` while the interface still reports `UP`, which the plugin logs explicitly rather than letting it surface as every motor going silent.

## 5. Configuration

### Hardware parameters

| Param | Meaning | Default |
|---|---|---|
| `master_id` | Host CAN id used as the source address of outgoing frames | `253` (`0xFD`) |
| `number_of_joints` | Optional cross-check against the number of `<joint>` blocks | (unchecked) |
| `error_timeout_ms` | Time without fresh feedback before a joint counts as lost | `4000` |
| `motor_can_timeout_ms` | Each motor's own `CAN_TIMEOUT` watchdog (`0x7028`): after this long without a command the motor releases torque. `0` disables it, so when the bus dies a motor keeps holding the last target it received (the freeze hold). Motors that lose power still come back limp. A clean shutdown disables torque unless `hold_torque_on_deactivate` says otherwise | `error_timeout_ms` |
| `freeze_on_joint_loss` | See §6 | `true` |
| `torque_enable` | Whether activation engages torque. `false` brings the rig up readable but limp: joints report state and can be moved by hand until `~/set_torque` is called | `true` |
| `hold_torque_on_deactivate` | Whether `on_deactivate()` leaves torque engaged. `false` disables every motor, which drops a gravity-loaded arm wherever it is standing. `true` holds the pose through a clean stop, and then only while `motor_can_timeout_ms` is `0` — an armed motor watchdog releases torque that long after the bus goes quiet, whatever this is set to. `on_init()` warns on that combination | `false` |

### Joints and gpios

Each motor is described twice, separating the two things that are otherwise easy to confuse:

- **`<joint>`** — the `ros2_control` contract: which motor id it drives, which command interface it is driven through, which states it reports.
- **`<gpio>`** — the motor as a device: model, bus, run mode and gains, plus interfaces for reading health and retuning gains at runtime.

The two are matched by id — a joint's `id` must equal some gpio's `ID`, and `on_init` refuses to start otherwise.

Joint params:

| Param | Meaning |
|---|---|
| `id` | Motor's CAN id. Must match a gpio's `ID`. |

gpio params:

| Param | Meaning |
|---|---|
| `type` | `robstride` |
| `ID` | Motor's CAN id |
| `can_interface` | SocketCAN interface this motor is on. Motors may use different buses, e.g. one per quadruped leg. |
| `actuator_type` | `"00"`..`"06"`, `"0"`..`"6"`, or `"RS00"`..`"RS06"` |
| `control_mode` | `motion` (default), `position_pp`, `velocity`, `current`, `position_csp` |
| `kp`, `kd` | Initial gains. `motion` mode only; the other modes are closed by the motor's own loops. |
| `direction` | `1` (default) or `-1`. Joint frame = `direction` × motor frame for position, velocity and effort, so a motor mounted opposite to the URDF axis needs no model change. Raw parameter writes through `~/set_data_to_robstride` stay in the motor frame. |

gpio interfaces:

| Interface | Kind | Meaning |
|---|---|---|
| `kp`, `kd` | command | Retune gains while running. Same destination as `~/set_data_to_robstride`. |
| `temperature` | state | Motor temperature (°C) |
| `run_state` | state | `RunState` enum (`2` = torqued) |
| `fault_bits` | state | Latched fault word |

**The joint's command interface must match its gpio's `control_mode`**: `velocity` mode is driven through `velocity`, `current` through `effort`, and the position modes through `position`. `on_init` rejects any other pairing — a controller claiming an interface the joint's mode never writes would claim it successfully and then achieve nothing, with no error to explain why the joint does not move.

Only declared interfaces are exported, so a joint cannot be claimed through an interface it did not declare.

```xml
<ros2_control name="robstride" type="system">
  <hardware>
    <plugin>robstride_hardware_interface/RobstrideHardware</plugin>
    <param name="master_id">253</param>
    <param name="error_timeout_ms">4000</param>
  </hardware>

  <joint name="rs_joint1">
    <param name="id">1</param>
    <command_interface name="velocity"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    <state_interface name="effort"/>
  </joint>

  <gpio name="rs1">
    <param name="type">robstride</param>
    <param name="ID">1</param>
    <param name="actuator_type">00</param>
    <param name="can_interface">can0</param>
    <param name="control_mode">velocity</param>
    <param name="kp">5.0</param>
    <param name="kd">0.3</param>
    <param name="direction">-1</param>
    <command_interface name="kp"/>
    <command_interface name="kd"/>
    <state_interface name="temperature"/>
    <state_interface name="run_state"/>
    <state_interface name="fault_bits"/>
  </gpio>
</ros2_control>
```

A joint driven through `velocity` should be `type="continuous"` in the URDF, not `revolute`. With command limit enforcement on, a `revolute` joint that ends up outside its position limits has its velocity clamped to zero in **both** directions, including the one that would bring it back, so any overshoot is unrecoverable without intervention. Position-commanded joints are unaffected: their commands are clamped to the limit rather than zeroed, which drives them back into range.

For a bench test without hardware, swap the `<hardware>` block for `mock_components/GenericSystem`.

## 6. Activation and joint loss

### Activation never produces motion

Engaging torque may only ever mean "hold where you already are". The closed-loop modes keep their target inside the motor, in RAM that survives a deactivate/reactivate cycle, so enabling with a stale `loc_ref` still loaded would make the motor servo back to wherever it was last aimed. `on_activate()` therefore preloads every target before any torque exists:

1. Open each bus and start its read thread.
2. Arm each motor's `CAN_TIMEOUT` watchdog and select its `run_mode`.
3. Send a Stop frame to every motor and wait for the reply. Stop is answered with the same Type-2 feedback as any other command, so this doubles as a position probe that cannot move anything — and it also clears any torque left on by a run that died before `on_deactivate`.
4. Preload each mode's target (`loc_ref` / `spd_ref` / `iq_ref`) from that measured position, and seed the command interfaces from the same reading.
5. If `torque_enable` is `false`, stop here: the motors stay readable and free to move.
6. Otherwise send Enable and wait up to `error_timeout_ms` for each motor to report `run_state == MOTOR`, resending every 200 ms since CAN frames are fire-and-forget.
7. If any joint never confirms, disable everything, tear down the buses and return `ERROR`.

If a joint reports no position in step 3, activation is refused rather than engaging torque against an unknown target.

### Losing a joint

A joint counts as lost when either:

- `error_timeout_ms` passes without fresh feedback, or
- with torque requested, it leaves the enabled state after having been confirmed enabled. A motor whose supply blips reboots with torque off and can answer again well inside `error_timeout_ms`, so staleness alone would miss it. A fault that drops the motor out of the enabled state counts the same way.

With `freeze_on_joint_loss` at its default of `true`, the component **freezes**:

- The lost joint is disabled and is **not** re-enabled. (Its own `CAN_TIMEOUT` watchdog has usually released torque already.)
- Every reachable joint keeps torque and holds the position it had at the moment of the loss.
- Controller commands are ignored, since the controllers are advancing setpoints against partly unknown state.
- `read()` returns `OK`, so nothing is torn down and no controllers are deactivated.

The freeze is latched. A recovered joint stays limp and control is **not** handed back: a rebooted motor's position may be a whole turn off, a link that dropped once can drop again, and the controllers' setpoints have moved on in the meantime. Only a fresh activation resumes control. Because a fault now freezes too, `~/reboot_robstride` clears the fault but does not by itself bring the joint back; reactivate afterwards.

Setting `freeze_on_joint_loss` to `false` restores the conventional behavior: `read()` returns `ERROR`, the framework deactivates the component, and torque is cut on every joint.

Independently of this, `write()` substitutes a Disable frame for any joint that is stale or reporting a fault, rather than continuing to drive it toward a possibly outdated target. While a joint is untorqued, `read()` tracks its command position to its measured position, so it re-engages targeting where it actually is.

### Deactivation

By default `on_deactivate()` disables every motor. That is right for a rig that can sit
unpowered, and wrong for an arm that gravity acts on: the joints release wherever they happen to
be standing and the arm drops onto whatever is below it.

`hold_torque_on_deactivate` makes a clean stop hold the pose instead. Nothing new is commanded
either way — the motors simply keep the target they were already holding. Two consequences worth
stating plainly:

- The arm stays **rigid** after the stack stops, until motor power is cut. There is no software
  path back to limp once the component is gone.
- It only holds while `motor_can_timeout_ms` is `0`. Deactivation closes the bus, so a motor
  whose own watchdog is armed releases torque that long afterwards no matter what this setting
  says. `on_init()` warns when it sees that combination.

A hard kill already behaves this way: the destructor stops and closes the buses without sending
Disable, so the motors hold until their own watchdog fires. Setting
`hold_torque_on_deactivate` makes the clean path agree with the crash path rather than
contradict it.

### Position after a motor power cycle

A motor only knows its angle within one turn when it powers up, so after a power cycle its reading can come back offset by a whole multiple of 2π. The reported `position` is therefore wrapped into a one-turn window centered on the middle of the joint's URDF limits (0 for a joint without position limits), and `write()` adds the dropped turns back before sending a target. A joint whose range is narrower than a full turn reads the same after a power cycle as before it. A joint without position limits is treated as continuous: its targets take the shortest way round.

This recovers whole turns only. A mechanical zero the motor forgets across a power cycle shifts the reading by an arbitrary amount, and the wrapped value then looks plausible but is wrong. Make sure the zero is stored in the motor before relying on this.

**Position feedback wraps at ±4π** — a firmware characteristic, not something this driver adds. See [robstride_sdk's README](https://github.com/kmj060703/RobstrideSDK/blob/main/README.md#5-actuator-models-and-limits) for what that means for continuously-rotating joints.

## 7. Services and active controllers

`~/set_torque` and `~/set_zero_robstride` talk only to the motor. Neither knows about, nor can override, whichever controller currently claims the command interface — an active `joint_trajectory_controller` keeps writing its own held setpoint every cycle regardless. Deactivate it first:

```bash
ros2 control switch_controllers --deactivate robstride_controller --controller-manager /robstride/controller_manager
ros2 service call /robstride/robstride_hardware_interface/set_torque std_srvs/srv/SetBool "{data: false}"
# ... move the joint by hand, call set_zero_robstride, etc ...
ros2 service call /robstride/robstride_hardware_interface/set_torque std_srvs/srv/SetBool "{data: true}"
ros2 control switch_controllers --activate robstride_controller --controller-manager /robstride/controller_manager
```

`joint_trajectory_controller` reads the measured position when it activates, so it resumes from wherever the joint actually ended up.

`~/set_zero_robstride` is the more dangerous of the two, for a different reason. It moves nothing itself, but it redefines what zero means at the joint's current position. If a controller is holding a setpoint of, say, `3.0` at that moment, the physical spot that used to read `3.0` now reads `0.0`, and the controller — still chasing the literal number — drives the motor somewhere new to match. The motion comes entirely from the controller reacting to the reference frame shifting underneath it.

## 8. Topics and services

All under the hardware component's own node namespace (`~`):

| Name | Type | Description |
|---|---|---|
| `~/robstride_state` | `robstride_interfaces/msg/RobstrideState` | Per-joint id/enabled/run_state/fault_bits, published every `read()` cycle |
| `~/get_data_from_robstride` | `robstride_interfaces/srv/GetDataFromRobstride` | Read live telemetry for one motor |
| `~/set_data_to_robstride` | `robstride_interfaces/srv/SetDataToRobstride` | Set `kp`/`kd`, or write an arbitrary parameter index |
| `~/set_zero_robstride` | `robstride_interfaces/srv/SetZeroRobstride` | Set a motor's mechanical zero |
| `~/set_torque` | `std_srvs/srv/SetBool` | Enable/disable all configured motors |
| `~/reboot_robstride` | `robstride_interfaces/srv/RebootRobstride` | Clear a motor's latched fault. Unconditional: the caller decides which faults are safe to clear. The joint re-enables only after the hardware is reactivated (see §6). |

See [robstride_interfaces' README](https://github.com/kmj060703/robstride_interfaces/blob/main/README.md) for field descriptions.

## 9. Threading model

This plugin inherits `rclcpp::Node` for the services above, so it has to spin itself. That spinning runs on its own executor thread (`service_executor_`, started in `on_init`), independent of the RT cycle.

An earlier version called `rclcpp::spin_some()` at the end of `read()` instead. That ran every callback synchronously inside the RT cycle, and a callback that blocked for a few seconds froze `read()`/`write()` for that whole duration — long enough for the resulting CAN silence to trip this plugin's own staleness check and force-deactivate the hardware.

The tradeoff is that state shared between a callback and the RT cycle is now touched by two genuinely concurrent threads. Service callbacks must not write the RT cycle's plain buffers directly; they park values in an atomic that `write()` picks up (see `pending_kp_` and `torque_enabled_desired_`).

## 10. Bus bandwidth

A 1 Mbps CAN bus is shared by every motor on it. Each cycle needs `2 · N_motors` frames — one command out, one feedback frame back per motor — and an 8-byte extended frame takes roughly 130–190 µs on the wire depending on bit stuffing:

```
utilization ≈ 2 · N_motors · update_rate_hz · frame_time_s
```

Measure rather than trust the estimate:

```bash
canbusload can0@1000000 -r -t -b -e
```

If utilization is near saturation, or staleness errors appear under load, reduce `update_rate` or the number of motors per bus. Leave headroom: a bus at 100% has no margin for retries, parameter writes, or momentary noise, and starts intermittently missing the `error_timeout_ms` deadline.

## 11. License

Apache License 2.0.
