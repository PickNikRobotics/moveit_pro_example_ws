# robstride_interfaces

## 1. Introduction

Message and service definitions used by [robstride_hardware_interface](https://github.com/kmj060703/robstride_hardware_interface).

## 2. Installation

Usually pulled in as a dependency of `robstride_hardware_interface`. To build it on its own:

```bash
cd ~/${WORKSPACE}/src
git clone https://github.com/kmj060703/robstride_interfaces.git

cd ~/${WORKSPACE}
colcon build --packages-select robstride_interfaces
source install/setup.bash
```

## 3. Messages

### `RobstrideState.msg`

Published on `~/robstride_state` every `read()` cycle.

```
std_msgs/Header header

int32 comm_state
int32[] id
bool[] enabled
int32[] run_state
uint32[] fault_bits

int32 COMM_STATE_OK = 0
int32 COMM_STATE_BUS_NOT_OPEN = -1
int32 COMM_STATE_SEND_FAIL = -2
```

- The four arrays are parallel, one entry per configured joint, in the same order as `ros2_control`'s joint list.
- `run_state`: `0` reset, `1` calibration, `2` motor (torqued).
- `fault_bits`: bitfield; see `robstride_sdk::FaultBit` for bit meanings.

## 4. Services

All served by `robstride_hardware_interface` under its own node namespace.

### `GetDataFromRobstride.srv` — `~/get_data_from_robstride`

```
std_msgs/Header header
uint8 id
string item_name
float64 timeout_sec
---
float32 item_data
bool result
```

Reads one value from the already-decoded lock-free state: `position`, `velocity`, `torque`, `temperature`, `run_state` or `fault_bits`. There is no round trip to the motor, so `timeout_sec` is accepted but unused.

### `SetDataToRobstride.srv` — `~/set_data_to_robstride`

```
std_msgs/Header header
uint8 id
string item_name
float32 item_data
---
bool result
```

`item_name` of `kp` or `kd` updates that joint's gains, applied by the next `write()` cycle. Any other name is treated as a RobStride parameter index (hex or decimal) and sends a parameter write frame.

### `SetZeroRobstride.srv` — `~/set_zero_robstride`

```
std_msgs/Header header
uint8 id
---
bool result
```

Sets one motor's mechanical zero at its current position.

### `RebootRobstride.srv` — `~/reboot_robstride`

```
std_msgs/Header header
uint8 id
---
bool result
```

Sends a Stop frame with the clear-fault flag set, telling the motor to drop a latched fault rather than stay disabled. `write()` keeps resending Enable to any joint that is not confirmed torqued, so the motor re-enables on its own once it accepts the clear.

Not a literal MCU reboot — the RobStride protocol has no such command. A motor whose MCU has genuinely hung needs a power cycle, and shows up as feedback staleness rather than a fault report.

**Unconditional by design.** The frame is always sent, whatever `fault_bits` currently shows. Whether a given fault is safe to clear and retry is a policy call for the caller: read `fault_bits` first and decide before calling.

## 5. License

Apache License 2.0.
