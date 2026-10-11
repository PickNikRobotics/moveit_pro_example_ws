# robstride_sdk

## 1. Introduction

Standalone C++17 SDK for the RobStride CAN actuator protocol (RS00–RS06). No ROS dependency — this package only talks to SocketCAN. It is consumed by [robstride_hardware_interface](https://github.com/kmj060703/robstride_hardware_interface).

Designed to sustain a **≥300 Hz** control loop against multiple motors on one bus:

- One non-blocking `SOCK_RAW`/`CAN_RAW` socket per interface, with 1 MiB send/receive buffers.
- A background read thread per bus, driven by `epoll` and draining frames with `recvmmsg()` in batches of up to 64 rather than one blocking `recv()` per frame.
- `SendFrames()` submits a whole batch through a single `sendmmsg()` call.
- Each motor's decoded state is a set of `std::atomic` fields written only by the read thread, so the control loop never locks or waits on CAN I/O.

## 2. Installation

Usually pulled in as a dependency of `robstride_hardware_interface`. To build it on its own:

```bash
cd ~/${WORKSPACE}/src
git clone https://github.com/kmj060703/RobstrideSDK.git

cd ~/${WORKSPACE}
colcon build --packages-select robstride_sdk
source install/setup.bash
```

Consume it from another `ament_cmake` package via the exported target list:

```cmake
find_package(robstride_sdk REQUIRED)
target_link_libraries(your_target ${robstride_sdk_TARGETS})
target_include_directories(your_target PRIVATE ${robstride_sdk_INCLUDE_DIRS})
```

## 3. Layout

```
include/robstride_sdk/
  robstride_protocol.hpp   Wire format: comm types, run modes, per-model limits, parameter
                           index table, fault bits, float<->uint16 quantization, extended-ID
                           pack/parse.
  robstride_motor.hpp      RobstrideMotor: frame encoders (motion command, enable/disable,
                           set-zero, parameter read/write) and the HandleFrame() decoder.
                           MotorState: the lock-free telemetry snapshot.
  can_bus.hpp              CanBus: one SocketCAN interface, epoll read thread, batched I/O.
src/
  can_bus.cpp              CanBus implementation.
```

## 4. Control modes

`RunModeFromString()` maps a name to the motor's `run_mode` parameter (`0x7005`):

| Name | Mode | Driven by |
|---|---|---|
| `motion` (or `mit`) | MIT-style | Type-1 frames carrying position, velocity, kp, kd and torque together |
| `position_pp` (or `pp`) | Profile position | `loc_ref` (`0x7016`) |
| `velocity` | Velocity | `spd_ref` (`0x700A`) |
| `current` | Current | `iq_ref` (`0x7006`) |
| `position_csp` (or `csp`) | Cyclic sync position | `loc_ref` (`0x7016`) |

`run_mode` lives in the motor's RAM and is lost on a power cycle, so it has to be re-sent whenever a motor comes back.

Parameter frames carry their communication type explicitly (`EncodeParamReadFloat`, `EncodeParamWriteFloat`, `EncodeParamWriteU8`, `EncodeParamWriteU32`), so a write can never be tagged as a read.

## 5. Actuator models and limits

`GetActuatorLimits(ActuatorType)` returns the quantization ranges (position, velocity, torque, kp, kd, current) used to encode and decode Type-1 and Type-2 frames, taken from the official RobStride manuals. Position range is ±4π for every model; the rest differ per model.

`ActuatorTypeFromString()` accepts `"00"`..`"06"`, bare `"0"`..`"6"` (xacro drops leading zeros), or `"RS00"`..`"RS06"`.

### Position wraparound

Position feedback loops within ±4π rather than reporting a continuous angle. A joint crossing that boundary would otherwise appear to jump by ~8π in a single frame with no physical motion, which a position-holding controller reads as an enormous error.

`UnwrapPosition()` reconstructs a continuous value by watching for an implausibly large frame-to-frame delta and folding it into an accumulated offset. Two things reset that offset to a fresh baseline:

- **A gap of more than 1 s since the last frame from that motor.** A power cycle or downed bus may have reset the motor's own position framing with nothing linking it to the pre-gap value, so carrying the old offset across would produce exactly the phantom jump this exists to prevent. Automatic.
- **`ResetPositionTracking()`**, called after a set-zero command, since recalibrating the mechanical zero relabels the raw position the same way a physical wrap would.

Joints that stay within roughly ±π of zero never reach the boundary. This matters for continuously-rotating joints.

## 6. Detecting a wedged bus

`SendFrames()` returns how many frames reached the kernel, and `last_send_errno()` reports why a short send stopped. `ENOBUFS` means the interface is not draining its transmit queue at all — typically bus-off on a controller that cannot restart itself. Without checking this, a bus that transmits nothing looks identical to every motor having gone silent.

## 7. CAN interface setup

RobStride motors run at 1 Mbps CAN 2.0 with extended (29-bit) IDs. Bring the interface up with [setup_can.sh](https://github.com/kmj060703/robstride_hardware_interface/blob/main/scripts/setup_can.sh), which applies the bitrate, `restart-ms` where supported, and a `txqueuelen` large enough that a batched send is not silently truncated.

## 8. License

Apache License 2.0. See [LICENSE](LICENSE).
