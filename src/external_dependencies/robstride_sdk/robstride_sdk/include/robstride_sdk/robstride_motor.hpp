// Copyright 2026 Minjo Kim
// SPDX-License-Identifier: Apache-2.0

#ifndef ROBSTRIDE_SDK__ROBSTRIDE_MOTOR_HPP_
#define ROBSTRIDE_SDK__ROBSTRIDE_MOTOR_HPP_

#include <linux/can.h>
#include <atomic>
#include <chrono>
#include <cstring>
#include <string>

#include "robstride_sdk/robstride_protocol.hpp"

namespace robstride_sdk
{

// Monotonic timestamp (ns) used to detect stale motors.
inline uint64_t NowNs()
{
  return static_cast<uint64_t>(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count());
}

// Lock-free snapshot of a motor's decoded state.
// Written by the CAN read thread, read by the control loop.
struct MotorState
{
  std::atomic<float> position{0.0f};
  std::atomic<float> velocity{0.0f};
  std::atomic<float> torque{0.0f};
  std::atomic<float> temperature{0.0f};
  std::atomic<uint8_t> run_state{static_cast<uint8_t>(RunState::RESET)};
  std::atomic<bool> fault_present{false};
  std::atomic<uint32_t> fault_bits{0};
  std::atomic<uint64_t> last_update_ns{0};
};

class RobstrideMotor
{
public:
  RobstrideMotor(uint8_t motor_id, uint8_t master_id, ActuatorType type)
  : motor_id_(motor_id), master_id_(master_id), type_(type),
    limits_(GetActuatorLimits(type))
  {
  }

  uint8_t motor_id() const {return motor_id_;}
  const ActuatorLimits & limits() const {return limits_;}
  MotorState & state() {return state_;}
  const MotorState & state() const {return state_;}

  // Call after a set-zero command: the new mechanical zero would otherwise
  // look like a wraparound to UnwrapPosition().
  void ResetPositionTracking()
  {
    position_initialized_.store(false, std::memory_order_relaxed);
    position_offset_.store(0.0f, std::memory_order_relaxed);
  }

  can_frame EncodeMotionCommand(float position, float velocity, float kp, float kd, float torque)
  const
  {
    const uint16_t torque_u = FloatToUint16(torque, -limits_.torque_max, limits_.torque_max);
    can_frame frame{};
    frame.can_id =
      BuildExtendedId(COMM_MOTION_CONTROL, torque_u, motor_id_) | CAN_EFF_FLAG;
    frame.can_dlc = 8;
    const uint16_t pos_u = FloatToUint16(position, -limits_.position_max, limits_.position_max);
    const uint16_t vel_u = FloatToUint16(velocity, -limits_.velocity_max, limits_.velocity_max);
    const uint16_t kp_u = FloatToUint16(kp, 0.0f, static_cast<float>(limits_.kp_max));
    const uint16_t kd_u = FloatToUint16(kd, 0.0f, static_cast<float>(limits_.kd_max));
    frame.data[0] = static_cast<uint8_t>(pos_u >> 8);
    frame.data[1] = static_cast<uint8_t>(pos_u & 0xFF);
    frame.data[2] = static_cast<uint8_t>(vel_u >> 8);
    frame.data[3] = static_cast<uint8_t>(vel_u & 0xFF);
    frame.data[4] = static_cast<uint8_t>(kp_u >> 8);
    frame.data[5] = static_cast<uint8_t>(kp_u & 0xFF);
    frame.data[6] = static_cast<uint8_t>(kd_u >> 8);
    frame.data[7] = static_cast<uint8_t>(kd_u & 0xFF);
    return frame;
  }

  can_frame EncodeEnable() const {return EncodeSimple(COMM_MOTOR_ENABLE, 0);}

  can_frame EncodeDisable(bool clear_fault) const
  {
    return EncodeSimple(COMM_MOTOR_STOP, clear_fault ? 1 : 0);
  }

  can_frame EncodeSetZero() const {return EncodeSimple(COMM_SET_MECH_ZERO, 1);}

  can_frame EncodeParamReadFloat(uint16_t index) const
  {
    return EncodeParamHeader(COMM_PARAM_READ, index);
  }

  can_frame EncodeParamWriteFloat(uint16_t index, float value) const
  {
    can_frame frame = EncodeParamHeader(COMM_PARAM_WRITE, index);
    std::memcpy(&frame.data[4], &value, sizeof(value));
    return frame;
  }

  can_frame EncodeParamWriteU8(uint16_t index, uint8_t value) const
  {
    can_frame frame = EncodeParamHeader(COMM_PARAM_WRITE, index);
    frame.data[4] = value;
    return frame;
  }

  can_frame EncodeParamWriteU32(uint16_t index, uint32_t value) const
  {
    can_frame frame = EncodeParamHeader(COMM_PARAM_WRITE, index);
    std::memcpy(&frame.data[4], &value, sizeof(value));
    return frame;
  }

  // Decode a received frame and update the state snapshot.
  // Called only from the CAN read thread.
  void HandleFrame(uint8_t comm_type, uint16_t data_area_2, const uint8_t * data, uint8_t dlc)
  {
    const uint64_t now_ns = NowNs();
    const uint64_t prev_update_ns = state_.last_update_ns.load(std::memory_order_relaxed);
    state_.last_update_ns.store(now_ns, std::memory_order_relaxed);

    // A long gap (power cycle, bus down) invalidates the unwrap offset:
    // start a fresh baseline instead of carrying it across.
    constexpr uint64_t kStaleGapNs = 1'000'000'000ull;
    if (prev_update_ns != 0 && now_ns - prev_update_ns > kStaleGapNs) {
      ResetPositionTracking();
    }

    if (comm_type == COMM_MOTOR_FEEDBACK && dlc >= 8) {
      const uint16_t pos_u = (static_cast<uint16_t>(data[0]) << 8) | data[1];
      const uint16_t vel_u = (static_cast<uint16_t>(data[2]) << 8) | data[3];
      const uint16_t torque_u = (static_cast<uint16_t>(data[4]) << 8) | data[5];
      const uint16_t temp_u = (static_cast<uint16_t>(data[6]) << 8) | data[7];
      state_.position.store(
        UnwrapPosition(
          Uint16ToFloat(pos_u, -limits_.position_max, limits_.position_max)),
        std::memory_order_relaxed);
      state_.velocity.store(
        Uint16ToFloat(vel_u, -limits_.velocity_max, limits_.velocity_max),
        std::memory_order_relaxed);
      state_.torque.store(
        Uint16ToFloat(torque_u, -limits_.torque_max, limits_.torque_max),
        std::memory_order_relaxed);
      state_.temperature.store(static_cast<float>(temp_u) * 0.1f, std::memory_order_relaxed);
      const uint8_t fault6 = static_cast<uint8_t>((data_area_2 >> 8) & 0x3F);
      state_.fault_present.store(fault6 != 0, std::memory_order_relaxed);
      const uint8_t mode = static_cast<uint8_t>((data_area_2 >> 14) & 0x03);
      state_.run_state.store(mode, std::memory_order_relaxed);
    } else if (comm_type == COMM_FAULT_FEEDBACK && dlc >= 4) {
      const uint32_t fault = (static_cast<uint32_t>(data[0]) << 24) |
        (static_cast<uint32_t>(data[1]) << 16) |
        (static_cast<uint32_t>(data[2]) << 8) | data[3];
      state_.fault_bits.store(fault, std::memory_order_relaxed);
      state_.fault_present.store(fault != 0, std::memory_order_relaxed);
    }
  }

private:
  can_frame EncodeSimple(uint8_t comm_type, uint8_t byte0) const
  {
    can_frame frame{};
    frame.can_id =
      BuildExtendedId(comm_type, static_cast<uint16_t>(master_id_) << 8, motor_id_) |
      CAN_EFF_FLAG;
    frame.can_dlc = 8;
    std::memset(frame.data, 0, 8);
    frame.data[0] = byte0;
    return frame;
  }

  // Common header for param read/write frames. comm_type is always
  // explicit so a write can never be tagged as a read.
  can_frame EncodeParamHeader(uint8_t comm_type, uint16_t index) const
  {
    can_frame frame{};
    frame.can_id =
      BuildExtendedId(comm_type, static_cast<uint16_t>(master_id_) << 8, motor_id_) |
      CAN_EFF_FLAG;
    frame.can_dlc = 8;
    std::memset(frame.data, 0, 8);
    frame.data[0] = static_cast<uint8_t>(index & 0xFF);
    frame.data[1] = static_cast<uint8_t>(index >> 8);
    return frame;
  }

  // Position feedback wraps at +/-position_max (+/-4pi). Rebuild a
  // continuous value by folding each wrap into an accumulated offset.
  float UnwrapPosition(float raw_position)
  {
    if (!position_initialized_.exchange(true, std::memory_order_relaxed)) {
      last_raw_position_.store(raw_position, std::memory_order_relaxed);
      return raw_position;
    }
    const float last_raw = last_raw_position_.load(std::memory_order_relaxed);
    const float delta = raw_position - last_raw;
    const float wrap_span = static_cast<float>(2.0 * limits_.position_max);
    // Only the CAN read thread does this read-modify-write, so a plain
    // load-then-store is safe.
    float offset = position_offset_.load(std::memory_order_relaxed);
    if (delta > static_cast<float>(limits_.position_max)) {
      offset -= wrap_span;
      position_offset_.store(offset, std::memory_order_relaxed);
    } else if (delta < -static_cast<float>(limits_.position_max)) {
      offset += wrap_span;
      position_offset_.store(offset, std::memory_order_relaxed);
    }
    last_raw_position_.store(raw_position, std::memory_order_relaxed);
    return raw_position + offset;
  }

  uint8_t motor_id_;
  uint8_t master_id_;
  ActuatorType type_;
  ActuatorLimits limits_;
  MotorState state_;
  std::atomic<bool> position_initialized_{false};
  std::atomic<float> last_raw_position_{0.0f};
  std::atomic<float> position_offset_{0.0f};
};

}  // namespace robstride_sdk

#endif  // ROBSTRIDE_SDK__ROBSTRIDE_MOTOR_HPP_
