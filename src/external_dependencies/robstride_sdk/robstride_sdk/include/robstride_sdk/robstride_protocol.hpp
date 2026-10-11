// Copyright 2026 Minjo Kim
// SPDX-License-Identifier: Apache-2.0

#ifndef ROBSTRIDE_SDK__ROBSTRIDE_PROTOCOL_HPP_
#define ROBSTRIDE_SDK__ROBSTRIDE_PROTOCOL_HPP_

#include <cstdint>
#include <cmath>
#include <map>
#include <stdexcept>
#include <string>

namespace robstride_sdk
{

// Communication types, bits 24~28 of the 29-bit extended CAN ID.
enum CommType : uint8_t
{
  COMM_GET_DEVICE_ID = 0x00,
  COMM_MOTION_CONTROL = 0x01,
  COMM_MOTOR_FEEDBACK = 0x02,
  COMM_MOTOR_ENABLE = 0x03,
  COMM_MOTOR_STOP = 0x04,
  COMM_SET_MECH_ZERO = 0x06,
  COMM_SET_CAN_ID = 0x07,
  COMM_PARAM_READ = 0x11,
  COMM_PARAM_WRITE = 0x12,
  COMM_FAULT_FEEDBACK = 0x15,
  COMM_DATA_SAVE = 0x16,
  COMM_BAUD_RATE_MODIFY = 0x17,
  COMM_ACTIVE_REPORT = 0x18,
  COMM_PROTOCOL_MODIFY = 0x19,
};

// Motor run_mode values (index 0x7005).
enum RunMode : uint8_t
{
  RUN_MODE_MOTION = 0,
  RUN_MODE_POSITION_PP = 1,
  RUN_MODE_VELOCITY = 2,
  RUN_MODE_CURRENT = 3,
  RUN_MODE_SET_ZERO = 4,
  RUN_MODE_POSITION_CSP = 5,
};

enum class ActuatorType : uint8_t
{
  RS00 = 0,
  RS01 = 1,
  RS02 = 2,
  RS03 = 3,
  RS04 = 4,
  RS05 = 5,
  RS06 = 6,
};

// Per-model quantization ranges for Type-1 and Type-2 frame fields.
struct ActuatorLimits
{
  double position_max;  // rad, symmetric range is [-position_max, +position_max]
  double velocity_max;  // rad/s
  double torque_max;    // N*m
  double kp_max;
  double kd_max;
  double current_max;   // A, from the iq_ref / limit_cur parameter range
};

inline const std::map<ActuatorType, ActuatorLimits> & GetActuatorLimitsTable()
{
  static const std::map<ActuatorType, ActuatorLimits> table = {
    {ActuatorType::RS00, {4.0 * M_PI, 33.0, 14.0, 500.0, 5.0, 16.0}},
    {ActuatorType::RS01, {4.0 * M_PI, 44.0, 17.0, 500.0, 5.0, 23.0}},
    {ActuatorType::RS02, {4.0 * M_PI, 44.0, 17.0, 500.0, 5.0, 23.0}},
    {ActuatorType::RS03, {4.0 * M_PI, 20.0, 60.0, 5000.0, 100.0, 43.0}},
    {ActuatorType::RS04, {4.0 * M_PI, 15.0, 120.0, 5000.0, 100.0, 90.0}},
    {ActuatorType::RS05, {4.0 * M_PI, 33.0, 17.0, 500.0, 5.0, 23.0}},
    {ActuatorType::RS06, {4.0 * M_PI, 50.0, 36.0, 5000.0, 100.0, 57.0}},
  };
  return table;
}

inline const ActuatorLimits & GetActuatorLimits(ActuatorType type)
{
  return GetActuatorLimitsTable().at(type);
}

inline ActuatorType ActuatorTypeFromString(const std::string & name)
{
  // Accepts "00".."06", "0".."6" or "RS00".."RS06".
  if (name == "00" || name == "0" || name == "RS00") {return ActuatorType::RS00;}
  if (name == "01" || name == "1" || name == "RS01") {return ActuatorType::RS01;}
  if (name == "02" || name == "2" || name == "RS02") {return ActuatorType::RS02;}
  if (name == "03" || name == "3" || name == "RS03") {return ActuatorType::RS03;}
  if (name == "04" || name == "4" || name == "RS04") {return ActuatorType::RS04;}
  if (name == "05" || name == "5" || name == "RS05") {return ActuatorType::RS05;}
  if (name == "06" || name == "6" || name == "RS06") {return ActuatorType::RS06;}
  throw std::invalid_argument("Unknown RobStride actuator_type: " + name);
}

inline RunMode RunModeFromString(const std::string & name)
{
  // "motion" uses Type-1 frames; the other modes set run_mode (0x7005)
  // and are driven by loc_ref/spd_ref/iq_ref writes.
  if (name.empty() || name == "motion" || name == "mit" || name == "0") {
    return RunMode::RUN_MODE_MOTION;
  }
  if (name == "position_pp" || name == "pp" || name == "1") {return RunMode::RUN_MODE_POSITION_PP;}
  if (name == "velocity" || name == "2") {return RunMode::RUN_MODE_VELOCITY;}
  if (name == "current" || name == "3") {return RunMode::RUN_MODE_CURRENT;}
  if (name == "position_csp" || name == "csp" || name == "5") {
    return RunMode::RUN_MODE_POSITION_CSP;
  }
  throw std::invalid_argument("Unknown RobStride control_mode: " + name);
}

// Parameter index list, common across motor models.
namespace ParamIndex
{
constexpr uint16_t RUN_MODE = 0x7005;
constexpr uint16_t IQ_REF = 0x7006;
constexpr uint16_t SPD_REF = 0x700A;
constexpr uint16_t LIMIT_TORQUE = 0x700B;
constexpr uint16_t CUR_KP = 0x7010;
constexpr uint16_t CUR_KI = 0x7011;
constexpr uint16_t CUR_FILT_GAIN = 0x7014;
constexpr uint16_t LOC_REF = 0x7016;
constexpr uint16_t LIMIT_SPD = 0x7017;
constexpr uint16_t LIMIT_CUR = 0x7018;
constexpr uint16_t MECH_POS = 0x7019;
constexpr uint16_t IQF = 0x701A;
constexpr uint16_t MECH_VEL = 0x701B;
constexpr uint16_t VBUS = 0x701C;
constexpr uint16_t LOC_KP = 0x701E;
constexpr uint16_t SPD_KP = 0x701F;
constexpr uint16_t SPD_KI = 0x7020;
constexpr uint16_t SPD_FILT_GAIN = 0x7021;
constexpr uint16_t ACC_RAD = 0x7022;
constexpr uint16_t VEL_MAX = 0x7024;
constexpr uint16_t ACC_SET = 0x7025;
constexpr uint16_t EPSCAN_TIME = 0x7026;
constexpr uint16_t CAN_TIMEOUT = 0x7028;
constexpr uint16_t ZERO_STA = 0x7029;
constexpr uint16_t DAMPER = 0x702A;
constexpr uint16_t ADD_OFFSET = 0x702B;
}  // namespace ParamIndex

// Fault bits from the Type 0x15 fault feedback frame.
namespace FaultBit
{
constexpr uint32_t OVERTEMPERATURE = 1u << 0;
constexpr uint32_t DRIVER_CHIP = 1u << 1;
constexpr uint32_t UNDERVOLTAGE = 1u << 2;
constexpr uint32_t OVERVOLTAGE = 1u << 3;
constexpr uint32_t PHASE_B_OVERCURRENT = 1u << 4;
constexpr uint32_t PHASE_C_OVERCURRENT = 1u << 5;
constexpr uint32_t ENCODER_UNCALIBRATED = 1u << 7;
constexpr uint32_t HARDWARE_ID = 1u << 8;
constexpr uint32_t POSITION_INIT = 1u << 9;
constexpr uint32_t STALL_OVERLOAD = 1u << 14;
constexpr uint32_t PHASE_A_OVERCURRENT = 1u << 16;
}  // namespace FaultBit

enum class RunState : uint8_t
{
  RESET = 0,
  CALIBRATION = 1,
  MOTOR = 2,
};

// Quantize a float in [x_min, x_max] to an unsigned integer.
inline uint16_t FloatToUint16(float x, float x_min, float x_max)
{
  if (x < x_min) {x = x_min;}
  if (x > x_max) {x = x_max;}
  const float span = x_max - x_min;
  return static_cast<uint16_t>(((x - x_min) * 65535.0f) / span);
}

// Inverse of FloatToUint16.
inline float Uint16ToFloat(uint16_t x, float x_min, float x_max)
{
  const float span = x_max - x_min;
  return (static_cast<float>(x) * span) / 65535.0f + x_min;
}

// Build the 29-bit extended CAN id: [comm_type:5][data_area_2:16][dest_id:8].
inline uint32_t BuildExtendedId(uint8_t comm_type, uint16_t data_area_2, uint8_t dest_id)
{
  return (static_cast<uint32_t>(comm_type) << 24) |
         (static_cast<uint32_t>(data_area_2) << 8) |
         static_cast<uint32_t>(dest_id);
}

struct ParsedId
{
  uint8_t comm_type;
  uint16_t data_area_2;
  uint8_t dest_id;
};

inline ParsedId ParseExtendedId(uint32_t can_id)
{
  ParsedId parsed;
  parsed.comm_type = static_cast<uint8_t>((can_id >> 24) & 0x1F);
  parsed.data_area_2 = static_cast<uint16_t>((can_id >> 8) & 0xFFFF);
  parsed.dest_id = static_cast<uint8_t>(can_id & 0xFF);
  return parsed;
}

}  // namespace robstride_sdk

#endif  // ROBSTRIDE_SDK__ROBSTRIDE_PROTOCOL_HPP_
