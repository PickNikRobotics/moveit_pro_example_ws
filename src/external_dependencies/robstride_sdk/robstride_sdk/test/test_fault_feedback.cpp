// Copyright 2026 Griswald Brooks
// SPDX-License-Identifier: Apache-2.0

#include <cstring>
#include <iostream>
#include <stdexcept>
#include <string>

#include "robstride_sdk/can_bus.hpp"

namespace robstride_sdk
{
// Exercise the receive path without opening a SocketCAN interface.
struct CanBusTestAccess
{
  static void Dispatch(CanBus & bus, const can_frame & frame)
  {
    bus.DispatchFrame(frame);
  }
};
}  // namespace robstride_sdk

namespace
{
using namespace robstride_sdk;

void Check(bool condition, const char * message)
{
  // Keep checks active in Release builds too.
  if (!condition) {
    throw std::runtime_error(message);
  }
}

void TestRouting(bool register_zero)
{
  const ParsedId parsed = ParseExtendedId(0x15000200);
  Check(parsed.comm_type == COMM_FAULT_FEEDBACK, "incorrect communication type");
  Check(parsed.data_area_2 == 2 && parsed.dest_id == 0, "incorrect ID fields");
  RobstrideMotor motor_zero(0, 0, ActuatorType::RS06);
  RobstrideMotor motor_two(2, 0, ActuatorType::RS06);
  CanBus bus("unused");
  if (register_zero) {
    bus.RegisterMotor(&motor_zero);
  }
  bus.RegisterMotor(&motor_two);
  can_frame frame{};
  frame.can_id = 0x15000200 | CAN_EFF_FLAG;
  frame.can_dlc = 8;
  frame.data[0] = 0x20;
  frame.data[1] = 0x40;
  CanBusTestAccess::Dispatch(bus, frame);
  Check(motor_two.state().last_update_ns.load() != 0, "motor 2 did not receive frame");
  Check(motor_two.state().fault_present.load(), "motor 2 fault not received");
  Check(motor_zero.state().last_update_ns.load() == 0, "frame routed to motor 0");
}

void TestFaultBits()
{
  RobstrideMotor motor(2, 0, ActuatorType::RS06);
  const uint8_t data[8] = {0x20, 0x40, 0, 0, 0, 0, 0, 0};
  motor.HandleFrame(COMM_FAULT_FEEDBACK, 2, data, 8);
  const uint32_t bits = motor.state().fault_bits.load();
  Check(bits == 0x4020, "fault word is not little-endian");
  Check((bits & FaultBit::STALL_OVERLOAD) != 0, "missing stall overload bit");
  Check((bits & FaultBit::PHASE_C_OVERCURRENT) != 0, "missing phase C overcurrent bit");
  Check(motor.state().fault_present.load(), "missing fault_present");
}

void TestOvertemperature()
{
  RobstrideMotor motor(2, 0, ActuatorType::RS00);
  const uint8_t data[8] = {1, 0, 0, 0, 0, 0, 0, 0};
  motor.HandleFrame(COMM_FAULT_FEEDBACK, 2, data, 8);
  Check(motor.state().fault_bits.load() == FaultBit::OVERTEMPERATURE, "missing bit 0");
  Check(motor.state().fault_present.load(), "missing fault_present for bit 0");
}

void TestWarningBits()
{
  RobstrideMotor motor(2, 0, ActuatorType::RS06);
  const uint8_t data[8] = {0, 0, 0, 0, 0x78, 0x56, 0x34, 0x92};
  motor.HandleFrame(COMM_FAULT_FEEDBACK, 2, data, 8);
  const MotorState & state = motor.state();
  Check(state.warning_bits.load() == 0x92345678u, "warning word is not little-endian");
  Check(state.fault_bits.load() == 0, "warning word changed fault bits");
  Check(!state.fault_present.load(), "warning alone set fault_present");
  const uint8_t clear[8] = {};
  motor.HandleFrame(COMM_FAULT_FEEDBACK, 2, clear, 8);
  Check(state.warning_bits.load() == 0, "warning word did not clear");
}

void TestShortFrame()
{
  RobstrideMotor motor(2, 0, ActuatorType::RS06);
  motor.state().warning_bits.store(0x12345678);
  const uint8_t data[4] = {1, 0, 0, 0};
  motor.HandleFrame(COMM_FAULT_FEEDBACK, 2, data, 4);
  Check(motor.state().fault_bits.load() == 1, "four-byte fault word not decoded");
  Check(motor.state().fault_present.load(), "four-byte fault not present");
  Check(motor.state().warning_bits.load() == 0x12345678, "short frame changed warnings");
  motor.HandleFrame(COMM_FAULT_FEEDBACK, 2, data, 3);
  Check(motor.state().fault_bits.load() == 1, "truncated fault changed bits");
}

void TestMotorFeedback()
{
  RobstrideMotor motor(2, 0, ActuatorType::RS06);
  CanBus bus("unused");
  bus.RegisterMotor(&motor);
  motor.state().fault_bits.store(0x4020);
  motor.state().warning_bits.store(1);
  can_frame frame{};
  frame.can_id = BuildExtendedId(COMM_MOTOR_FEEDBACK, (2u << 14) | 2, 0) | CAN_EFF_FLAG;
  frame.can_dlc = 8;
  const uint8_t data[8] = {128, 0, 128, 0, 128, 0, 0x01, 0x90};
  std::memcpy(frame.data, data, 8);
  CanBusTestAccess::Dispatch(bus, frame);
  Check(motor.state().temperature.load() == 40.0f, "Type 2 payload byte order changed");
  Check(motor.state().run_state.load() == 2, "Type 2 run state changed");
  Check(!motor.state().fault_present.load(), "Type 2 fault flags changed");
  Check(motor.state().fault_bits.load() == 0x4020, "Type 2 cleared detailed fault bits");
  Check(motor.state().warning_bits.load() == 1, "Type 2 cleared detailed warning bits");
}

void TestTemperatureFreshness()
{
  RobstrideMotor motor(2, 0, ActuatorType::RS00);
  const auto & state = motor.state();
  Check(state.last_feedback_ns.load() == 0, "default temperature looks measured");
  const uint8_t data[8] = {128, 0, 128, 0, 128, 0, 0x01, 0x90};
  motor.HandleFrame(COMM_MOTOR_FEEDBACK, 2, data, 7);
  motor.HandleFrame(COMM_FAULT_FEEDBACK, 2, data, 8);
  Check(state.last_feedback_ns.load() == 0, "non-temperature frame invented a sample");
  motor.HandleFrame(COMM_MOTOR_FEEDBACK, 2, data, 8);
  const auto first = state.last_feedback_ns.load(std::memory_order_acquire);
  Check(first != 0 && state.temperature.load() == 40.0f, "missing valid sample");
  motor.HandleFrame(COMM_FAULT_FEEDBACK, 2, data, 8);
  motor.HandleFrame(COMM_MOTOR_FEEDBACK, 2, data, 0);
  motor.HandleFrame(COMM_PARAM_READ, 2, data, 8);
  Check(state.last_feedback_ns.load() == first, "unrelated frame refreshed temperature");
  motor.HandleFrame(COMM_MOTOR_FEEDBACK, 2, data, 8);
  Check(state.last_feedback_ns.load() > first, "new valid feedback did not refresh age");
}
}  // namespace

int main(int argc, char ** argv)
{
  try {
    Check(argc == 2, "expected test case argument");
    const std::string name = argv[1];
    if (name == "routing") {
      TestRouting(true);
    } else if (name == "routing_without_motor_zero") {
      TestRouting(false);
    } else if (name == "fault_bits") {
      TestFaultBits();
    } else if (name == "overtemperature") {
      TestOvertemperature();
    } else if (name == "warning_bits") {
      TestWarningBits();
    } else if (name == "short_frame") {
      TestShortFrame();
    } else if (name == "motor_feedback") {
      TestMotorFeedback();
    } else if (name == "temperature_freshness") {
      TestTemperatureFreshness();
    } else {
      throw std::runtime_error("unknown test case");
    }
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
  return 0;
}
