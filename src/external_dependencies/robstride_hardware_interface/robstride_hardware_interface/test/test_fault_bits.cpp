// Copyright 2026 PickNik Inc.
// SPDX-License-Identifier: Apache-2.0
//
// The fault word of a Type 0x15 frame, decoded from a frame captured on the
// reBot bench while joint3/joint4 were latched in a fault.

#include <gtest/gtest.h>

#include <cstdint>

#include "robstride_hardware_interface/robstride_hardware_interface.hpp"
#include "robstride_sdk/robstride_motor.hpp"
#include "robstride_sdk/robstride_protocol.hpp"

TEST(FaultBits, CapturedFrameDecodesLittleEndian)
{
  robstride_sdk::RobstrideMotor motor(4, 0xFD, robstride_sdk::ActuatorType::RS00);
  const uint8_t data[8] = {0x10, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  motor.HandleFrame(robstride_sdk::COMM_FAULT_FEEDBACK, 0, data, 8);

  const uint32_t bits = robstride_hardware_interface::FaultBits(motor.state());
  EXPECT_EQ(bits, 0x4010u);
  EXPECT_EQ(
    bits, robstride_sdk::FaultBit::PHASE_B_OVERCURRENT | robstride_sdk::FaultBit::STALL_OVERLOAD);
}
