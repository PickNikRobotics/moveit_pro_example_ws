// Copyright 2026 PickNik Inc.
// SPDX-License-Identifier: Apache-2.0
//
// What a clean deactivation does to torque. The default disables every motor,
// which drops a gravity-loaded arm; hold_torque_on_deactivate must leave the
// motors holding instead. Both paths are driven through the real lifecycle
// against a fake motor on a virtual CAN interface, so nothing here needs an
// arm, a CAN adapter, or a power supply.
//
// Needs a vcan interface (NET_ADMIN, which the MoveIt Pro dev container has):
//   sudo modprobe vcan
//   sudo ip link add dev vcan0 type vcan && sudo ip link set vcan0 up
// Without one the test skips rather than failing, so it cannot be mistaken
// for coverage it did not provide.

#include <algorithm>
#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include <net/if.h>

#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_component_interface_params.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "robstride_hardware_interface/robstride_hardware_interface.hpp"
#include "robstride_sdk/robstride_protocol.hpp"

#include "fake_robstride_motor.hpp"

namespace
{

constexpr uint8_t kMotorId = 1;
constexpr uint8_t kMasterId = 0xFD;

const char * TestInterface()
{
  const char * from_env = std::getenv("ROBSTRIDE_TEST_CAN_IFACE");
  return from_env != nullptr ? from_env : "vcan0";
}

hardware_interface::HardwareInfo MakeInfo(bool hold_torque_on_deactivate)
{
  hardware_interface::HardwareInfo info;
  info.name = "robstride_test";
  info.type = "system";
  info.hardware_plugin_name = "robstride_hardware_interface/RobstrideHardware";
  info.hardware_parameters["master_id"] = std::to_string(kMasterId);
  // Short enough to keep a failing activation from stalling the suite.
  info.hardware_parameters["error_timeout_ms"] = "1000";
  // 0 keeps the motor's own watchdog out of the picture: this test is about
  // what on_deactivate() sends, not what the motor does afterwards.
  info.hardware_parameters["motor_can_timeout_ms"] = "0";
  info.hardware_parameters["hold_torque_on_deactivate"] =
    hold_torque_on_deactivate ? "true" : "false";

  hardware_interface::ComponentInfo joint;
  joint.name = "test_joint";
  joint.type = "joint";
  joint.parameters["id"] = std::to_string(kMotorId);
  hardware_interface::InterfaceInfo position_command;
  position_command.name = "position";
  joint.command_interfaces.push_back(position_command);
  for (const char * name : {"position", "velocity", "effort"}) {
    hardware_interface::InterfaceInfo state;
    state.name = name;
    joint.state_interfaces.push_back(state);
  }
  info.joints.push_back(joint);

  hardware_interface::ComponentInfo gpio;
  gpio.name = "test_motor";
  gpio.type = "gpio";
  gpio.parameters["type"] = "robstride";
  gpio.parameters["ID"] = std::to_string(kMotorId);
  gpio.parameters["actuator_type"] = "RS00";
  gpio.parameters["can_interface"] = TestInterface();
  gpio.parameters["control_mode"] = "motion";
  gpio.parameters["kp"] = "20.0";
  gpio.parameters["kd"] = "1.0";
  info.gpios.push_back(gpio);

  return info;
}

/// Stop frames the motor saw after the last Enable. The activation sequence
/// sends Stop first as a position probe, so only what follows the Enable can
/// be the deactivation disabling torque.
size_t StopFramesAfterEnable(const std::vector<uint8_t> & received)
{
  const auto last_enable = std::find(
    received.rbegin(), received.rend(),
    static_cast<uint8_t>(robstride_sdk::COMM_MOTOR_ENABLE));
  if (last_enable == received.rend()) {
    return 0;
  }
  return static_cast<size_t>(
    std::count(
      received.rbegin(), last_enable,
      static_cast<uint8_t>(robstride_sdk::COMM_MOTOR_STOP)));
}

class DeactivateTorqueTest : public testing::Test
{
protected:
  void SetUp() override
  {
    if (::if_nametoindex(TestInterface()) == 0) {
      GTEST_SKIP() << "no '" << TestInterface() <<
        "' interface; see the header comment for how to create one";
    }
  }

  /// Run one full activate/deactivate cycle and report what the motor saw.
  std::vector<uint8_t> RunCycle(bool hold_torque_on_deactivate)
  {
    FakeRobstrideMotor motor(TestInterface(), kMotorId, kMasterId);
    if (!motor.ok()) {
      ADD_FAILURE() << "could not open " << TestInterface();
      return {};
    }

    robstride_hardware_interface::RobstrideHardware hardware;
    hardware_interface::HardwareComponentInterfaceParams params;
    params.hardware_info = MakeInfo(hold_torque_on_deactivate);
    if (hardware.on_init(params) != hardware_interface::CallbackReturn::SUCCESS) {
      ADD_FAILURE() << "on_init failed";
      return {};
    }

    const rclcpp_lifecycle::State unconfigured;
    if (hardware.on_activate(unconfigured) != hardware_interface::CallbackReturn::SUCCESS) {
      ADD_FAILURE() << "on_activate failed; the fake motor did not answer the probe";
      return {};
    }
    // The component is active and the motor reports MOTOR; whatever Stop
    // frames arrive from here on came from the deactivation.
    hardware.on_deactivate(unconfigured);
    // The last frames are in flight on the fake's socket, not yet read.
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    return motor.received();
  }
};

TEST_F(DeactivateTorqueTest, DisablesTorqueByDefault)
{
  const auto received = RunCycle(false);
  ASSERT_FALSE(received.empty());
  EXPECT_GT(StopFramesAfterEnable(received), 0u)
    << "the default deactivation must disable torque";
}

TEST_F(DeactivateTorqueTest, HoldsTorqueWhenAsked)
{
  const auto received = RunCycle(true);
  ASSERT_FALSE(received.empty());
  // A Stop frame here is torque released, and a raised arm on the floor.
  EXPECT_EQ(StopFramesAfterEnable(received), 0u)
    << "hold_torque_on_deactivate must not disable torque";
}

}  // namespace

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
