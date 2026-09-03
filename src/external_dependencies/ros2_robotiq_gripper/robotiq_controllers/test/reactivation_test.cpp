// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "robotiq_controllers/internal/reactivation.hpp"
#include "robotiq_controllers/robotiq_activation_controller.hpp"

namespace robotiq_controllers::internal
{
namespace
{
using testing::HasSubstr;

TEST(ReactivateGripper, SimulatedReactivationHasNoDeferredHardwareCommand)
{
  // GIVEN the retained simulation-only activation implementation
  // WHEN reactivation is requested
  const ReactivationResult result = simulated_reactivation();

  // THEN it succeeds synchronously without creating a pending hardware command.
  EXPECT_TRUE(result.success);
  EXPECT_THAT(result.message, HasSubstr("simulation"));
}

TEST(RobotiqActivationController, SimulatedControllerClaimsNoHardwareInterfaces)
{
  // GIVEN the activation controller retained for simulation configurations
  const robotiq_controllers::RobotiqActivationController controller;

  // WHEN its hardware command requirements are queried
  const controller_interface::InterfaceConfiguration configuration =
      controller.command_interface_configuration();

  // THEN no physical activation command or response interface is claimed
  EXPECT_EQ(configuration.type,
            controller_interface::interface_configuration_type::NONE);
  EXPECT_TRUE(configuration.names.empty());
}

TEST(RobotiqActivationController, ActivationWithoutSimulationAssertionFailsClosed)
{
  // GIVEN a controller loaded without an explicit simulation-only assertion.
  rclcpp::init(0, nullptr);
  robotiq_controllers::RobotiqActivationController controller;
  controller_interface::ControllerInterfaceParams controller_params;
  controller_params.controller_name = "hardware_mode_reactivation_test";
  controller_params.controller_manager_update_rate = 100;
  ASSERT_EQ(controller.init(controller_params), controller_interface::return_type::OK);

  // WHEN lifecycle activation is requested in the default hardware-capable mode.
  const controller_interface::CallbackReturn result =
      controller.on_activate(rclcpp_lifecycle::State{});

  // THEN the simulation no-op cannot become an active false-success service.
  EXPECT_EQ(result, controller_interface::CallbackReturn::ERROR);
  rclcpp::shutdown();
}

TEST(RobotiqActivationController, ExplicitSimulationAssertionAllowsActivation)
{
  // GIVEN a controller explicitly configured for simulation-only operation.
  rclcpp::init(0, nullptr);
  robotiq_controllers::RobotiqActivationController controller;
  controller_interface::ControllerInterfaceParams controller_params;
  controller_params.controller_name = "simulation_reactivation_test";
  controller_params.controller_manager_update_rate = 100;
  controller_params.node_options = rclcpp::NodeOptions().parameter_overrides(
      { rclcpp::Parameter("simulation_only", true) });
  ASSERT_EQ(controller.init(controller_params), controller_interface::return_type::OK);
  EXPECT_TRUE(controller.get_node()->describe_parameter("simulation_only").read_only);

  // WHEN lifecycle activation is requested with the assertion enabled.
  const controller_interface::CallbackReturn result =
      controller.on_activate(rclcpp_lifecycle::State{});

  // THEN the simulation compatibility service may activate.
  EXPECT_EQ(result, controller_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(controller.on_deactivate(rclcpp_lifecycle::State{}),
            controller_interface::CallbackReturn::SUCCESS);
  rclcpp::shutdown();
}

}  // namespace
}  // namespace robotiq_controllers::internal
