// Copyright 2026 PickNik Inc.
// SPDX-License-Identifier: BSD-3-Clause

#include <fstream>
#include <iterator>
#include <memory>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>
#include <controller_manager/controller_manager.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/rclcpp.hpp>

namespace
{
class GravityConfigTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    rclcpp::init(0, nullptr);
  }
  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }
};
}  // namespace

TEST_F(GravityConfigTest, DefaultMockCanConfigureGravityInactiveButCannotActivateEffort)
{
  // This URDF is expanded from the shipped Xacro with hardware_interface=mock
  // by CMake. Never load the real driver or a physical transport in this test.
  std::ifstream urdf_stream(TEST_URDF);
  ASSERT_TRUE(urdf_stream.good());
  const std::string urdf((std::istreambuf_iterator<char>(urdf_stream)), std::istreambuf_iterator<char>());
  const auto config = YAML::LoadFile(std::string(PACKAGE_DIR) + "/config/control/rebot.ros2_control.yaml");
  const std::string name = "gravity_compensation_controller";
  const auto type = config["controller_manager"]["ros__parameters"][name]["type"].as<std::string>();
  const auto gravity = config[name]["ros__parameters"];
  const auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  auto options = controller_manager::get_cm_node_options();
  options.parameter_overrides({ rclcpp::Parameter("update_rate", 100) });
  const auto manager = std::make_shared<controller_manager::ControllerManager>(
      executor, urdf, true, "test_gravity_config_manager", "", options);
  ASSERT_TRUE(manager->is_resource_manager_initialized());
  const auto controller = manager->load_controller(name, type);
  ASSERT_NE(controller, nullptr);
  std::vector<rclcpp::Parameter> parameters;
  parameters.emplace_back("joints", gravity["joints"].as<std::vector<std::string>>());
  for (const auto* key : { "root_link", "tip_link" })
  {
    parameters.emplace_back(key, gravity[key].as<std::string>());
  }
  for (const auto* key : { "gains", "offsets", "gravity_vector", "payload_com" })
  {
    parameters.emplace_back(key, gravity[key].as<std::vector<double>>());
  }
  parameters.emplace_back("payload_mass", gravity["payload_mass"].as<double>());
  ASSERT_TRUE(controller->get_node()->set_parameters_atomically(parameters).successful);
  ASSERT_EQ(manager->configure_controller(name), controller_interface::return_type::OK);
  EXPECT_TRUE(controller->is_chainable());
  EXPECT_FALSE(controller->is_in_chained_mode());
  EXPECT_EQ(controller->get_lifecycle_id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  EXPECT_EQ(controller->command_interface_configuration().names,
            (std::vector<std::string>{ "joint1/effort", "joint2/effort", "joint3/effort", "joint4/effort",
                                       "joint5/effort", "joint6/effort" }));
  // Missing hardware effort must fail synchronously during preparation, not
  // silently gain command authority on the unchanged default mock model.
  EXPECT_EQ(manager->switch_controller({ name }, {}, controller_manager_msgs::srv::SwitchController::Request::STRICT,
                                       true, rclcpp::Duration::from_seconds(1.0)),
            controller_interface::return_type::ERROR);
  EXPECT_EQ(controller->get_lifecycle_id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  EXPECT_TRUE(manager->get_loaded_controllers().front().info.claimed_interfaces.empty());
}
