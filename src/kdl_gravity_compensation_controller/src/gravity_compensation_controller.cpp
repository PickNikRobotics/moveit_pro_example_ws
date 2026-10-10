// Copyright 2026 PickNik Inc.
// SPDX-License-Identifier: BSD-3-Clause

#include "kdl_gravity_compensation_controller/gravity_compensation_controller.hpp"

#include <exception>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace kdl_gravity_compensation_controller
{

using controller_interface::CallbackReturn;
using controller_interface::interface_configuration_type;
using controller_interface::InterfaceConfiguration;

CallbackReturn GravityCompensationController::on_init()
{
  auto_declare<std::vector<std::string>>("joints", {});
  auto_declare<std::string>("root_link", "base_link");
  auto_declare<std::string>("tip_link", "");
  auto_declare<std::vector<double>>("gravity_vector", { 0.0, 0.0, -9.81 });
  auto_declare<double>("payload_mass", 0.0);
  auto_declare<std::vector<double>>("payload_com", { 0.0, 0.0, 0.0 });
  auto_declare<std::vector<double>>("gains", {});
  auto_declare<std::vector<double>>("offsets", {});
  return CallbackReturn::SUCCESS;
}

InterfaceConfiguration GravityCompensationController::command_interface_configuration() const
{
  InterfaceConfiguration config{ interface_configuration_type::INDIVIDUAL, {} };
  for (const auto& joint : joints_)
  {
    config.names.push_back(joint + "/" + hardware_interface::HW_IF_EFFORT);
  }
  return config;
}

InterfaceConfiguration GravityCompensationController::state_interface_configuration() const
{
  InterfaceConfiguration config{ interface_configuration_type::INDIVIDUAL, {} };
  for (const auto& joint : joints_)
  {
    config.names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
  }
  return config;
}

CallbackReturn GravityCompensationController::on_configure(const rclcpp_lifecycle::State&)
{
  const auto node = get_node();
  const auto logger = node->get_logger();
  joints_ = node->get_parameter("joints").as_string_array();
  const auto gravity = node->get_parameter("gravity_vector").as_double_array();
  const auto payload_com = node->get_parameter("payload_com").as_double_array();
  const double payload_mass = node->get_parameter("payload_mass").as_double();
  gains_ = node->get_parameter("gains").as_double_array();
  offsets_ = node->get_parameter("offsets").as_double_array();
  if (gains_.empty())
  {
    gains_.assign(joints_.size(), 1.0);
  }
  if (offsets_.empty())
  {
    offsets_.assign(joints_.size(), 0.0);
  }
  if (joints_.empty() || gains_.size() != joints_.size() || offsets_.size() != joints_.size() || gravity.size() != 3 ||
      payload_com.size() != 3 || payload_mass < 0.0)
  {
    RCLCPP_ERROR(logger, "Need a non-empty joints list, gains and offsets empty or one per joint, a "
                         "3-element gravity_vector and payload_com, and payload_mass >= 0.");
    return CallbackReturn::ERROR;
  }

  try
  {
    model_ = std::make_unique<GravityModel>(get_robot_description(), node->get_parameter("root_link").as_string(),
                                            node->get_parameter("tip_link").as_string(),
                                            KDL::Vector(gravity[0], gravity[1], gravity[2]), payload_mass,
                                            KDL::Vector(payload_com[0], payload_com[1], payload_com[2]));
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(logger, "%s", e.what());
    return CallbackReturn::ERROR;
  }
  // The chain's own joints, in order: a mismatch would apply one joint's
  // torque to another.
  if (model_->joint_names() != joints_)
  {
    RCLCPP_ERROR(logger, "joints must list the chain's movable joints from root_link to tip_link.");
    return CallbackReturn::ERROR;
  }
  q_.resize(joints_.size());
  tau_.resize(joints_.size());
  return CallbackReturn::SUCCESS;
}

CallbackReturn GravityCompensationController::on_deactivate(const rclcpp_lifecycle::State&)
{
  // Command interfaces keep their last value after release; leave no torque behind.
  for (auto& command : command_interfaces_)
  {
    (void)command.set_value(0.0);
  }
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type GravityCompensationController::update(const rclcpp::Time&, const rclcpp::Duration&)
{
  for (size_t i = 0; i < joints_.size(); ++i)
  {
    const auto position = state_interfaces_[i].get_optional();
    if (!position)
    {
      return controller_interface::return_type::OK;  // Keep last cycle's torques.
    }
    q_(i) = *position;
  }
  if (!model_->Compute(q_, tau_))
  {
    return controller_interface::return_type::ERROR;
  }
  for (size_t i = 0; i < joints_.size(); ++i)
  {
    (void)command_interfaces_[i].set_value(gains_[i] * tau_(i) + offsets_[i]);
  }
  return controller_interface::return_type::OK;
}

}  // namespace kdl_gravity_compensation_controller

PLUGINLIB_EXPORT_CLASS(kdl_gravity_compensation_controller::GravityCompensationController,
                       controller_interface::ControllerInterface)
