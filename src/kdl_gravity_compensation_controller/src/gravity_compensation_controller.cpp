// Copyright 2026 PickNik Inc.
// SPDX-License-Identifier: BSD-3-Clause

#include "kdl_gravity_compensation_controller/gravity_compensation_controller.hpp"

#include <algorithm>
#include <cmath>
#include <exception>
#include <limits>

#include <controller_interface/helpers.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace kdl_gravity_compensation_controller
{
namespace
{
bool all_finite(const std::vector<double>& values)
{
  return std::all_of(values.begin(), values.end(), [](double value) { return std::isfinite(value); });
}
}  // namespace

using controller_interface::CallbackReturn;
using controller_interface::interface_configuration_type;
using controller_interface::InterfaceConfiguration;
using controller_interface::return_type;

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
  trim_validation_callback_.reset();
  trim_update_callback_.reset();
  joints_ = node->get_parameter("joints").as_string_array();
  const auto gravity = node->get_parameter("gravity_vector").as_double_array();
  const auto payload_com = node->get_parameter("payload_com").as_double_array();
  const double payload_mass = node->get_parameter("payload_mass").as_double();
  Trim trim{ node->get_parameter("gains").as_double_array(), node->get_parameter("offsets").as_double_array() };
  if (trim.gains.empty())
  {
    trim.gains.assign(joints_.size(), 1.0);
  }
  if (trim.offsets.empty())
  {
    trim.offsets.assign(joints_.size(), 0.0);
  }
  if (joints_.empty() || trim.gains.size() != joints_.size() || trim.offsets.size() != joints_.size() ||
      gravity.size() != 3 || payload_com.size() != 3 || !std::isfinite(payload_mass) || payload_mass < 0.0 ||
      !all_finite(gravity) || !all_finite(payload_com) || !all_finite(trim.gains) || !all_finite(trim.offsets))
  {
    RCLCPP_ERROR(logger, "Need a non-empty joints list, finite gains and offsets empty or one per joint, finite "
                         "3-element gravity_vector and payload_com, and finite payload_mass >= 0.");
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
  // Configuration order must be KDL order. Loaned interfaces are independently
  // ordered by name on activation, never assumed to arrive in this order.
  if (model_->joint_names() != joints_)
  {
    RCLCPP_ERROR(logger, "joints must list the chain's movable joints from root_link to tip_link.");
    return CallbackReturn::ERROR;
  }
  q_.resize(joints_.size());
  tau_.resize(joints_.size());
  output_efforts_.assign(joints_.size(), 0.0);
  ordered_commands_.reserve(joints_.size());
  ordered_positions_.reserve(joints_.size());
  effort_references_.clear();
  effort_references_.reserve(joints_.size());
  for (const auto& joint : joints_)
  {
    effort_references_.push_back(std::make_shared<hardware_interface::CommandInterface>(
        std::string(node->get_name()) + "/" + joint, hardware_interface::HW_IF_EFFORT));
  }
  trim_.writeFromNonRT(trim);

  // Validation must not change RT state: a later callback can still reject the
  // transaction. Only the post-set callback installs accepted gains/offsets.
  trim_validation_callback_ =
      node->add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter>& parameters) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        for (const auto& parameter : parameters)
        {
          if (parameter.get_name() != "gains" && parameter.get_name() != "offsets")
          {
            continue;
          }
          if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY ||
              parameter.as_double_array().size() != joints_.size() || !all_finite(parameter.as_double_array()))
          {
            result.successful = false;
            result.reason = parameter.get_name() + " needs one finite value per joint";
            return result;
          }
        }
        return result;
      });
  trim_update_callback_ =
      node->add_post_set_parameters_callback([this](const std::vector<rclcpp::Parameter>& parameters) {
        Trim updated = *trim_.readFromNonRT();
        for (const auto& parameter : parameters)
        {
          if (parameter.get_name() == "gains")
          {
            updated.gains = parameter.as_double_array();
          }
          else if (parameter.get_name() == "offsets")
          {
            updated.offsets = parameter.as_double_array();
          }
        }
        trim_.writeFromNonRT(updated);
      });

  publisher_ = node->create_publisher<sensor_msgs::msg::JointState>("~/commanded_effort", rclcpp::SystemDefaultsQoS());
  rt_publisher_ = std::make_unique<realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>>(publisher_);
  rt_publisher_->msg_.name = joints_;
  rt_publisher_->msg_.position.resize(joints_.size());
  rt_publisher_->msg_.effort.resize(joints_.size());
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::CommandInterface::SharedPtr>
GravityCompensationController::on_export_reference_interfaces_list()
{
  return effort_references_;
}

bool GravityCompensationController::clear_references()
{
  bool success = true;
  for (auto& reference : effort_references_)
  {
    success = reference->set_value(std::numeric_limits<double>::quiet_NaN()) && success;
  }
  return success;
}

bool GravityCompensationController::zero_commands()
{
  bool success = true;
  for (auto& command : command_interfaces_)
  {
    success = command.set_value(0.0) && success;
  }
  return success;
}

bool GravityCompensationController::on_set_chained_mode(bool)
{
  // Controller manager changes mode while inactive. A cleared (NaN) reference
  // contributes zero, including joints not claimed by an upstream controller.
  return clear_references();
}

CallbackReturn GravityCompensationController::on_activate(const rclcpp_lifecycle::State&)
{
  ordered_commands_.clear();
  ordered_positions_.clear();
  if (!controller_interface::get_ordered_interfaces(command_interfaces_, joints_, hardware_interface::HW_IF_EFFORT,
                                                    ordered_commands_) ||
      !controller_interface::get_ordered_interfaces(state_interfaces_, joints_, hardware_interface::HW_IF_POSITION,
                                                    ordered_positions_) ||
      command_interfaces_.size() != joints_.size() || state_interfaces_.size() != joints_.size())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Need exactly one effort command and position state interface per joint.");
    return CallbackReturn::FAILURE;
  }
  for (size_t i = 0; i < joints_.size(); ++i)
  {
    const auto& command = ordered_commands_[i].get();
    const auto& position = ordered_positions_[i].get();
    // The ordering helper checks counts; duplicated loans can still hide a
    // missing joint. Check every mapped name before using the interfaces.
    if (command.get_name() != joints_[i] + "/" + hardware_interface::HW_IF_EFFORT ||
        position.get_name() != joints_[i] + "/" + hardware_interface::HW_IF_POSITION ||
        command.get_data_type() != hardware_interface::HandleDataType::DOUBLE ||
        position.get_data_type() != hardware_interface::HandleDataType::DOUBLE)
    {
      RCLCPP_ERROR(get_node()->get_logger(), "Need unique, correctly named double effort/position interfaces.");
      return CallbackReturn::FAILURE;
    }
    const auto value = position.get_optional();
    if (!value || !std::isfinite(*value))
    {
      RCLCPP_ERROR(get_node()->get_logger(), "Cannot activate without finite joint positions.");
      return CallbackReturn::FAILURE;
    }
  }
  const bool references_reset = clear_references();
  const bool commands_reset = zero_commands();
  return references_reset && commands_reset ? CallbackReturn::SUCCESS : CallbackReturn::FAILURE;
}

CallbackReturn GravityCompensationController::on_deactivate(const rclcpp_lifecycle::State&)
{
  // Command handles retain their values after release. Do not leave a previous
  // producer's effort behind for a later standalone/chained activation.
  const bool references_reset = clear_references();
  const bool commands_reset = zero_commands();
  ordered_commands_.clear();
  ordered_positions_.clear();
  return references_reset && commands_reset ? CallbackReturn::SUCCESS : CallbackReturn::ERROR;
}

CallbackReturn GravityCompensationController::on_cleanup(const rclcpp_lifecycle::State&)
{
  trim_validation_callback_.reset();
  trim_update_callback_.reset();
  effort_references_.clear();
  ordered_commands_.clear();
  ordered_positions_.clear();
  rt_publisher_.reset();
  publisher_.reset();
  model_.reset();
  return CallbackReturn::SUCCESS;
}

CallbackReturn GravityCompensationController::on_error(const rclcpp_lifecycle::State& state)
{
  return on_deactivate(state);
}

CallbackReturn GravityCompensationController::on_shutdown(const rclcpp_lifecycle::State& state)
{
  return on_deactivate(state);
}

return_type GravityCompensationController::update_reference_from_subscribers(const rclcpp::Time&,
                                                                             const rclcpp::Duration&)
{
  // There is deliberately no topic command path. Standalone is gravity-only,
  // even if an old reference handle retained a nonzero value.
  return clear_references() ? return_type::OK : fail_update();
}

return_type GravityCompensationController::fail_update()
{
  // Best effort only: report ERROR even if the zero write also fails. Never
  // forward an invalid sum or label an unreadable input as a healthy cycle.
  (void)zero_commands();
  return return_type::ERROR;
}

return_type GravityCompensationController::update_and_write_commands(const rclcpp::Time& time, const rclcpp::Duration&)
{
  // Consume every reference once, even on invalid input: a producer must write
  // each cycle, so a released or skipped reference cannot leave effort behind.
  bool inputs_valid = true;
  for (size_t i = 0; i < joints_.size(); ++i)
  {
    const auto position = ordered_positions_[i].get().get_optional();
    const auto reference = effort_references_[i]->get_optional();
    const bool cleared = effort_references_[i]->set_value(std::numeric_limits<double>::quiet_NaN());
    if (!cleared || !position || !reference || !std::isfinite(*position) || std::isinf(*reference))
    {
      inputs_valid = false;
      continue;
    }
    q_(i) = *position;
    output_efforts_[i] = std::isnan(*reference) ? 0.0 : *reference;
  }
  if (!inputs_valid || !model_->Compute(q_, tau_))
  {
    return fail_update();
  }
  const Trim& trim = *trim_.readFromRT();
  // Validate the complete output before writing any joint. The trim belongs
  // only to gravity: it must not scale an upstream effort or accumulate it.
  for (size_t i = 0; i < joints_.size(); ++i)
  {
    output_efforts_[i] += trim.gains[i] * tau_(i) + trim.offsets[i];
    if (!std::isfinite(tau_(i)) || !std::isfinite(output_efforts_[i]))
    {
      return fail_update();
    }
  }
  for (size_t i = 0; i < joints_.size(); ++i)
  {
    if (!ordered_commands_[i].get().set_value(output_efforts_[i]))
    {
      return fail_update();
    }
  }
  if (rt_publisher_->trylock())
  {
    for (size_t i = 0; i < joints_.size(); ++i)
    {
      rt_publisher_->msg_.position[i] = q_(i);
      rt_publisher_->msg_.effort[i] = output_efforts_[i];
    }
    rt_publisher_->msg_.header.stamp = time;
    rt_publisher_->unlockAndPublish();
  }
  return return_type::OK;
}

}  // namespace kdl_gravity_compensation_controller

PLUGINLIB_EXPORT_CLASS(kdl_gravity_compensation_controller::GravityCompensationController,
                       controller_interface::ChainableControllerInterface)
