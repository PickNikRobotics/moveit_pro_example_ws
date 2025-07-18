// Copyright (c) 2023 Franka Robotics GmbH
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <franka_force_torque_sensor/franka_force_torque_sensor.hpp>

#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace
{
constexpr auto kArmId = "fr3";
constexpr auto kSensorName = "franka_ft_sensor";
const std::vector<std::string> kForceTorqueInterfaces = {
  kSensorName + std::string("/force.x"),  kSensorName + std::string("/force.y"),  kSensorName + std::string("/force.z"),
  kSensorName + std::string("/torque.x"), kSensorName + std::string("/torque.y"), kSensorName + std::string("/torque.z")
};
}  // namespace

namespace franka_force_torque_sensor
{

controller_interface::CallbackReturn ForceTorqueSensor::on_init()
{
  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration ForceTorqueSensor::command_interface_configuration() const
{
  return controller_interface::InterfaceConfiguration
  {
    return { interface_configuration_type::INDIVIDUAL, kForceTorqueInterfaces };
  };
}

controller_interface::InterfaceConfiguration ForceTorqueSensor::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto& franka_robot_model_name : franka_robot_model_->get_state_interface_names())
  {
    state_interfaces_config.names.push_back(franka_robot_model_name);
  }
  return state_interfaces_config;
}

controller_interface::CallbackReturn ForceTorqueSensor::on_configure(const rclcpp_lifecycle::State& /*previous_state*/)
{
  franka_robot_model_ =
      std::make_unique<franka_semantic_components::FrankaRobotModel>(franka_semantic_components::FrankaRobotModel(
          kArmId + "/" + k_robot_model_interface_name, kArmId + "/" + k_robot_state_interface_name));

  RCLCPP_DEBUG(get_node()->get_logger(), "configured successfully");
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ForceTorqueSensor::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  franka_robot_model_->assign_loaned_state_interfaces(state_interfaces_);
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ForceTorqueSensor::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  franka_robot_model_->release_interfaces();
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type ForceTorqueSensor::update(const rclcpp::Time& /*time*/,
                                                            const rclcpp::Duration& /*period*/)
{
  std::array<double, 49> mass = franka_robot_model_->getMassMatrix();
  std::array<double, 7> coriolis = franka_robot_model_->getCoriolisForceVector();
  std::array<double, 7> gravity = franka_robot_model_->getGravityForceVector();
  std::array<double, 16> pose = franka_robot_model_->getPoseMatrix(franka::Frame::kJoint4);
  std::array<double, 42> joint4_body_jacobian_wrt_joint4 = franka_robot_model_->getBodyJacobian(franka::Frame::kJoint4);
  std::array<double, 42> endeffector_jacobian_wrt_base =
      franka_robot_model_->getZeroJacobian(franka::Frame::kEndEffector);

  return controller_interface::return_type::OK;
}

}  // namespace franka_force_torque_sensor

#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(franka_force_torque_sensor::ForceTorqueSensor, controller_interface::ControllerInterface)
