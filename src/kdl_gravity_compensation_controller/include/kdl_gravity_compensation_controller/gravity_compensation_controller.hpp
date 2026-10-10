// Copyright 2026 PickNik Inc.
// SPDX-License-Identifier: BSD-3-Clause

#pragma once

#include <memory>
#include <string>
#include <vector>

#include <controller_interface/controller_interface.hpp>
#include <kdl/jntarray.hpp>

#include "kdl_gravity_compensation_controller/gravity_model.hpp"

namespace kdl_gravity_compensation_controller
{

// Claims <joint>/effort for each configured joint and writes
// gain * gravity torque + offset to it every cycle, from <joint>/position.
// It claims nothing else, so a trajectory controller can keep each joint's
// position interface. Parameters are read on configure; reconfigure the
// controller to apply a change.
class GravityCompensationController : public controller_interface::ControllerInterface
{
public:
  controller_interface::CallbackReturn on_init() override;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
  std::vector<std::string> joints_;
  std::vector<double> gains_;
  std::vector<double> offsets_;
  std::unique_ptr<GravityModel> model_;
  KDL::JntArray q_;
  KDL::JntArray tau_;
};

}  // namespace kdl_gravity_compensation_controller
