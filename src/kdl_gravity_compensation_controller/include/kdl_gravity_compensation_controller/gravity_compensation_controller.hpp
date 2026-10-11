// Copyright 2026 PickNik Inc.
// SPDX-License-Identifier: BSD-3-Clause

#pragma once

#include <memory>
#include <string>
#include <vector>

#include <controller_interface/controller_interface.hpp>
#include <kdl/jntarray.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <realtime_tools/realtime_publisher.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "kdl_gravity_compensation_controller/gravity_model.hpp"

namespace kdl_gravity_compensation_controller
{

// Claims <joint>/effort for each configured joint and writes
// gain * gravity torque + offset to it every cycle, from <joint>/position.
// It claims nothing else, so a trajectory controller can keep each joint's
// position interface. gains and offsets apply as soon as they are set; the
// other parameters are read on configure. Publishes the positions it used and
// the efforts it wrote on ~/commanded_effort.
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
  struct Trim
  {
    std::vector<double> gains;
    std::vector<double> offsets;
  };

  std::vector<std::string> joints_;
  // Written by the parameter callback, read by update().
  realtime_tools::RealtimeBuffer<Trim> trim_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr trim_callback_;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::JointState>> publisher_;
  std::unique_ptr<realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>> rt_publisher_;
  std::unique_ptr<GravityModel> model_;
  KDL::JntArray q_;
  KDL::JntArray tau_;
};

}  // namespace kdl_gravity_compensation_controller
