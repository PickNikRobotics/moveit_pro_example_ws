// Copyright 2026 PickNik Inc.
// SPDX-License-Identifier: BSD-3-Clause

#pragma once

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <controller_interface/chainable_controller_interface.hpp>
#include <kdl/jntarray.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <realtime_tools/realtime_publisher.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "kdl_gravity_compensation_controller/gravity_model.hpp"

namespace kdl_gravity_compensation_controller
{

// The sole writer of <joint>/effort. Upstream controllers write the exported
// <controller>/<joint>/effort references, not the hardware interfaces, every
// cycle. Output: upstream effort + gain * KDL gravity torque + offset. Each
// update consumes the references and clears them to NaN; a NaN (unwritten)
// reference contributes zero. Standalone has no input subscriber and uses zero
// upstream effort. All parameters except gains and offsets take effect on
// configure; those two apply live.
class GravityCompensationController : public controller_interface::ChainableControllerInterface
{
public:
  controller_interface::CallbackReturn on_init() override;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::CallbackReturn on_error(const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State& previous_state) override;

protected:
  std::vector<hardware_interface::CommandInterface::SharedPtr> on_export_reference_interfaces_list() override;
  bool on_set_chained_mode(bool chained_mode) override;
  controller_interface::return_type update_reference_from_subscribers(const rclcpp::Time& time,
                                                                      const rclcpp::Duration& period) override;
  controller_interface::return_type update_and_write_commands(const rclcpp::Time& time,
                                                              const rclcpp::Duration& period) override;

private:
  struct Trim
  {
    std::vector<double> gains;
    std::vector<double> offsets;
  };

  bool clear_references();
  bool zero_commands();
  controller_interface::return_type fail_update();

  std::vector<std::string> joints_;
  // Handle-owned storage remains stable until cleanup/reconfigure; shared with
  // the resource manager, never resized or allocated by update().
  std::vector<hardware_interface::CommandInterface::SharedPtr> effort_references_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> ordered_commands_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> ordered_positions_;
  std::vector<double> output_efforts_;
  // Written only after a parameter transaction is accepted, read by update().
  realtime_tools::RealtimeBuffer<Trim> trim_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr trim_validation_callback_;
  rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr trim_update_callback_;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::JointState>> publisher_;
  std::unique_ptr<realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>> rt_publisher_;
  std::unique_ptr<GravityModel> model_;
  KDL::JntArray q_;
  KDL::JntArray tau_;
};

}  // namespace kdl_gravity_compensation_controller
