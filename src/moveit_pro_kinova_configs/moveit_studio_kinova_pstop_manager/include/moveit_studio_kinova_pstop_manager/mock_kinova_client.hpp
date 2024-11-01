// Copyright 2022 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <example_interfaces/srv/trigger.hpp>
#include <example_interfaces/msg/bool.hpp>

namespace moveit_studio::kinova_pstop_manager
{
/**
 * \brief This class is intended for use in simulation, and mocks some of the services that the DashboardClientROS class
 * provides for real hardware.
 */
class MockKinovaClient : public rclcpp::Node
{
public:
  MockKinovaClient(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  /**
   * @brief Returns the result of fault reset try
   *
   * @param request The request object.
   * @param response The response (always OK).
   */
  void handleFaultResetQuery(const example_interfaces::srv::Trigger::Request::SharedPtr request,
                             example_interfaces::srv::Trigger::Response::SharedPtr response);

  void publishFaultState();

  void mockFaultState(const example_interfaces::srv::Trigger::Request::SharedPtr request,
                      example_interfaces::srv::Trigger::Response::SharedPtr response);

  rclcpp::Service<example_interfaces::srv::Trigger>::SharedPtr reset_fault_service_;
  rclcpp::Service<example_interfaces::srv::Trigger>::SharedPtr mock_fault_service_;

  rclcpp::Publisher<example_interfaces::msg::Bool>::SharedPtr fault_state_pub_;
  bool in_fault_;
  rclcpp::TimerBase::SharedPtr fault_state_timer_;
};
}  // namespace moveit_studio::kinova_pstop_manager
