// Copyright 2022 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

// Implementing this feature as a node that provides a service that calls other ROS services is not ideal.  Please see
// the README for a discussion of alternatives.

#pragma once

#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <moveit_studio_agent_msgs/msg/fault_status.hpp>
#include <std_msgs/msg/bool.hpp>
#include <example_interfaces/msg/bool.hpp>
#include <example_interfaces/srv/trigger.hpp>

namespace moveit_studio::kinova_pstop_manager
{
class ProtectiveStopManager : public rclcpp::Node
{
public:
  ProtectiveStopManager(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  using SwitchController = controller_manager_msgs::srv::SwitchController;

  /**
   * @brief This is the callback for the "recover_from_protective_stop" service.  It unlocks the protective stop, stops
   * the program currently running on the arm, and re-sends the control program.
   *
   * @param request An empty request.
   * @param response Indicates whether the protective stop was successfully released.
   */
  void recoverFromProtectiveStop(const std_srvs::srv::Trigger::Request::SharedPtr request,
                                 std_srvs::srv::Trigger::Response::SharedPtr response);

  /**
   * @brief Helper function to check whether a service is unavailable.  If the service is unavailable, this function
   * also sets displays the appropriate error message and sets the Response.
   *
   * @param client The client for the service we are checking.
   * @param response The response object, which will indicate success or failure.
   * @return true The service is unavailable.
   * @return false The service is available.
   */
  bool indicateUnavailableService(rclcpp::ClientBase::SharedPtr client,
                                  std_srvs::srv::Trigger::Response::SharedPtr response = nullptr);

  /**
   * @brief Callback function that publishes the current fault status of the robot.
   *
   */
  void publishFaultStatus();

  /**
   * @brief Determines whether the Kinova robot is in fault.
   *
   * @return true The robot is in fault.
   * @return false The robot is not in fault.
   * @return std::nullopt An error occurred when attempting to call the service.
   */
  bool isRobotInFault();

  std::vector<std::string> all_controller_names;
  std::vector<std::string> active_controller_names;

  rclcpp::CallbackGroup::SharedPtr reentrant_callback_group_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr recovery_service_;
  rclcpp::Client<SwitchController>::SharedPtr switch_controller_client_;

  // generic moveit studio fault publisher
  rclcpp::Publisher<moveit_studio_agent_msgs::msg::FaultStatus>::SharedPtr fault_status_publisher_;
  rclcpp::TimerBase::SharedPtr fault_status_timer_;
  std::atomic<bool> in_fault_;

  // time measurement
  rclcpp::Time last_fault_status_update_;

  // connect to the fault_controller that is communication pipe to/from the driver
  rclcpp::Subscription<example_interfaces::msg::Bool>::SharedPtr fault_ctrl_sub_;
  rclcpp::Client<example_interfaces::srv::Trigger>::SharedPtr fault_reset_client_;
};
}  // namespace moveit_studio::kinova_pstop_manager
