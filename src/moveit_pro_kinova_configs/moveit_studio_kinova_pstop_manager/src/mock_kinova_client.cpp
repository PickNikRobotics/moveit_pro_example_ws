// Copyright 2022 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include <moveit_studio_kinova_pstop_manager/mock_kinova_client.hpp>

namespace moveit_studio::kinova_pstop_manager
{
constexpr auto kNodeName = "fault_controller";
constexpr auto kFaultTopic = "~/internal_fault";
constexpr auto kResetFault = "~/reset_fault";
constexpr auto kMockFault = "~/mock_fault";

MockKinovaClient::MockKinovaClient(const rclcpp::NodeOptions& options)
  : rclcpp::Node(kNodeName, options)
  , reset_fault_service_(create_service<example_interfaces::srv::Trigger>(
        kResetFault,
        [this](const example_interfaces::srv::Trigger::Request::SharedPtr request,
               example_interfaces::srv::Trigger::Response::SharedPtr response) {
          handleFaultResetQuery(request, response);
        }))
  , mock_fault_service_(create_service<example_interfaces::srv::Trigger>(
        kMockFault,
        [this](const example_interfaces::srv::Trigger::Request::SharedPtr request,
               example_interfaces::srv::Trigger::Response::SharedPtr response) { mockFaultState(request, response); }))
  , fault_state_pub_(create_publisher<example_interfaces::msg::Bool>(kFaultTopic, rclcpp::QoS(1).transient_local()))
  , in_fault_(false)
  , fault_state_timer_(create_wall_timer(std::chrono::milliseconds(50), [&]() { publishFaultState(); }))
{
}

void MockKinovaClient::handleFaultResetQuery(const example_interfaces::srv::Trigger::Request::SharedPtr /*request*/,
                                             example_interfaces::srv::Trigger::Response::SharedPtr response)
{
  in_fault_ = false;
  response->success = true;
}

void MockKinovaClient::publishFaultState()
{
  example_interfaces::msg::Bool msg;
  msg.data = in_fault_;
  fault_state_pub_->publish(msg);
}

void MockKinovaClient::mockFaultState(const example_interfaces::srv::Trigger::Request::SharedPtr /*request*/,
                                      example_interfaces::srv::Trigger::Response::SharedPtr response)
{
  in_fault_ = true;
  response->success = true;
}
}  // namespace moveit_studio::kinova_pstop_manager
