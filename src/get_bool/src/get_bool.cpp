// Copyright 2025 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include <get_bool/get_bool.hpp>

#include "fmt/format.h"

#include "moveit_studio_behavior_interface/get_required_ports.hpp"
#include "moveit_studio_behavior_interface/metadata_fields.hpp"

namespace get_bool
{

// Port names.
constexpr auto kPortIdBoolTopicName = "bool_topic_name";
constexpr auto kPortIdBoolValue = "subscribed_bool";

GetBool::GetBool(const std::string& name, const BT::NodeConfiguration& config,
                 const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources)
  : moveit_studio::behaviors::SharedResourcesNode<BT::StatefulActionNode>(name, config, shared_resources)
{
}

BT::PortsList GetBool::providedPorts()
{
  return BT::PortsList({ BT::InputPort<std::string>(kPortIdBoolTopicName, "bool_topic",
                                                    "The name of the std_msgs::msg::Bool topic to subscribe to."),
                         BT::OutputPort<bool>(kPortIdBoolValue, "{subscribed_bool}",
                                             "Subscribed boolean value.") });
}

BT::KeyValueVector GetBool::metadata()
{
  return { { moveit_studio::behaviors::kSubcategoryMetadataKey, "User Created Behaviors" },
           { moveit_studio::behaviors::kDescriptionMetadataKey,
             "Subscribe to a bool message and store it on the blackboard." } };
}

BT::NodeStatus GetBool::onStart()
{
  const auto ports = moveit_studio::behaviors::getRequiredInputs(getInput<std::string>(kPortIdBoolTopicName));

  if (!ports.has_value())
  {
    shared_resources_->logger->publishFailureMessage(
        name(), fmt::format("Failed to get required value from input data port: {}", ports.error()));
    return BT::NodeStatus::FAILURE;
  }

  const auto& [bool_topic_name] = ports.value();

  // Subscribe to the bool topic.
  bool_subscriber_ = shared_resources_->node->create_subscription<std_msgs::msg::Bool>(
      bool_topic_name, rclcpp::SystemDefaultsQoS(), [this](const std_msgs::msg::Bool::SharedPtr msg) {
        std::unique_lock lock(bool_mutex_);
        current_bool_value_ = msg->data;
      });

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus GetBool::onRunning()
{
  // Update output port.
  {
    std::shared_lock lock(bool_mutex_);
    setOutput(kPortIdBoolValue, current_bool_value_);
  }

  return BT::NodeStatus::RUNNING;
}

void GetBool::onHalted()
{
  // Stop subscriber.
  bool_subscriber_.reset();
}

}  // namespace get_bool