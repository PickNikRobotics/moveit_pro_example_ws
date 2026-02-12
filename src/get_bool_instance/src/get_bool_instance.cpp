#include <get_bool_instance/get_bool_instance.hpp>

#include <std_msgs/msg/bool.hpp>

#include "fmt/format.h"

#include "moveit_pro_behavior_interface/get_required_ports.hpp"
#include "moveit_pro_behavior_interface/metadata_fields.hpp"

namespace get_bool_instance
{

constexpr auto kPortIdBoolTopicName = "bool_topic_name";
constexpr auto kPortIdBoolValue = "subscribed_bool_instance";

GetBoolInstance::GetBoolInstance(
    const std::string& name,
    const BT::NodeConfiguration& config,
    const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
  : moveit_pro::behaviors::SharedResourcesNode<BT::StatefulActionNode>(
        name, config, shared_resources)
{
  bool_subscriber_ =
      shared_resources_->node->create_subscription<std_msgs::msg::Bool>(
          "bool_topic",  // temporary default; overridden in onStart if needed
          rclcpp::SystemDefaultsQoS(),
          [this](const std_msgs::msg::Bool::SharedPtr msg)
          {
            std::unique_lock<std::shared_mutex> lock(bool_mutex_);
            current_bool_ = *msg;
          });
}

BT::PortsList GetBoolInstance::providedPorts()
{
  return {
      BT::InputPort<std::string>(
          kPortIdBoolTopicName,
          "bool_topic",
          "The name of the std_msgs::msg::Bool topic to subscribe to."),
      BT::OutputPort<std_msgs::msg::Bool>(
          kPortIdBoolValue,
          "{subscribed_bool_instance}",
          "Subscribed boolean message.")
  };
}

BT::KeyValueVector GetBoolInstance::metadata()
{
  return {
      { moveit_pro::behaviors::kSubcategoryMetadataKey,
        "User Created Behaviors" },
      { moveit_pro::behaviors::kDescriptionMetadataKey,
        "Subscribe to a single boolean message and store it on the blackboard." }
  };
}

BT::NodeStatus GetBoolInstance::onStart()
{
  const auto ports =
      moveit_pro::behaviors::getRequiredInputs(
          getInput<std::string>(kPortIdBoolTopicName));

  if (!ports)
  {
    shared_resources_->logger->publishFailureMessage(
        name(), ports.error());
    return BT::NodeStatus::FAILURE;
  }

  const auto& [bool_topic_name] = ports.value();

  // Recreate subscriber only if topic changed
  if (!bool_subscriber_ || bool_topic_name != bool_topic_name_)
  {
    bool_topic_name_ = bool_topic_name;
    bool_subscriber_ =
        shared_resources_->node->create_subscription<std_msgs::msg::Bool>(
            bool_topic_name_,
            rclcpp::SystemDefaultsQoS(),
            [this](const std_msgs::msg::Bool::SharedPtr msg)
            {
              std::unique_lock<std::shared_mutex> lock(bool_mutex_);
              current_bool_ = *msg;
            });
  }

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus GetBoolInstance::onRunning()
{
  std::optional<std_msgs::msg::Bool> bool_copy;

  {
    std::shared_lock<std::shared_mutex> lock(bool_mutex_);
    if (!current_bool_)
    {
      return BT::NodeStatus::RUNNING;
    }
    bool_copy = current_bool_;
  }

  setOutput(kPortIdBoolValue, *bool_copy);

  // Stop listening — snapshot behavior
  bool_subscriber_.reset();

  return BT::NodeStatus::SUCCESS;
}

void GetBoolInstance::onHalted()
{
  bool_subscriber_.reset();

  std::unique_lock<std::shared_mutex> lock(bool_mutex_);
  current_bool_.reset();
}

}  // namespace get_bool_instance
