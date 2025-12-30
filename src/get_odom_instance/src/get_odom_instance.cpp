#include <get_odom_instance/get_odom_instance.hpp>

#include <nav_msgs/msg/odometry.hpp>

#include "fmt/format.h"

#include "moveit_studio_behavior_interface/get_required_ports.hpp"
#include "moveit_studio_behavior_interface/metadata_fields.hpp"

namespace get_odom_instance
{

constexpr auto kPortIdOdomTopicName = "odom_topic_name";
constexpr auto kPortIdOdomValue = "subscribed_odom_instance";

GetOdomInstance::GetOdomInstance(
    const std::string& name,
    const BT::NodeConfiguration& config,
    const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources)
  : moveit_studio::behaviors::SharedResourcesNode<BT::StatefulActionNode>(
        name, config, shared_resources)
{
  odom_subscriber_ =
      shared_resources_->node->create_subscription<nav_msgs::msg::Odometry>(
          "odom",  // temporary default; overridden in onStart if needed
          rclcpp::SystemDefaultsQoS(),
          [this](const nav_msgs::msg::Odometry::SharedPtr msg)
          {
            std::unique_lock<std::shared_mutex> lock(odom_mutex_);
            current_odometry_ = *msg;
          });
}

BT::PortsList GetOdomInstance::providedPorts()
{
  return {
      BT::InputPort<std::string>(
          kPortIdOdomTopicName,
          "odom",
          "The name of the nav_msgs::msg::Odometry topic to subscribe to."),
      BT::OutputPort<nav_msgs::msg::Odometry>(
          kPortIdOdomValue,
          "{subscribed_odom_instance}",
          "Subscribed odometry message.")
  };
}

BT::KeyValueVector GetOdomInstance::metadata()
{
  return {
      { moveit_studio::behaviors::kSubcategoryMetadataKey,
        "User Created Behaviors" },
      { moveit_studio::behaviors::kDescriptionMetadataKey,
        "Subscribe to a single odometry message and store it on the blackboard." }
  };
}

BT::NodeStatus GetOdomInstance::onStart()
{
  const auto ports =
      moveit_studio::behaviors::getRequiredInputs(
          getInput<std::string>(kPortIdOdomTopicName));

  if (!ports)
  {
    shared_resources_->logger->publishFailureMessage(
        name(), ports.error());
    return BT::NodeStatus::FAILURE;
  }

  const auto& [odom_topic_name] = ports.value();

  // Recreate subscriber only if topic changed
  if (!odom_subscriber_ || odom_topic_name != odom_topic_name_)
  {
    odom_topic_name_ = odom_topic_name;
    odom_subscriber_ =
        shared_resources_->node->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic_name_,
            rclcpp::SystemDefaultsQoS(),
            [this](const nav_msgs::msg::Odometry::SharedPtr msg)
            {
              std::unique_lock<std::shared_mutex> lock(odom_mutex_);
              current_odometry_ = *msg;
            });
  }

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus GetOdomInstance::onRunning()
{
  std::optional<nav_msgs::msg::Odometry> odom_copy;

  {
    std::shared_lock<std::shared_mutex> lock(odom_mutex_);
    if (!current_odometry_)
    {
      return BT::NodeStatus::RUNNING;
    }
    odom_copy = current_odometry_;
  }

  setOutput(kPortIdOdomValue, *odom_copy);

  // Stop listening — snapshot behavior
  odom_subscriber_.reset();

  return BT::NodeStatus::SUCCESS;
}

void GetOdomInstance::onHalted()
{
  odom_subscriber_.reset();

  std::unique_lock<std::shared_mutex> lock(odom_mutex_);
  current_odometry_.reset();
}

}  // namespace get_odom_instance
