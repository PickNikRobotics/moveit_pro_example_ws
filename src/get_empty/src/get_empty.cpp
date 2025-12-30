#include <get_empty/get_empty.hpp>

#include "spdlog/spdlog.h"
#include "fmt/format.h"

namespace get_empty
{
GetEmpty::GetEmpty(
    const std::string& name, const BT::NodeConfiguration& config,
    const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources)
  : moveit_studio::behaviors::SharedResourcesNode<BT::StatefulActionNode>(name, config, shared_resources)
  , message_count_(0)
{
}

BT::PortsList GetEmpty::providedPorts()
{
  return BT::PortsList({ 
    BT::InputPort<std::string>("empty_topic_name", "empty_topic",
                               "The name of the std_msgs::msg::Empty topic to subscribe to."),
    BT::OutputPort<bool>("message_received", "{message_received}",
                        "Boolean indicating if at least one message has been received."),
    BT::OutputPort<uint64_t>("message_count", "{message_count}",
                            "Total count of empty messages received.") 
  });
}

BT::KeyValueVector GetEmpty::metadata()
{
  return { 
    {"description", "Subscribe to an empty message topic and track received messages on the blackboard."},
    {"subcategory", "Messages"} 
  };
}

BT::NodeStatus GetEmpty::onStart()
{
  // Get the topic name from input port
  const auto topic_name_result = getInput<std::string>("empty_topic_name");
  if (!topic_name_result)
  {
    shared_resources_->logger->publishFailureMessage(
        name(), fmt::format("Failed to get required value from input port 'empty_topic_name': {}", 
                           topic_name_result.error()));
    return BT::NodeStatus::FAILURE;
  }

  const std::string empty_topic_name = topic_name_result.value();

  // Reset message count when starting
  message_count_ = 0;

  // Subscribe to the empty topic
  empty_subscriber_ = shared_resources_->node->create_subscription<std_msgs::msg::Empty>(
      empty_topic_name, rclcpp::SystemDefaultsQoS(), 
      [this](const std_msgs::msg::Empty::SharedPtr msg) {
        (void)msg;  // Empty message has no data
        std::unique_lock lock(empty_mutex_);
        message_count_++;
      });

  // shared_resources_->logger->publishInfoMessage(
  //     name(), fmt::format("Subscribed to empty topic: {}", empty_topic_name));

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus GetEmpty::onRunning()
{
  uint64_t count;
  {
    std::shared_lock lock(empty_mutex_);
    count = message_count_;
  }

  setOutput("message_received", count > 0);
  setOutput("message_count", count);

  if (count > 0)
  {
    // Stop listening once condition is satisfied
    empty_subscriber_.reset();

    // shared_resources_->logger->publishInfoMessage(
    //     name(), "Empty message received — exiting behavior successfully.");

    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::RUNNING;
}

void GetEmpty::onHalted()
{
  // Stop subscribers
  empty_subscriber_.reset();
  
  spdlog::info("GetEmpty behavior halted.");
}

}  // namespace get_empty