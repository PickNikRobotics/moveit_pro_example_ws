#include <example_behaviors/example_delayed_message.hpp>

namespace example_behaviors
{
ExampleDelayedMessage::ExampleDelayedMessage(
    const std::string& name, const BT::NodeConfiguration& config,
    const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
  : moveit_pro::behaviors::SharedResourcesNode<BT::StatefulActionNode>(name, config, shared_resources)
{
}

BT::PortsList ExampleDelayedMessage::providedPorts()
{
  // delay_duration: Number of seconds to wait before logging a message
  return BT::PortsList(
      { BT::InputPort<double>("delay_duration", "5", "The duration, in seconds, to wait before logging a message.") });
}

BT::KeyValueVector ExampleDelayedMessage::metadata()
{
  return { { "subcategory", "Example Behaviors" },
           { "description", "After some time, log a message that says \"Hello, world!\"." } };
}

BT::NodeStatus ExampleDelayedMessage::onStart()
{
  // Store the time at which this node was first ticked
  start_time_ = std::chrono::steady_clock::now();

  // getInput returns a BT::Expected so we'll store the result temporarily while we check if it was set correctly
  const auto maybe_duration = getInput<double>("delay_duration");

  // The maybe_error function returns a std::optional with an error message if the port was set incorrectly
  if (const auto error = moveit_pro::behaviors::maybe_error(maybe_duration); error)
  {
    // If the port was set incorrectly, we will log an error message to the UI and the node will return FAILURE
    shared_resources_->logger->publishFailureMessage(name(), "Failed to get required values from input data ports." +
                                                                 error.value());
    return BT::NodeStatus::FAILURE;
  }

  // Store the value of the port
  delay_duration_ = maybe_duration.value();

  // If the duration is zero, we can log the message immediately
  if (delay_duration_ <= 0)
  {
    // Log the "Hello, world!" message.
    // Setting the third argument to false ensures the message will be shown immediately
    shared_resources_->logger->publishInfoMessage(name(), "Hello, world!");
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ExampleDelayedMessage::onRunning()
{
  // If the delay duration has not elapsed since this node was started, return RUNNING
  if (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - start_time_).count() <
      delay_duration_)
  {
    // Log the "Hello, world!" message.
    // Setting the third argument to false ensures the message will be shown immediately
    shared_resources_->logger->publishInfoMessage(name(), "Hello, world!");
    return BT::NodeStatus::SUCCESS;
  }
  else
  {
    return BT::NodeStatus::SUCCESS;
  }
}

void ExampleDelayedMessage::onHalted()
{
}

}  // namespace example_behaviors
