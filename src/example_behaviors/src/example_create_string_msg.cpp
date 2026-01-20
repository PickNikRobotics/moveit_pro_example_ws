#include <example_behaviors/example_create_string_msg.hpp>

#include <moveit_pro_behavior_interface/check_for_error.hpp>
#include <std_msgs/msg/string.hpp>

namespace example_behaviors
{
ExampleCreateStringMsg::ExampleCreateStringMsg(
    const std::string& name, const BT::NodeConfiguration& config,
    const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources)
  : moveit_studio::behaviors::SharedResourcesNode<BT::SyncActionNode>(name, config, shared_resources)
{
}

BT::PortsList ExampleCreateStringMsg::providedPorts()
{
  // This node has no input or output ports
  return BT::PortsList(
      { BT::InputPort<std::string>("string", "example_string", "The value of the string ROS message to be created."),
        BT::OutputPort<std_msgs::msg::String>("string_msg", "{string_msg}", "The string ROS message.") });
}

BT::KeyValueVector ExampleCreateStringMsg::metadata()
{
  return { { "subcategory", "Example Behaviors" }, { "description", "Create a ROS String msg on the blackboard." } };
}

BT::NodeStatus ExampleCreateStringMsg::tick()
{
  // getInput returns a BT::Expected so we'll store the result temporarily while we check if it was set correctly
  const auto expected_string = getInput<std::string>("string");

  // The maybe_error function returns a std::optional with an error message if the port was set incorrectly
  if (const auto error = moveit_studio::behaviors::maybe_error(expected_string); error)
  {
    // If the port was set incorrectly, we will log an error message to the UI and the node will return FAILURE
    shared_resources_->logger->publishFailureMessage(name(), "Failed to get required values from input data ports." +
                                                                 error.value());
    return BT::NodeStatus::FAILURE;
  }

  // Create the string msg with the input string.
  std_msgs::msg::String string_msg;
  string_msg.data = expected_string.value();

  setOutput<std_msgs::msg::String>("string_msg", string_msg);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace example_behaviors
