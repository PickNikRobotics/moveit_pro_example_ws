#include <example_behaviors/example_get_string_from_topic.hpp>

// Include the template implementation for GetMessageFromTopicBehaviorBase<T>.
#include <moveit_pro_behavior_interface/impl/get_message_from_topic_impl.hpp>

namespace example_behaviors
{
ExampleGetStringFromTopic::ExampleGetStringFromTopic(const std::string& name, const BT::NodeConfiguration& config,
                                                     const std::shared_ptr<BehaviorContext>& shared_resources)
  : GetMessageFromTopicBehaviorBase<std_msgs::msg::String>(name, config, shared_resources)
{
}

BT::PortsList ExampleGetStringFromTopic::providedPorts()
{
  // This node has one input port and one output port
  return BT::PortsList({
      BT::InputPort<std::string>("topic_name", "/chatter", "The name of the topic the Behavior subscribes to."),
      BT::OutputPort<std::vector<int>>("message_out", "{my_string}", "The output String message."),
  });
}

BT::KeyValueVector ExampleGetStringFromTopic::metadata()
{
  return { { "subcategory", "Example Behaviors" },
           { "description", "Captures a string message and makes it available on an output port." } };
}

}  // namespace example_behaviors

// Template specialization of GetMessageFromTopicBehaviorBase<T> for the String message type
template class moveit_studio::behaviors::GetMessageFromTopicBehaviorBase<std_msgs::msg::String>;
