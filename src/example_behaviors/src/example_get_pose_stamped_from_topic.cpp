#include <example_behaviors/example_get_pose_stamped_from_topic.hpp>

#include <moveit_studio_behavior_interface/impl/get_message_from_topic_impl.hpp>

namespace example_behaviors
{
// Mirrors the PoseStamped specialization produced by generate_topic_interface_behaviors.py so developers
// can see the generated pattern inside example_behaviors.
ExampleGetPoseStampedFromTopic::ExampleGetPoseStampedFromTopic(
    const std::string& name, const BT::NodeConfiguration& config,
    const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources)
  : GetMessageFromTopicBehaviorBase<geometry_msgs::msg::PoseStamped>(name, config, shared_resources)
{
}

BT::PortsList ExampleGetPoseStampedFromTopic::providedPorts()
{
  return BT::PortsList({
      BT::InputPort<std::string>(kPortIDTopicName, "/pose_stamped",
                                 "Topic that publishes <code>geometry_msgs::msg::PoseStamped</code> messages."),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>(kPortIDMessageOut, "{pose_stamped}",
                                                      "Output pose from the subscribed topic."),
  });
}

BT::KeyValueVector ExampleGetPoseStampedFromTopic::metadata()
{
  return { { "subcategory", "Example Behaviors" },
           { "description", "Capture a pose stamped message from a topic and store it on the blackboard." } };
}

}  // namespace example_behaviors

template class moveit_studio::behaviors::GetMessageFromTopicBehaviorBase<geometry_msgs::msg::PoseStamped>;
