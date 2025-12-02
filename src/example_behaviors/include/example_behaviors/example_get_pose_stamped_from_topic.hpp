#pragma once

#include <moveit_studio_behavior_interface/get_message_from_topic.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace example_behaviors
{
class ExampleGetPoseStampedFromTopic final
  : public moveit_studio::behaviors::GetMessageFromTopicBehaviorBase<geometry_msgs::msg::PoseStamped>
{
public:
  ExampleGetPoseStampedFromTopic(const std::string& name, const BT::NodeConfiguration& config,
                                 const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources);

  static BT::PortsList providedPorts();

  static BT::KeyValueVector metadata();

private:
  std::shared_future<tl::expected<bool, std::string>>& getFuture() override
  {
    return future_;
  }

  std::shared_future<tl::expected<bool, std::string>> future_;
};
}  // namespace example_behaviors
