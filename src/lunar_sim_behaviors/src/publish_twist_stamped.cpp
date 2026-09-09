#include <lunar_sim_behaviors/publish_twist_stamped.hpp>
#include <moveit_pro_behavior_interface/metadata_fields.hpp>

namespace lunar_sim_behaviors
{
inline constexpr auto kDescriptionPublishTwistStamped = R"(
                <p>
                    Publish a <code>geometry_msgs::msg::TwistStamped</code> message to a topic.
                </p>
            )";

using Base = moveit_pro::behaviors::SendMessageToTopicBehaviorBase<geometry_msgs::msg::TwistStamped>;

PublishTwistStamped::PublishTwistStamped(const std::string& name, const BT::NodeConfiguration& config,
                                         const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
  : Base(name, config, shared_resources)
{
}

BT::PortsList PublishTwistStamped::providedPorts()
{
  return BT::PortsList({
      BT::InputPort<geometry_msgs::msg::TwistStamped>(Base::kPortIDMessage, "The TwistStamped message to publish."),
      BT::InputPort<std::string>(Base::kPortIDTopicName, "", "The topic the message should be published to."),
      BT::InputPort<size_t>(Base::kPortIDQueueSize, 1, "The queue size for the publisher."),
      BT::InputPort<bool>(Base::kPortIDUseBestEffort, false,
                          "Whether the publisher's reliability should be best effort (true) or reliable (false)."),
  });
}

BT::KeyValueVector PublishTwistStamped::metadata()
{
  return { { moveit_pro::behaviors::kSubcategoryMetadataKey, "ROS Messaging" },
           { moveit_pro::behaviors::kDescriptionMetadataKey, kDescriptionPublishTwistStamped } };
}

}  // namespace lunar_sim_behaviors

template class moveit_pro::behaviors::SendMessageToTopicBehaviorBase<geometry_msgs::msg::TwistStamped>;
