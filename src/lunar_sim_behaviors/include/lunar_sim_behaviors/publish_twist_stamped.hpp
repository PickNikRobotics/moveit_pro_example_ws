#pragma once

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <memory>
#include <moveit_pro_behavior_interface/send_message_to_topic.hpp>

namespace lunar_sim_behaviors
{

/**
 * @brief Publish a geometry_msgs::msg::TwistStamped message to a topic.
 *
 * @details
 * | Data Port Name  | Port Type | Object Type                       |
 * |-----------------|-----------|------------------------------------|
 * | message         | Input     | geometry_msgs::msg::TwistStamped  |
 * | topic           | Input     | std::string                       |
 * | queue_size      | Input     | size_t                            |
 * | use_best_effort | Input     | bool                              |
 */
class PublishTwistStamped final
  : public moveit_pro::behaviors::SendMessageToTopicBehaviorBase<geometry_msgs::msg::TwistStamped>
{
public:
  PublishTwistStamped(const std::string& name, const BT::NodeConfiguration& config,
                      const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources);

  static BT::PortsList providedPorts();

  static BT::KeyValueVector metadata();
};

}  // namespace lunar_sim_behaviors
