#pragma once

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/bt_factory.h>
#include <moveit_studio_behavior_interface/behavior_context.hpp>
#include <moveit_studio_behavior_interface/shared_resources_node.hpp>
#include <rclcpp/publisher.hpp>
#include <std_msgs/msg/color_rgba.hpp>

#include <memory>
#include <string>

namespace example_behaviors
{
class PublishColorRGBA final : public moveit_studio::behaviors::SharedResourcesNode<BT::SyncActionNode>
{
public:
  PublishColorRGBA(const std::string& name, const BT::NodeConfiguration& config,
                   const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources);

  static BT::PortsList providedPorts();

  static BT::KeyValueVector metadata();

  BT::NodeStatus tick() override;

private:
  std::shared_ptr<rclcpp::Publisher<std_msgs::msg::ColorRGBA>> publisher_;
};
}  // namespace example_behaviors
