#pragma once

#include <std_msgs/msg/empty.hpp>
#include <shared_mutex>

#include "behaviortree_cpp/action_node.h"
#include "moveit_pro_behavior_interface/shared_resources_node.hpp"

namespace get_empty
{

class GetEmpty : public moveit_pro::behaviors::SharedResourcesNode<BT::StatefulActionNode>
{
public:
  GetEmpty(const std::string& name, const BT::NodeConfiguration& config,
           const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources);

  static BT::PortsList providedPorts();

  static BT::KeyValueVector metadata();

  BT::NodeStatus onStart() override;

  BT::NodeStatus onRunning() override;

  void onHalted() override;

private:
  std::shared_ptr<rclcpp::Subscription<std_msgs::msg::Empty>> empty_subscriber_;
  std::shared_mutex empty_mutex_;
  uint64_t message_count_;
};

}  // namespace get_empty