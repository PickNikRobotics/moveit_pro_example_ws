// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <memory>
#include <string>

#include <behaviortree_cpp/action_node.h>
#include <control_msgs/action/gripper_command.hpp>
#include <moveit_pro_behavior_interface/shared_resources_node.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

namespace vla_sim_behaviors
{
/**
 * @brief Sends a gripper position goal and succeeds as soon as it is dispatched.
 */
class SendGripperCommand : public moveit_pro::behaviors::SharedResourcesNode<BT::SyncActionNode>
{
public:
  SendGripperCommand(const std::string& name, const BT::NodeConfiguration& config,
                     const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources);

  [[nodiscard]] static BT::PortsList providedPorts();
  [[nodiscard]] static BT::KeyValueVector metadata();

  BT::NodeStatus tick() override;

private:
  rclcpp_action::Client<control_msgs::action::GripperCommand>::SharedPtr client_;
  std::string action_name_;
};
}  // namespace vla_sim_behaviors
