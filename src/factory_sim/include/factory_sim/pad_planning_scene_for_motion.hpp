// Copyright 2026 PickNik Inc.
// SPDX-License-Identifier: BSD-3-Clause
#pragma once
#include <behaviortree_cpp/action_node.h>
#include <moveit_pro_behavior_interface/shared_resources_node.hpp>
namespace factory_sim
{
/**
 * @brief Copy a full scene with a minimum robot-link padding.
 * @details Preserves larger existing padding and leaves the input scene unchanged.
 *
 * | Data Port Name | Port Type | Object Type |
 * | --- | --- | --- |
 * | planning_scene | input | moveit_msgs::msg::PlanningScene |
 * | minimum_padding_m | input | double |
 * | padded_scene | output | moveit_msgs::msg::PlanningScene |
 */
class PadPlanningSceneForMotion final : public moveit_pro::behaviors::SharedResourcesNode<BT::SyncActionNode>
{
public:
  using SharedResourcesNode::SharedResourcesNode;
  static BT::PortsList providedPorts();
  static BT::KeyValueVector metadata();
  BT::NodeStatus tick() override;
};
}  // namespace factory_sim
