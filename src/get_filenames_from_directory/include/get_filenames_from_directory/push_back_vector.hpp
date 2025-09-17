// Copyright 2025 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <behaviortree_cpp/action_node.h>
#include <moveit_studio_behavior_interface/shared_resources_node.hpp>

namespace moveit_studio::behaviors
{
/**
 * @brief Insert an element into a vector at the given index.
 *
 * @details
 * | Data Port Name     | Port Type   | Object Type                       |
 * | ------------------ | ----------- | -------------                     |
 * | input_vector       | input       | std::vector<BT::Any>              |
 * | index              | input       | int                               |
 * | element            | input       | BT::Any                           |
 * | output_vector      | output      | std::vector<BT::Any>              |
 */
class PushBackVector final : public SharedResourcesNode<BT::SyncActionNode>
{
public:
  PushBackVector(const std::string& name, const BT::NodeConfiguration& config,
                 const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources);

  static BT::PortsList providedPorts();

  static BT::KeyValueVector metadata();

  BT::NodeStatus tick() override;
};
}  // namespace moveit_studio::behaviors
