// Copyright 2026 PickNik Inc.
// SPDX-License-Identifier: BSD-3-Clause

#pragma once

#include <behaviortree_cpp/condition_node.h>

namespace factory_sim
{
/**
 * @brief Read an enabled flag through a typed boolean port.
 * @details Accepts boolean blackboard values and literal XML inputs.
 *
 * | Data Port Name | Port Type | Object Type |
 * | --- | --- | --- |
 * | enabled | input | bool |
 */
class IsLookaheadEnabled final : public BT::ConditionNode
{
public:
  using BT::ConditionNode::ConditionNode;
  [[nodiscard]] static BT::PortsList providedPorts();
  [[nodiscard]] static BT::KeyValueVector metadata();
  [[nodiscard]] BT::NodeStatus tick() override;
};
}  // namespace factory_sim
