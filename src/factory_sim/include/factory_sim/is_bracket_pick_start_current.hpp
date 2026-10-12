// Copyright 2026 PickNik Inc.
// SPDX-License-Identifier: BSD-3-Clause

#pragma once

#include <behaviortree_cpp/condition_node.h>

namespace factory_sim
{
/**
 * @brief Check a prepared Fanuc bracket-pick start against fresh stopped feedback.
 * @details Requires all six bounded arm joints, finite positions and velocities, a full scene, and stamped feedback on
 * the receipt clock. Rejects multi-DOF trajectories; FAILURE requests replanning without commanding motion.
 *
 * | Data Port Name | Port Type | Object Type |
 * | --- | --- | --- |
 * | solution | input | moveit_task_constructor_msgs::msg::Solution |
 * | planning_scene | input | moveit_msgs::msg::PlanningScene |
 * | joint_state | input | sensor_msgs::msg::JointState |
 * | current_time | input | double |
 * | maximum_state_age_s | input | double |
 * | velocity_tolerance | input | double |
 * | position_tolerance | input | double |
 * | reason | output | std::string |
 */
class IsBracketPickStartCurrent final : public BT::ConditionNode
{
public:
  using BT::ConditionNode::ConditionNode;
  [[nodiscard]] static BT::PortsList providedPorts();
  [[nodiscard]] static BT::KeyValueVector metadata();
  [[nodiscard]] BT::NodeStatus tick() override;
};
}  // namespace factory_sim
