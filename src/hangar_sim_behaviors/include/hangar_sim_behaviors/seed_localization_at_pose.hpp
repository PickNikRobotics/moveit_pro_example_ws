// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <behaviortree_cpp/action_node.h>
#include <moveit_pro_behavior_interface/behavior_context.hpp>
#include <moveit_pro_behavior_interface/shared_resources_node.hpp>

#include <memory>
#include <string>

namespace hangar_sim_behaviors
{
/**
 * @brief Re-seed the particle filter at a pose the caller supplies, with a stated spread.
 *
 * @details Core's `SetInitialPose` seeds from TF -- that is, from the estimate the filter already
 * holds -- which collapses an over-dispersed cloud but cannot move it. This Behavior seeds from an
 * arbitrary pose instead, which is what an operator correcting the robot's position on the map
 * needs, and what restoring that correction afterwards needs.
 *
 * The spread is not a claim about how accurate the pose is. It sets the resolution of anything that
 * refines from here (2000 particles over 0.5 m sit roughly 5 cm apart) and it bounds how far a
 * refinement can travel from the seed, because a forced no-motion update selects among the
 * particles the seed drew and never creates a new one. Widening it in yaw is how a filter converges
 * facing the wrong way; leave `yaw_std_dev` alone unless you know why you are changing it.
 *
 * The pose is projected to the plane: x, y and yaw are used, and roll, pitch and z are discarded,
 * because the localizer is 2D. A pose that is not already in the map frame is refused rather than
 * reinterpreted.
 *
 * | Data Port Name     | Port Type | Object Type                     |
 * | ------------------ | --------- | ------------------------------- |
 * | pose               | Input     | geometry_msgs::msg::PoseStamped |
 * | xy_std_dev         | Input     | double                          |
 * | yaw_std_dev        | Input     | double                          |
 * | initial_pose_topic | Input     | std::string                     |
 * | subscriber_timeout | Input     | double                          |
 */
class SeedLocalizationAtPose final : public moveit_pro::behaviors::SharedResourcesNode<BT::SyncActionNode>
{
public:
  SeedLocalizationAtPose(const std::string& name, const BT::NodeConfiguration& config,
                         const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources);

  static BT::PortsList providedPorts();

  static BT::KeyValueVector metadata();

  BT::NodeStatus tick() override;
};

}  // namespace hangar_sim_behaviors
