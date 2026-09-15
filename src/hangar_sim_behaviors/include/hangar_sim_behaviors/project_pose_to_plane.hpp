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
 * @brief Flatten a pose into the plane the 2D localizer works in: keep x, y and yaw, zero the rest.
 *
 * @details `SeedLocalizationAtPose` flattens the pose it is handed before publishing it, because
 * the localizer is 2D. Anything that later measures a distance against that seed has to compare
 * against the flattened pose, not the pose that was handed in -- otherwise the measurement picks up
 * the z, roll and pitch that were never seeded, and a limit on how far the estimate moved becomes a
 * limit on how tilted the click surface was. Both use the same projection, from
 * `hangar_sim_behaviors/planar_pose.hpp`.
 *
 * The frame is passed through untouched: this changes the pose's shape, not the frame it is in.
 *
 * | Data Port Name | Port Type | Object Type                     |
 * | -------------- | --------- | ------------------------------- |
 * | pose           | Input     | geometry_msgs::msg::PoseStamped |
 * | planar_pose    | Output    | geometry_msgs::msg::PoseStamped |
 */
class ProjectPoseToPlane final : public moveit_pro::behaviors::SharedResourcesNode<BT::SyncActionNode>
{
public:
  ProjectPoseToPlane(const std::string& name, const BT::NodeConfiguration& config,
                     const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources);

  static BT::PortsList providedPorts();

  static BT::KeyValueVector metadata();

  BT::NodeStatus tick() override;
};

}  // namespace hangar_sim_behaviors
