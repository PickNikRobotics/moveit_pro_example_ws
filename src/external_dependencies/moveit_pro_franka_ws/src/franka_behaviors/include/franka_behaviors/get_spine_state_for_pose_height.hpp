// Copyright 2026 PickNik Inc.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the PickNik Inc. nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#pragma once

#include <behaviortree_cpp/action_node.h>
#include <moveit_pro_behavior_interface/shared_resources_node.hpp>

namespace franka_behaviors
{
/**
 * @brief Computes a spine-only goal that puts the spine link at the height of a target pose.
 *
 * @details
 * The pose and the spine link are compared in the spine joint's parent frame, where the joint axis is +Z.
 * The current spine value comes from the planning scene. The goal is clamped to the spine travel.
 * This Behavior does not move the robot: feed its output to "Move to Joint State" on the spine group.
 *
 * | Data Port Name      | Port Type | Object Type                          |
 * | ------------------- | --------- | ------------------------------------ |
 * | target_pose         | Input     | geometry_msgs::msg::PoseStamped      |
 * | planning_scene      | Input     | moveit_msgs::msg::PlanningScene      |
 * | spine_joint_name    | Input     | std::string                          |
 * | spine_parent_frame  | Input     | std::string                          |
 * | spine_link_name     | Input     | std::string                          |
 * | height_bias         | Input     | double                               |
 * | min_spine_value     | Input     | double                               |
 * | max_spine_value     | Input     | double                               |
 * | spine_joint_state   | Output    | moveit_msgs::msg::RobotState         |
 * | target_spine_value  | Output    | double                               |
 */
class GetSpineStateForPoseHeight : public moveit_pro::behaviors::SharedResourcesNode<BT::SyncActionNode>
{
public:
  GetSpineStateForPoseHeight(const std::string& name, const BT::NodeConfiguration& config,
                             const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources);

  static BT::PortsList providedPorts();

  static BT::KeyValueVector metadata();

  BT::NodeStatus tick() override;
};
}  // namespace franka_behaviors
