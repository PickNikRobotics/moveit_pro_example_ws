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
#include <sensor_msgs/msg/joint_state.hpp>
#include <string>
#include <tl/expected.hpp>

namespace moveit_pro::base
{
class RobotModel;
}  // namespace moveit_pro::base

namespace rebot_bench_behaviors
{
/** Clamps each named joint into the robot model's position bounds; fails for a joint it cannot bound. */
tl::expected<void, std::string> clampToBounds(const moveit_pro::base::RobotModel& model,
                                              sensor_msgs::msg::JointState& joints);

/**
 * @brief Clamps each joint of a robot state into the robot model's position bounds.
 *
 * @details A joint held in impedance mode can sag past a bound. A command anchored to the clamped state keeps a
 * trajectory within the bounds from its first point. Fails for a joint the robot model does not know, or a non-finite
 * position.
 *
 * | Data Port Name | Port Type | Object Type                  |
 * | -------------- | --------- | ---------------------------- |
 * | robot_state    | input     | moveit_msgs::msg::RobotState |
 * | bounded_state  | output    | moveit_msgs::msg::RobotState |
 */
class ClampRobotStateToBounds final : public moveit_pro::behaviors::SharedResourcesNode<BT::SyncActionNode>
{
public:
  using SharedResourcesNode::SharedResourcesNode;
  static BT::PortsList providedPorts();
  static BT::KeyValueVector metadata();
  BT::NodeStatus tick() override;
};
}  // namespace rebot_bench_behaviors
