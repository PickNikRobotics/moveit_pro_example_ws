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

#include <memory>
#include <string>

#include <behaviortree_cpp/action_node.h>
#include <moveit_pro_behavior_interface/behavior_context.hpp>
#include <moveit_pro_behavior_interface/shared_resources_node.hpp>

namespace vla_sim_behaviors
{
/**
 * @brief Turns a trial index into that trial's cube layout and the cube pair to stack.
 *
 * @details Gets the trial from sampleCubeStackTrial(), so the same trial_index and seed give the same layout and pair
 * every time. A reset Objective places the cubes at cube_poses with SetMujocoFreeBodyPoses, a task Objective passes
 * prompt to the policy, and a success check tests that top_color rests on bottom_color. Each of them runs this
 * Behavior with the same trial_index and seed, so they agree on the trial.
 *
 * | Data Port Name | Port Type | Object Type                                  |
 * | -------------- | --------- | -------------------------------------------- |
 * | trial_index    | input     | int                                          |
 * | seed           | input     | int                                          |
 * | cube_poses     | output    | std::vector<geometry_msgs::msg::PoseStamped> |
 * | top_color      | output    | std::string                                  |
 * | bottom_color   | output    | std::string                                  |
 * | prompt         | output    | std::string                                  |
 */
class SampleCubeStackTrial final : public moveit_pro::behaviors::SharedResourcesNode<BT::SyncActionNode>
{
public:
  SampleCubeStackTrial(const std::string& name, const BT::NodeConfiguration& config,
                       const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources);

  static BT::PortsList providedPorts();

  static BT::KeyValueVector metadata();

  BT::NodeStatus tick() override;
};
}  // namespace vla_sim_behaviors
