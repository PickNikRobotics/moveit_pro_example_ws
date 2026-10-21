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
#include <moveit_pro_behavior/behaviors/core/plan_to_joint_goal.hpp>
#include <moveit_pro_behavior_interface/service_client_interface.hpp>
#include <string>

namespace rebot_bench_behaviors
{
/**
 * @brief PlanToJointGoal, planned from the current state clamped into the robot model's position bounds.
 *
 * @details A joint held in impedance mode can sag past a bound. PlanToJointGoal then plans from past the bound, and
 * refuses the start once the sag puts it in self-collision. This plans from that joint's bound instead; every port,
 * collision check and detour is PlanToJointGoal's. Anchor the plan to the same clamped state, from
 * ClampRobotStateToBounds, so it also executes from there.
 */
class PlanToJointGoalFromBoundedStart final : public moveit_pro::behaviors::PlanToJointGoal
{
public:
  PlanToJointGoalFromBoundedStart(const std::string& name, const BT::NodeConfiguration& config,
                                  const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources);

  /** @brief Plans from the scene that @p scene_client returns, clamped, rather than from the scene service. */
  PlanToJointGoalFromBoundedStart(
      const std::string& name, const BT::NodeConfiguration& config,
      const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources,
      std::unique_ptr<moveit_pro::behaviors::ClientInterfaceBase<moveit_pro::behaviors::GetPlanningScene>> scene_client);

  static BT::KeyValueVector metadata();
};
}  // namespace rebot_bench_behaviors
