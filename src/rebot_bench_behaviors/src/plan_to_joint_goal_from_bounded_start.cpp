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
#include <rebot_bench_behaviors/plan_to_joint_goal_from_bounded_start.hpp>

#include <moveit_pro_base/robot_model/robot_model.hpp>
#include <moveit_pro_behavior_interface/metadata_fields.hpp>
#include <rebot_bench_behaviors/clamp_robot_state_to_bounds.hpp>
#include <utility>

namespace rebot_bench_behaviors
{
namespace
{
using moveit_pro::behaviors::BehaviorContext;
using moveit_pro::behaviors::ClientInterfaceBase;
using moveit_pro::behaviors::GetPlanningScene;

class BoundedSceneClient final : public ClientInterfaceBase<GetPlanningScene>
{
public:
  BoundedSceneClient(std::shared_ptr<BehaviorContext> context,
                     std::unique_ptr<ClientInterfaceBase<GetPlanningScene>> scene_client)
    : context_(std::move(context)), scene_client_(std::move(scene_client))
  {
  }

  void initialize(const std::string& service_name, std::chrono::duration<double> wait_for_server_timeout,
                  std::chrono::duration<double> response_timeout) override
  {
    scene_client_->initialize(service_name, wait_for_server_timeout, response_timeout);
  }

  bool waitForServiceServer() const override
  {
    return scene_client_->waitForServiceServer();
  }

  tl::expected<GetPlanningScene::Response, std::string>
  syncSendRequest(const GetPlanningScene::Request& request) override
  {
    auto response = scene_client_->syncSendRequest(request);
    if (!response)
      return response;
    if (!context_->robot_model)
      return tl::make_unexpected("No robot model is loaded.");
    if (const auto bounded = clampToBounds(*context_->robot_model, response->scene.robot_state.joint_state); !bounded)
      return tl::make_unexpected(bounded.error());
    return response;
  }

  void cancelRequest() override
  {
    scene_client_->cancelRequest();
  }

private:
  std::shared_ptr<BehaviorContext> context_;
  std::unique_ptr<ClientInterfaceBase<GetPlanningScene>> scene_client_;
};
}  // namespace

PlanToJointGoalFromBoundedStart::PlanToJointGoalFromBoundedStart(
    const std::string& name, const BT::NodeConfiguration& config,
    const std::shared_ptr<BehaviorContext>& shared_resources)
  : PlanToJointGoalFromBoundedStart(
        name, config, shared_resources,
        std::make_unique<moveit_pro::behaviors::RclcppClientInterface<GetPlanningScene>>(shared_resources))
{
}

PlanToJointGoalFromBoundedStart::PlanToJointGoalFromBoundedStart(
    const std::string& name, const BT::NodeConfiguration& config,
    const std::shared_ptr<BehaviorContext>& shared_resources,
    std::unique_ptr<ClientInterfaceBase<GetPlanningScene>> scene_client)
  : PlanToJointGoal(name, config, shared_resources,
                    std::make_unique<BoundedSceneClient>(shared_resources, std::move(scene_client)))
{
}

BT::KeyValueVector PlanToJointGoalFromBoundedStart::metadata()
{
  return { { moveit_pro::behaviors::kSubcategoryMetadataKey, "reBot Bench" },
           { moveit_pro::behaviors::kDescriptionMetadataKey,
             "PlanToJointGoal from the current state clamped into the robot model's position bounds, so a joint "
             "that sagged past a bound plans from the bound. Ports, collision checks and detours are "
             "PlanToJointGoal's." } };
}
}  // namespace rebot_bench_behaviors
