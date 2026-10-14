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
#include <moveit_pro_behavior_interface/shared_resources_node_loader.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rebot_bench_behaviors/anchor_trajectory_to_command.hpp>
#include <rebot_bench_behaviors/fit_cube_to_mask.hpp>
#include <rebot_bench_behaviors/hold_gripper_in_trajectory.hpp>
#include <rebot_bench_behaviors/is_joint_at_least.hpp>
#include <rebot_bench_behaviors/load_motion_scales.hpp>
#include <rebot_bench_behaviors/record_rebot_trajectory.hpp>
#include <rebot_bench_behaviors/report_unreachable_cube.hpp>

namespace rebot_bench_behaviors
{
class RebotBenchBehaviorsLoader : public moveit_pro::behaviors::SharedResourcesNodeLoaderBase
{
public:
  void registerBehaviors(BT::BehaviorTreeFactory& factory,
                         const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& context) override
  {
    moveit_pro::behaviors::registerBehavior<RecordRebotTrajectory>(factory, "RecordRebotTrajectory", context);
    moveit_pro::behaviors::registerBehavior<AnchorTrajectoryToCommand>(factory, "AnchorTrajectoryToCommand", context);
    moveit_pro::behaviors::registerBehavior<FitCubeToMask>(factory, "FitCubeToMask", context);
    moveit_pro::behaviors::registerBehavior<HoldGripperInTrajectory>(factory, "HoldGripperInTrajectory", context);
    moveit_pro::behaviors::registerBehavior<IsJointAtLeast>(factory, "IsJointAtLeast", context);
    moveit_pro::behaviors::registerBehavior<LoadMotionScales>(factory, "LoadMotionScales", context);
    moveit_pro::behaviors::registerBehavior<ReportUnreachableCube>(factory, "ReportUnreachableCube", context);
  }
};
}  // namespace rebot_bench_behaviors
PLUGINLIB_EXPORT_CLASS(rebot_bench_behaviors::RebotBenchBehaviorsLoader,
                       moveit_pro::behaviors::SharedResourcesNodeLoaderBase)
