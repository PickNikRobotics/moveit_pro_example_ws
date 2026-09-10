// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include <behaviortree_cpp/bt_factory.h>
#include <moveit_pro_behavior_interface/behavior_context.hpp>
#include <moveit_pro_behavior_interface/shared_resources_node_loader.hpp>

#include <hangar_sim_behaviors/call_empty_service.hpp>
#include <hangar_sim_behaviors/project_pose_to_plane.hpp>
#include <hangar_sim_behaviors/scan_match_residual.hpp>
#include <hangar_sim_behaviors/seed_localization_at_pose.hpp>

#include <pluginlib/class_list_macros.hpp>

namespace hangar_sim_behaviors
{
class HangarSimBehaviorsLoader : public moveit_pro::behaviors::SharedResourcesNodeLoaderBase
{
public:
  void registerBehaviors(BT::BehaviorTreeFactory& factory,
                         const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources) override
  {
    moveit_pro::behaviors::registerBehavior<CallEmptyService>(factory, "CallEmptyService", shared_resources);
    moveit_pro::behaviors::registerBehavior<ProjectPoseToPlane>(factory, "ProjectPoseToPlane", shared_resources);
    moveit_pro::behaviors::registerBehavior<ScanMatchResidual>(factory, "ScanMatchResidual", shared_resources);
    moveit_pro::behaviors::registerBehavior<SeedLocalizationAtPose>(factory, "SeedLocalizationAtPose", shared_resources);
  }
};
}  // namespace hangar_sim_behaviors

PLUGINLIB_EXPORT_CLASS(hangar_sim_behaviors::HangarSimBehaviorsLoader,
                       moveit_pro::behaviors::SharedResourcesNodeLoaderBase);
