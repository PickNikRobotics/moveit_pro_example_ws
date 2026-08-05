// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include <behaviortree_cpp/bt_factory.h>
#include <moveit_pro_behavior_interface/behavior_context.hpp>
#include <moveit_pro_behavior_interface/shared_resources_node_loader.hpp>

#include <lab_sim_behaviors/compute_tray_place_positions_using_apriltags.hpp>
#include <lab_sim_behaviors/generate_surface_coverage_path.hpp>
#include <lab_sim_behaviors/get_mask2d_from_region.hpp>

#include <pluginlib/class_list_macros.hpp>

namespace lab_sim_behaviors
{
class LabSimBehaviorsLoader : public moveit_pro::behaviors::SharedResourcesNodeLoaderBase
{
public:
  void registerBehaviors(BT::BehaviorTreeFactory& factory,
                         const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources) override
  {
    moveit_pro::behaviors::registerBehavior<ComputeTrayPlacePositionsUsingAprilTags>(
        factory, "ComputeTrayPlacePositionsUsingAprilTags", shared_resources);
    moveit_pro::behaviors::registerBehavior<GenerateSurfaceCoveragePath>(factory, "GenerateSurfaceCoveragePath",
                                                                         shared_resources);
    moveit_pro::behaviors::registerBehavior<GetMask2DFromRegion>(factory, "GetMask2DFromRegion", shared_resources);
  }
};
}  // namespace lab_sim_behaviors

PLUGINLIB_EXPORT_CLASS(lab_sim_behaviors::LabSimBehaviorsLoader, moveit_pro::behaviors::SharedResourcesNodeLoaderBase);
