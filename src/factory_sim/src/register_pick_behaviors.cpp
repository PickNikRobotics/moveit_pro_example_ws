// Copyright 2026 PickNik Inc.
// SPDX-License-Identifier: BSD-3-Clause

#include <factory_sim/get_joint_state_from_topic.hpp>
#include <factory_sim/is_bracket_pick_start_current.hpp>
#include <factory_sim/is_lookahead_enabled.hpp>
#include <factory_sim/pad_planning_scene_for_motion.hpp>
#include <moveit_pro_behavior_interface/shared_resources_node_loader.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace factory_sim
{
class PickBehaviorsLoader final : public moveit_pro::behaviors::SharedResourcesNodeLoaderBase
{
public:
  void registerBehaviors(BT::BehaviorTreeFactory& factory,
                         const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& resources) override
  {
    moveit_pro::behaviors::registerBehavior<PadPlanningSceneForMotion>(factory, "PadPlanningSceneForMotion", resources);
    moveit_pro::behaviors::registerBehavior<IsBracketPickStartCurrent>(factory, "IsBracketPickStartCurrent");
    moveit_pro::behaviors::registerBehavior<IsLookaheadEnabled>(factory, "IsLookaheadEnabled");
    moveit_pro::behaviors::registerBehavior<GetJointStateFromTopic>(factory, "GetJointStateFromTopic", resources);
  }
};
}  // namespace factory_sim

PLUGINLIB_EXPORT_CLASS(factory_sim::PickBehaviorsLoader, moveit_pro::behaviors::SharedResourcesNodeLoaderBase);
