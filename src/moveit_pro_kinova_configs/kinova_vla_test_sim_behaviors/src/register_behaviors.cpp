// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include <behaviortree_cpp/bt_factory.h>
#include <moveit_pro_behavior_interface/behavior_context.hpp>
#include <moveit_pro_behavior_interface/shared_resources_node_loader.hpp>

#include <kinova_vla_test_sim_behaviors/compute_top_down_keyposes.hpp>
#include <kinova_vla_test_sim_behaviors/plan_joint_spline_through_poses.hpp>
#include <kinova_vla_test_sim_behaviors/send_gripper_command.hpp>

#include <pluginlib/class_list_macros.hpp>

namespace kinova_vla_test_sim_behaviors
{
class KinovaVlaTestSimBehaviorsLoader : public moveit_pro::behaviors::SharedResourcesNodeLoaderBase
{
public:
  void registerBehaviors(BT::BehaviorTreeFactory& factory,
                         const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources) override
  {
    moveit_pro::behaviors::registerBehavior<ComputeTopDownKeyposes>(factory, "ComputeTopDownKeyposes", shared_resources);
    moveit_pro::behaviors::registerBehavior<PlanJointSplineThroughPoses>(factory, "PlanJointSplineThroughPoses",
                                                                         shared_resources);
    moveit_pro::behaviors::registerBehavior<SendGripperCommand>(factory, "SendGripperCommand", shared_resources);
  }
};
}  // namespace kinova_vla_test_sim_behaviors

PLUGINLIB_EXPORT_CLASS(kinova_vla_test_sim_behaviors::KinovaVlaTestSimBehaviorsLoader,
                       moveit_pro::behaviors::SharedResourcesNodeLoaderBase);
