#include <behaviortree_cpp/bt_factory.h>
#include <moveit_pro_behavior_interface/behavior_context.hpp>
#include <moveit_pro_behavior_interface/shared_resources_node_loader.hpp>
#include <pi0_behaviors/get_pi0_trajectory.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace pi0_behaviors
{
class Pi0BehaviorsLoader : public moveit_pro::behaviors::SharedResourcesNodeLoaderBase
{
public:
  void registerBehaviors(BT::BehaviorTreeFactory& factory,
                         const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources) override
  {
    moveit_pro::behaviors::registerBehavior<GetPi0Trajectory>(factory, "GetPi0Trajectory", shared_resources);
  }
};
}  // namespace pi0_behaviors

PLUGINLIB_EXPORT_CLASS(pi0_behaviors::Pi0BehaviorsLoader, moveit_pro::behaviors::SharedResourcesNodeLoaderBase);
