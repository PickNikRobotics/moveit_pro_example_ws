#include <behaviortree_cpp/bt_factory.h>
#include <moveit_pro_behavior_interface/behavior_context.hpp>
#include <moveit_pro_behavior_interface/shared_resources_node_loader.hpp>

#include <franka_behaviors/franka_grasp_action.hpp>

#include <pluginlib/class_list_macros.hpp>

namespace franka_behaviors
{
class FrankaBehaviorsLoader : public moveit_studio::behaviors::SharedResourcesNodeLoaderBase
{
public:
  void registerBehaviors(BT::BehaviorTreeFactory& factory,
                         const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources) override
  {
    moveit_studio::behaviors::registerBehavior<GraspAction>(factory, "FrankaGraspAction", shared_resources);
  }
};
}  // namespace franka_behaviors

PLUGINLIB_EXPORT_CLASS(franka_behaviors::FrankaBehaviorsLoader, moveit_studio::behaviors::SharedResourcesNodeLoaderBase);
