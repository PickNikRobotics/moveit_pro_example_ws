#include <behaviortree_cpp/bt_factory.h>
#include <moveit_studio_behavior_interface/behavior_context.hpp>
#include <moveit_studio_behavior_interface/shared_resources_node_loader.hpp>

#include <graspnet_example_behavior/graspnet_example_behavior.hpp>

#include <pluginlib/class_list_macros.hpp>

namespace graspnet_example_behavior
{
class GraspnetExampleBehaviorBehaviorsLoader : public moveit_studio::behaviors::SharedResourcesNodeLoaderBase
{
public:
  void registerBehaviors(BT::BehaviorTreeFactory& factory,
    [[maybe_unused]] const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources) override
  {
    moveit_studio::behaviors::registerBehavior<GraspnetExampleBehavior>(factory, "GraspnetExampleBehavior", shared_resources);
    
  }
};
}  // namespace graspnet_example_behavior

PLUGINLIB_EXPORT_CLASS(graspnet_example_behavior::GraspnetExampleBehaviorBehaviorsLoader,
                       moveit_studio::behaviors::SharedResourcesNodeLoaderBase);
