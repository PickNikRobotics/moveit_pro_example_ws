#include <behaviortree_cpp/bt_factory.h>
#include <moveit_studio_behavior_interface/behavior_context.hpp>
#include <moveit_studio_behavior_interface/shared_resources_node_loader.hpp>

#include <example_behaviors/T_pushing.hpp>

#include <pluginlib/class_list_macros.hpp>

namespace example_behaviors
{
class ExampleBehaviorsLoader : public moveit_studio::behaviors::SharedResourcesNodeLoaderBase
{
public:
  void registerBehaviors(BT::BehaviorTreeFactory& factory,
                         const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources) override
  {
    moveit_studio::behaviors::registerBehavior<T_pushing>(factory, "T_pushing", shared_resources);
  }
};
}  // namespace example_behaviors

PLUGINLIB_EXPORT_CLASS(example_behaviors::ExampleBehaviorsLoader,
                       moveit_studio::behaviors::SharedResourcesNodeLoaderBase);
