#include <behaviortree_cpp/bt_factory.h>
#include <moveit_studio_behavior_interface/behavior_context.hpp>
#include <moveit_studio_behavior_interface/shared_resources_node_loader.hpp>

#include <custom_mpc_behavior/custom_mpc_behavior.hpp>

#include <pluginlib/class_list_macros.hpp>

namespace custom_mpc_behavior
{
class CustomMpcBehaviorBehaviorsLoader : public moveit_studio::behaviors::SharedResourcesNodeLoaderBase
{
public:
  void registerBehaviors(
      BT::BehaviorTreeFactory& factory,
      [[maybe_unused]] const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources) override
  {
    moveit_studio::behaviors::registerBehavior<CustomMpcBehavior>(factory, "CustomMpcBehavior", shared_resources);
  }
};
}  // namespace custom_mpc_behavior

PLUGINLIB_EXPORT_CLASS(custom_mpc_behavior::CustomMpcBehaviorBehaviorsLoader,
                       moveit_studio::behaviors::SharedResourcesNodeLoaderBase);
