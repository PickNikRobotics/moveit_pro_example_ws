#include <behaviortree_cpp/bt_factory.h>
#include <moveit_pro_behavior_interface/behavior_context.hpp>
#include <moveit_pro_behavior_interface/shared_resources_node_loader.hpp>

#include <lunar_sim_behaviors/publish_twist_stamped.hpp>

#include <pluginlib/class_list_macros.hpp>

namespace lunar_sim_behaviors
{
class LunarSimBehaviorsLoader : public moveit_pro::behaviors::SharedResourcesNodeLoaderBase
{
public:
  void registerBehaviors(BT::BehaviorTreeFactory& factory,
                         const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources) override
  {
    moveit_pro::behaviors::registerBehavior<PublishTwistStamped>(factory, "PublishTwistStamped", shared_resources);
  }
};
}  // namespace lunar_sim_behaviors

PLUGINLIB_EXPORT_CLASS(lunar_sim_behaviors::LunarSimBehaviorsLoader,
                       moveit_pro::behaviors::SharedResourcesNodeLoaderBase);
