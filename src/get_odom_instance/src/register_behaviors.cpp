#include <behaviortree_cpp/bt_factory.h>
#include <moveit_pro_behavior_interface/behavior_context.hpp>
#include <moveit_pro_behavior_interface/shared_resources_node_loader.hpp>

#include <get_odom_instance/get_odom_instance.hpp>

#include <pluginlib/class_list_macros.hpp>

namespace get_odom_instance
{
class GetOdomInstanceBehaviorsLoader : public moveit_pro::behaviors::SharedResourcesNodeLoaderBase
{
public:
  void registerBehaviors(
      BT::BehaviorTreeFactory& factory,
      [[maybe_unused]] const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources) override
  {
    moveit_pro::behaviors::registerBehavior<GetOdomInstance>(factory, "GetOdomInstance", shared_resources);
  }
};
}  // namespace get_odom_instance

PLUGINLIB_EXPORT_CLASS(get_odom_instance::GetOdomInstanceBehaviorsLoader,
                       moveit_pro::behaviors::SharedResourcesNodeLoaderBase);
