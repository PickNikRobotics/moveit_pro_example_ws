#include <behaviortree_cpp/bt_factory.h>
#include <moveit_studio_behavior_interface/behavior_context.hpp>
#include <moveit_studio_behavior_interface/shared_resources_node_loader.hpp>

#include <convert_odom_to_pose_stamped/convert_odom_to_pose_stamped.hpp>

#include <pluginlib/class_list_macros.hpp>

namespace convert_odom_to_pose_stamped
{
class ConvertOdomToPoseStampedBehaviorsLoader : public moveit_studio::behaviors::SharedResourcesNodeLoaderBase
{
public:
  void registerBehaviors(BT::BehaviorTreeFactory& factory,
    [[maybe_unused]] const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources) override
  {
    moveit_studio::behaviors::registerBehavior<ConvertOdomToPoseStamped>(factory, "ConvertOdomToPoseStamped");
    
  }
};
}  // namespace convert_odom_to_pose_stamped

PLUGINLIB_EXPORT_CLASS(convert_odom_to_pose_stamped::ConvertOdomToPoseStampedBehaviorsLoader,
                       moveit_studio::behaviors::SharedResourcesNodeLoaderBase);
