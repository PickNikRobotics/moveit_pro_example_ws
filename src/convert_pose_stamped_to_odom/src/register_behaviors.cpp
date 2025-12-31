#include <behaviortree_cpp/bt_factory.h>
#include <moveit_studio_behavior_interface/behavior_context.hpp>
#include <moveit_studio_behavior_interface/shared_resources_node_loader.hpp>

#include <convert_pose_stamped_to_odom/convert_pose_stamped_to_odom.hpp>

#include <pluginlib/class_list_macros.hpp>

namespace convert_pose_stamped_to_odom
{
class ConvertPoseStampedToOdomBehaviorsLoader : public moveit_studio::behaviors::SharedResourcesNodeLoaderBase
{
public:
  void registerBehaviors(BT::BehaviorTreeFactory& factory,
    [[maybe_unused]] const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources) override
  {
    moveit_studio::behaviors::registerBehavior<ConvertPoseStampedToOdom>(factory, "ConvertPoseStampedToOdom");
    
  }
};
}  // namespace convert_pose_stamped_to_odom

PLUGINLIB_EXPORT_CLASS(convert_pose_stamped_to_odom::ConvertPoseStampedToOdomBehaviorsLoader,
                       moveit_studio::behaviors::SharedResourcesNodeLoaderBase);
