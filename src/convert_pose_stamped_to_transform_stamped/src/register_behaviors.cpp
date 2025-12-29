#include <behaviortree_cpp/bt_factory.h>
#include <moveit_studio_behavior_interface/behavior_context.hpp>
#include <moveit_studio_behavior_interface/shared_resources_node_loader.hpp>

#include <convert_pose_stamped_to_transform_stamped/convert_pose_stamped_to_transform_stamped.hpp>

#include <pluginlib/class_list_macros.hpp>

namespace convert_pose_stamped_to_transform_stamped
{
class ConvertPoseStampedToTransformStampedBehaviorsLoader : public moveit_studio::behaviors::SharedResourcesNodeLoaderBase
{
public:
  void registerBehaviors(BT::BehaviorTreeFactory& factory,
    [[maybe_unused]] const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources) override
  {
    moveit_studio::behaviors::registerBehavior<ConvertPoseStampedToTransformStamped>(factory, "ConvertPoseStampedToTransformStamped", shared_resources);
    
  }
};
}  // namespace convert_pose_stamped_to_transform_stamped

PLUGINLIB_EXPORT_CLASS(convert_pose_stamped_to_transform_stamped::ConvertPoseStampedToTransformStampedBehaviorsLoader,
                       moveit_studio::behaviors::SharedResourcesNodeLoaderBase);
