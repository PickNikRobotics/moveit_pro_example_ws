#include <behaviortree_cpp/bt_factory.h>
#include <moveit_pro_behavior_interface/behavior_context.hpp>
#include <moveit_pro_behavior_interface/shared_resources_node_loader.hpp>

#include <transform_odom_with_pose/transform_odom_with_pose.hpp>

#include <pluginlib/class_list_macros.hpp>

namespace transform_odom_with_pose
{
class TransformOdomWithPoseBehaviorsLoader : public moveit_pro::behaviors::SharedResourcesNodeLoaderBase
{
public:
  void registerBehaviors(BT::BehaviorTreeFactory& factory,
    [[maybe_unused]] const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources) override
  {
    moveit_pro::behaviors::registerBehavior<TransformOdomWithPose>(factory, "TransformOdomWithPose", shared_resources);
    
  }
};
}  // namespace transform_odom_with_pose

PLUGINLIB_EXPORT_CLASS(transform_odom_with_pose::TransformOdomWithPoseBehaviorsLoader,
                       moveit_pro::behaviors::SharedResourcesNodeLoaderBase);
