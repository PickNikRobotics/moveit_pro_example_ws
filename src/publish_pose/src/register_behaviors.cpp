#include <behaviortree_cpp/bt_factory.h>
#include <moveit_studio_behavior_interface/behavior_context.hpp>
#include <moveit_studio_behavior_interface/shared_resources_node_loader.hpp>

#include <publish_pose/publish_pose.hpp>

#include <pluginlib/class_list_macros.hpp>

namespace publish_pose
{
class PublishPoseBehaviorsLoader : public moveit_studio::behaviors::SharedResourcesNodeLoaderBase
{
public:
  void registerBehaviors(BT::BehaviorTreeFactory& factory,
    [[maybe_unused]] const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources) override
  {
    moveit_studio::behaviors::registerBehavior<PublishPose>(factory, "PublishPose", shared_resources);
    
  }
};
}  // namespace publish_pose

PLUGINLIB_EXPORT_CLASS(publish_pose::PublishPoseBehaviorsLoader,
                       moveit_studio::behaviors::SharedResourcesNodeLoaderBase);
