#include <behaviortree_cpp/bt_factory.h>
#include <moveit_studio_behavior_interface/behavior_context.hpp>
#include <moveit_studio_behavior_interface/shared_resources_node_loader.hpp>

#include <detect_aruco_tag/detect_aruco_tag.hpp>

#include <pluginlib/class_list_macros.hpp>

namespace detect_aruco_tag
{
class DetectArucoTagBehaviorsLoader : public moveit_studio::behaviors::SharedResourcesNodeLoaderBase
{
public:
  void registerBehaviors(
      BT::BehaviorTreeFactory& factory,
      [[maybe_unused]] const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources) override
  {
    moveit_studio::behaviors::registerBehavior<DetectArucoTag>(factory, "DetectArucoTag", shared_resources);
  }
};
}  // namespace detect_aruco_tag

PLUGINLIB_EXPORT_CLASS(detect_aruco_tag::DetectArucoTagBehaviorsLoader,
                       moveit_studio::behaviors::SharedResourcesNodeLoaderBase);
