#include <behaviortree_cpp/bt_factory.h>
#include <moveit_studio_behavior_interface/behavior_context.hpp>
#include <moveit_studio_behavior_interface/shared_resources_node_loader.hpp>

#include <distance_to_collision/distance_to_collision.hpp>

#include <pluginlib/class_list_macros.hpp>

namespace distance_to_collision
{
class DistanceToCollisionBehaviorsLoader : public moveit_studio::behaviors::SharedResourcesNodeLoaderBase
{
public:
  void registerBehaviors(
      BT::BehaviorTreeFactory& factory,
      [[maybe_unused]] const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources) override
  {
    moveit_studio::behaviors::registerBehavior<DistanceToCollision>(factory, "DistanceToCollision", shared_resources);
  }
};
}  // namespace distance_to_collision

PLUGINLIB_EXPORT_CLASS(distance_to_collision::DistanceToCollisionBehaviorsLoader,
                       moveit_studio::behaviors::SharedResourcesNodeLoaderBase);
