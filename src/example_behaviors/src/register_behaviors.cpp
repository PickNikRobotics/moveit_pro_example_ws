#include <behaviortree_cpp/bt_factory.h>
#include <moveit_studio_behavior_interface/behavior_context.hpp>
#include <moveit_studio_behavior_interface/json_serialization.hpp>
#include <moveit_studio_behavior_interface/shared_resources_node_loader.hpp>
#include <std_msgs/msg/string.hpp>

#include <example_behaviors/example_hello_world.hpp>

#include <pluginlib/class_list_macros.hpp>

namespace example_behaviors
{
class ExampleBehaviorsLoader : public moveit_studio::behaviors::SharedResourcesNodeLoaderBase
{
public:
  void registerBehaviors(BT::BehaviorTreeFactory& factory,
                         const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources) override
  {
    moveit_studio::behaviors::registerBehavior<ExampleHelloWorld>(factory, "ExampleHelloWorld", shared_resources);
    // Register ROS messages for blackboard viewer.
    register_ros_msg<std_msgs::msg::String>();
  }
};
}  // namespace example_behaviors

PLUGINLIB_EXPORT_CLASS(example_behaviors::ExampleBehaviorsLoader,
                       moveit_studio::behaviors::SharedResourcesNodeLoaderBase);
