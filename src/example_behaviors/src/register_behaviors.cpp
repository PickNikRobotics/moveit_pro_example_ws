#include <behaviortree_cpp/bt_factory.h>
#include <moveit_studio_behavior_interface/behavior_context.hpp>
#include <moveit_studio_behavior_interface/shared_resources_node_loader.hpp>

#include <example_behaviors/hello_world.hpp>
#include <example_behaviors/delayed_message.hpp>
#include <example_behaviors/setup_mtc_wave_hand.hpp>
#include <example_behaviors/add_two_ints_service_client.hpp>
#include <example_behaviors/fibonacci_action_client.hpp>
#include <example_behaviors/get_string_from_topic.hpp>
#include <example_behaviors/publish_color_rgba.hpp>
#include <example_behaviors/setup_mtc_pick_from_pose.hpp>
#include <example_behaviors/setup_mtc_place_from_pose.hpp>

#include <pluginlib/class_list_macros.hpp>

namespace example_behaviors
{
class ExampleBehaviorsLoader : public moveit_studio::behaviors::SharedResourcesNodeLoaderBase
{
public:
  void registerBehaviors(BT::BehaviorTreeFactory& factory,
                         const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources) override
  {
    moveit_studio::behaviors::registerBehavior<HelloWorld>(factory, "HelloWorld", shared_resources);
    moveit_studio::behaviors::registerBehavior<DelayedMessage>(factory, "DelayedMessage", shared_resources);
    moveit_studio::behaviors::registerBehavior<SetupMTCWaveHand>(factory, "SetupMTCWaveHand", shared_resources);
    moveit_studio::behaviors::registerBehavior<GetStringFromTopic>(factory, "GetStringFromTopic", shared_resources);
    moveit_studio::behaviors::registerBehavior<AddTwoIntsServiceClient>(factory, "AddTwoIntsServiceClient",
                                                                        shared_resources);
    moveit_studio::behaviors::registerBehavior<FibonacciActionClient>(factory, "FibonacciActionClient",
                                                                      shared_resources);
    moveit_studio::behaviors::registerBehavior<PublishColorRGBA>(factory, "PublishColorRGBA", shared_resources);
    moveit_studio::behaviors::registerBehavior<SetupMtcPickFromPose>(factory, "SetupMtcPickFromPose", shared_resources);
    moveit_studio::behaviors::registerBehavior<SetupMtcPlaceFromPose>(factory, "SetupMtcPlaceFromPose",
                                                                      shared_resources);
  }
};
}  // namespace example_behaviors

PLUGINLIB_EXPORT_CLASS(example_behaviors::ExampleBehaviorsLoader,
                       moveit_studio::behaviors::SharedResourcesNodeLoaderBase);
