#include <behaviortree_cpp/bt_factory.h>
#include <moveit_pro_behavior_interface/behavior_context.hpp>
#include <moveit_pro_behavior_interface/json_serialization.hpp>
#include <moveit_pro_behavior_interface/shared_resources_node_loader.hpp>
#include <std_msgs/msg/string.hpp>

#include <example_behaviors/example_add_two_ints_service_client.hpp>
#include <example_behaviors/example_convert_mtc_solution_to_joint_trajectory.hpp>
#include <example_behaviors/example_create_string_msg.hpp>
#include <example_behaviors/example_delayed_message.hpp>
#include <example_behaviors/example_fibonacci_action_client.hpp>
#include <example_behaviors/example_get_string_from_topic.hpp>
#include <example_behaviors/example_hello_world.hpp>
#include <example_behaviors/example_ndt_registration.hpp>
#include <example_behaviors/example_publish_color_rgba.hpp>
#include <example_behaviors/example_ransac_registration.hpp>
#include <example_behaviors/example_sam2_segmentation.hpp>
#include <example_behaviors/example_setup_mtc_wave_hand.hpp>

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
    moveit_studio::behaviors::registerBehavior<ExampleConvertMtcSolutionToJointTrajectory>(
        factory, "ExampleConvertMtcSolutionToJointTrajectory", shared_resources);
    moveit_studio::behaviors::registerBehavior<ExampleDelayedMessage>(factory, "ExampleDelayedMessage",
                                                                      shared_resources);
    moveit_studio::behaviors::registerBehavior<ExampleSetupMTCWaveHand>(factory, "ExampleSetupMTCWaveHand",
                                                                        shared_resources);
    moveit_studio::behaviors::registerBehavior<ExampleGetStringFromTopic>(factory, "ExampleGetStringFromTopic",
                                                                          shared_resources);
    moveit_studio::behaviors::registerBehavior<ExampleAddTwoIntsServiceClient>(
        factory, "ExampleAddTwoIntsServiceClient", shared_resources);
    moveit_studio::behaviors::registerBehavior<ExampleFibonacciActionClient>(factory, "ExampleFibonacciActionClient",
                                                                             shared_resources);
    moveit_studio::behaviors::registerBehavior<ExamplePublishColorRGBA>(factory, "ExamplePublishColorRGBA",
                                                                        shared_resources);
    moveit_studio::behaviors::registerBehavior<ExampleNDTRegistration>(factory, "ExampleNDTRegistration",
                                                                       shared_resources);
    moveit_studio::behaviors::registerBehavior<ExampleRANSACRegistration>(factory, "ExampleRANSACRegistration",
                                                                          shared_resources);
    moveit_studio::behaviors::registerBehavior<ExampleSAM2Segmentation>(factory, "ExampleSAM2Segmentation",
                                                                        shared_resources);
    moveit_studio::behaviors::registerBehavior<ExampleCreateStringMsg>(factory, "ExampleCreateStringMsg",
                                                                       shared_resources);
    // Register ROS messages for blackboard viewer.
    register_ros_msg<std_msgs::msg::String>();
  }
};
}  // namespace example_behaviors

PLUGINLIB_EXPORT_CLASS(example_behaviors::ExampleBehaviorsLoader,
                       moveit_studio::behaviors::SharedResourcesNodeLoaderBase);
