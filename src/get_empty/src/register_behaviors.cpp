#include <behaviortree_cpp/bt_factory.h>
#include <moveit_pro_behavior_interface/behavior_context.hpp>
#include <moveit_pro_behavior_interface/shared_resources_node_loader.hpp>
#include <moveit_pro_behavior_interface/impl/get_message_from_topic_impl.hpp>

#include <get_empty/get_empty.hpp>

#include <pluginlib/class_list_macros.hpp>

// Explicitly instantiate the template before registration
template class moveit_pro::behaviors::GetMessageFromTopicBehaviorBase<std_msgs::msg::Empty>;

namespace get_empty
{
class GetEmptyBehaviorsLoader : public moveit_pro::behaviors::SharedResourcesNodeLoaderBase
{
public:
  void registerBehaviors(BT::BehaviorTreeFactory& factory,
    [[maybe_unused]] const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources) override
  {
    moveit_pro::behaviors::registerBehavior<GetEmpty>(factory, "GetEmpty", shared_resources);
  }
};
}  // namespace get_empty

PLUGINLIB_EXPORT_CLASS(get_empty::GetEmptyBehaviorsLoader,
                       moveit_pro::behaviors::SharedResourcesNodeLoaderBase);