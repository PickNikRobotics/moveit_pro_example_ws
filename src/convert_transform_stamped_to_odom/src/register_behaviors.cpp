#include <behaviortree_cpp/bt_factory.h>
#include <moveit_pro_behavior_interface/behavior_context.hpp>
#include <moveit_pro_behavior_interface/shared_resources_node_loader.hpp>

#include <convert_transform_stamped_to_odom/convert_transform_stamped_to_odom.hpp>

#include <pluginlib/class_list_macros.hpp>

namespace convert_transform_stamped_to_odom
{
class ConvertTransformStampedToOdomBehaviorsLoader : public moveit_pro::behaviors::SharedResourcesNodeLoaderBase
{
public:
  void registerBehaviors(BT::BehaviorTreeFactory& factory,
    [[maybe_unused]] const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources) override
  {
    moveit_pro::behaviors::registerBehavior<ConvertTransformStampedToOdom>(factory, "ConvertTransformStampedToOdom", shared_resources);
  }
};
}  // namespace convert_transform_stamped_to_odom

PLUGINLIB_EXPORT_CLASS(convert_transform_stamped_to_odom::ConvertTransformStampedToOdomBehaviorsLoader,
                       moveit_pro::behaviors::SharedResourcesNodeLoaderBase);