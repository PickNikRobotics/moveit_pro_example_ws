#include <behaviortree_cpp/bt_factory.h>
#include <moveit_studio_behavior_interface/behavior_context.hpp>
#include <moveit_studio_behavior_interface/shared_resources_node_loader.hpp>
#include "get_filenames_from_directory/push_back_vector.hpp"

#include <get_filenames_from_directory/get_filenames_from_directory.hpp>

#include <pluginlib/class_list_macros.hpp>

namespace get_filenames_from_directory
{
class GetFilenamesFromDirectoryBehaviorsLoader : public moveit_studio::behaviors::SharedResourcesNodeLoaderBase
{
public:
  void registerBehaviors(
      BT::BehaviorTreeFactory& factory,
      [[maybe_unused]] const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources) override
  {
    moveit_studio::behaviors::registerBehavior<GetFilenamesFromDirectory>(factory, "GetFilenamesFromDirectory");
    moveit_studio::behaviors::registerBehavior<moveit_studio::behaviors::PushBackVector>(factory, "PushBackVector",
                                                                                         shared_resources);
  }
};
}  // namespace get_filenames_from_directory

PLUGINLIB_EXPORT_CLASS(get_filenames_from_directory::GetFilenamesFromDirectoryBehaviorsLoader,
                       moveit_studio::behaviors::SharedResourcesNodeLoaderBase);
