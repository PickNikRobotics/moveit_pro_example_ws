#include <behaviortree_cpp/bt_factory.h>
#include <moveit_studio_behavior_interface/behavior_context.hpp>
#include <moveit_studio_behavior_interface/shared_resources_node_loader.hpp>

#include <pcl_decompose_to_spheres/pcl_decompose_to_spheres.hpp>

#include <pluginlib/class_list_macros.hpp>

namespace pcl_decompose_to_spheres
{
class PclDecomposeToSpheresBehaviorsLoader : public moveit_studio::behaviors::SharedResourcesNodeLoaderBase
{
public:
  void registerBehaviors(BT::BehaviorTreeFactory& factory,
    [[maybe_unused]] const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources) override
  {
    moveit_studio::behaviors::registerBehavior<PclDecomposeToSpheres>(factory, "PclDecomposeToSpheres", shared_resources);
    
  }
};
}  // namespace pcl_decompose_to_spheres

PLUGINLIB_EXPORT_CLASS(pcl_decompose_to_spheres::PclDecomposeToSpheresBehaviorsLoader,
                       moveit_studio::behaviors::SharedResourcesNodeLoaderBase);
