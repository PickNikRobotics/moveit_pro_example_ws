#include <behaviortree_cpp/bt_factory.h>
#include <moveit_studio_behavior_interface/behavior_context.hpp>
#include <moveit_studio_behavior_interface/shared_resources_node_loader.hpp>

#include <print_mtc_solution/print_mtc_solution.hpp>

#include <pluginlib/class_list_macros.hpp>

namespace print_mtc_solution
{
class PrintMtcSolutionBehaviorsLoader : public moveit_studio::behaviors::SharedResourcesNodeLoaderBase
{
public:
  void registerBehaviors(BT::BehaviorTreeFactory& factory,
    [[maybe_unused]] const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources) override
  {
    moveit_studio::behaviors::registerBehavior<PrintMtcSolution>(factory, "PrintMtcSolution", shared_resources);
    
  }
};
}  // namespace print_mtc_solution

PLUGINLIB_EXPORT_CLASS(print_mtc_solution::PrintMtcSolutionBehaviorsLoader,
                       moveit_studio::behaviors::SharedResourcesNodeLoaderBase);
