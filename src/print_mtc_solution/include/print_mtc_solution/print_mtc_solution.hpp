#pragma once

#include <behaviortree_cpp/action_node.h>
#include <moveit_studio_behavior_interface/async_behavior_base.hpp>
#include <tl_expected/expected.hpp>

#include <future>
#include <string>

namespace print_mtc_solution
{
/**
 * @brief Print MTC introspection information after PlanMTCTask failure.
 */
class PrintMtcSolution : public moveit_studio::behaviors::AsyncBehaviorBase
{
public:
  PrintMtcSolution(const std::string& name, const BT::NodeConfiguration& config,
                   const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources);

  static BT::PortsList providedPorts();
  static BT::KeyValueVector metadata();

  tl::expected<bool, std::string> doWork() override;
  tl::expected<void, std::string> doHalt() override;

private:
  std::shared_future<tl::expected<bool, std::string>>& getFuture() override
  {
    return future_;
  }

  // ---- Members ---------------------------------------------------------------
  std::shared_future<tl::expected<bool, std::string>> future_;
};
}  // namespace print_mtc_solution
