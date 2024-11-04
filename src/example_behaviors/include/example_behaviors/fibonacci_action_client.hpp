#pragma once

#include <moveit_studio_behavior_interface/action_client_behavior_base.hpp>
#include <example_interfaces/action/fibonacci.hpp>

using moveit_studio::behaviors::ActionClientBehaviorBase;
using moveit_studio::behaviors::BehaviorContext;
using Fibonacci = example_interfaces::action::Fibonacci;

namespace example_behaviors
{
class FibonacciActionClient final : public ActionClientBehaviorBase<Fibonacci>
{
public:
  FibonacciActionClient(const std::string& name, const BT::NodeConfiguration& config,
                        const std::shared_ptr<BehaviorContext>& shared_resources);

  /** @brief Implementation of the required providedPorts() function for the hello_world Behavior. */
  static BT::PortsList providedPorts();

  /**
   * @brief Implementation of the metadata() function for displaying metadata, such as Behavior description and
   * subcategory, in the MoveIt Studio Developer Tool.
   * @return A BT::KeyValueVector containing the Behavior metadata.
   */
  static BT::KeyValueVector metadata();

private:
  /** @brief User-provided function to get the name of the action when initializing the action client. */
  tl::expected<std::string, std::string> getActionName() override;

  /** @brief User-provided function to create the action goal before sending the action goal request. */
  tl::expected<Fibonacci::Goal, std::string> createGoal() override;

  /** @brief Optional user-provided function to process the action result after the action has finished. */
  tl::expected<bool, std::string> processResult(const std::shared_ptr<Fibonacci::Result> result) override;

  /** @brief Optional user-provided function to process feedback sent by the action server. */
  void processFeedback(const std::shared_ptr<const Fibonacci::Feedback> feedback) override;

  /** @brief Classes derived from AsyncBehaviorBase must implement getFuture() so that it returns a shared_future class member */
  std::shared_future<tl::expected<bool, std::string>>& getFuture() override
  {
    return future_;
  }

  /** @brief Classes derived from AsyncBehaviorBase must have this shared_future as a class member */
  std::shared_future<tl::expected<bool, std::string>> future_;
};
}  // namespace example_behaviors
