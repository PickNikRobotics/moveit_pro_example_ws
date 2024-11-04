

#pragma once

#include <behaviortree_cpp/action_node.h>
#include <moveit_studio_behavior_interface/shared_resources_node.hpp>
#include <moveit_studio_behavior_interface/behavior_context.hpp>
#include <moveit_studio_behavior_interface/check_for_error.hpp>
#include <moveit_studio_behavior_interface/async_behavior_base.hpp>

namespace example_behaviors
{
/**
 * @brief The SetupMTCWaveHand behavior makes any robot move the end of its arm back and forth.
 * @details This is an example of a behavior that uses MoveIt Task Constructor to configure an MTC task,
 * which can be planned and executed by MoveIt Pro's core PlanMTCTask and ExecuteMTCTask Behaviors.
 * It is derived from AsyncBehaviorBase, an extension of the templated SharedResourcesNode type,
 * which augments the core BehaviorTree.Cpp types with an additional constructor parameter
 * to allow the Behavior to access a rclcpp Node owned by the Agent's ObjectiveServerNode.
 * The AsyncBehaviorBase requires a user-implemented function doWork()
 * which is called within an async process in a separate thread.
 * It returns an tl::expected which contains a bool indicating task success
 * If the task fails, doWork() returns a tl::unexpected which contains a string indicating the error
 * if the process completed successfully or was canceled,
 * or an error message if the process failed unexpectedly.
 */
class SetupMTCWaveHand final : public moveit_studio::behaviors::AsyncBehaviorBase
{
public:
  /**
   * @brief Constructor for the SetupMTCWaveHand behavior.
   * @param name The name of a particular instance of this Behavior. This will be set by the behavior
   * tree factory when this Behavior is created within a new behavior tree.
   * @param shared_resources A shared_ptr to a BehaviorContext that is shared among all
   * SharedResourcesNode Behaviors in the behavior tree. This BehaviorContext is owned by the MoveIt Pro
   * Agent's ObjectiveServerNode.
   * @param config This contains runtime configuration info for this Behavior, such as the mapping
   * between the Behavior's data ports on the behavior tree's blackboard. This will be set by the
   * behavior tree factory when this Behavior is created within a new behavior tree.
   */
  SetupMTCWaveHand(const std::string& name, const BT::NodeConfiguration& config,
                   const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources);

  /**
   * @brief Implementation of the required providedPorts() function for the SetupMTCWaveHand Behavior.
   * @details The BehaviorTree.CPP library requires that Behaviors must implement a static function
   * named providedPorts() which defines their input and output ports. If the Behavior does not use
   * any ports, this function must return an empty BT::PortsList.
   * This function returns a list of ports with their names and port info, which is used internally
   * by the behavior tree.
   * @return SetupMTCWaveHand has one data port: a bidirectional port named "task", which is a shared_ptr
   * to an MTC task object. This function returns a BT::PortsList that declares this single port.
   */
  static BT::PortsList providedPorts();

  /**
   * @brief Implementation of the metadata() function for displaying metadata, such as Behavior description and
   * subcategory, in the MoveIt Studio Developer Tool.
   * @return A BT::KeyValueVector containing the Behavior metadata.
   */
  static BT::KeyValueVector metadata();

private:
  /**
   * @brief Async thread for SetupMTCWaveHand. Adds MTC stages to an MTC task provided on a data port.
   * @details This function is where the Behavior performs its work asynchronously while the behavior tree ticks.
   * It is very important that behaviors return from being ticked very quickly because if it blocks before returning
   * it will block execution of the entire behavior tree, which may have undesirable consequences for other Behaviors
   * that require a fast update rate to work correctly.
   * @return An tl::expected which contains a true if the MTC stages were configured and added to the MTC task,
   * or a string if it failed to retrieve the MTC task from the "task" data port.
   */
  tl::expected<bool, std::string> doWork() override;

  /** @brief Classes derived from AsyncBehaviorBase must implement getFuture() so that it returns a shared_future class member */
  std::shared_future<tl::expected<bool, std::string>>& getFuture() override
  {
    return future_;
  }

  /** @brief Classes derived from AsyncBehaviorBase must have this shared_future as a class member */
  std::shared_future<tl::expected<bool, std::string>> future_;
};
}  // namespace example_behaviors
