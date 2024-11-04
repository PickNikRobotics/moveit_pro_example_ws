#pragma once

#include <behaviortree_cpp/action_node.h>

// This header includes the SharedResourcesNode type
#include <moveit_studio_behavior_interface/shared_resources_node.hpp>

#include <moveit_msgs/msg/move_it_error_codes.hpp>

#include <moveit_studio_behavior_interface/check_for_error.hpp>

namespace example_behaviors
{
/**
 * @brief DelayedMessage will use FailureLoggerROS to log a "Hello World" message after the duration specified on the input port
 */
class DelayedMessage : public moveit_studio::behaviors::SharedResourcesNode<BT::StatefulActionNode>
{
private:
  std::chrono::time_point<std::chrono::steady_clock> start_time_;
  double delay_duration_;

public:
  /**
   * @brief Constructor for the delayed_message behavior.
   * @param name The name of a particular instance of this Behavior. This will be set by the behavior tree factory when
   * this Behavior is created within a new behavior tree.
   * @param shared_resources A shared_ptr to a BehaviorContext that is shared among all SharedResourcesNode Behaviors in
   * the behavior tree. This BehaviorContext is owned by the Studio Agent's ObjectiveServerNode.
   * @param config This contains runtime configuration info for this Behavior, such as the mapping between the
   * Behavior's data ports on the behavior tree's blackboard. This will be set by the behavior tree factory when this
   * Behavior is created within a new behavior tree.
   * @details An important limitation is that the members of the base Behavior class are not instantiated until after
   * the initialize() function is called, so these classes should not be used within the constructor.
   */
  DelayedMessage(const std::string& name, const BT::NodeConfiguration& config,
                 const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources);

  /**
   * @brief Implementation of the required providedPorts() function for the delayed_message Behavior.
   * @details The BehaviorTree.CPP library requires that Behaviors must implement a static function named
   * providedPorts() which defines their input and output ports. If the Behavior does not use any ports, this function
   * must return an empty BT::PortsList. This function returns a list of ports with their names and port info, which is
   * used internally by the behavior tree.
   * @return If delayed_message does not expose any ports, this function returns an empty list.
   */
  static BT::PortsList providedPorts();

  /**
   * @brief Implementation of the metadata() function for displaying metadata, such as Behavior description and
   * subcategory, in the MoveIt Studio Developer Tool.
   * @return A BT::KeyValueVector containing the Behavior metadata.
   */
  static BT::KeyValueVector metadata();

  /**
   * @brief Implementation of onStart(). Runs when the Behavior is ticked for the first time.
   * @return Always returns BT::NodeStatus::RUNNING, since the success of Behavior's initialization is checked in @ref
   * onRunning().
   */
  BT::NodeStatus onStart() override;

  /**
   * @brief Implementation of onRunning(). Checks the status of the Behavior when it is ticked after it starts running.
   * @return BT::NodeStatus::RUNNING, BT::NodeStatus::SUCCESS, or BT::NodeStatus::FAILURE depending on the Behavior status.
   */
  BT::NodeStatus onRunning() override;

  void onHalted() override;
};
}  // namespace example_behaviors
