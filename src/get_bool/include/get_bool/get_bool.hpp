#pragma once

#include <shared_mutex>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

#include <behaviortree_cpp/action_node.h>
#include <moveit_studio_behavior_interface/get_required_ports.hpp>

// This header includes the SharedResourcesNode type
#include <moveit_studio_behavior_interface/shared_resources_node.hpp>

namespace get_bool
{
/**
 * @brief Subscribes to a std_msgs::msg::Bool topic and outputs the boolean value to the blackboard.
 */
class GetBool : public moveit_studio::behaviors::SharedResourcesNode<BT::StatefulActionNode>
{
public:
  /**
   * @brief Constructor for the get_bool behavior.
   * @param name The name of a particular instance of this Behavior. This will be set by the behavior tree factory when this Behavior is created within a new behavior tree.
   * @param config This contains runtime configuration info for this Behavior, such as the mapping between the Behavior's data ports on the behavior tree's blackboard. This will be set by the behavior tree factory when this Behavior is created within a new behavior tree.
   * @param shared_resources A shared_ptr to a BehaviorContext that is shared among all SharedResourcesNode Behaviors in the behavior tree. This BehaviorContext is owned by the Studio Agent's ObjectiveServerNode.
   * @details An important limitation is that the members of the base Behavior class are not instantiated until after the initialize() function is called, so these classes should not be used within the constructor.
   */
  GetBool(const std::string& name, const BT::NodeConfiguration& config, const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources);

  /**
   * @brief Implementation of the required providedPorts() function for the get_bool Behavior.
   * @details The BehaviorTree.CPP library requires that Behaviors must implement a static function named providedPorts() which defines their input and output ports. If the Behavior does not use any ports, this function must return an empty BT::PortsList.
   * This function returns a list of ports with their names and port info, which is used internally by the behavior tree.
   * @return A BT::PortsList containing the input port for the topic name and output port for the boolean value.
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
   * @brief Called when the behavior is started. Sets up the subscription to the bool topic.
   * @return BT::NodeStatus::RUNNING if successful, BT::NodeStatus::FAILURE otherwise.
   */
  BT::NodeStatus onStart() override;

  /**
   * @brief Called while the behavior is running. Updates the output port with the current boolean value.
   * @return BT::NodeStatus::RUNNING to continue execution.
   */
  BT::NodeStatus onRunning() override;

  /**
   * @brief Called when the behavior is halted. Cleans up the subscription.
   */
  void onHalted() override;

  /** @brief Subscriber for the bool topic. */
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr bool_subscriber_;

  /** @brief Mutex for thread-safe access to the current bool value. */
  std::shared_mutex bool_mutex_;

  /** @brief The most recently received boolean value. */
  bool current_bool_value_{ false };
};
}  // namespace get_bool