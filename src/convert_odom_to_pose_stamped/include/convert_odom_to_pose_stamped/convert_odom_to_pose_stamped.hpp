#pragma once

#include <behaviortree_cpp/action_node.h>
#include <moveit_pro_behavior_interface/get_required_ports.hpp>





namespace convert_odom_to_pose_stamped
{
/**
 * @brief TODO(...)
 */
class ConvertOdomToPoseStamped : public BT::SyncActionNode
{
public:
  /**
   * @brief Constructor for the convert_odom_to_pose_stamped behavior.
   * @param name The name of a particular instance of this Behavior. This will be set by the behavior tree factory when this Behavior is created within a new behavior tree.
   * @param config This contains runtime configuration info for this Behavior, such as the mapping between the Behavior's data ports on the behavior tree's blackboard. This will be set by the behavior tree factory when this Behavior is created within a new behavior tree.
   * @details An important limitation is that the members of the base Behavior class are not instantiated until after the initialize() function is called, so these classes should not be used within the constructor.
   */
  ConvertOdomToPoseStamped(const std::string& name, const BT::NodeConfiguration& config);

  
  /**
   * @brief Implementation of the required providedPorts() function for the convert_odom_to_pose_stamped Behavior.
   * @details The BehaviorTree.CPP library requires that Behaviors must implement a static function named providedPorts() which defines their input and output ports. If the Behavior does not use any ports, this function must return an empty BT::PortsList.
   * This function returns a list of ports with their names and port info, which is used internally by the behavior tree.
   * @return convert_odom_to_pose_stamped does not expose any ports, so this function returns an empty list.
   */
  static BT::PortsList providedPorts();

  /**
   * @brief Implementation of the metadata() function for displaying metadata, such as Behavior description and
   * subcategory, in the MoveIt Studio Developer Tool.
   * @return A BT::KeyValueVector containing the Behavior metadata.
   */
  static BT::KeyValueVector metadata();

  /**
   * @brief Implementation of BT::SyncActionNode::tick() for ConvertOdomToPoseStamped.
   * @details This function is where the Behavior performs its work when the behavior tree is being run. Since ConvertOdomToPoseStamped is derived from BT::SyncActionNode, it is very important that its tick() function always finishes very quickly. If tick() blocks before returning, it will block execution of the entire behavior tree, which may have undesirable consequences for other Behaviors that require a fast update rate to work correctly.
   */
  BT::NodeStatus tick() override;
  
};
}  // namespace convert_odom_to_pose_stamped
