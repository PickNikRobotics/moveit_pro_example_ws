#pragma once

#include <behaviortree_cpp/action_node.h>
#include <moveit_pro_behavior_interface/get_required_ports.hpp>

// This header includes the SharedResourcesNode type
#include <moveit_pro_behavior_interface/shared_resources_node.hpp>

namespace convert_pose_stamped_to_transform_stamped
{
/**
 * @brief Converts a geometry_msgs::msg::PoseStamped message into a geometry_msgs::msg::TransformStamped message.
 */
class ConvertPoseStampedToTransformStamped : public moveit_pro::behaviors::SharedResourcesNode<BT::SyncActionNode>
{
public:
  /**
   * @brief Constructor for the convert_pose_stamped_to_transform_stamped behavior.
   * @param name The name of a particular instance of this Behavior. This will be set by the behavior tree factory when this Behavior is created within a new behavior tree.
   * @param config This contains runtime configuration info for this Behavior, such as the mapping between the Behavior's data ports on the behavior tree's blackboard. This will be set by the behavior tree factory when this Behavior is created within a new behavior tree.
   * @param shared_resources A shared_ptr to a BehaviorContext that is shared among all SharedResourcesNode Behaviors in the behavior tree. This BehaviorContext is owned by the Studio Agent's ObjectiveServerNode.
   * @details An important limitation is that the members of the base Behavior class are not instantiated until after the initialize() function is called, so these classes should not be used within the constructor.
   */
  ConvertPoseStampedToTransformStamped(const std::string& name, const BT::NodeConfiguration& config, const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources);

  /**
   * @brief Implementation of the required providedPorts() function for the convert_pose_stamped_to_transform_stamped Behavior.
   * @details The BehaviorTree.CPP library requires that Behaviors must implement a static function named providedPorts() which defines their input and output ports. If the Behavior does not use any ports, this function must return an empty BT::PortsList.
   * This function returns a list of ports with their names and port info, which is used internally by the behavior tree.
   * @return A BT::PortsList containing the input ports for PoseStamped and child_frame_id, and output port for TransformStamped.
   */
  static BT::PortsList providedPorts();

  /**
   * @brief Implementation of the metadata() function for displaying metadata, such as Behavior description and
   * subcategory, in the MoveIt Studio Developer Tool.
   * @return A BT::KeyValueVector containing the Behavior metadata.
   */
  static BT::KeyValueVector metadata();

  /**
   * @brief Implementation of BT::SyncActionNode::tick() for ConvertPoseStampedToTransformStamped.
   * @details This function is where the Behavior performs its work when the behavior tree is being run. Since ConvertPoseStampedToTransformStamped is derived from BT::SyncActionNode, it is very important that its tick() function always finishes very quickly. If tick() blocks before returning, it will block execution of the entire behavior tree, which may have undesirable consequences for other Behaviors that require a fast update rate to work correctly.
   */
  BT::NodeStatus tick() override;
};
}  // namespace convert_pose_stamped_to_transform_stamped