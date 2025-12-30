#pragma once

#include <behaviortree_cpp/action_node.h>
#include <moveit_studio_behavior_interface/get_required_ports.hpp>
#include <moveit_studio_behavior_interface/shared_resources_node.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace transform_odom_with_pose
{

/**
 * @brief Transform an Odometry message into a different reference frame using
 *        a PoseStamped-defined transform.
 *
 * @details
 * This Behavior applies a transform defined by a geometry_msgs::msg::PoseStamped
 * to an input nav_msgs::msg::Odometry message. The pose component is fully
 * transformed, while the twist component is rotated (not translated).
 *
 * This Behavior is synchronous and intended for lightweight, deterministic
 * frame transformations (e.g., logging, frame alignment, or data conditioning).
 */
class TransformOdomWithPose
  : public moveit_studio::behaviors::SharedResourcesNode<BT::SyncActionNode>
{
public:
  /**
   * @brief Constructor for the TransformOdomWithPose Behavior.
   *
   * @param name The name of this Behavior instance in the behavior tree.
   * @param config Runtime configuration provided by the behavior tree factory,
   *               including port mappings.
   * @param shared_resources Shared BehaviorContext owned by the Studio Agent.
   *
   * @note Members of the base Behavior class are not initialized until after
   * initialize() is called. They must not be used in the constructor.
   */
  TransformOdomWithPose(
      const std::string& name,
      const BT::NodeConfiguration& config,
      const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources);

  /**
   * @brief Define the input and output ports for this Behavior.
   *
   * Ports:
   * - Input:  nav_msgs::msg::Odometry        ("odom_in")
   * - Input:  geometry_msgs::msg::PoseStamped ("transform_pose")
   * - Output: nav_msgs::msg::Odometry        ("odom_out")
   *
   * @return The list of ports used by this Behavior.
   */
  static BT::PortsList providedPorts();

  /**
   * @brief Metadata for display in the MoveIt Studio Developer Tool.
   *
   * @return A key-value vector describing the Behavior.
   */
  static BT::KeyValueVector metadata();

  /**
   * @brief Execute the odometry transformation.
   *
   * @details
   * This function retrieves the input odometry and transform pose from the
   * behavior tree, applies the transform, and outputs the transformed odometry.
   * Since this is a synchronous action, it must complete quickly and must not
   * block.
   *
   * @return BT::NodeStatus::SUCCESS on successful transformation,
   *         BT::NodeStatus::FAILURE otherwise.
   */
  BT::NodeStatus tick() override;
};

}  // namespace transform_odom_with_pose
