#pragma once

#include <behaviortree_cpp/action_node.h>
#include <moveit_pro_behavior_interface/get_required_ports.hpp>
#include <moveit_pro_behavior_interface/shared_resources_node.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/publisher.hpp>

namespace publish_odom
{

/**
 * @brief Publish a nav_msgs::msg::Odometry message to a ROS 2 topic.
 *
 * @details
 * This Behavior publishes an Odometry message provided via a behavior tree
 * input port to a specified topic. The publisher is created lazily on the
 * first tick and reused on subsequent ticks.
 *
 * This Behavior is synchronous and must return quickly.
 */
class PublishOdom
  : public moveit_pro::behaviors::SharedResourcesNode<BT::SyncActionNode>
{
public:
  /**
   * @brief Constructor for the PublishOdom Behavior.
   *
   * @param name The name of this Behavior instance in the behavior tree.
   * @param config Runtime configuration provided by the behavior tree factory.
   * @param shared_resources Shared BehaviorContext owned by the Studio Agent.
   */
  PublishOdom(
      const std::string& name,
      const BT::NodeConfiguration& config,
      const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources);

  /**
   * @brief Define the input ports for the PublishOdom Behavior.
   *
   * Ports:
   * - Input: nav_msgs::msg::Odometry ("message")
   * - Input: std::string             ("topic_name")
   * - Input: size_t                  ("queue_size")
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
   * @brief Publish the odometry message.
   *
   * @return BT::NodeStatus::SUCCESS if published successfully,
   *         BT::NodeStatus::FAILURE otherwise.
   */
  BT::NodeStatus tick() override;

private:
  /// Cached publisher (created on first tick)
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;

  /// Cached topic name to detect changes
  std::string topic_name_;
};

}  // namespace publish_odom
