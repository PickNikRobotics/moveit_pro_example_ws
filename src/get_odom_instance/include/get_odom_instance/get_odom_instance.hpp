#pragma once

#include <shared_mutex>
#include <optional>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <behaviortree_cpp/action_node.h>
#include <moveit_studio_behavior_interface/get_required_ports.hpp>
#include <moveit_studio_behavior_interface/shared_resources_node.hpp>

namespace get_odom_instance
{
/**
 * @brief Subscribes to a single odometry message and publishes it to the blackboard.
 *
 * The behavior exits successfully after the first message is received.
 */
class GetOdomInstance
  : public moveit_studio::behaviors::SharedResourcesNode<BT::StatefulActionNode>
{
public:
  GetOdomInstance(const std::string& name,
                  const BT::NodeConfiguration& config,
                  const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources);

  static BT::PortsList providedPorts();
  static BT::KeyValueVector metadata();

private:
  /** @brief Called once when the behavior starts. Sets up the subscription. */
  BT::NodeStatus onStart() override;

  /**
   * @brief Called while the behavior is running.
   * @return RUNNING until an odometry message is received, then SUCCESS.
   */
  BT::NodeStatus onRunning() override;

  /** @brief Called when the behavior is halted. Cleans up the subscription. */
  void onHalted() override;

  /** @brief Subscriber for the odom topic. */
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;

  /** @brief Mutex for thread-safe access to odometry data. */
  std::shared_mutex odom_mutex_;

  /** @brief Latest received odometry message, if any. */
  std::optional<nav_msgs::msg::Odometry> current_odometry_;
};

}  // namespace get_odom_instance
