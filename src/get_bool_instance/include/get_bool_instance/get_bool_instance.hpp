#pragma once

#include <shared_mutex>
#include <optional>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

#include <behaviortree_cpp/action_node.h>
#include <moveit_pro_behavior_interface/get_required_ports.hpp>
#include <moveit_pro_behavior_interface/shared_resources_node.hpp>

namespace get_bool_instance
{
/**
 * @brief Subscribes to a single boolean message and publishes it to the blackboard.
 *
 * The behavior exits successfully after the first message is received.
 */
class GetBoolInstance
  : public moveit_pro::behaviors::SharedResourcesNode<BT::StatefulActionNode>
{
public:
  GetBoolInstance(const std::string& name,
                  const BT::NodeConfiguration& config,
                  const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources);

  static BT::PortsList providedPorts();
  static BT::KeyValueVector metadata();

private:
  /** @brief Called once when the behavior starts. Sets up the subscription. */
  BT::NodeStatus onStart() override;

  /**
   * @brief Called while the behavior is running.
   * @return RUNNING until a boolean message is received, then SUCCESS.
   */
  BT::NodeStatus onRunning() override;

  /** @brief Called when the behavior is halted. Cleans up the subscription. */
  void onHalted() override;

  /** @brief Subscriber for the bool topic. */
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr bool_subscriber_;

  /** @brief Mutex for thread-safe access to boolean data. */
  std::shared_mutex bool_mutex_;

  /** @brief Latest received boolean message, if any. */
  std::optional<std_msgs::msg::Bool> current_bool_;

  std::string bool_topic_name_;
};

}  // namespace get_bool_instance
