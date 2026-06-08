#pragma once

#include <behaviortree_cpp/action_node.h>
#include <moveit_pro_behavior_interface/behavior_context.hpp>
#include <moveit_pro_behavior_interface/shared_resources_node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

namespace pi0_behaviors
{
/**
 * @brief Waits for a JointTrajectory message on /pi0_trajectory and outputs it.
 *
 * An external pi0 bridge node publishes trajectories computed by the pi0 policy
 * server to /pi0_trajectory. This behavior receives one trajectory per tick and
 * passes it downstream to ExecuteTrajectory.
 *
 * | Data Port Name       | Port Type | Object Type                              |
 * | -------------------- | --------- | ---------------------------------------- |
 * | joint_trajectory_msg | Output    | trajectory_msgs::msg::JointTrajectory    |
 */
class GetPi0Trajectory final
  : public moveit_pro::behaviors::SharedResourcesNode<BT::StatefulActionNode>
{
public:
  GetPi0Trajectory(const std::string& name, const BT::NodeConfiguration& config,
                   const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr sub_;
  trajectory_msgs::msg::JointTrajectory::SharedPtr latest_;
};

}  // namespace pi0_behaviors
