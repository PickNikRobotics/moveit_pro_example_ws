#pragma once

#include <behaviortree_cpp/action_node.h>

// This header includes the SharedResourcesNode type
#include "OsqpEigen/OsqpEigen.h"

#include <moveit_studio_behavior_interface/async_behavior_base.hpp>
#include <moveit_studio_behavior_interface/shared_resources_node.hpp>

#include <rclcpp/publisher.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace franka_behaviors
{
/**
 * @brief The T_pushing Behavior uses FailureLoggerROS to log a "Hello World" message and will always
 * return SUCCESS
 */
class T_pushing final : public moveit_studio::behaviors::SharedResourcesNode<BT::StatefulActionNode>
{
public:
  /**
   * @brief Constructor for the hello_world behavior.
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
  T_pushing(const std::string& name, const BT::NodeConfiguration& config,
            const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources);

  /**
   * @brief Implementation of the required providedPorts() function for the hello_world Behavior.
   * @details The BehaviorTree.CPP library requires that Behaviors must implement a static function named
   * providedPorts() which defines their input and output ports. If the Behavior does not use any ports, this function
   * must return an empty BT::PortsList. This function returns a list of ports with their names and port info, which is
   * used internally by the behavior tree.
   * @return hello_world does not expose any ports, so this function returns an empty list.
   */
  static BT::PortsList providedPorts();

  /**
   * @brief Implementation of the metadata() function for displaying metadata, such as Behavior description and
   * subcategory, in the MoveIt Studio Developer Tool.
   * @return A BT::KeyValueVector containing the Behavior metadata.
   */
  static BT::KeyValueVector metadata();

  // Validate inputs and register a timer callback that will publish the command at a fixed rate.
  BT::NodeStatus onStart() override;

  // Check if a halt was requested, and delete the timer callback if so.
  BT::NodeStatus onRunning() override;

  // Flag that a halt was requested.
  void onHalted() override;

private:
  Eigen::Vector3d get_velocities(const Eigen::Vector3d& rp, const Eigen::Vector3d& vp, const Eigen::Vector3d& N);
  std::tuple<double, Eigen::Vector3d> get_cost(const Eigen::Vector3d& rp, const Eigen::Vector3d& vp,
                                               const Eigen::Vector3d& N, const Eigen::Isometry3d& target_com_transform);

  std::shared_ptr<rclcpp::Publisher<visualization_msgs::msg::MarkerArray>> marker_publisher_;

  Eigen::Matrix<double, 3, 3> H;
  Eigen::Vector<double, 3> c;
  Eigen::Vector<double, 3> com_;

  Eigen::Vector3d best_rp_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d best_vp_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d best_N_ = Eigen::Vector3d::Zero();
  rclcpp::Time alpha_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  rclcpp::Time last_update_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
};
}  // namespace franka_behaviors
