#pragma once

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/bt_factory.h>
#include <spdlog/spdlog.h>
#include <yaml-cpp/yaml.h>
#include <Eigen/Core>
#include <cartesian_planning/trajectory_utils.hpp>
#include <moveit/robot_model_loader/robot_model_loader.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit_pro_behavior_interface/behavior_context.hpp>
#include <moveit_pro_behavior_interface/check_for_error.hpp>
#include <moveit_pro_behavior_interface/shared_resources_node.hpp>
#include <moveit_task_constructor_msgs/msg/solution.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

namespace example_behaviors
{
/**
 * @brief Converts a MoveIt Task Constructor Solution into a JointTrajectory.
 *
 * @details
 * | Data Port Name    | Port Type     | Object Type                                             |
 * | ----------------- |---------------|---------------------------------------------------------|
 * | solution          | Input         | moveit_task_constructor_msgs::msg::Solution             |
 * | joint_group       | Input         | std::string                                             |
 * | velocity_scaling_factor | Input   | double                                                  |
 * | acceleration_scaling_factor | Input | double                                               |
 * | sampling_rate     | Input         | int                                                     |
 * | joint_trajectory  | Output        | trajectory_msgs::msg::JointTrajectory                   |
 */
class ExampleConvertMtcSolutionToJointTrajectory final
  : public moveit_studio::behaviors::SharedResourcesNode<BT::SyncActionNode>
{
public:
  ExampleConvertMtcSolutionToJointTrajectory(
      const std::string& name, const BT::NodeConfiguration& config,
      const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources);

  static BT::PortsList providedPorts();

  static BT::KeyValueVector metadata();

  BT::NodeStatus tick() override;

private:
  std::unique_ptr<robot_model_loader::RobotModelLoader> robot_model_loader_;

  const std::vector<Eigen::VectorXd>& extractJointPositions(const moveit_task_constructor_msgs::msg::Solution& solution);
};
}  // namespace example_behaviors
