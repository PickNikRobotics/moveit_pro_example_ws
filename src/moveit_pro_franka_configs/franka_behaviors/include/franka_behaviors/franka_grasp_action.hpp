// Copyright 2023 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <franka_msgs/action/grasp.hpp>
#include <franka_msgs/action/move.hpp>
#include <moveit_pro_behavior_interface/action_client_behavior_base.hpp>
#include <moveit_pro_behavior_interface/shared_resources_node.hpp>

namespace franka_behaviors
{
using Grasp = franka_msgs::action::Grasp;

/**
 * @brief Actuate a gripper through its driver node's Grasp action. Given the name of the action topic and a
 * target gripper position, move the gripper to the specified position.
 *
 * @details
 * | Data Port Name              | Port Type | Object Type |
 * | --------------------------- |-----------|-------------|
 * | gripper_action_name         | Input     | std::string |
 * | timeout                     | Input     | double      |
 * | width                       | Input     | double      |
 * | speed                       | Input     | double      |
 * | force                       | Input     | double      |
 */
class GraspAction final : public moveit_pro::behaviors::ActionClientBehaviorBase<Grasp>
{
public:
  /**
   * @brief Constructor for GraspAction behavior.
   * @param name Name of the node. Must match the name used for this node in the behavior tree definition file (the .xml
   * file).
   * @param config Node configuration. Only used here because the BehaviorTree.CPP expects constructor signature with
   * name and config first before custom constructor parameters.
   * @param shared_resources Provides access to common resources such as the node handle and failure logger that are
   * shared between all the behaviors that inherit from moveit_pro::behaviors::SharedResourcesNode.
   */
  GraspAction(const std::string& name, const BT::NodeConfiguration& config,
              const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources);

  /**
   * @brief Custom tree nodes that have input and/or output ports must define them in this static function.
   * @details This function must be static. It is a requirement set by the BehaviorTree.CPP library.
   * @return List of ports with the names and port info. The return value is for the internal use of the behavior tree.
   */
  static BT::PortsList providedPorts();

  static BT::KeyValueVector metadata();

private:
  /**
   * @brief Defines the action name by getting the value from the `gripper_command_action_name` input data port.
   * @return The name of the action for the Grasp action client.
   */
  tl::expected<std::string, std::string> getActionName() override;

  /**
   * @brief Sets the maximum duration to wait for the gripper action goal to succeed before failing.
   * @details This returns a constant duration which isn't configurable by the end user. The main purpose of this timeout
   * is to make sure objective execution does not get stuck forever if the gripper action server becomes unresponsive.
   * @return The timeout duration.
   */
  tl::expected<std::chrono::duration<double>, std::string> getResultTimeout() override;

  /**
   * @brief Creates the action goal by getting the target gripper position from the `position` input data port.
   * @return Returns the action goal message if successful.
   */
  tl::expected<Grasp::Goal, std::string> createGoal() override;

  /** @brief Classes derived from ActionClientBehaviorBase must implement getFuture() so that it returns a shared_future class member */
  std::shared_future<tl::expected<bool, std::string>>& getFuture() override
  {
    return future_;
  }

  /** @brief Classes derived from ActionClientBehaviorBase must have this shared_future as a class member */
  std::shared_future<tl::expected<bool, std::string>> future_;
};
}  // namespace franka_behaviors
