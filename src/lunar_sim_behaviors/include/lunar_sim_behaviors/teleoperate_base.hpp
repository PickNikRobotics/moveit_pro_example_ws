// Copyright 2026 PickNik Inc.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the PickNik Inc. nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#pragma once

#include <mutex>
#include <string>

#include <behaviortree_cpp/action_node.h>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>

#include <moveit_pro_behavior_interface/shared_resources_node.hpp>
#include <moveit_pro_controllers_msgs/msg/velocity_force_command.hpp>

namespace lunar_sim_behaviors
{

/**
 * @brief Converts a VelocityForceCommand into a clamped planar Twist (linear.x, angular.z only).
 * @details Pure function, no ROS dependency, so it can be unit tested directly.
 */
[[nodiscard]] geometry_msgs::msg::Twist clampTwistCommand(
    const moveit_pro_controllers_msgs::msg::VelocityForceCommand& command, double max_linear_velocity,
    double max_angular_velocity);

/**
 * @brief Drives a differential-drive base's `/cmd_vel` from the same VelocityForceCommand stream the Desktop App's
 * Pose tab, gamepad, and Quest headset already publish for teleoperated jogging.
 *
 * @details Subscribes to `/pose_jog/<planning_group>` and forwards `linear.x`/`angular.z`, clamped to the configured
 * limits, as a freshly-restamped `geometry_msgs::msg::TwistStamped` on `cmd_vel_topic`. All other twist components are
 * dropped. Modeled on the forward-with-restamp half of `PoseJog`; unlike `PoseJog` this Behavior does no collision
 * checking and controls a single planning group directly by topic name.
 *
 * | Data Port Name       | Port Type | Object Type |
 * |-----------------------|-----------|-------------|
 * | planning_group        | Input     | std::string |
 * | cmd_vel_topic         | Input     | std::string |
 * | frame_id              | Input     | std::string |
 * | max_linear_velocity   | Input     | double      |
 * | max_angular_velocity  | Input     | double      |
 */
class TeleoperateBase : public moveit_pro::behaviors::SharedResourcesNode<BT::StatefulActionNode>
{
public:
  static constexpr auto kPortIdPlanningGroup = "planning_group";
  static constexpr auto kPortIdCmdVelTopic = "cmd_vel_topic";
  static constexpr auto kPortIdFrameId = "frame_id";
  static constexpr auto kPortIdMaxLinearVelocity = "max_linear_velocity";
  static constexpr auto kPortIdMaxAngularVelocity = "max_angular_velocity";

  TeleoperateBase(const std::string& name, const BT::NodeConfiguration& config,
                  const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources);

  static BT::PortsList providedPorts();

  static BT::KeyValueVector metadata();

  // Creates the VelocityForceCommand subscription and the TwistStamped publisher. Returns RUNNING.
  BT::NodeStatus onStart() override;

  // Forwards the latest command to cmd_vel, restamped, whenever a new one has arrived since the last forward.
  BT::NodeStatus onRunning() override;

  // Publishes one zero twist, then drops the subscription and publisher.
  void onHalted() override;

private:
  std::string frame_id_;
  double max_linear_velocity_ = 0.0;
  double max_angular_velocity_ = 0.0;

  rclcpp::Subscription<moveit_pro_controllers_msgs::msg::VelocityForceCommand>::SharedPtr command_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_publisher_;

  std::mutex command_mutex_;
  moveit_pro_controllers_msgs::msg::VelocityForceCommand latest_command_;
  bool has_new_command_ = false;
};

}  // namespace lunar_sim_behaviors
