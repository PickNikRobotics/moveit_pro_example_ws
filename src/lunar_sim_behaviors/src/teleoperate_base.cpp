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

#include <lunar_sim_behaviors/teleoperate_base.hpp>

#include <algorithm>

#include <fmt/format.h>

#include <moveit_pro_behavior_interface/get_required_ports.hpp>
#include <moveit_pro_behavior_interface/metadata_fields.hpp>

namespace lunar_sim_behaviors
{
namespace
{
inline constexpr auto kDescriptionTeleoperateBase = R"(
                <p>
                    Drives a differential-drive base's <code>cmd_vel</code> from the VelocityForceCommand stream
                    published on <code>/pose_jog/&lt;planning_group&gt;</code> by the Desktop App's Pose tab,
                    gamepad, and Quest headset teleoperation sources.
                </p>
            )";

constexpr auto kCommandTopicPrefix = "/pose_jog/";

// Use system-defaults QoS, matching PoseJog's VelocityForceCommand subscription.
const auto kQoSProfile = rclcpp::SystemDefaultsQoS();

}  // namespace

geometry_msgs::msg::Twist clampTwistCommand(const moveit_pro_controllers_msgs::msg::VelocityForceCommand& command,
                                            const double max_linear_velocity, const double max_angular_velocity)
{
  geometry_msgs::msg::Twist twist;
  twist.linear.x = std::clamp(command.twist.linear.x, -max_linear_velocity, max_linear_velocity);
  twist.angular.z = std::clamp(command.twist.angular.z, -max_angular_velocity, max_angular_velocity);
  return twist;
}

TeleoperateBase::TeleoperateBase(const std::string& name, const BT::NodeConfiguration& config,
                                 const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
  : moveit_pro::behaviors::SharedResourcesNode<BT::StatefulActionNode>(name, config, shared_resources)
{
}

BT::PortsList TeleoperateBase::providedPorts()
{
  return {
    BT::InputPort<std::string>(kPortIdPlanningGroup, "base",
                               "Planning group whose jog commands to follow. Subscribes to "
                               "/pose_jog/<planning_group>."),
    BT::InputPort<std::string>(kPortIdCmdVelTopic, "/cmd_vel", "Topic to publish the resulting TwistStamped to."),
    BT::InputPort<std::string>(kPortIdFrameId, "footprint", "Frame ID to stamp the published TwistStamped with."),
    BT::InputPort<double>(kPortIdMaxLinearVelocity, 0.8, "Maximum magnitude, in m/s, for the forwarded linear.x."),
    BT::InputPort<double>(kPortIdMaxAngularVelocity, 2.0, "Maximum magnitude, in rad/s, for the forwarded angular.z."),
  };
}

BT::KeyValueVector TeleoperateBase::metadata()
{
  return { { moveit_pro::behaviors::kSubcategoryMetadataKey, "MuJoCo Simulation" },
           { moveit_pro::behaviors::kDescriptionMetadataKey, kDescriptionTeleoperateBase } };
}

BT::NodeStatus TeleoperateBase::onStart()
{
  has_new_command_ = false;

  const auto ports = moveit_pro::behaviors::getRequiredInputs(
      getInput<std::string>(kPortIdPlanningGroup), getInput<std::string>(kPortIdCmdVelTopic),
      getInput<std::string>(kPortIdFrameId), getInput<double>(kPortIdMaxLinearVelocity),
      getInput<double>(kPortIdMaxAngularVelocity));
  if (!ports.has_value())
  {
    getBehaviorContext()->logger->publishFailureMessage(
        name(), "Failed to get required value from input data port: " + ports.error());
    return BT::NodeStatus::FAILURE;
  }
  const auto& [planning_group, cmd_vel_topic, frame_id, max_linear_velocity, max_angular_velocity] = ports.value();
  frame_id_ = frame_id;
  max_linear_velocity_ = max_linear_velocity;
  max_angular_velocity_ = max_angular_velocity;

  const std::string command_topic = fmt::format("{}{}", kCommandTopicPrefix, planning_group);
  command_subscription_ =
      getBehaviorContext()->node->create_subscription<moveit_pro_controllers_msgs::msg::VelocityForceCommand>(
          command_topic, kQoSProfile,
          [this](const moveit_pro_controllers_msgs::msg::VelocityForceCommand::SharedPtr msg) {
            const std::scoped_lock lock(command_mutex_);
            latest_command_ = *msg;
            has_new_command_ = true;
          });

  cmd_vel_publisher_ =
      getBehaviorContext()->node->create_publisher<geometry_msgs::msg::TwistStamped>(cmd_vel_topic, kQoSProfile);

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus TeleoperateBase::onRunning()
{
  moveit_pro_controllers_msgs::msg::VelocityForceCommand command;
  {
    const std::scoped_lock lock(command_mutex_);
    if (!has_new_command_)
    {
      return BT::NodeStatus::RUNNING;
    }
    command = latest_command_;
    has_new_command_ = false;
  }

  // Jazzy diff_drive_controller drops commands older than its cmd_vel_timeout (0.5s), so this message must be
  // created here, with the current time, rather than forwarding the browser's original stamp.
  geometry_msgs::msg::TwistStamped twist_stamped;
  twist_stamped.header.stamp = getBehaviorContext()->node->now();
  twist_stamped.header.frame_id = frame_id_;
  twist_stamped.twist = clampTwistCommand(command, max_linear_velocity_, max_angular_velocity_);
  cmd_vel_publisher_->publish(twist_stamped);

  return BT::NodeStatus::RUNNING;
}

void TeleoperateBase::onHalted()
{
  if (cmd_vel_publisher_)
  {
    geometry_msgs::msg::TwistStamped zero_twist_stamped;
    zero_twist_stamped.header.stamp = getBehaviorContext()->node->now();
    zero_twist_stamped.header.frame_id = frame_id_;
    cmd_vel_publisher_->publish(zero_twist_stamped);
  }
  command_subscription_.reset();
  cmd_vel_publisher_.reset();
}

}  // namespace lunar_sim_behaviors
