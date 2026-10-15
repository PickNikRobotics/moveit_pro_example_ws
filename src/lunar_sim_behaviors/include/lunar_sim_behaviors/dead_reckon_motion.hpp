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

#include <chrono>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include <behaviortree_cpp/action_node.h>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/time.hpp>

#include <moveit_pro_behavior_interface/shared_resources_node.hpp>

namespace lunar_sim_behaviors
{

struct PlanarPose
{
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
};

struct MotionCommand
{
  geometry_msgs::msg::Twist twist;
  bool done = false;
};

// Control laws, exposed for unit tests.
namespace detail
{
/** @brief Wraps an angle to [-pi, pi]. */
[[nodiscard]] double wrapAngle(double angle);

/** @brief Yaw of a quaternion, about +Z. */
[[nodiscard]] double yawFromQuaternion(const geometry_msgs::msg::Quaternion& q);

/**
 * @brief Velocity that drives `distance` metres forward along the start heading.
 * @details Progress is the displacement projected onto the start heading. Speed tapers toward the end and the yaw
 * error to the start heading is steered out; forward motion pauses while that error exceeds 0.5 rad. `done` is set,
 * with a zero twist, once the remaining distance is within tolerance.
 */
[[nodiscard]] MotionCommand driveDistanceCommand(const PlanarPose& start, const PlanarPose& current, double distance,
                                                 double max_linear_velocity, double max_angular_velocity);

/**
 * @brief Velocity that turns in place until `turned` reaches `angle`, both in radians, positive counter-clockwise.
 * @details The rate tapers toward the target and `done` is set, with a zero twist, once within tolerance.
 */
[[nodiscard]] MotionCommand turnCommand(double turned, double angle, double max_angular_velocity);
}  // namespace detail

/**
 * @brief Base for Behaviors that move a differential-drive base by a measured amount, closing the loop on an
 * odometry estimate (dead reckoning) rather than on commanded durations.
 *
 * @details Subscribes to `odometry_topic`, records the first pose received after start, and publishes a freshly
 * stamped TwistStamped on `cmd_vel_topic` every tick until the derived class reports the motion done. Fails if no
 * odometry arrives, if the odometry stamp grows stale, or if the motion is not done within `timeout_sec`; it publishes
 * a zero twist whenever it stops, and fails on non-finite odometry.
 */
class DeadReckonMotionBase : public moveit_pro::behaviors::SharedResourcesNode<BT::StatefulActionNode>
{
public:
  static constexpr auto kPortIdOdometryTopic = "odometry_topic";
  static constexpr auto kPortIdCmdVelTopic = "cmd_vel_topic";
  static constexpr auto kPortIdFrameId = "frame_id";
  static constexpr auto kPortIdTimeoutSec = "timeout_sec";

  using moveit_pro::behaviors::SharedResourcesNode<BT::StatefulActionNode>::SharedResourcesNode;

  static BT::PortsList commonPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

protected:
  // Reads and validates the derived class's own ports. Returns an error message on failure.
  virtual std::optional<std::string> readMotionPorts() = 0;

  // `turned` is the yaw accumulated since start, unwrapped.
  virtual MotionCommand computeCommand(const PlanarPose& start, const PlanarPose& current, double turned) const = 0;

  // Progress for the timeout message, such as "1.20 of 1.80 m".
  virtual std::string describeProgress(const PlanarPose& start, const PlanarPose& current, double turned) const = 0;

private:
  void publishTwist(const geometry_msgs::msg::Twist& twist) const;
  BT::NodeStatus stop(BT::NodeStatus status);

  std::string odometry_topic_;
  std::string frame_id_;
  std::chrono::duration<double> timeout_{ 0.0 };
  std::chrono::steady_clock::time_point start_time_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_publisher_;

  std::mutex odometry_mutex_;
  std::optional<PlanarPose> latest_pose_;
  rclcpp::Time latest_pose_stamp_;

  std::optional<PlanarPose> start_pose_;
  PlanarPose previous_pose_;
  double turned_ = 0.0;
};

/**
 * @brief Drives a differential-drive base straight ahead by a measured distance, holding the start heading.
 *
 * | Data Port Name       | Port Type | Object Type |
 * |----------------------|-----------|-------------|
 * | distance             | Input     | double      |
 * | max_linear_velocity  | Input     | double      |
 * | max_angular_velocity | Input     | double      |
 * | odometry_topic       | Input     | std::string |
 * | cmd_vel_topic        | Input     | std::string |
 * | frame_id             | Input     | std::string |
 * | timeout_sec          | Input     | double      |
 */
class DriveDistance final : public DeadReckonMotionBase
{
public:
  static constexpr auto kPortIdDistance = "distance";
  static constexpr auto kPortIdMaxLinearVelocity = "max_linear_velocity";
  static constexpr auto kPortIdMaxAngularVelocity = "max_angular_velocity";

  using DeadReckonMotionBase::DeadReckonMotionBase;

  static BT::PortsList providedPorts();
  static BT::KeyValueVector metadata();

private:
  std::optional<std::string> readMotionPorts() override;
  MotionCommand computeCommand(const PlanarPose& start, const PlanarPose& current, double turned) const override;
  std::string describeProgress(const PlanarPose& start, const PlanarPose& current, double turned) const override;

  double distance_ = 0.0;
  double max_linear_velocity_ = 0.0;
  double max_angular_velocity_ = 0.0;
};

/**
 * @brief Turns a differential-drive base in place by a measured angle.
 *
 * | Data Port Name       | Port Type | Object Type |
 * |----------------------|-----------|-------------|
 * | angle                | Input     | double      |
 * | max_angular_velocity | Input     | double      |
 * | odometry_topic       | Input     | std::string |
 * | cmd_vel_topic        | Input     | std::string |
 * | frame_id             | Input     | std::string |
 * | timeout_sec          | Input     | double      |
 */
class TurnInPlace final : public DeadReckonMotionBase
{
public:
  static constexpr auto kPortIdAngle = "angle";
  static constexpr auto kPortIdMaxAngularVelocity = "max_angular_velocity";

  using DeadReckonMotionBase::DeadReckonMotionBase;

  static BT::PortsList providedPorts();
  static BT::KeyValueVector metadata();

private:
  std::optional<std::string> readMotionPorts() override;
  MotionCommand computeCommand(const PlanarPose& start, const PlanarPose& current, double turned) const override;
  std::string describeProgress(const PlanarPose& start, const PlanarPose& current, double turned) const override;

  double angle_ = 0.0;
  double max_angular_velocity_ = 0.0;
};

}  // namespace lunar_sim_behaviors
