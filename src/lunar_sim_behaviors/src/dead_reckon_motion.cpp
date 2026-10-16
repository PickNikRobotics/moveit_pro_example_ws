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

#include <lunar_sim_behaviors/dead_reckon_motion.hpp>

#include <algorithm>
#include <cmath>

#include <fmt/format.h>

#include <moveit_pro_behavior_interface/get_required_ports.hpp>
#include <moveit_pro_behavior_interface/metadata_fields.hpp>

namespace lunar_sim_behaviors
{
namespace
{
inline constexpr auto kDescriptionDriveDistance = R"(
                <p>
                    Drives a differential-drive base straight ahead by a measured distance, holding its start
                    heading. Closes the loop on an odometry estimate, such as wheel odometry fused with an IMU,
                    instead of on commanded durations.
                </p>
            )";
inline constexpr auto kDescriptionTurnInPlace = R"(
                <p>
                    Turns a differential-drive base in place by a measured angle, positive counter-clockwise.
                    Closes the loop on an odometry estimate, such as wheel odometry fused with an IMU, instead of on
                    commanded durations.
                </p>
            )";

constexpr double kDistanceTolerance = 0.01;      // m
constexpr double kLinearGain = 1.0;              // (m/s) per m remaining
constexpr double kMinLinearVelocity = 0.05;      // m/s, so the taper does not stall short of the goal
constexpr double kHeadingGain = 2.0;             // (rad/s) per rad of heading error
constexpr double kMaxDrivingHeadingError = 0.5;  // rad; beyond this, turn back before driving on
constexpr double kAngleTolerance = 0.01;         // rad
constexpr double kAngularGain = 1.5;             // (rad/s) per rad remaining
constexpr double kMinAngularVelocity = 0.15;     // rad/s, so skid-steer scrub does not stall the turn
constexpr std::chrono::duration<double> kOdometryTimeout{ 1.0 };

const auto kQoSProfile = rclcpp::SystemDefaultsQoS();

double taper(const double remaining, const double gain, const double min_velocity, const double max_velocity)
{
  return std::clamp(gain * remaining, std::min(min_velocity, max_velocity), max_velocity);
}

// Displacement from `start` projected onto the start heading.
double progressAlong(const PlanarPose& start, const PlanarPose& current)
{
  return (current.x - start.x) * std::cos(start.yaw) + (current.y - start.y) * std::sin(start.yaw);
}

bool isPositiveFinite(const double value)
{
  return std::isfinite(value) && value > 0.0;
}
}  // namespace

namespace detail
{
double wrapAngle(const double angle)
{
  return std::remainder(angle, 2.0 * M_PI);
}

double yawFromQuaternion(const geometry_msgs::msg::Quaternion& q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

MotionCommand driveDistanceCommand(const PlanarPose& start, const PlanarPose& current, const double distance,
                                   const double max_linear_velocity, const double max_angular_velocity)
{
  const double remaining = distance - progressAlong(start, current);
  MotionCommand command;
  if (remaining <= kDistanceTolerance)
  {
    command.done = true;
    return command;
  }
  const double heading_error = wrapAngle(start.yaw - current.yaw);
  command.twist.linear.x = std::abs(heading_error) > kMaxDrivingHeadingError ?
                               0.0 :
                               taper(remaining, kLinearGain, kMinLinearVelocity, max_linear_velocity);
  command.twist.angular.z = std::clamp(kHeadingGain * heading_error, -max_angular_velocity, max_angular_velocity);
  return command;
}

MotionCommand turnCommand(const double turned, const double angle, const double max_angular_velocity)
{
  const double remaining = angle - turned;
  MotionCommand command;
  if (std::abs(remaining) <= kAngleTolerance)
  {
    command.done = true;
    return command;
  }
  command.twist.angular.z =
      std::copysign(taper(std::abs(remaining), kAngularGain, kMinAngularVelocity, max_angular_velocity), remaining);
  return command;
}
}  // namespace detail

BT::PortsList DeadReckonMotionBase::commonPorts()
{
  return {
    BT::InputPort<std::string>(kPortIdOdometryTopic, "/odom_filtered",
                               "Odometry estimate to close the loop on, in a world-fixed frame."),
    BT::InputPort<std::string>(kPortIdCmdVelTopic, "/cmd_vel", "Topic to publish the TwistStamped commands to."),
    BT::InputPort<std::string>(kPortIdFrameId, "footprint", "Frame ID to stamp the published TwistStamped with."),
    BT::InputPort<double>(kPortIdTimeoutSec, 30.0, "Seconds to allow for the motion before failing."),
  };
}

BT::NodeStatus DeadReckonMotionBase::onStart()
{
  const auto ports = moveit_pro::behaviors::getRequiredInputs(getInput<std::string>(kPortIdOdometryTopic),
                                                              getInput<std::string>(kPortIdCmdVelTopic),
                                                              getInput<std::string>(kPortIdFrameId),
                                                              getInput<double>(kPortIdTimeoutSec));
  if (!ports.has_value())
  {
    getBehaviorContext()->logger->publishFailureMessage(name(), "Failed to get required value from input data port: " +
                                                                    ports.error());
    return BT::NodeStatus::FAILURE;
  }
  const auto& [odometry_topic, cmd_vel_topic, frame_id, timeout_sec] = ports.value();
  if (!isPositiveFinite(timeout_sec))
  {
    getBehaviorContext()->logger->publishFailureMessage(name(), "timeout_sec must be finite and positive.");
    return BT::NodeStatus::FAILURE;
  }
  if (const auto error = readMotionPorts())
  {
    getBehaviorContext()->logger->publishFailureMessage(name(), *error);
    return BT::NodeStatus::FAILURE;
  }
  odometry_topic_ = odometry_topic;
  frame_id_ = frame_id;
  timeout_ = std::chrono::duration<double>(timeout_sec);
  start_time_ = std::chrono::steady_clock::now();
  start_pose_.reset();
  turned_ = 0.0;
  {
    const std::scoped_lock lock(odometry_mutex_);
    latest_pose_.reset();
  }

  odometry_subscription_ = getBehaviorContext()->node->create_subscription<nav_msgs::msg::Odometry>(
      odometry_topic_, kQoSProfile, [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        const std::scoped_lock lock(odometry_mutex_);
        latest_pose_ = PlanarPose{ msg->pose.pose.position.x, msg->pose.pose.position.y,
                                   detail::yawFromQuaternion(msg->pose.pose.orientation) };
        latest_pose_stamp_ = msg->header.stamp;
      });
  // Kept across activations so the zero twist published on stop is not lost to the publisher's destruction.
  const auto& node = getBehaviorContext()->node;
  if (!cmd_vel_publisher_ ||
      node->get_node_topics_interface()->resolve_topic_name(cmd_vel_topic) != cmd_vel_publisher_->get_topic_name())
  {
    cmd_vel_publisher_ = node->create_publisher<geometry_msgs::msg::TwistStamped>(cmd_vel_topic, kQoSProfile);
  }
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus DeadReckonMotionBase::onRunning()
{
  const auto now = std::chrono::steady_clock::now();
  std::optional<PlanarPose> pose;
  rclcpp::Time pose_stamp;
  {
    const std::scoped_lock lock(odometry_mutex_);
    pose = latest_pose_;
    pose_stamp = latest_pose_stamp_;
  }

  if (!pose)
  {
    if (now - start_time_ > kOdometryTimeout)
    {
      getBehaviorContext()->logger->publishFailureMessage(
          name(), fmt::format("No odometry on {} within {:.1f} s. Check that the state estimator is running.",
                              odometry_topic_, kOdometryTimeout.count()));
      return stop(BT::NodeStatus::FAILURE);
    }
    return BT::NodeStatus::RUNNING;
  }
  // The stamp, not the arrival time: an estimator that stalls can keep republishing its last pose.
  const double pose_age = (getBehaviorContext()->node->now() - pose_stamp).seconds();
  if (pose_age > kOdometryTimeout.count())
  {
    getBehaviorContext()->logger->publishFailureMessage(
        name(), fmt::format("Odometry on {} is {:.1f} s old, over the {:.1f} s limit; stopped the base.",
                            odometry_topic_, pose_age, kOdometryTimeout.count()));
    return stop(BT::NodeStatus::FAILURE);
  }
  if (!std::isfinite(pose->x) || !std::isfinite(pose->y) || !std::isfinite(pose->yaw))
  {
    getBehaviorContext()->logger->publishFailureMessage(
        name(), fmt::format("Odometry on {} is not finite; stopped the base.", odometry_topic_));
    return stop(BT::NodeStatus::FAILURE);
  }

  if (!start_pose_)
  {
    start_pose_ = *pose;
    previous_pose_ = *pose;
  }
  turned_ += detail::wrapAngle(pose->yaw - previous_pose_.yaw);
  previous_pose_ = *pose;

  const MotionCommand command = computeCommand(*start_pose_, *pose, turned_);
  if (command.done)
  {
    return stop(BT::NodeStatus::SUCCESS);
  }
  if (now - start_time_ > timeout_)
  {
    getBehaviorContext()->logger->publishFailureMessage(
        name(), fmt::format("Timed out after {:.1f} s at {}; stopped the base. The base may be stuck, or timeout_sec "
                            "may be too short.",
                            timeout_.count(), describeProgress(*start_pose_, *pose, turned_)));
    return stop(BT::NodeStatus::FAILURE);
  }
  publishTwist(command.twist);
  return BT::NodeStatus::RUNNING;
}

void DeadReckonMotionBase::onHalted()
{
  (void)stop(BT::NodeStatus::IDLE);
}

void DeadReckonMotionBase::publishTwist(const geometry_msgs::msg::Twist& twist) const
{
  // diff_drive_controller drops commands older than its cmd_vel_timeout, so every message gets a fresh stamp.
  geometry_msgs::msg::TwistStamped twist_stamped;
  twist_stamped.header.stamp = getBehaviorContext()->node->now();
  twist_stamped.header.frame_id = frame_id_;
  twist_stamped.twist = twist;
  cmd_vel_publisher_->publish(twist_stamped);
}

BT::NodeStatus DeadReckonMotionBase::stop(const BT::NodeStatus status)
{
  if (cmd_vel_publisher_)
  {
    publishTwist(geometry_msgs::msg::Twist());
  }
  odometry_subscription_.reset();
  return status;
}

BT::PortsList DriveDistance::providedPorts()
{
  auto ports = commonPorts();
  ports.insert(BT::InputPort<double>(kPortIdDistance, 1.0, "Distance to drive forward, in metres."));
  ports.insert(BT::InputPort<double>(kPortIdMaxLinearVelocity, 0.3, "Forward speed, in m/s."));
  ports.insert(BT::InputPort<double>(kPortIdMaxAngularVelocity, 0.3,
                                     "Maximum yaw rate, in rad/s, used to hold the start heading."));
  return ports;
}

BT::KeyValueVector DriveDistance::metadata()
{
  return { { moveit_pro::behaviors::kSubcategoryMetadataKey, "Mobile Base" },
           { moveit_pro::behaviors::kDescriptionMetadataKey, kDescriptionDriveDistance } };
}

std::optional<std::string> DriveDistance::readMotionPorts()
{
  const auto ports = moveit_pro::behaviors::getRequiredInputs(getInput<double>(kPortIdDistance),
                                                              getInput<double>(kPortIdMaxLinearVelocity),
                                                              getInput<double>(kPortIdMaxAngularVelocity));
  if (!ports.has_value())
  {
    return "Failed to get required value from input data port: " + ports.error();
  }
  const auto& [distance, max_linear_velocity, max_angular_velocity] = ports.value();
  if (!isPositiveFinite(distance) || !isPositiveFinite(max_linear_velocity) || !isPositiveFinite(max_angular_velocity))
  {
    return "distance, max_linear_velocity and max_angular_velocity must be finite and positive.";
  }
  distance_ = distance;
  max_linear_velocity_ = max_linear_velocity;
  max_angular_velocity_ = max_angular_velocity;
  return std::nullopt;
}

MotionCommand DriveDistance::computeCommand(const PlanarPose& start, const PlanarPose& current, double /*turned*/) const
{
  return detail::driveDistanceCommand(start, current, distance_, max_linear_velocity_, max_angular_velocity_);
}

std::string DriveDistance::describeProgress(const PlanarPose& start, const PlanarPose& current, double /*turned*/) const
{
  return fmt::format("{:.2f} of {:.2f} m", progressAlong(start, current), distance_);
}

BT::PortsList TurnInPlace::providedPorts()
{
  auto ports = commonPorts();
  ports.insert(BT::InputPort<double>(kPortIdAngle, M_PI_2, "Angle to turn, in radians, positive counter-clockwise."));
  ports.insert(BT::InputPort<double>(kPortIdMaxAngularVelocity, 0.5, "Turn rate, in rad/s."));
  return ports;
}

BT::KeyValueVector TurnInPlace::metadata()
{
  return { { moveit_pro::behaviors::kSubcategoryMetadataKey, "Mobile Base" },
           { moveit_pro::behaviors::kDescriptionMetadataKey, kDescriptionTurnInPlace } };
}

std::optional<std::string> TurnInPlace::readMotionPorts()
{
  const auto ports = moveit_pro::behaviors::getRequiredInputs(getInput<double>(kPortIdAngle),
                                                              getInput<double>(kPortIdMaxAngularVelocity));
  if (!ports.has_value())
  {
    return "Failed to get required value from input data port: " + ports.error();
  }
  const auto& [angle, max_angular_velocity] = ports.value();
  if (!std::isfinite(angle) || !isPositiveFinite(max_angular_velocity))
  {
    return "angle must be finite and max_angular_velocity finite and positive.";
  }
  angle_ = angle;
  max_angular_velocity_ = max_angular_velocity;
  return std::nullopt;
}

MotionCommand TurnInPlace::computeCommand(const PlanarPose& /*start*/, const PlanarPose& /*current*/,
                                          const double turned) const
{
  return detail::turnCommand(turned, angle_, max_angular_velocity_);
}

std::string TurnInPlace::describeProgress(const PlanarPose& /*start*/, const PlanarPose& /*current*/,
                                          const double turned) const
{
  return fmt::format("{:.1f} of {:.1f} deg", turned * 180.0 / M_PI, angle_ * 180.0 / M_PI);
}

}  // namespace lunar_sim_behaviors
