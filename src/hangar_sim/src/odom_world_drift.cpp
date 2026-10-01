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

#include <hangar_sim/odom_world_drift.hpp>

#include <algorithm>
#include <cmath>
#include <iterator>

#include <tf2/utils.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace hangar_sim
{
namespace
{
tf2::Transform poseOf(const nav_msgs::msg::Odometry& m)
{
  tf2::Transform t;
  tf2::fromMsg(m.pose.pose, t);
  return t;
}
}  // namespace

tf2::Transform odomToWorld(const tf2::Transform& est, const tf2::Transform& truth)
{
  const tf2::Transform drift = est * truth.inverse();
  // Truth is planar, but fuse solves in full 3D, so the SE(3) difference is not. Project it: see
  // the header for why odom -> world has to stay planar.
  tf2::Quaternion yaw_only;
  yaw_only.setRPY(0.0, 0.0, tf2::getYaw(drift.getRotation()));
  return tf2::Transform(yaw_only, tf2::Vector3(drift.getOrigin().x(), drift.getOrigin().y(), 0.0));
}

geometry_msgs::msg::TransformStamped toTransform(const tf2::Transform& odom_to_world, const rclcpp::Time& stamp)
{
  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp = stamp;
  tf.header.frame_id = "odom";
  tf.child_frame_id = "world";
  tf.transform = tf2::toMsg(odom_to_world);
  return tf;
}

bool isStale(double est_age_sec)
{
  return est_age_sec > kEstStaleSec;
}

bool isTeleport(const tf2::Transform& prev, const tf2::Transform& next, double dt_sec)
{
  const double dt = std::clamp(dt_sec, 0.0, kMaxSampleGapSec);
  const tf2::Transform step = prev.inverse() * next;
  return std::hypot(step.getOrigin().x(), step.getOrigin().y()) > kTeleportJumpM + kMaxBaseSpeedMps * dt ||
         std::abs(tf2::getYaw(step.getRotation())) > kTeleportJumpRad + kMaxBaseYawRateRps * dt;
}

bool TruthHistory::add(const rclcpp::Time& stamp, const tf2::Transform& pose)
{
  const bool rewound = !samples_.empty() && stamp < samples_.back().stamp;
  if (rewound)
  {
    samples_.clear();
  }
  samples_.push_back({ stamp, pose });
  while (samples_.size() > kTruthHistoryMax ||
         (samples_.size() > 1 && (stamp - samples_.front().stamp).seconds() > kTruthHistorySec))
  {
    samples_.pop_front();
  }
  return !rewound;
}

std::optional<tf2::Transform> TruthHistory::at(const rclcpp::Time& when) const
{
  if (samples_.empty() || when < samples_.front().stamp)
  {
    return std::nullopt;
  }
  if (when >= samples_.back().stamp)
  {
    return (when - samples_.back().stamp).seconds() <= kEstAheadToleranceSec ? std::optional(samples_.back().pose) :
                                                                               std::nullopt;
  }
  // front <= when < back, so `after` exists, `before` exists, and before.stamp <= when < after.stamp.
  const auto after = std::upper_bound(samples_.begin(), samples_.end(), when,
                                      [](const rclcpp::Time& t, const Sample& s) { return t < s.stamp; });
  const auto before = std::prev(after);
  const double fraction = (when - before->stamp).seconds() / (after->stamp - before->stamp).seconds();
  // slerp takes the short way round, so a pair straddling +/-pi does not spin the long way.
  return tf2::Transform(before->pose.getRotation().slerp(after->pose.getRotation(), fraction),
                        before->pose.getOrigin().lerp(after->pose.getOrigin(), fraction));
}

OdomWorldDrift::OdomWorldDrift(std::shared_ptr<rclcpp::Node> node) : node_(std::move(node)), tf_broadcaster_(*node_)
{
  // Only the latest estimate is used; truth is kept as a short stamped history (see TruthHistory).
  est_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
      "/odom_filtered", 10, [this](const nav_msgs::msg::Odometry::ConstSharedPtr& m) { onEst(*m); });
  truth_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", rclcpp::SensorDataQoS(), [this](const nav_msgs::msg::Odometry::ConstSharedPtr& m) { onTruth(*m); });
  fuse_set_pose_ = node_->create_client<fuse_msgs::srv::SetPose>("/state_estimator/set_pose");
  initial_pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 1);
  amcl_pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/pose", 10, [this](const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr& m) { onAmclPose(*m); });
  // Node clock, not wall clock, so the tick advances on sim time under use_sim_time.
  timer_ = rclcpp::create_timer(node_, node_->get_clock(), rclcpp::Duration::from_seconds(kPubPeriod),
                                [this] { publish(); });
}

void OdomWorldDrift::onEst(const nav_msgs::msg::Odometry& msg)
{
  est_ = poseOf(msg);
  // Arrival time, not the sender's stamp: staleness means how long we have gone without one.
  est_stamp_ = node_->get_clock()->now();
  // The sender's stamp, separately: this is the instant the estimate describes, and it is what
  // truth has to be sampled at for the difference to mean anything.
  est_msg_stamp_ = rclcpp::Time(msg.header.stamp);
}

void OdomWorldDrift::onTruth(const nav_msgs::msg::Odometry& msg)
{
  // A reset teleports truth (or rewinds sim time): drop the stale estimate and history; a teleport also resets both.
  const rclcpp::Time stamp(msg.header.stamp);
  const tf2::Transform truth = poseOf(msg);
  const bool teleported =
      !truth_.empty() && isTeleport(truth_.newest(), truth, (stamp - truth_.newestStamp()).seconds());
  if (teleported)
  {
    truth_.clear();
  }
  const bool rewound = !truth_.add(stamp, truth);
  if (teleported || rewound)
  {
    est_.reset();
  }
  if (teleported)
  {
    resetFuse(truth);
  }
}

void OdomWorldDrift::resetFuse(const tf2::Transform& truth)
{
  const uint64_t generation = ++reseed_generation_;
  reseed_ = Reseed{ generation, node_->get_clock()->now(), std::nullopt, std::nullopt };
  if (!fuse_set_pose_->service_is_ready())
  {
    abandonReseed(std::string(fuse_set_pose_->get_service_name()) + " is not available");
    return;
  }
  auto request = std::make_shared<fuse_msgs::srv::SetPose::Request>();
  request->pose.header.stamp = node_->get_clock()->now();
  request->pose.header.frame_id = "odom";
  tf2::toMsg(truth, request->pose.pose.pose);
  auto& covariance = request->pose.pose.covariance;  // x, y, z, roll, pitch, yaw
  covariance[0] = kReseedXYVariance;
  covariance[7] = kReseedXYVariance;
  covariance[14] = kReseedZVariance;
  covariance[21] = kReseedTiltVariance;
  covariance[28] = kReseedTiltVariance;
  covariance[35] = kReseedYawVariance;
  fuse_set_pose_->async_send_request(
      request, [this, generation](rclcpp::Client<fuse_msgs::srv::SetPose>::SharedFuture future) {
        if (!reseed_.has_value() || reseed_->generation != generation)
        {
          return;  // a later teleport superseded this request, or its deadline already passed
        }
        const auto& response = future.get();
        if (!response->success)
        {
          abandonReseed("fuse rejected set_pose (" + response->message + ")");
          return;
        }
        reseed_->fuse_reset_at = node_->get_clock()->now();
      });
}

void OdomWorldDrift::abandonReseed(const std::string& why)
{
  reseed_.reset();
  RCLCPP_ERROR(node_->get_logger(),
               "sim teleport detected but %s -- fuse and AMCL may keep their pre-teleport estimates; "
               "re-localize with a 2D pose estimate before navigating.",
               why.c_str());
}

void OdomWorldDrift::onAmclPose(const geometry_msgs::msg::PoseWithCovarianceStamped& msg)
{
  if (reseed_.has_value() && reseed_->broadcast_at.has_value() &&
      rclcpp::Time(msg.header.stamp) > reseed_->broadcast_at.value() && !truth_.empty())
  {
    seedAmcl(truth_.newest());
  }
}

void OdomWorldDrift::seedAmcl(const tf2::Transform& truth)
{
  reseed_.reset();
  geometry_msgs::msg::PoseWithCovarianceStamped seed;
  seed.header.stamp = node_->get_clock()->now();
  seed.header.frame_id = "map";
  tf2::toMsg(truth, seed.pose.pose);
  seed.pose.covariance[0] = kReseedXYVariance;
  seed.pose.covariance[7] = kReseedXYVariance;
  seed.pose.covariance[35] = kReseedYawVariance;
  initial_pose_pub_->publish(seed);
  RCLCPP_INFO(node_->get_logger(), "sim teleport: reset fuse and re-seeded AMCL at truth (%.2f, %.2f, %.1f deg)",
              truth.getOrigin().x(), truth.getOrigin().y(), tf2::getYaw(truth.getRotation()) * 180.0 / M_PI);
}

void OdomWorldDrift::publish()
{
  // Ahead of the withhold paths below, which would otherwise stall a re-seed silently.
  if (reseed_.has_value() && (node_->get_clock()->now() - reseed_->requested_at).seconds() > kReseedDeadlineSec)
  {
    abandonReseed("AMCL was not re-seeded within " + std::to_string(std::lround(kReseedDeadlineSec)) + " s");
  }
  if (!est_.has_value() || truth_.empty())
  {
    return;
  }
  const double est_age = (node_->get_clock()->now() - est_stamp_).seconds();
  if (isStale(est_age))
  {
    // Withhold so lookups fail loudly instead.
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                         "/odom_filtered is %.2f s stale (fuse down?) -- withholding odom->world "
                         "rather than localizing against a frozen estimate.",
                         est_age);
    return;
  }
  const auto truth_at_est = truth_.at(est_msg_stamp_);
  if (!truth_at_est.has_value())
  {
    // The estimate's stamp falls outside the truth history: the two streams are not overlapping
    // (one stalled, or the clock jumped). Withhold rather than difference across the gap.
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                         "no truth sample bracketing the estimate's stamp -- withholding "
                         "odom->world rather than differencing across a timing gap.");
    return;
  }
  tf_broadcaster_.sendTransform(toTransform(odomToWorld(est_.value(), truth_at_est.value()), node_->get_clock()->now()));
  if (!reseed_.has_value() || !reseed_->fuse_reset_at.has_value())
  {
    return;
  }
  const rclcpp::Time now = node_->get_clock()->now();
  if (!reseed_->broadcast_at.has_value() && est_msg_stamp_ > reseed_->fuse_reset_at.value())
  {
    reseed_->broadcast_at = now;
  }
  if (reseed_->broadcast_at.has_value() && (now - reseed_->broadcast_at.value()).seconds() > kAmclUpdateWaitSec)
  {
    seedAmcl(truth_.newest());
  }
}

}  // namespace hangar_sim
