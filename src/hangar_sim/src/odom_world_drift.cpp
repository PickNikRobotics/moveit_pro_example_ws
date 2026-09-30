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
Pose2 fromOdom(const nav_msgs::msg::Odometry& m)
{
  return { m.pose.pose.position.x, m.pose.pose.position.y, tf2::getYaw(m.pose.pose.orientation) };
}

Pose2 invert(const Pose2& p)
{
  const double c = std::cos(p.yaw), s = std::sin(p.yaw);
  return { -c * p.x - s * p.y, s * p.x - c * p.y, -p.yaw };
}

Pose2 compose(const Pose2& a, const Pose2& b)
{
  const double c = std::cos(a.yaw), s = std::sin(a.yaw);
  return { a.x + c * b.x - s * b.y, a.y + s * b.x + c * b.y, a.yaw + b.yaw };
}

double wrap(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

Pose2 lerp(const Pose2& a, const Pose2& b, double fraction)
{
  return { a.x + fraction * (b.x - a.x), a.y + fraction * (b.y - a.y), a.yaw + fraction * wrap(b.yaw - a.yaw) };
}

Pose2 odomToWorld(const Pose2& est, const Pose2& truth)
{
  return compose(est, invert(truth));
}

geometry_msgs::msg::TransformStamped toTransform(const Pose2& odom_to_world, const rclcpp::Time& stamp)
{
  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp = stamp;
  tf.header.frame_id = "odom";
  tf.child_frame_id = "world";
  tf.transform.translation.x = odom_to_world.x;
  tf.transform.translation.y = odom_to_world.y;
  tf.transform.rotation.z = std::sin(odom_to_world.yaw / 2.0);
  tf.transform.rotation.w = std::cos(odom_to_world.yaw / 2.0);
  return tf;
}

bool isStale(double est_age_sec)
{
  return est_age_sec > kEstStaleSec;
}

bool TruthHistory::add(const rclcpp::Time& stamp, const Pose2& pose)
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

std::optional<Pose2> TruthHistory::at(const rclcpp::Time& when) const
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
  return lerp(before->pose, after->pose, fraction);
}

OdomWorldDrift::OdomWorldDrift(std::shared_ptr<rclcpp::Node> node) : node_(std::move(node)), tf_broadcaster_(*node_)
{
  // Only the latest estimate is used; truth is kept as a short stamped history (see TruthHistory).
  est_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
      "/odom_filtered", 10, [this](const nav_msgs::msg::Odometry::ConstSharedPtr& m) { onEst(*m); });
  truth_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", rclcpp::SensorDataQoS(), [this](const nav_msgs::msg::Odometry::ConstSharedPtr& m) { onTruth(*m); });
  // Node clock, not wall clock, so the tick advances on sim time under use_sim_time.
  timer_ = rclcpp::create_timer(node_, node_->get_clock(), rclcpp::Duration::from_seconds(kPubPeriod),
                                [this] { publish(); });
}

void OdomWorldDrift::onEst(const nav_msgs::msg::Odometry& msg)
{
  est_ = fromOdom(msg);
  // Arrival time, not the sender's stamp: staleness means how long we have gone without one.
  est_stamp_ = node_->get_clock()->now();
  // The sender's stamp, separately: this is the instant the estimate describes, and it is what
  // truth has to be sampled at for the difference to mean anything.
  est_msg_stamp_ = rclcpp::Time(msg.header.stamp);
}

void OdomWorldDrift::onTruth(const nav_msgs::msg::Odometry& msg)
{
  // A sim reset rewinds the clock. Drop the estimate along with the history: a pre-reset estimate
  // paired with post-reset truth is a meaningless offset.
  if (!truth_.add(rclcpp::Time(msg.header.stamp), fromOdom(msg)))
  {
    est_.reset();
  }
}

void OdomWorldDrift::publish()
{
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
}

}  // namespace hangar_sim
