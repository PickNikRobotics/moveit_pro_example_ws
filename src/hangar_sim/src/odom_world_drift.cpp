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

// Publishes odom -> world = est(odom -> base) (+) inverse(truth(world -> base)), so navigation reads
// fuse's estimate while the arm planner and the hangar meshes under 'world' keep MuJoCo truth.
// base_link can have only one TF parent; broadcasting the difference lets one tree carry both.
// Replaces the static identity when use_fuse:=true. Sim-only and planar, hence 2D pose algebra.

#include <algorithm>
#include <cmath>
#include <deque>
#include <functional>
#include <optional>
#include <utility>

#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/utils.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace
{
using std::placeholders::_1;

constexpr double kPubPeriod = 0.02;   // 50 Hz -- keeps odom->world fresh for AMCL's motion model.
constexpr double kEstStaleSec = 0.5;  // ~5x fuse's 10 Hz publish period.
// Truth history, so the estimate is differenced against truth at ITS OWN stamp. The cap only
// guards against an unbounded queue; neither is a tuning knob.
constexpr double kTruthHistorySec = 1.0;
constexpr size_t kTruthHistoryMax = 4096;
// fuse stamps its estimate where it predicted to, which can land just ahead of the newest truth.
// Clamp to newest within this window; beyond it the streams have diverged and we withhold.
constexpr double kEstAheadToleranceSec = 0.05;

/// Planar pose (x, y, yaw). Local to this file; tf2::Transform is full SE(3).
struct Pose2
{
  double x = 0.0, y = 0.0, yaw = 0.0;
};

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

double wrap(double a)
{
  return std::atan2(std::sin(a), std::cos(a));
}

/// Lerp between two truth samples; yaw goes through the wrapped difference so a pair straddling
/// +/-pi does not spin the long way round.
Pose2 lerp(const Pose2& a, const Pose2& b, double f)
{
  return { a.x + f * (b.x - a.x), a.y + f * (b.y - a.y), a.yaw + f * wrap(b.yaw - a.yaw) };
}
}  // namespace

class OdomWorldDrift : public rclcpp::Node
{
public:
  OdomWorldDrift() : Node("odom_world_drift"), tf_broadcaster_(*this)
  {
    // Only the latest estimate is used; truth is kept as a short stamped history (see truthAt).
    est_sub_ =
        create_subscription<nav_msgs::msg::Odometry>("/odom_filtered", 10, std::bind(&OdomWorldDrift::onEst, this, _1));
    truth_sub_ = create_subscription<nav_msgs::msg::Odometry>("/odom", rclcpp::SensorDataQoS(),
                                                              std::bind(&OdomWorldDrift::onTruth, this, _1));
    // Node clock, not wall clock, so the tick advances on sim time under use_sim_time.
    timer_ = rclcpp::create_timer(this, get_clock(), rclcpp::Duration::from_seconds(kPubPeriod), [this] { publish(); });
  }

private:
  void onEst(const nav_msgs::msg::Odometry::ConstSharedPtr& m)
  {
    est_ = fromOdom(*m);
    // Arrival time, not the sender's stamp: staleness means how long we have gone without one.
    est_stamp_ = get_clock()->now();
    // The sender's stamp, separately: this is the instant the estimate describes, and it is what
    // truth has to be sampled at for the difference to mean anything.
    est_msg_stamp_ = rclcpp::Time(m->header.stamp);
  }

  void onTruth(const nav_msgs::msg::Odometry::ConstSharedPtr& m)
  {
    const rclcpp::Time stamp(m->header.stamp);
    // MuJoCo publishes truth monotonically, but a sim reset rewinds the clock. Drop the history
    // AND the estimate: a pre-reset estimate paired with post-reset truth is a meaningless offset.
    if (!truth_hist_.empty() && stamp < truth_hist_.back().first)
    {
      truth_hist_.clear();
      est_.reset();
    }
    truth_hist_.emplace_back(stamp, fromOdom(*m));
    while (truth_hist_.size() > kTruthHistoryMax ||
           (truth_hist_.size() > 1 && (stamp - truth_hist_.front().first).seconds() > kTruthHistorySec))
    {
      truth_hist_.pop_front();
    }
  }

  /// Truth at `when`, interpolated between the samples that bracket it. Pairing the newest of each
  /// instead would difference two different instants, which reads as omega * age of spurious yaw.
  /// Returns nullopt outside the buffer, so publish() withholds rather than extrapolating.
  std::optional<Pose2> truthAt(const rclcpp::Time& when) const
  {
    if (truth_hist_.empty() || when < truth_hist_.front().first)
    {
      return std::nullopt;
    }
    if (when > truth_hist_.back().first)
    {
      return (when - truth_hist_.back().first).seconds() <= kEstAheadToleranceSec ?
                 std::optional<Pose2>(truth_hist_.back().second) :
                 std::nullopt;
    }
    if (truth_hist_.size() < 2)
    {
      return truth_hist_.front().second;
    }
    const auto hi = std::lower_bound(truth_hist_.begin(), truth_hist_.end(), when,
                                     [](const auto& e, const rclcpp::Time& t) { return e.first < t; });
    if (hi == truth_hist_.begin())
    {
      return hi->second;
    }
    const auto lo = std::prev(hi);
    const double span = (hi->first - lo->first).seconds();
    if (span <= 0.0)
    {
      return lo->second;
    }
    return lerp(lo->second, hi->second, (when - lo->first).seconds() / span);
  }

  void publish()
  {
    if (!est_.has_value() || truth_hist_.empty())
    {
      return;
    }
    // Arrival-time staleness only: this catches fuse going silent, not fuse publishing a frozen
    // estimate with fresh stamps. Withhold on silence so lookups fail loudly instead.
    const double est_age = (get_clock()->now() - est_stamp_).seconds();
    if (est_age > kEstStaleSec)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "/odom_filtered is %.2f s stale (fuse down?) -- withholding odom->world "
                           "rather than localizing against a frozen estimate.",
                           est_age);
      return;
    }
    const auto truth_at_est = truthAt(est_msg_stamp_);
    if (!truth_at_est.has_value())
    {
      // The estimate's stamp falls outside the truth history: the two streams are not overlapping
      // (one stalled, or the clock jumped). Withhold rather than difference across the gap.
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "no truth sample bracketing the estimate's stamp -- withholding "
                           "odom->world rather than differencing across a timing gap.");
      return;
    }
    const Pose2 t = compose(est_.value(), invert(truth_at_est.value()));

    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = get_clock()->now();
    tf.header.frame_id = "odom";
    tf.child_frame_id = "world";
    tf.transform.translation.x = t.x;
    tf.transform.translation.y = t.y;
    tf.transform.rotation.z = std::sin(t.yaw / 2.0);
    tf.transform.rotation.w = std::cos(t.yaw / 2.0);
    tf_broadcaster_.sendTransform(tf);
  }

  // Touched only by the subscriptions and timer, which share one mutually-exclusive callback
  // group, so no locking is needed.
  std::optional<Pose2> est_;    // fuse estimate, odom -> base
  rclcpp::Time est_stamp_;      // arrival time of the last est_, for the staleness guard
  rclcpp::Time est_msg_stamp_;  // the instant the last est_ describes, for pairing with truth
  // MuJoCo ground truth, world -> base, with stamps: a short history rather than just the latest,
  // so the estimate can be differenced against truth from the same instant.
  std::deque<std::pair<rclcpp::Time, Pose2>> truth_hist_;
  // ROS entities last, so callbacks stop before the state above destructs.
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr est_sub_, truth_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomWorldDrift>());
  rclcpp::shutdown();
  return 0;
}
