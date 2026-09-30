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
// Replaces the static identity when use_fuse:=true. Sim-only and planar.

#pragma once

#include <cstddef>
#include <deque>
#include <memory>
#include <optional>

#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Transform.hpp>

namespace hangar_sim
{
constexpr double kPubPeriod = 0.02;   // 50 Hz -- keeps odom->world fresh for AMCL's motion model.
constexpr double kEstStaleSec = 0.5;  // ~5x fuse's 10 Hz publish period.
// The truth history exists so the estimate is differenced against truth at ITS OWN stamp. The cap
// only guards against an unbounded queue; neither is a tuning knob.
constexpr double kTruthHistorySec = 1.0;
constexpr size_t kTruthHistoryMax = 4096;
// fuse stamps its estimate where it predicted to, which can land just ahead of the newest truth.
// Clamp to newest within this window; beyond it the streams have diverged and we withhold.
constexpr double kEstAheadToleranceSec = 0.05;

/// odom -> world, given the estimate (odom -> base) and truth (world -> base) of the same instant.
/// Projected to planar (z = 0, yaw only). The algebra is SE(3), and truth is planar by
/// construction (odom_planar), but fuse solves in 3D and its z/roll/pitch would otherwise lift and
/// tilt everything under 'world' -- the hangar meshes, the MoveIt collision geometry and the arm's
/// own chain -- relative to map and odom, which no 2D consumer downstream would report.
tf2::Transform odomToWorld(const tf2::Transform& est, const tf2::Transform& truth);

geometry_msgs::msg::TransformStamped toTransform(const tf2::Transform& odom_to_world, const rclcpp::Time& stamp);

/// A frozen estimate would still broadcast with a fresh stamp, and AMCL would silently localize
/// against a base that appears not to move.
bool isStale(double est_age_sec);

/// Time-ordered ground-truth poses (world -> base) over the last kTruthHistorySec.
class TruthHistory
{
public:
  /// Appends a sample. Returns false, after discarding every older sample, if `stamp` is earlier
  /// than the newest one: MuJoCo publishes monotonically, so that means the sim clock was reset.
  bool add(const rclcpp::Time& stamp, const tf2::Transform& pose);

  bool empty() const
  {
    return samples_.empty();
  }

  /// Truth at `when`. Pairing the newest of each stream instead would difference two different
  /// instants, which reads as omega * age of spurious yaw. The rule, in full:
  ///   - before the oldest sample:                    nullopt (never extrapolate backwards)
  ///   - between the oldest and newest sample:        lerp/slerp of the bracketing pair
  ///   - up to kEstAheadToleranceSec past the newest: the newest sample
  ///   - later than that:                             nullopt (the streams have diverged)
  std::optional<tf2::Transform> at(const rclcpp::Time& when) const;

private:
  struct Sample
  {
    rclcpp::Time stamp;
    tf2::Transform pose;
  };
  std::deque<Sample> samples_;  // oldest first
};

/// Owns no node: the caller creates one, hands it in, and spins it.
class OdomWorldDrift
{
public:
  explicit OdomWorldDrift(std::shared_ptr<rclcpp::Node> node);

  void onEst(const nav_msgs::msg::Odometry& msg);
  void onTruth(const nav_msgs::msg::Odometry& msg);
  /// Broadcasts odom -> world, or withholds it (with a throttled warning) when the inputs cannot
  /// support one.
  void publish();

private:
  std::shared_ptr<rclcpp::Node> node_;
  // Touched only by the subscriptions and timer, which share one mutually-exclusive callback
  // group, so no locking is needed.
  std::optional<tf2::Transform> est_;  // fuse estimate, odom -> base
  rclcpp::Time est_stamp_;             // arrival time of the last est_, for the staleness guard
  rclcpp::Time est_msg_stamp_;         // the instant the last est_ describes, for pairing with truth
  TruthHistory truth_;                 // MuJoCo ground truth, world -> base
  // ROS entities last, so callbacks stop before the state above destructs.
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr est_sub_, truth_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace hangar_sim
