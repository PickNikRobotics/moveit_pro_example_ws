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

// Publish odom -> world so that odom -> base equals fuse's (drifty) estimate,
// while robot_state_publisher keeps world -> base as ground truth.
//
//     odom -> world = fuse_estimate(odom->base) (+) inverse(world->base_true)
// makes the TF lookup odom -> base resolve to fuse's estimate; AMCL then has real
// drift to correct. Replaces the static odom -> world identity when use_fuse:=true.
//
// Sim-only. The drift lives in the ground plane, so everything here is 2D pose
// algebra (x, y, yaw), shared with the other hangar_sim nodes via se2.hpp. The
// ground-truth base pose is read straight from the virtual-rail joints on
// /joint_states rather than a tf2 listener: the sim floods /tf at ~700 Hz across
// ~140 frames, and buffering that just to read one pose is wasteful. (The Python
// predecessor also avoided numpy here to dodge OpenBLAS's spin-waiting worker
// threads; in C++ there is no such dependency to avoid.)

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <optional>
#include <string>
#include <vector>

#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2/utils.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "hangar_sim/odom_world_drift_logic.hpp"
#include "hangar_sim/se2.hpp"

namespace
{
using hangar_sim::se2::compose;
using hangar_sim::se2::invert;
using hangar_sim::se2::Pose2;
// Rail-joint list + index resolver live in an internal-namespace header so their branches are
// unit-tested (test_odom_world_drift); the frame/kinematics assumptions are documented there.
using odom_world_drift::internal::kRailJoints;
using odom_world_drift::internal::resolveRailIndices;

constexpr double kPubPeriod = 0.02;   // 50 Hz, keeps odom->world fresh for AMCL
constexpr double kEstStaleSec = 0.5;  // ~5x fuse's ~10 Hz publish period: treat /odom_filtered as
                                      // stale (fuse crashed/stalled) beyond this and stop
                                      // broadcasting, rather than replaying a frozen estimate
                                      // against a still-advancing ground-truth base_ stamp.
}  // namespace

class OdomWorldDrift : public rclcpp::Node
{
public:
  OdomWorldDrift() : Node("odom_world_drift"), tf_broadcaster_(*this)
  {
    // Depth-10 default-reliable on both: fuse publishes /odom_filtered reliable, and
    // joint_state_broadcaster publishes /joint_states reliable. The node only ever uses the latest
    // sample of each (no history), so the small queue is intentional -- it just tolerates a brief
    // callback-scheduling burst.
    est_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/odom_filtered", 10, [this](nav_msgs::msg::Odometry::ConstSharedPtr m) { onEst(*m); });
    joints_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10, [this](sensor_msgs::msg::JointState::ConstSharedPtr m) { onJoints(*m); });
    // Node-clock timer (not wall): the node runs use_sim_time, so the 50 Hz tick must advance on
    // sim time too, matching the stamp taken from the same clock in publish().
    timer_ =
        rclcpp::create_timer(this, get_clock(), rclcpp::Duration::from_seconds(kPubPeriod), [this]() { publish(); });
  }

private:
  void onEst(const nav_msgs::msg::Odometry& m)
  {
    const auto& p = m.pose.pose;
    est_ = Pose2{ p.position.x, p.position.y, tf2::getYaw(p.orientation) };
    est_stamp_ = get_clock()->now();  // arrival time, not the sender's stamp: staleness is judged
                                      // against how long WE have gone without a fresh sample.
  }

  void onJoints(const sensor_msgs::msg::JointState& msg)
  {
    // /joint_states is multi-publisher: messages can omit the rail joints or order them
    // differently, so re-resolve the indices whenever the name list changes rather than trusting
    // a stale cache. Cache the name list unconditionally -- including on a failed resolve -- so a
    // repeated rail-less list is recognized and skipped without re-searching every message.
    if (msg.name != cached_names_)
    {
      cached_names_ = msg.name;
      idx_ = resolveRailIndices(msg.name);
    }
    if (!idx_.has_value())
    {
      return;
    }
    // position and name normally align, but position may be shorter than a stale index implies;
    // bounds-check before operator[] (unlike Python, an out-of-range read here is UB, not an
    // exception that merely drops the callback).
    const auto& idx = idx_.value();
    if (std::any_of(idx.begin(), idx.end(), [&](std::size_t i) { return i >= msg.position.size(); }))
    {
      return;
    }
    // Read the rail joints directly as world->base (x, y, yaw). Exact only under the sim's rail
    // kinematics and zero base-link offset -- see kRailJoints in odom_world_drift_logic.hpp.
    base_ = Pose2{ msg.position[idx[0]], msg.position[idx[1]], msg.position[idx[2]] };
  }

  void publish()
  {
    if (!est_.has_value() || !base_.has_value())
    {
      return;
    }
    // base_ keeps updating from /joint_states at 50 Hz independent of fuse's health, so a stale
    // est_ (fuse crashed or stalled -- now recoverable via respawn) would otherwise broadcast
    // odom->world with an advancing stamp but frozen content: the TF lookup stays "available" and
    // AMCL silently localizes against a base that appears not to be moving. Withhold instead.
    const double est_age = (get_clock()->now() - est_stamp_).seconds();
    if (est_age > kEstStaleSec)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "odom_world_drift: /odom_filtered is %.2fs stale (fuse down?) -- "
                           "withholding odom->world so downstream TF lookups fail loudly instead "
                           "of localizing against a frozen estimate.",
                           est_age);
      return;
    }
    // odom->world = odom->base(est) (+) inverse(world->base(true))
    // Note: est (fuse, ~10 Hz) and base (rail joints, ~50 Hz) are the latest of each, sampled at
    // different instants, so during motion this carries up to ~speed*0.1s of timing-skew noise
    // (~5 cm at 0.5 m/s). Second-order vs the 300 ms lag the gate's latency-comp corrects, and it
    // just presents as extra apparent drift for AMCL to absorb -- left as-is intentionally.
    const Pose2 t = compose(est_.value(), invert(base_.value()));

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

  // All state below is written and read only from the two subscription callbacks and the timer,
  // which share the node's default mutually-exclusive callback group -- so they never run
  // concurrently and the state needs no locking.
  std::optional<Pose2> est_;                                        // fuse estimate odom->base
  rclcpp::Time est_stamp_;                                          // arrival time of the last est_
  std::optional<Pose2> base_;                                       // ground-truth world->base
  std::vector<std::string> cached_names_;                           // /joint_states name list idx_ resolves against
  std::optional<std::array<std::size_t, kRailJoints.size()>> idx_;  // cached rail-joint indices into that list
  // ROS entities declared last so they destruct (and stop firing callbacks) before the state above.
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr est_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joints_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomWorldDrift>());
  rclcpp::shutdown();
  return 0;
}
