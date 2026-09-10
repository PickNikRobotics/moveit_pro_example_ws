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

// Publish odom -> world, so that a lookup of odom -> ridgeback_base_link returns fuse's estimate
// while robot_state_publisher keeps world -> ridgeback_base_link as MuJoCo ground truth:
//
//     odom -> world = est(odom -> base) (+) inverse(truth(world -> base))
//
// ridgeback_base_link can have only one TF parent, but navigation has to read the estimate while
// the arm planner and the 66 hangar collision meshes under 'world' keep reading truth. Publishing
// the live difference between the two as odom -> world is what lets one transform tree carry both.
// Replaces the static odom -> world identity whenever use_fuse:=true.
//
// Both inputs arrive as nav_msgs/Odometry, so there is no TF buffer here and no joint-name
// resolution: /odom_filtered from fuse, and /odom straight from MuJoCo -- the true base pose,
// planar, at 150 Hz (odom_planar / odom_rate in picknik_ur_mujoco_ros2_control.xacro). /odom is
// still stamped "odom -> ridgeback_base_link" from when those two frames coincided; the pose it
// carries is exactly the ground truth wanted here.
//
// Sim-only, and the difference lives in the ground plane, so this is 2D pose algebra. There is
// deliberately no synthetic drift or noise term: MuJoCo models contact, and the estimate drifts on
// the sim's own physics.

#include <cmath>
#include <functional>
#include <optional>

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

/// Planar pose (x, y, yaw), defaulting to the identity. Kept local rather than in a shared header:
/// this is the only consumer, and tf2::Transform is full SE(3).
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
}  // namespace

class OdomWorldDrift : public rclcpp::Node
{
public:
  OdomWorldDrift() : Node("odom_world_drift"), tf_broadcaster_(*this)
  {
    // Only the latest sample of each input is ever used, so the small queues just absorb a
    // scheduling burst. Best-effort on the truth input matches either publisher -- /odom has been
    // reliable since moveit_pro#21948 and best-effort before it.
    est_sub_ =
        create_subscription<nav_msgs::msg::Odometry>("/odom_filtered", 10, std::bind(&OdomWorldDrift::onEst, this, _1));
    truth_sub_ = create_subscription<nav_msgs::msg::Odometry>("/odom", rclcpp::SensorDataQoS(),
                                                              std::bind(&OdomWorldDrift::onTruth, this, _1));
    // Node clock, not wall clock: under use_sim_time the tick has to advance on sim time, matching
    // the stamp publish() takes from that same clock.
    timer_ = rclcpp::create_timer(this, get_clock(), rclcpp::Duration::from_seconds(kPubPeriod), [this] { publish(); });
  }

private:
  void onEst(const nav_msgs::msg::Odometry::ConstSharedPtr& m)
  {
    est_ = fromOdom(*m);
    // Arrival time, not the sender's stamp: staleness here means how long *we* have gone without a
    // fresh estimate.
    est_stamp_ = get_clock()->now();
  }

  void onTruth(const nav_msgs::msg::Odometry::ConstSharedPtr& m)
  {
    truth_ = fromOdom(*m);
  }

  void publish()
  {
    if (!est_.has_value() || !truth_.has_value())
    {
      return;
    }
    // truth_ keeps advancing at 150 Hz whatever fuse is doing, so a frozen est_ (fuse crashed or
    // stalled) would broadcast odom->world with an advancing stamp but stale content: the lookup
    // stays "available" and AMCL silently localizes against a base that appears not to move.
    // Withhold instead, so downstream TF lookups fail loudly.
    const double est_age = (get_clock()->now() - est_stamp_).seconds();
    if (est_age > kEstStaleSec)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "/odom_filtered is %.2f s stale (fuse down?) -- withholding odom->world "
                           "rather than localizing against a frozen estimate.",
                           est_age);
      return;
    }
    // The two are the latest of each and sampled at different instants, so during motion this
    // carries up to ~speed * 0.1 s of timing skew -- more apparent drift for AMCL to absorb.
    const Pose2 t = compose(est_.value(), invert(truth_.value()));

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

  // Everything below is touched only by the two subscriptions and the timer, which share the node's
  // default mutually-exclusive callback group, so none of it needs locking.
  std::optional<Pose2> est_;    // fuse estimate, odom -> base
  rclcpp::Time est_stamp_;      // arrival time of the last est_
  std::optional<Pose2> truth_;  // MuJoCo ground truth, world -> base
  // ROS entities last, so they stop firing callbacks before the state above destructs.
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
