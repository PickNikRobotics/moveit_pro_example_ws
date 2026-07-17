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
// AND ANY EXPRESS OR IMPLIED WARRANTIES ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DAMAGES ARISING IN ANY WAY
// OUT OF THE USE OF THIS SOFTWARE.

// Forward the active platform controller's odometry as one continuous stream with
// a yaw covariance that grows during spin, for fuse's wheel_odom_sensor.
//
// Two mecanum controllers drive the base -- platform_velocity_controller_nav2 for
// Nav2, platform_velocity_controller for whole-body -- and only the active one
// publishes odom. Fuse needs a single unbroken input across every nav <-> whole-
// body switch, so this forwards whichever controller is publishing and stitches
// the handoff: each source's pose is rigidly offset to continue from the last
// output, so the stream never jumps (fuse consumes it differentially). Forwarding
// the controllers' own ~600 Hz odometry keeps their accuracy rather than
// re-deriving it from wheel states. The yaw covariance ramps from kBaseCov (trust
// wheel yaw, bounding the IMU gyro's drift) toward kSpinCov (defer to the IMU) as
// spin accumulates, because mecanum rollers slip in place.
//
// The deactivating controller stops publishing a beat before the activating one
// starts, so at each switch there is a brief window where neither source is live.
// A timer bridges it by holding the last pose at zero velocity (see hold()): fuse
// then never loses its wheel constraint and cannot coast, which otherwise jumped
// the estimate ~1 m at the switch and flipped the map during whole-body motion.

#include <algorithm>
#include <chrono>
#include <cmath>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

namespace
{
constexpr double kBaseCov = 0.03;     // straight-line yaw covariance
constexpr double kSpinCov = 0.2;      // fully-slipping yaw covariance
constexpr double kTau = 0.5;          // slip relaxation time constant [s]
constexpr double kSlipFull = 0.5;     // leaky-integrated |yaw rate| for full distrust
constexpr double kMaxDt = 0.1;        // clamp dt so a stale gap can't spike the integral
constexpr double kSourceGap = 0.2;    // a source quiet longer than this counts as switched out [s]
constexpr int kYaw = 35;              // (yaw, yaw) pose element and (vyaw, vyaw) twist element
constexpr double kHoldPeriod = 0.02;  // 50 Hz: rate at which to check for a silent source
constexpr double kHoldGap = 0.015;    // hold the last pose once the source is quiet this long [s]

// Planar SE(2) pose (x, y, yaw).
struct Pose2
{
  double x, y, yaw;
};
[[nodiscard]] Pose2 invert(const Pose2& p)
{
  const double c = std::cos(p.yaw), s = std::sin(p.yaw);
  return { -c * p.x - s * p.y, s * p.x - c * p.y, -p.yaw };
}
[[nodiscard]] Pose2 compose(const Pose2& a, const Pose2& b)
{
  const double c = std::cos(a.yaw), s = std::sin(a.yaw);
  return { a.x + c * b.x - s * b.y, a.y + s * b.x + c * b.y, a.yaw + b.yaw };
}
}  // namespace

class SlipAwareOdom : public rclcpp::Node
{
public:
  SlipAwareOdom() : Node("slip_aware_odom")
  {
    odom_.header.frame_id = "odom";
    odom_.child_frame_id = "ridgeback_base_link";
    // The published pose is a re-anchored stream seeded at the origin, not a true
    // absolute odom->base -- fuse consumes it differentially, so only its deltas are
    // meaningful. No TF is published from it. An absolute consumer would be wrong.
    pub_ = create_publisher<nav_msgs::msg::Odometry>("/odom_slip_aware", 10);
    // source 0 = nav2 controller, source 1 = whole-body controller.
    nav_sub_ = create_subscription<nav_msgs::msg::Odometry>("/platform_velocity_controller_nav2/odom", 10,
                                                            [this](nav_msgs::msg::Odometry::ConstSharedPtr m) {
                                                              relay(*m, 0);
                                                            });
    body_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/platform_velocity_controller/odom", 10, [this](nav_msgs::msg::Odometry::ConstSharedPtr m) { relay(*m, 1); });
    last_relay_ = now();
    hold_timer_ = create_wall_timer(std::chrono::duration<double>(kHoldPeriod), [this]() { hold(); });
  }

private:
  void relay(const nav_msgs::msg::Odometry& m, int source)
  {
    // Forward every message: fuse consumes each one as a differential yaw constraint,
    // and that full density is what holds AMCL locked through the fast base turns.
    // Throttling (tested at 50/200 Hz) starves those constraints and AMCL diverges
    // mid-turn, so the controller's native rate is the necessary rate here. That rate
    // is ~500 Hz (measured): the forked clearpath_mecanum_drive_controller publishes
    // odom every update() and ignores its `publish_rate: 50.0` param, so kHoldGap is
    // sized for ~500 Hz, not the 50 Hz the config implies.
    const double t = rclcpp::Time(m.header.stamp).seconds();
    const auto& q = m.pose.pose.orientation;
    const Pose2 p{ m.pose.pose.position.x, m.pose.pose.position.y,
                   std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)) };

    // Only the active controller publishes odom (the inactive one is silent), so the
    // two sources never interleave and a source change always means a genuine handoff.
    // On that switch or a stale gap, re-anchor so the output continues seamlessly:
    // pick the offset for which compose(offset, p) == the last output pose.
    if (source != active_ || last_t_ <= 0.0 || t - last_t_ > kSourceGap)
    {
      offset_ = compose(out_, invert(p));
      active_ = source;
    }
    out_ = compose(offset_, p);

    const double dt = (last_t_ > 0.0) ? std::clamp(t - last_t_, 0.0, kMaxDt) : 0.0;
    last_t_ = t;
    // Leaky integral of |yaw rate|: grows over sustained spin, decays with kTau.
    slip_ = slip_ * std::exp(-dt / kTau) + std::abs(m.twist.twist.angular.z) * dt;
    const double cov = kBaseCov + (kSpinCov - kBaseCov) * std::min(1.0, slip_ / kSlipFull);

    odom_.header.stamp = m.header.stamp;
    odom_.pose.pose.position.x = out_.x;
    odom_.pose.pose.position.y = out_.y;
    odom_.pose.pose.orientation.z = std::sin(out_.yaw / 2.0);
    odom_.pose.pose.orientation.w = std::cos(out_.yaw / 2.0);
    odom_.pose.covariance = m.pose.covariance;  // keep the controller's x/y covariance
    odom_.twist = m.twist;                      // body-frame twist is source-independent
    // Grow both the pose-yaw and twist-yaw-rate covariance so fuse defers to the IMU
    // gyro -- for orientation and yaw rate alike -- while the mecanum wheels slip.
    odom_.pose.covariance[kYaw] = cov;
    odom_.twist.covariance[kYaw] = cov;
    pub_->publish(odom_);
    last_relay_ = now();
  }

  // Bridge the brief source silence at a controller handoff: once the active
  // controller has been quiet for kHoldGap, republish the last pose at zero
  // velocity so fuse keeps a live wheel constraint and cannot coast. During
  // normal driving the ~500 Hz stream keeps last_relay_ fresh and this no-ops.
  void hold()
  {
    if (active_ < 0)
    {
      return;
    }
    const double gap = (now() - last_relay_).seconds();
    if (gap < kHoldGap)
    {
      return;
    }
    // Stamp = last relayed message stamp (last_t_, on the controllers' clock) advanced
    // by the elapsed gap, so held samples stay on the relayed stream's clock base and
    // strictly increase -- independent of whether the node clock matches it.
    const double held = last_t_ + gap;
    odom_.header.stamp.sec = static_cast<int32_t>(held);
    odom_.header.stamp.nanosec = static_cast<uint32_t>((held - std::floor(held)) * 1e9);
    odom_.twist.twist.linear.x = 0.0;
    odom_.twist.twist.linear.y = 0.0;
    odom_.twist.twist.angular.z = 0.0;
    pub_->publish(odom_);
  }

  // All shared state below is written and read only from relay() and hold(), which
  // share the node's default mutually-exclusive callback group -- so they never run
  // concurrently and the state needs no locking. Do NOT move either callback to a
  // separate or reentrant callback group without adding synchronization.
  nav_msgs::msg::Odometry odom_;
  Pose2 out_{ 0.0, 0.0, 0.0 };     // continuous output pose
  Pose2 offset_{ 0.0, 0.0, 0.0 };  // current source -> output-frame offset
  int active_ = -1;                // source id currently being forwarded
  double slip_ = 0.0, last_t_ = 0.0;
  rclcpp::Time last_relay_;  // node-clock time of the last forwarded message
  // ROS entities declared last so they destruct (and stop firing callbacks) before
  // the state above that those callbacks read.
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr nav_sub_, body_sub_;
  rclcpp::TimerBase::SharedPtr hold_timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SlipAwareOdom>());
  rclcpp::shutdown();
  return 0;
}
