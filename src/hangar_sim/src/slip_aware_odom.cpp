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

#include <cmath>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/utils.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "hangar_sim/slip_aware_odom_logic.hpp"

namespace
{
constexpr int kYaw = 35;                          // (yaw, yaw) pose element and (vyaw, vyaw) twist element
constexpr double kHoldPeriod = 0.02;              // 50 Hz: rate at which to check for a silent source
constexpr double kHoldGap = 0.015;                // hold the last pose once the source is quiet this long [s]
constexpr double kHoldMaxSec = 0.5;               // stop holding (and warn) once the source has been quiet this
                                                  // long: well beyond the ~15 ms handoff beat kHoldGap bridges,
                                                  // so a hold reaching this means the active controller died,
                                                  // not a handoff -- keep publishing a fabricated "parked" state
                                                  // forever would tell fuse the base is stationary indefinitely.
constexpr int64_t kHoldMaxWarnThrottleMs = 2000;  // min interval between hold-timeout warnings [ms]

using slip_aware_odom::Pose2;
using slip_aware_odom::RelayParams;
using slip_aware_odom::RelayState;
using slip_aware_odom::updateRelay;
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
    // Node-clock timer (not wall): the hold-gap check in hold() compares against now() (sim time),
    // so the tick must advance on sim time too, not wall time, under use_sim_time.
    hold_timer_ =
        rclcpp::create_timer(this, get_clock(), rclcpp::Duration::from_seconds(kHoldPeriod), [this]() { hold(); });
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
    const Pose2 p{ m.pose.pose.position.x, m.pose.pose.position.y, tf2::getYaw(q) };

    // Only the active controller publishes odom (the inactive one is silent), so the
    // two sources never interleave and a source change always means a genuine handoff.
    // updateRelay re-anchors on that switch (or a stale gap) so the output continues
    // seamlessly, and ramps the yaw covariance toward spin_cov as the leaky-integrated
    // |yaw rate| accumulates. Pure logic lives in slip_aware_odom_logic.hpp -- unit-tested
    // in test/test_slip_aware_odom.cpp.
    const auto result = updateRelay(p, t, m.twist.twist.angular.z, source, relay_params_, relay_state_);

    odom_.header.stamp = m.header.stamp;
    odom_.pose.pose.position.x = result.pose.x;
    odom_.pose.pose.position.y = result.pose.y;
    odom_.pose.pose.orientation.z = std::sin(result.pose.yaw / 2.0);
    odom_.pose.pose.orientation.w = std::cos(result.pose.yaw / 2.0);
    odom_.pose.covariance = m.pose.covariance;  // keep the controller's x/y covariance
    odom_.twist = m.twist;                      // body-frame twist is source-independent
    // Grow both the pose-yaw and twist-yaw-rate covariance so fuse defers to the IMU
    // gyro -- for orientation and yaw rate alike -- while the mecanum wheels slip.
    odom_.pose.covariance[kYaw] = result.yaw_cov;
    odom_.twist.covariance[kYaw] = result.yaw_cov;
    pub_->publish(odom_);
    last_relay_ = now();
  }

  // Bridge the brief source silence at a controller handoff: once the active
  // controller has been quiet for kHoldGap, republish the last pose at zero
  // velocity so fuse keeps a live wheel constraint and cannot coast. During
  // normal driving the ~500 Hz stream keeps last_relay_ fresh and this no-ops.
  void hold()
  {
    const double gap = (now() - last_relay_).seconds();
    if (gap > kHoldMaxSec)
    {
      // The active source has been quiet far longer than a handoff beat -- it died (crashed
      // controller, deactivated with nothing taking over) rather than switched. Stop fabricating
      // zero-velocity odometry: let fuse's wheel constraint lapse loudly instead of asserting
      // "parked" indefinitely.
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), kHoldMaxWarnThrottleMs,
                           "slip_aware_odom: active source has been silent for %.2fs (> %.2fs) -- "
                           "treating it as dead, not a handoff. Withholding held odometry.",
                           gap, kHoldMaxSec);
      return;
    }
    // computeHoldStamp returns the stamp (last relayed message time, on the controllers'
    // clock, advanced by the elapsed gap) to publish a held sample at, or nullopt if
    // nothing has been relayed yet or the gap hasn't reached kHoldGap. Pure logic in
    // slip_aware_odom_logic.hpp -- unit-tested in test/test_slip_aware_odom.cpp.
    const auto held_opt = slip_aware_odom::computeHoldStamp(relay_state_.active, relay_state_.last_t, gap, kHoldGap);
    if (!held_opt.has_value())
    {
      return;
    }
    const double held = held_opt.value();
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
  RelayParams relay_params_;  // relay/covariance-ramp tunables (defaults match the prior constants)
  RelayState relay_state_;    // mutable re-anchor/slip state, updated in relay()
  rclcpp::Time last_relay_;   // node-clock time of the last forwarded message
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
