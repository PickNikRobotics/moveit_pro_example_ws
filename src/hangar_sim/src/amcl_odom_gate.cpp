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

// Degeneracy-aware map->odom gate (ROS node).
//
// AMCL matches the lidar to a static map. Where the scene is degenerate for that
// match -- pressed against the large smooth fuselage (slide-along ambiguity), or
// among unmapped boxes -- the estimate lurches/teleports and the map flips.
// odom->base (fuse's wheel+IMU dead-reckoning) stays locally accurate through those
// zones, so this node holds the last good map->odom and coasts on odom until AMCL is
// trustworthy again, then blends smoothly back. It replaces AMCL's own broadcast:
// set AMCL's tf_broadcast:=false so this node is the sole map->odom publisher.
//
// This file is only the ROS I/O: read /particle_cloud, distil it to (mean, spread),
// look up odom->base, and hand the implied map->odom to the pure decision function
// detail::updateGate (amcl_odom_gate_logic.hpp), which is unit-tested in isolation.
//
// ASSUMPTION (documented, load-bearing): odom->base is trustworthy for the DURATION
// of a degenerate zone. This holds because the zones are transient (a few seconds
// passing the fuselage or boxes) and fuse's drift over that span is small. If the
// robot were held in a degenerate zone long enough for odom to drift materially, or
// odometry failed grossly (severe wheel slip) exactly there, the gate would coast on
// bad data and degrade to "as good as odometry" -- the design's outer limit.

#include <chrono>
#include <cmath>
#include <vector>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/utils.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "hangar_sim/amcl_odom_gate_logic.hpp"

namespace
{
constexpr double kPubPeriod = 0.033;            // 30 Hz map->odom broadcast
constexpr double kTransformTolerance = 0.1;     // future-date the transform so TF can extrapolate [s]
constexpr double kCoastAlpha = 0.5;             // follow-fraction below which the gate counts as coasting
constexpr double kCoastWarnSeconds = 5.0;       // warn once a continuous coast exceeds this [s]
constexpr int64_t kCoastWarnThrottleMs = 2000;  // min interval between coast warnings [ms]
constexpr double kStaleInputSeconds = 3.0;      // publish() warns if onCloud has not updated in this long:
                                                // beluga publishes /particle_cloud faster than this even at
                                                // rest, so a longer gap means input starvation (beluga/odom
                                                // TF gone) and the node is republishing a frozen map->odom.
constexpr int64_t kStaleWarnThrottleMs = 2000;  // min interval between stale-input warnings [ms]
constexpr double kOdomBufferSec = 1.0;          // odom->base history kept for latency compensation [s].
                                                // Usable compensation ceiling is this minus the newest
                                                // sample's age (the odom TF lag), not the full window.
}  // namespace

using amcl_odom_gate::Pose2;

class AmclOdomGate : public rclcpp::Node
{
public:
  AmclOdomGate() : Node("amcl_odom_gate"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_), tf_broadcaster_(*this)
  {
    params_.spread_hold = declare_parameter("spread_hold", params_.spread_hold);
    params_.spread_resume = declare_parameter("spread_resume", params_.spread_resume);
    params_.jump_hold = declare_parameter("jump_hold", params_.jump_hold);
    params_.jump_hold_yaw = declare_parameter("jump_hold_yaw", params_.jump_hold_yaw);
    params_.provisional_tol = declare_parameter("provisional_tol", params_.provisional_tol);
    params_.provisional_tol_yaw = declare_parameter("provisional_tol_yaw", params_.provisional_tol_yaw);
    params_.persist_time = declare_parameter("persist_time", params_.persist_time);
    params_.spread_accept_max = declare_parameter("spread_accept_max", params_.spread_accept_max);
    params_.latency_compensation_sec = declare_parameter("latency_compensation_sec", params_.latency_compensation_sec);
    params_.alpha_slew = declare_parameter("alpha_slew", params_.alpha_slew);
    if (params_.latency_compensation_sec >= kOdomBufferSec)
    {
      // interpolateOdom would clamp to the oldest sample, silently degrading compensation. The two
      // constants live apart (a parameter vs. a hardcoded window), so warn rather than fail silently.
      RCLCPP_WARN(get_logger(),
                  "latency_compensation_sec (%.2f s) >= odom buffer window (%.2f s); compensation will "
                  "clamp to the oldest sample. Reduce it or enlarge kOdomBufferSec.",
                  params_.latency_compensation_sec, kOdomBufferSec);
    }
    // alpha_slew <= 0 freezes the gate: once s.alpha snaps to 0 on a hold, min(s.alpha + alpha_slew, 1)
    // can never climb back, so the node would broadcast a frozen map->odom for the rest of the run.
    // Fall back to the default rather than run permanently stuck.
    if (params_.alpha_slew <= 0.0)
    {
      const double default_alpha_slew = amcl_odom_gate::GateParams{}.alpha_slew;
      RCLCPP_WARN(get_logger(),
                  "alpha_slew (%.3f) must be > 0; the gate could never resume after a hold. Using the default (%.3f).",
                  params_.alpha_slew, default_alpha_slew);
      params_.alpha_slew = default_alpha_slew;
    }
    // spread_resume must sit below spread_hold or the hysteresis loses its dead band and the hold latch
    // can flicker every update.
    if (params_.spread_resume >= params_.spread_hold)
    {
      RCLCPP_WARN(get_logger(),
                  "spread_resume (%.2f) >= spread_hold (%.2f); the hold hysteresis has no dead band and may flicker.",
                  params_.spread_resume, params_.spread_hold);
    }
    // Negative persist_time makes the persistence check vacuous (now - provisional_since >= negative is
    // always true), accepting every large innovation at once -- the teleport guard is effectively off.
    if (params_.persist_time < 0.0)
    {
      RCLCPP_WARN(get_logger(), "persist_time (%.2f s) is negative; the teleport-persistence guard is disabled.",
                  params_.persist_time);
    }
    // beluga publishes /particle_cloud BEST_EFFORT; match it or we receive nothing.
    cloud_sub_ = create_subscription<geometry_msgs::msg::PoseArray>(
        "/particle_cloud", rclcpp::SensorDataQoS(),
        [this](geometry_msgs::msg::PoseArray::ConstSharedPtr m) { onCloud(*m); });
    // Node-clock timer (not wall): the node runs use_sim_time, and publish()'s coast/stale windows
    // are measured in now() (sim time), so the tick must advance on sim time too or the two drift
    // apart when the sim is not real-time.
    timer_ =
        rclcpp::create_timer(this, get_clock(), rclcpp::Duration::from_seconds(kPubPeriod), [this]() { publish(); });
  }

private:
  void onCloud(const geometry_msgs::msg::PoseArray& cloud)
  {
    const size_t n = cloud.poses.size();
    if (n < 2)
    {
      return;
    }
    // Refresh the odom buffer here too (not only in the publish timer): onCloud fires asynchronously
    // to that 30 Hz timer, so without this the newest buffered sample could be up to a publish period
    // stale relative to now, and interpolateOdom at (now - 0) would clamp to a stale pose -- making
    // even latency_compensation_sec = 0 differ from a fresh latest lookup. Sampling now keeps the
    // buffer as fresh as this callback and preserves the "0 == old compose-at-latest" invariant.
    sampleOdomForBuffer();
    // Circular mean (AMCL's estimate) and RMS spread (its confidence) over the cloud.
    double sx = 0.0, sy = 0.0, ss = 0.0, sc = 0.0;
    for (const auto& p : cloud.poses)
    {
      sx += p.position.x;
      sy += p.position.y;
      const double y = tf2::getYaw(p.orientation);
      ss += std::sin(y);
      sc += std::cos(y);
    }
    const Pose2 mean{ sx / n, sy / n, std::atan2(ss, sc) };
    double var = 0.0;
    for (const auto& p : cloud.poses)
    {
      const double dx = p.position.x - mean.x, dy = p.position.y - mean.y;
      var += dx * dx + dy * dy;
    }
    const double spread = std::sqrt(var / n);

    // odom->base is fuse's estimate. Compose the AMCL mean with odom->base from the cloud's own
    // content time (cloud stamp - latency_compensation_sec), NOT the latest: beluga's cloud CONTENT
    // trails the true pose by ~that lag during motion (its stamp under-reports it -- a lookup at the
    // cloud stamp itself fails as future extrapolation), so composing with the latest odom leaves
    // map->odom stale by speed*lag. Referencing the cloud's OWN header stamp (not now()) keeps the
    // compensation immune to onCloud transport/scheduling jitter, so the parameter measures one
    // physical thing. The live odom->base then carries the consistent map->odom forward to now.
    // odom_buf_ is filled at the publish rate and at the top of this callback; interpolateOdom clamps
    // if the target predates the buffer (startup) and returns nullopt only if it is empty, in which
    // case fall back to the latest TF lookup. lag = 0 reproduces compose-at-latest.
    // INVARIANT: mean (from beluga's /particle_cloud, in ITS base_frame) and this odom->base lookup
    // must refer to the SAME base link (ridgeback_base_link); a frame mismatch would bias every
    // candidate by a constant offset that persistence would then accept as a real correction.
    const double now_sec = this->now().seconds();
    const double odom_target = rclcpp::Time(cloud.header.stamp).seconds() - params_.latency_compensation_sec;
    Pose2 odom_base{ 0.0, 0.0, 0.0 };
    if (const auto interp = amcl_odom_gate::interpolateOdom(odom_buf_, odom_target); interp.has_value())
    {
      odom_base = interp.value();
      if (params_.latency_compensation_sec > 0.0 && !odom_buf_.empty() && odom_target > odom_buf_.back().t)
      {
        // Compensation was requested but the target landed AFTER the newest odom sample, so
        // interpolateOdom clamped to it -- i.e. the odom->base TF latency exceeds
        // latency_compensation_sec and the feature is silently under-compensating. Surface it.
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), kStaleWarnThrottleMs,
                             "amcl_odom_gate: latency-compensation target is newer than the odom buffer "
                             "(odom TF lag exceeds latency_compensation_sec=%.2f s); clamped, under-compensating.",
                             params_.latency_compensation_sec);
      }
    }
    else
    {
      // interpolateOdom returns nullopt only when odom_buf_ is empty, which -- given the
      // sampleOdomForBuffer() at the top of this callback -- only happens in the narrow startup
      // window where that lookup just failed. Retry once here; if it still fails, skip this cloud.
      try
      {
        const auto t = tf_buffer_.lookupTransform("odom", "ridgeback_base_link", tf2::TimePointZero);
        odom_base = { t.transform.translation.x, t.transform.translation.y, tf2::getYaw(t.transform.rotation) };
      }
      catch (const tf2::TransformException&)
      {
        return;  // odom->base not available yet: skip this cloud (timer keeps rebroadcasting last good).
      }
    }

    // AMCL's implied map->odom = (map->base) o inv(odom->base); gate decides what to broadcast.
    // The persist_time clock uses now() (wall/sim), not the cloud stamp; under steady AMCL
    // latency the constant offset leaves elapsed-duration measurement correct.
    const Pose2 candidate = amcl_odom_gate::compose(mean, amcl_odom_gate::invert(odom_base));
    map_odom_ = amcl_odom_gate::detail::updateGate(candidate, spread, now_sec, params_, state_);
    have_map_odom_ = true;

    // Watchdog: the gate is the SOLE map->odom source, so a prolonged hold (coasting on odom
    // through a severe-spread wrong lock) silently degrades localization to unbounded dead
    // reckoning. Surface a long coast so it does not quietly move the base's map pose out from
    // under the nav stack -- the state_.alpha follow-fraction is near 0 whenever the gate holds.
    if (state_.alpha < kCoastAlpha)
    {
      const auto t_now = now();  // one read: keep the logged elapsed consistent with the check
      if (!coasting_)
      {
        coasting_ = true;
        coast_start_ = t_now;
      }
      else if (const double coast_s = (t_now - coast_start_).seconds(); coast_s > kCoastWarnSeconds)
      {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), kCoastWarnThrottleMs,
                             "amcl_odom_gate coasting on odom for %.1f s (AMCL untrusted, spread=%.2f); "
                             "map->odom is dead-reckoning and will drift until AMCL re-locks.",
                             coast_s, spread);
      }
    }
    else
    {
      coasting_ = false;
    }
    last_update_ = now();  // liveness: publish() warns if onCloud stops updating (input starvation).
  }

  // Sample odom->base at the latest available TF and append to the ring buffer keyed by the TF's
  // OWN stamp (which trails now by the odom publish latency, so labelling by now would mis-time it),
  // trimming samples older than kOdomBufferSec. Runs at the publish rate; feeds interpolateOdom() in
  // onCloud so a latency-lagged AMCL correction can be composed at its content time (now - lag).
  void sampleOdomForBuffer()
  {
    try
    {
      const auto tf = tf_buffer_.lookupTransform("odom", "ridgeback_base_link", tf2::TimePointZero);
      amcl_odom_gate::appendOdomSample(
          odom_buf_,
          { rclcpp::Time(tf.header.stamp).seconds(),
            { tf.transform.translation.x, tf.transform.translation.y, tf2::getYaw(tf.transform.rotation) } },
          kOdomBufferSec);
    }
    catch (const tf2::TransformException&)
    {
      // odom->base not ready yet; leave the buffer as-is.
    }
  }

  void publish()
  {
    sampleOdomForBuffer();  // keep the latency-compensation buffer filled at the publish rate

    if (!have_map_odom_)
    {
      // No seed yet: /particle_cloud hasn't arrived or the odom->base lookup keeps failing, so this
      // sole map->odom publisher is emitting nothing. The starvation watchdog below sits after this
      // return and can't see a never-started gate, so surface it here (throttled).
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), kStaleWarnThrottleMs,
                           "amcl_odom_gate: no map->odom yet -- waiting for /particle_cloud and the "
                           "odom->ridgeback_base_link TF; nothing is being broadcast.");
      return;
    }
    // Liveness watchdog: this timer keeps firing regardless of input, so it is the only place
    // that can see onCloud STARVATION -- if beluga stops publishing /particle_cloud or odom->base
    // disappears, updateGate never runs, state_.alpha freezes, and the coast watchdog in onCloud
    // is never reached, yet we keep broadcasting a frozen (future-dated) map->odom. Surface that
    // here so the worst silent failure for a sole broadcaster does not pass unnoticed.
    if (const double stale_s = (now() - last_update_).seconds(); stale_s > kStaleInputSeconds)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), kStaleWarnThrottleMs,
                           "amcl_odom_gate: no /particle_cloud update for %.1f s -- broadcasting a frozen "
                           "map->odom. Check that beluga_amcl and the odom->base TF are alive.",
                           stale_s);
    }
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = now() + rclcpp::Duration::from_seconds(kTransformTolerance);
    t.header.frame_id = "map";
    t.child_frame_id = "odom";
    t.transform.translation.x = map_odom_.x;
    t.transform.translation.y = map_odom_.y;
    t.transform.rotation.z = std::sin(map_odom_.yaw / 2.0);
    t.transform.rotation.w = std::cos(map_odom_.yaw / 2.0);
    tf_broadcaster_.sendTransform(t);
  }

  // onCloud (subscription) writes state_/map_odom_/coasting_/coast_start_/last_update_ and reads
  // odom_buf_; publish (timer) reads map_odom_/last_update_ and writes odom_buf_. main() spins on the
  // default SingleThreadedExecutor, so this node's two callbacks never run concurrently and the
  // access is safe without locking. (tf2_ros::TransformListener spins its own thread that writes
  // tf_buffer_, but that is internally synchronized and touches none of these members.) If this node
  // is ever moved to a MultiThreadedExecutor or separate callback groups, guard the shared state
  // (state_/map_odom_/last_update_/odom_buf_).
  amcl_odom_gate::GateParams params_;
  amcl_odom_gate::detail::GateState state_;
  Pose2 map_odom_{ 0.0, 0.0, 0.0 };
  bool have_map_odom_ = false;
  bool coasting_ = false;  // watchdog: is the gate currently holding (alpha < kCoastAlpha)?
  // coast_start_/last_update_ are default-constructed RCL_SYSTEM_TIME; now() is RCL_ROS_TIME under
  // use_sim_time and subtracting mismatched clock types throws. Both are only subtracted AFTER being
  // assigned now() on a prior tick (coast_start_ behind coasting_, last_update_ behind have_map_odom_),
  // so the clock types always match at the subtraction sites -- keep that ordering if refactoring.
  rclcpp::Time coast_start_;  // when the current continuous coast began (valid only while coasting_)
  rclcpp::Time last_update_;  // time of the last successful onCloud update (valid once have_map_odom_)
  std::vector<amcl_odom_gate::OdomSample> odom_buf_;  // recent odom->base samples for latency compensation
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr cloud_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AmclOdomGate>());
  rclcpp::shutdown();
  return 0;
}
