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

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include "hangar_sim/amcl_odom_gate_logic.hpp"

namespace
{
constexpr double kPubPeriod = 0.033;         // 30 Hz map->odom broadcast
constexpr double kTransformTolerance = 0.1;  // future-date the transform so TF can extrapolate [s]

[[nodiscard]] double yawOf(const geometry_msgs::msg::Quaternion& q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}
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
    params_.alpha_slew = declare_parameter("alpha_slew", params_.alpha_slew);
    // beluga publishes /particle_cloud BEST_EFFORT; match it or we receive nothing.
    cloud_sub_ = create_subscription<geometry_msgs::msg::PoseArray>(
        "/particle_cloud", rclcpp::SensorDataQoS(),
        [this](geometry_msgs::msg::PoseArray::ConstSharedPtr m) { onCloud(*m); });
    timer_ = create_wall_timer(std::chrono::duration<double>(kPubPeriod), [this]() { publish(); });
  }

private:
  void onCloud(const geometry_msgs::msg::PoseArray& cloud)
  {
    const size_t n = cloud.poses.size();
    if (n < 2)
    {
      return;
    }
    // Circular mean (AMCL's estimate) and RMS spread (its confidence) over the cloud.
    double sx = 0.0, sy = 0.0, ss = 0.0, sc = 0.0;
    for (const auto& p : cloud.poses)
    {
      sx += p.position.x;
      sy += p.position.y;
      const double y = yawOf(p.orientation);
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

    // odom->base is fuse's estimate (via odom->world->base); skip if TF isn't ready.
    Pose2 odom_base{ 0.0, 0.0, 0.0 };
    try
    {
      const auto t = tf_buffer_.lookupTransform("odom", "ridgeback_base_link", tf2::TimePointZero);
      odom_base = { t.transform.translation.x, t.transform.translation.y, yawOf(t.transform.rotation) };
    }
    catch (const tf2::TransformException&)
    {
      return;
    }

    // AMCL's implied map->odom = (map->base) o inv(odom->base); gate decides what to broadcast.
    const Pose2 candidate = amcl_odom_gate::compose(mean, amcl_odom_gate::invert(odom_base));
    map_odom_ = amcl_odom_gate::detail::updateGate(candidate, spread, this->now().seconds(), params_, state_);
    have_map_odom_ = true;
  }

  void publish()
  {
    if (!have_map_odom_)
    {
      return;
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

  // onCloud (subscription) writes state_/map_odom_ and publish (timer) reads map_odom_.
  // main() spins on the default SingleThreadedExecutor, so these callbacks never run
  // concurrently and the access is safe without locking. If this node is ever moved to
  // a MultiThreadedExecutor or separate callback groups, guard state_/map_odom_.
  amcl_odom_gate::GateParams params_;
  amcl_odom_gate::detail::GateState state_;
  Pose2 map_odom_{ 0.0, 0.0, 0.0 };
  bool have_map_odom_ = false;
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
