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

#include <chrono>
#include <cmath>
#include <memory>
#include <thread>
#include <vector>

#include <gtest/gtest.h>
#include <fuse_msgs/srv/set_pose.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <tf2/utils.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

using namespace hangar_sim;
using namespace std::chrono_literals;

namespace
{
constexpr double kEps = 1e-9;
// Longer than one kPubPeriod tick, so anything already sent is delivered before the slate is wiped.
constexpr auto kDrainInterval = std::chrono::milliseconds(50);

// Message stamps are arbitrary; the node only compares them with each other.
rclcpp::Time stamp(double t)
{
  return rclcpp::Time(static_cast<int64_t>(std::llround((1000.0 + t) * 1e9)));
}

double wrap(double a)
{
  return std::atan2(std::sin(a), std::cos(a));
}

tf2::Transform pose(double x, double y, double yaw)
{
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  return tf2::Transform(q, tf2::Vector3(x, y, 0.0));
}

nav_msgs::msg::Odometry odom(double t, const tf2::Transform& p)
{
  nav_msgs::msg::Odometry m;
  m.header.stamp = stamp(t);
  tf2::toMsg(p, m.pose.pose);
  return m;
}

void expectPose(const tf2::Transform& actual, const tf2::Transform& expected)
{
  EXPECT_NEAR(actual.getOrigin().x(), expected.getOrigin().x(), 1e-6);
  EXPECT_NEAR(actual.getOrigin().y(), expected.getOrigin().y(), 1e-6);
  EXPECT_NEAR(actual.getOrigin().z(), expected.getOrigin().z(), 1e-6);
  EXPECT_NEAR(wrap(tf2::getYaw(actual.getRotation()) - tf2::getYaw(expected.getRotation())), 0.0, 1e-6);
}
}  // namespace

// ---- free functions -------------------------------------------------------------------------------------------

TEST(PoseAlgebra, OdomToWorldIsIdentityWhenEstimateEqualsTruth)
{
  const tf2::Transform p = pose(3.0, -4.0, 1.0);
  expectPose(odomToWorld(p, p), pose(0.0, 0.0, 0.0));
}

TEST(PoseAlgebra, OdomToWorldIsEstimateTimesInverseTruth)
{
  const tf2::Transform est = pose(1.0, 2.0, 0.3), truth = pose(0.5, -1.0, -0.2);
  // Applying the result to truth must give back the estimate: (odom -> world) (world -> base).
  expectPose(odomToWorld(est, truth) * truth, est);
}

TEST(PoseAlgebra, ToTransformFillsFramesStampAndPlanarRotation)
{
  const auto tf = toTransform(pose(1.0, 2.0, M_PI / 2.0), stamp(5.0));
  EXPECT_EQ(tf.header.frame_id, "odom");
  EXPECT_EQ(tf.child_frame_id, "world");
  EXPECT_EQ(rclcpp::Time(tf.header.stamp).nanoseconds(), stamp(5.0).nanoseconds());
  EXPECT_NEAR(tf.transform.translation.x, 1.0, kEps);
  EXPECT_NEAR(tf.transform.translation.y, 2.0, kEps);
  EXPECT_NEAR(tf.transform.translation.z, 0.0, kEps);
  EXPECT_NEAR(tf.transform.rotation.z, std::sin(M_PI / 4.0), kEps);
  EXPECT_NEAR(tf.transform.rotation.w, std::cos(M_PI / 4.0), kEps);
  EXPECT_NEAR(tf.transform.rotation.x, 0.0, kEps);
  EXPECT_NEAR(tf.transform.rotation.y, 0.0, kEps);
}

TEST(Staleness, LimitIsExclusive)
{
  EXPECT_FALSE(isStale(0.0));
  EXPECT_FALSE(isStale(kEstStaleSec));
  EXPECT_TRUE(isStale(kEstStaleSec + 0.01));
}

TEST(Teleport, DrivableStepsAreNotTeleports)
{
  EXPECT_FALSE(isTeleport(pose(1.0, 2.0, 0.3), pose(1.0 + kTeleportJumpM * 0.9, 2.0, 0.3), 0.0));
  EXPECT_FALSE(isTeleport(pose(1.0, 2.0, 0.3), pose(1.0, 2.0, 0.3 + kTeleportJumpRad * 0.9), 0.0));
}

TEST(Teleport, AJumpInPositionOrYawIsATeleport)
{
  EXPECT_TRUE(isTeleport(pose(1.0, 2.0, 0.3), pose(1.0, 2.0 + kTeleportJumpM * 1.1, 0.3), 0.0));
  EXPECT_TRUE(isTeleport(pose(1.0, 2.0, 0.3), pose(1.0, 2.0, 0.3 - kTeleportJumpRad * 1.1), 0.0));
}

TEST(Teleport, AllowsForWhatTheBaseCouldDriveAcrossASampleGap)
{
  // A dropped best-effort /odom sample while driving at full speed is not a teleport.
  const double gap = 0.5;
  EXPECT_FALSE(isTeleport(pose(0.0, 0.0, 0.0), pose(kTeleportJumpM + 0.9 * kMaxBaseSpeedMps * gap, 0.0, 0.0), gap));
  EXPECT_TRUE(isTeleport(pose(0.0, 0.0, 0.0), pose(kTeleportJumpM + 1.1 * kMaxBaseSpeedMps * gap, 0.0, 0.0), gap));
  // A reset after a long gap (a paused sim) is still a teleport: the allowance is capped.
  EXPECT_TRUE(isTeleport(pose(0.0, 0.0, 0.0),
                         pose(kTeleportJumpM + kMaxBaseSpeedMps * kMaxSampleGapSec + 0.1, 0.0, 0.0), 30.0));
}

TEST(Teleport, YawIsComparedTheShortWayAcrossPi)
{
  EXPECT_FALSE(isTeleport(pose(0.0, 0.0, M_PI - 0.05), pose(0.0, 0.0, -M_PI + 0.05), 0.0));
}

// ---- TruthHistory ---------------------------------------------------------------------------------------------

TEST(TruthHistory, EmptyHasNothingToReturn)
{
  TruthHistory h;
  EXPECT_TRUE(h.empty());
  EXPECT_FALSE(h.at(stamp(0.0)).has_value());
}

TEST(TruthHistory, ReturnsStoredSampleAtItsOwnStamp)
{
  TruthHistory h;
  h.add(stamp(0.0), pose(0.0, 0.0, 0.0));
  h.add(stamp(0.1), pose(1.0, 0.0, 0.0));
  h.add(stamp(0.2), pose(5.0, 0.0, 0.0));
  ASSERT_TRUE(h.at(stamp(0.0)).has_value());
  expectPose(*h.at(stamp(0.0)), pose(0.0, 0.0, 0.0));
  expectPose(*h.at(stamp(0.1)), pose(1.0, 0.0, 0.0));
  expectPose(*h.at(stamp(0.2)), pose(5.0, 0.0, 0.0));
}

TEST(TruthHistory, InterpolatesBetweenTheBracketingSamples)
{
  TruthHistory h;
  h.add(stamp(0.0), pose(0.0, 0.0, 0.0));
  h.add(stamp(0.1), pose(1.0, 2.0, 0.2));
  h.add(stamp(0.3), pose(5.0, 2.0, 0.2));
  expectPose(*h.at(stamp(0.05)), pose(0.5, 1.0, 0.1));  // halfway through the first interval
  expectPose(*h.at(stamp(0.2)), pose(3.0, 2.0, 0.2));   // halfway through the second, wider one
}

TEST(TruthHistory, InterpolatesYawAcrossPiTheShortWay)
{
  TruthHistory h;
  h.add(stamp(0.0), pose(0.0, 0.0, M_PI - 0.1));
  h.add(stamp(0.2), pose(0.0, 0.0, -M_PI + 0.1));
  EXPECT_NEAR(std::abs(wrap(tf2::getYaw(h.at(stamp(0.1))->getRotation()))), M_PI, 1e-6);
}

TEST(TruthHistory, NothingBeforeTheOldestSample)
{
  TruthHistory h;
  h.add(stamp(1.0), pose(0.0, 0.0, 0.0));
  h.add(stamp(1.1), pose(1.0, 0.0, 0.0));
  EXPECT_FALSE(h.at(stamp(0.99)).has_value());
}

TEST(TruthHistory, ClampsToNewestWithinTheAheadTolerance)
{
  TruthHistory h;
  h.add(stamp(0.0), pose(0.0, 0.0, 0.0));
  h.add(stamp(0.1), pose(1.0, 0.0, 0.0));
  expectPose(*h.at(stamp(0.1 + kEstAheadToleranceSec - 0.01)), pose(1.0, 0.0, 0.0));
}

TEST(TruthHistory, NothingBeyondTheAheadTolerance)
{
  TruthHistory h;
  h.add(stamp(0.0), pose(0.0, 0.0, 0.0));
  h.add(stamp(0.1), pose(1.0, 0.0, 0.0));
  EXPECT_FALSE(h.at(stamp(0.1 + kEstAheadToleranceSec + 0.01)).has_value());
}

TEST(TruthHistory, SingleSampleAnswersOnlyItsOwnInstantAndTheToleranceAfter)
{
  TruthHistory h;
  h.add(stamp(0.0), pose(2.0, 0.0, 0.0));
  expectPose(*h.at(stamp(0.0)), pose(2.0, 0.0, 0.0));
  expectPose(*h.at(stamp(0.03)), pose(2.0, 0.0, 0.0));
  EXPECT_FALSE(h.at(stamp(-0.01)).has_value());
}

TEST(TruthHistory, ClockRewindDiscardsTheOldSamples)
{
  TruthHistory h;
  EXPECT_TRUE(h.add(stamp(10.0), pose(0.0, 0.0, 0.0)));
  EXPECT_TRUE(h.add(stamp(10.1), pose(1.0, 0.0, 0.0)));
  EXPECT_FALSE(h.add(stamp(2.0), pose(9.0, 0.0, 0.0)));  // sim reset
  EXPECT_FALSE(h.at(stamp(10.1)).has_value());
  expectPose(*h.at(stamp(2.0)), pose(9.0, 0.0, 0.0));
}

TEST(TruthHistory, EqualStampIsNotARewind)
{
  TruthHistory h;
  EXPECT_TRUE(h.add(stamp(1.0), pose(0.0, 0.0, 0.0)));
  EXPECT_TRUE(h.add(stamp(1.0), pose(1.0, 0.0, 0.0)));
}

TEST(TruthHistory, DropsSamplesOlderThanTheHistoryWindow)
{
  TruthHistory h;
  h.add(stamp(0.0), pose(0.0, 0.0, 0.0));
  h.add(stamp(0.5), pose(1.0, 0.0, 0.0));
  h.add(stamp(1.5), pose(2.0, 0.0, 0.0));  // 0.0 is now 1.5 s old; 0.5 is exactly kTruthHistorySec old
  EXPECT_FALSE(h.at(stamp(0.2)).has_value());
  expectPose(*h.at(stamp(0.5)), pose(1.0, 0.0, 0.0));
}

TEST(TruthHistory, CapsTheNumberOfSamples)
{
  TruthHistory h;
  const int n = static_cast<int>(kTruthHistoryMax) + 100;
  for (int i = 0; i < n; ++i)
  {
    h.add(stamp(i * 1e-5), pose(static_cast<double>(i), 0.0, 0.0));  // all well inside the time window
  }
  EXPECT_FALSE(h.at(stamp(0.0)).has_value());  // the oldest 100 are gone
  expectPose(*h.at(stamp((n - 1) * 1e-5)), pose(static_cast<double>(n - 1), 0.0, 0.0));
  expectPose(*h.at(stamp(100 * 1e-5)), pose(100.0, 0.0, 0.0));  // the oldest survivor
}

// ---- OdomWorldDrift, composed with an injected node -------------------------------------------------------------

class OdomWorldDriftTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    node_ = std::make_shared<rclcpp::Node>("odom_world_drift_test");
    drift_ = std::make_unique<OdomWorldDrift>(node_);
    tf_sub_ = node_->create_subscription<tf2_msgs::msg::TFMessage>(
        "/tf", 100, [this](const tf2_msgs::msg::TFMessage::ConstSharedPtr& m) {
          received_.insert(received_.end(), m->transforms.begin(), m->transforms.end());
        });
    executor_.add_node(node_);
  }

  void spinFor(std::chrono::milliseconds duration)
  {
    const auto end = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < end)
    {
      executor_.spin_some(10ms);
    }
  }

  /// Transforms broadcast while spinning for `duration`. Drains first, so a sample published at
  /// the tail of an earlier window is not delivered into this one.
  std::vector<geometry_msgs::msg::TransformStamped> broadcastDuring(std::chrono::milliseconds duration)
  {
    spinFor(kDrainInterval);
    received_.clear();
    spinFor(duration);
    return received_;
  }

  std::shared_ptr<rclcpp::Node> node_;
  std::unique_ptr<OdomWorldDrift> drift_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::vector<geometry_msgs::msg::TransformStamped> received_;
};

TEST_F(OdomWorldDriftTest, SubscribesToTheEstimateAndTruthTopics)
{
  EXPECT_EQ(node_->get_subscriptions_info_by_topic("/odom_filtered").size(), 1u);
  EXPECT_EQ(node_->get_subscriptions_info_by_topic("/odom").size(), 1u);
}

TEST_F(OdomWorldDriftTest, PublishesNothingBeforeItHasBothInputs)
{
  EXPECT_TRUE(broadcastDuring(150ms).empty());
  drift_->onEst(odom(0.0, pose(1.0, 0.0, 0.0)));
  EXPECT_TRUE(broadcastDuring(150ms).empty());  // estimate but no truth
}

TEST_F(OdomWorldDriftTest, PublishesEstimateTimesInverseTruth)
{
  const tf2::Transform est = pose(1.0, 2.0, 0.3), truth = pose(0.5, -1.0, -0.2);
  drift_->onTruth(odom(0.0, truth));
  drift_->onTruth(odom(0.1, truth));
  drift_->onEst(odom(0.05, est));
  const auto tfs = broadcastDuring(200ms);
  ASSERT_FALSE(tfs.empty());
  EXPECT_EQ(tfs.back().header.frame_id, "odom");
  EXPECT_EQ(tfs.back().child_frame_id, "world");
  const auto expected = toTransform(odomToWorld(est, truth), stamp(0.0));
  EXPECT_NEAR(tfs.back().transform.translation.x, expected.transform.translation.x, 1e-6);
  EXPECT_NEAR(tfs.back().transform.translation.y, expected.transform.translation.y, 1e-6);
  EXPECT_NEAR(tfs.back().transform.rotation.z, expected.transform.rotation.z, 1e-6);
  EXPECT_NEAR(tfs.back().transform.rotation.w, expected.transform.rotation.w, 1e-6);
}

TEST_F(OdomWorldDriftTest, ProjectsANonPlanarEstimateBackOntoThePlane)
{
  tf2::Quaternion tilted;
  tilted.setRPY(0.05, -0.08, 0.3);
  const tf2::Transform est(tilted, tf2::Vector3(1.0, 2.0, 0.4));
  const tf2::Transform truth = pose(0.5, -1.0, -0.2);
  drift_->onTruth(odom(0.0, truth));
  drift_->onTruth(odom(0.1, truth));
  drift_->onEst(odom(0.05, est));
  const auto tfs = broadcastDuring(200ms);
  ASSERT_FALSE(tfs.empty());
  const auto& t = tfs.back().transform;
  EXPECT_NEAR(t.translation.z, 0.0, kEps);
  EXPECT_NEAR(t.rotation.x, 0.0, kEps);
  EXPECT_NEAR(t.rotation.y, 0.0, kEps);
  // The planar part of the SE(3) difference survives the projection.
  const tf2::Transform se3 = est * truth.inverse();
  tf2::Quaternion q;
  tf2::fromMsg(t.rotation, q);
  EXPECT_NEAR(wrap(tf2::getYaw(q) - tf2::getYaw(se3.getRotation())), 0.0, 1e-6);
  EXPECT_NEAR(t.translation.x, se3.getOrigin().x(), 1e-6);
  EXPECT_NEAR(t.translation.y, se3.getOrigin().y(), 1e-6);
}

TEST_F(OdomWorldDriftTest, DifferencesTruthAtTheEstimatesOwnStamp)
{
  drift_->onTruth(odom(0.0, pose(0.0, 0.0, 0.0)));
  drift_->onTruth(odom(0.2, pose(0.2, 0.0, 0.0)));
  drift_->onEst(odom(0.1, pose(0.1, 0.0, 0.0)));  // truth was at x = 0.1 then, so no drift
  const auto tfs = broadcastDuring(200ms);
  ASSERT_FALSE(tfs.empty());
  EXPECT_NEAR(tfs.back().transform.translation.x, 0.0, 1e-6);
}

TEST_F(OdomWorldDriftTest, WithholdsWhenTheEstimateFallsOutsideTheTruthHistory)
{
  drift_->onTruth(odom(0.0, pose(0.0, 0.0, 0.0)));
  drift_->onTruth(odom(0.1, pose(0.1, 0.0, 0.0)));
  drift_->onEst(odom(0.1 + kEstAheadToleranceSec + 0.1, pose(1.0, 0.0, 0.0)));
  EXPECT_TRUE(broadcastDuring(150ms).empty());
  drift_->onEst(odom(-0.5, pose(1.0, 0.0, 0.0)));
  EXPECT_TRUE(broadcastDuring(150ms).empty());
}

TEST_F(OdomWorldDriftTest, WithholdsOnceTheEstimateGoesStale)
{
  drift_->onTruth(odom(0.0, pose(0.0, 0.0, 0.0)));
  drift_->onTruth(odom(0.1, pose(0.1, 0.0, 0.0)));
  drift_->onEst(odom(0.05, pose(1.0, 0.0, 0.0)));
  EXPECT_FALSE(broadcastDuring(150ms).empty());
  std::this_thread::sleep_for(std::chrono::duration<double>(kEstStaleSec + 0.1));
  EXPECT_TRUE(broadcastDuring(150ms).empty());
  drift_->onEst(odom(0.05, pose(1.0, 0.0, 0.0)));  // a fresh estimate resumes publishing
  EXPECT_FALSE(broadcastDuring(150ms).empty());
}

TEST_F(OdomWorldDriftTest, SimResetDropsTheEstimate)
{
  drift_->onTruth(odom(5.0, pose(0.0, 0.0, 0.0)));
  drift_->onTruth(odom(5.1, pose(0.1, 0.0, 0.0)));
  drift_->onEst(odom(5.05, pose(1.0, 0.0, 0.0)));
  EXPECT_FALSE(broadcastDuring(150ms).empty());
  drift_->onTruth(odom(0.5, pose(0.0, 0.0, 0.0)));  // the sim clock went backwards
  EXPECT_TRUE(broadcastDuring(150ms).empty());
  drift_->onEst(odom(0.5, pose(0.0, 0.0, 0.0)));
  EXPECT_FALSE(broadcastDuring(150ms).empty());
}

/// Stands in for fuse's set_pose service and AMCL's /initialpose subscription.
class OdomWorldDriftTeleportTest : public OdomWorldDriftTest
{
protected:
  void SetUp() override
  {
    OdomWorldDriftTest::SetUp();
    set_pose_srv_ = node_->create_service<fuse_msgs::srv::SetPose>(
        "/state_estimator/set_pose", [this](const std::shared_ptr<fuse_msgs::srv::SetPose::Request> req,
                                            std::shared_ptr<fuse_msgs::srv::SetPose::Response> res) {
          set_pose_requests_.push_back(req->pose);
          res->success = accept_set_pose_;
        });
    seed_sub_ = node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/initialpose", 10,
        [this](const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr& m) { seeds_.push_back(*m); });
    spinFor(kDrainInterval);  // let the client see the service
  }

  /// Odometry stamped on the node clock, offset by `dt` seconds: the AMCL re-seed waits for an
  /// estimate stamped after fuse accepted its reset, which is a node-clock instant.
  nav_msgs::msg::Odometry odomNow(double dt, const tf2::Transform& p)
  {
    nav_msgs::msg::Odometry m;
    m.header.stamp = node_->get_clock()->now() + rclcpp::Duration::from_seconds(dt);
    tf2::toMsg(p, m.pose.pose);
    return m;
  }

  geometry_msgs::msg::PoseWithCovarianceStamped amclPoseNow(double dt)
  {
    geometry_msgs::msg::PoseWithCovarianceStamped m;
    m.header.stamp = node_->get_clock()->now() + rclcpp::Duration::from_seconds(dt);
    m.header.frame_id = "map";
    return m;
  }

  /// Drives a teleport through to the point where only AMCL's next update is outstanding.
  void teleportAndLetFuseReset(const tf2::Transform& spawn)
  {
    drift_->onTruth(odomNow(0.0, pose(1.3, 0.2, 1.6)));
    drift_->onEst(odomNow(0.0, pose(1.8, 0.3, 1.6)));
    spinFor(kDrainInterval);
    drift_->onTruth(odomNow(0.0, spawn));  // the reset
    spinFor(kDrainInterval);
    drift_->onTruth(odomNow(0.05, spawn));
    drift_->onEst(odomNow(0.05, spawn));  // fuse publishes from its reset pose
    spinFor(kDrainInterval);
  }

  rclcpp::Service<fuse_msgs::srv::SetPose>::SharedPtr set_pose_srv_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr seed_sub_;
  bool accept_set_pose_ = true;
  std::vector<geometry_msgs::msg::PoseWithCovarianceStamped> set_pose_requests_;
  std::vector<geometry_msgs::msg::PoseWithCovarianceStamped> seeds_;
};

TEST_F(OdomWorldDriftTeleportTest, ResetsFuseThenSeedsAmclAtTruthAfterItsNextUpdate)
{
  const tf2::Transform spawn = pose(-0.06, -0.02, -0.02);
  drift_->onAmclPose(amclPoseNow(0.0));  // updates before the reset are irrelevant
  // An AMCL update between the teleport and fuse's reset estimate reaching TF is too early: AMCL
  // has yet to see the reset's jump in odom -> base.
  drift_->onTruth(odomNow(0.0, pose(1.3, 0.2, 1.6)));
  drift_->onEst(odomNow(0.0, pose(1.8, 0.3, 1.6)));
  spinFor(kDrainInterval);
  drift_->onTruth(odomNow(0.0, spawn));
  spinFor(kDrainInterval);
  drift_->onAmclPose(amclPoseNow(0.0));
  spinFor(kDrainInterval);
  EXPECT_TRUE(seeds_.empty());
  drift_->onTruth(odomNow(0.05, spawn));
  drift_->onEst(odomNow(0.05, spawn));
  spinFor(kDrainInterval);
  ASSERT_EQ(set_pose_requests_.size(), 1u);
  EXPECT_EQ(set_pose_requests_[0].header.frame_id, "odom");
  tf2::Transform requested;
  tf2::fromMsg(set_pose_requests_[0].pose.pose, requested);
  expectPose(requested, spawn);
  drift_->onAmclPose(amclPoseNow(-1.0));  // an update that predates the reset estimate
  spinFor(kDrainInterval);
  EXPECT_TRUE(seeds_.empty());

  drift_->onAmclPose(amclPoseNow(0.0));
  spinFor(kDrainInterval);
  ASSERT_EQ(seeds_.size(), 1u);
  EXPECT_EQ(seeds_[0].header.frame_id, "map");
  tf2::Transform seeded;
  tf2::fromMsg(seeds_[0].pose.pose, seeded);
  expectPose(seeded, spawn);
  EXPECT_NEAR(seeds_[0].pose.covariance[0], kReseedXYVariance, kEps);
  EXPECT_NEAR(seeds_[0].pose.covariance[35], kReseedYawVariance, kEps);
  drift_->onAmclPose(amclPoseNow(0.0));  // one seed per teleport
  spinFor(kDrainInterval);
  EXPECT_EQ(seeds_.size(), 1u);
}

TEST_F(OdomWorldDriftTeleportTest, SeedsAmclAnywayIfItNeverUpdates)
{
  teleportAndLetFuseReset(pose(0.0, 0.0, 0.0));
  // Keep the estimate fresh, so odom -> world keeps broadcasting while AMCL stays silent.
  const auto end = std::chrono::steady_clock::now() + std::chrono::duration<double>(kAmclUpdateWaitSec + 0.3);
  while (std::chrono::steady_clock::now() < end && seeds_.empty())
  {
    drift_->onTruth(odomNow(0.0, pose(0.0, 0.0, 0.0)));
    drift_->onEst(odomNow(0.0, pose(0.0, 0.0, 0.0)));
    spinFor(50ms);
  }
  EXPECT_EQ(seeds_.size(), 1u);
}

TEST_F(OdomWorldDriftTeleportTest, DoesNotSeedAmclWhenFuseRejectsTheReset)
{
  accept_set_pose_ = false;
  teleportAndLetFuseReset(pose(0.0, 0.0, 0.0));
  ASSERT_EQ(set_pose_requests_.size(), 1u);
  drift_->onAmclPose(amclPoseNow(0.0));
  spinFor(kDrainInterval);
  EXPECT_TRUE(seeds_.empty());
}

TEST_F(OdomWorldDriftTest, StopsBroadcastingOnceDestroyed)
{
  drift_->onTruth(odom(0.0, pose(0.0, 0.0, 0.0)));
  drift_->onEst(odom(0.0, pose(1.0, 0.0, 0.0)));
  EXPECT_FALSE(broadcastDuring(150ms).empty());
  drift_.reset();
  EXPECT_TRUE(broadcastDuring(150ms).empty());
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
