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
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

using namespace hangar_sim;
using namespace std::chrono_literals;

namespace
{
constexpr double kEps = 1e-9;

// Message stamps are arbitrary; the node only compares them with each other.
rclcpp::Time stamp(double t)
{
  return rclcpp::Time(static_cast<int64_t>(std::llround((1000.0 + t) * 1e9)));
}

nav_msgs::msg::Odometry odom(double t, const Pose2& p)
{
  nav_msgs::msg::Odometry m;
  m.header.stamp = stamp(t);
  m.pose.pose.position.x = p.x;
  m.pose.pose.position.y = p.y;
  m.pose.pose.orientation.z = std::sin(p.yaw / 2.0);
  m.pose.pose.orientation.w = std::cos(p.yaw / 2.0);
  return m;
}

void expectPose(const Pose2& actual, const Pose2& expected)
{
  EXPECT_NEAR(actual.x, expected.x, 1e-6);
  EXPECT_NEAR(actual.y, expected.y, 1e-6);
  EXPECT_NEAR(wrap(actual.yaw - expected.yaw), 0.0, 1e-6);
}
}  // namespace

// ---- free functions -------------------------------------------------------------------------------------------

TEST(PoseAlgebra, WrapFoldsIntoMinusPiToPi)
{
  EXPECT_NEAR(wrap(0.3), 0.3, kEps);
  EXPECT_NEAR(wrap(2.0 * M_PI + 0.3), 0.3, kEps);
  EXPECT_NEAR(wrap(-2.0 * M_PI - 0.3), -0.3, kEps);
  EXPECT_NEAR(wrap(M_PI + 0.1), -M_PI + 0.1, kEps);
}

TEST(PoseAlgebra, FromOdomReadsPlanarPose)
{
  expectPose(fromOdom(odom(0.0, { 1.0, -2.0, 0.7 })), { 1.0, -2.0, 0.7 });
}

TEST(PoseAlgebra, ComposeRotatesTheSecondPoseIntoTheFirstFrame)
{
  expectPose(compose({ 1.0, 0.0, M_PI / 2.0 }, { 1.0, 0.0, 0.0 }), { 1.0, 1.0, M_PI / 2.0 });
  expectPose(compose({ 1.0, 2.0, 0.3 }, { 0.0, 0.0, 0.0 }), { 1.0, 2.0, 0.3 });
}

TEST(PoseAlgebra, InvertUndoesCompose)
{
  const Pose2 p{ 1.5, -0.5, 2.0 };
  expectPose(compose(p, invert(p)), { 0.0, 0.0, 0.0 });
  expectPose(compose(invert(p), p), { 0.0, 0.0, 0.0 });
}

TEST(PoseAlgebra, LerpInterpolatesPositionAndYaw)
{
  expectPose(lerp({ 0.0, 0.0, 0.0 }, { 2.0, 4.0, 0.4 }, 0.25), { 0.5, 1.0, 0.1 });
  expectPose(lerp({ 0.0, 0.0, 0.0 }, { 2.0, 4.0, 0.4 }, 0.0), { 0.0, 0.0, 0.0 });
  expectPose(lerp({ 0.0, 0.0, 0.0 }, { 2.0, 4.0, 0.4 }, 1.0), { 2.0, 4.0, 0.4 });
}

TEST(PoseAlgebra, LerpTakesTheShortWayAcrossPi)
{
  const Pose2 mid = lerp({ 0.0, 0.0, M_PI - 0.1 }, { 0.0, 0.0, -M_PI + 0.1 }, 0.5);
  EXPECT_NEAR(std::abs(wrap(mid.yaw)), M_PI, 1e-6);
}

TEST(PoseAlgebra, OdomToWorldIsIdentityWhenEstimateEqualsTruth)
{
  const Pose2 p{ 3.0, -4.0, 1.0 };
  expectPose(odomToWorld(p, p), { 0.0, 0.0, 0.0 });
}

TEST(PoseAlgebra, OdomToWorldIsEstimateTimesInverseTruth)
{
  const Pose2 est{ 1.0, 2.0, 0.3 }, truth{ 0.5, -1.0, -0.2 };
  // Applying the result to truth must give back the estimate: (odom -> world) (world -> base).
  expectPose(compose(odomToWorld(est, truth), truth), est);
}

TEST(PoseAlgebra, ToTransformFillsFramesStampAndPlanarRotation)
{
  const auto tf = toTransform({ 1.0, 2.0, M_PI / 2.0 }, stamp(5.0));
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
  h.add(stamp(0.0), { 0.0, 0.0, 0.0 });
  h.add(stamp(0.1), { 1.0, 0.0, 0.0 });
  h.add(stamp(0.2), { 5.0, 0.0, 0.0 });
  ASSERT_TRUE(h.at(stamp(0.0)).has_value());
  expectPose(*h.at(stamp(0.0)), { 0.0, 0.0, 0.0 });
  expectPose(*h.at(stamp(0.1)), { 1.0, 0.0, 0.0 });
  expectPose(*h.at(stamp(0.2)), { 5.0, 0.0, 0.0 });
}

TEST(TruthHistory, InterpolatesBetweenTheBracketingSamples)
{
  TruthHistory h;
  h.add(stamp(0.0), { 0.0, 0.0, 0.0 });
  h.add(stamp(0.1), { 1.0, 2.0, 0.2 });
  h.add(stamp(0.3), { 5.0, 2.0, 0.2 });
  expectPose(*h.at(stamp(0.05)), { 0.5, 1.0, 0.1 });  // halfway through the first interval
  expectPose(*h.at(stamp(0.2)), { 3.0, 2.0, 0.2 });   // halfway through the second, wider one
}

TEST(TruthHistory, InterpolatesYawAcrossPiTheShortWay)
{
  TruthHistory h;
  h.add(stamp(0.0), { 0.0, 0.0, M_PI - 0.1 });
  h.add(stamp(0.2), { 0.0, 0.0, -M_PI + 0.1 });
  EXPECT_NEAR(std::abs(wrap(h.at(stamp(0.1))->yaw)), M_PI, 1e-6);
}

TEST(TruthHistory, NothingBeforeTheOldestSample)
{
  TruthHistory h;
  h.add(stamp(1.0), { 0.0, 0.0, 0.0 });
  h.add(stamp(1.1), { 1.0, 0.0, 0.0 });
  EXPECT_FALSE(h.at(stamp(0.99)).has_value());
}

TEST(TruthHistory, ClampsToNewestWithinTheAheadTolerance)
{
  TruthHistory h;
  h.add(stamp(0.0), { 0.0, 0.0, 0.0 });
  h.add(stamp(0.1), { 1.0, 0.0, 0.0 });
  expectPose(*h.at(stamp(0.1 + kEstAheadToleranceSec - 0.01)), { 1.0, 0.0, 0.0 });
}

TEST(TruthHistory, NothingBeyondTheAheadTolerance)
{
  TruthHistory h;
  h.add(stamp(0.0), { 0.0, 0.0, 0.0 });
  h.add(stamp(0.1), { 1.0, 0.0, 0.0 });
  EXPECT_FALSE(h.at(stamp(0.1 + kEstAheadToleranceSec + 0.01)).has_value());
}

TEST(TruthHistory, SingleSampleAnswersOnlyItsOwnInstantAndTheToleranceAfter)
{
  TruthHistory h;
  h.add(stamp(0.0), { 2.0, 0.0, 0.0 });
  expectPose(*h.at(stamp(0.0)), { 2.0, 0.0, 0.0 });
  expectPose(*h.at(stamp(0.03)), { 2.0, 0.0, 0.0 });
  EXPECT_FALSE(h.at(stamp(-0.01)).has_value());
}

TEST(TruthHistory, ClockRewindDiscardsTheOldSamples)
{
  TruthHistory h;
  EXPECT_TRUE(h.add(stamp(10.0), { 0.0, 0.0, 0.0 }));
  EXPECT_TRUE(h.add(stamp(10.1), { 1.0, 0.0, 0.0 }));
  EXPECT_FALSE(h.add(stamp(2.0), { 9.0, 0.0, 0.0 }));  // sim reset
  EXPECT_FALSE(h.at(stamp(10.1)).has_value());
  expectPose(*h.at(stamp(2.0)), { 9.0, 0.0, 0.0 });
}

TEST(TruthHistory, EqualStampIsNotARewind)
{
  TruthHistory h;
  EXPECT_TRUE(h.add(stamp(1.0), { 0.0, 0.0, 0.0 }));
  EXPECT_TRUE(h.add(stamp(1.0), { 1.0, 0.0, 0.0 }));
}

TEST(TruthHistory, DropsSamplesOlderThanTheHistoryWindow)
{
  TruthHistory h;
  h.add(stamp(0.0), { 0.0, 0.0, 0.0 });
  h.add(stamp(0.5), { 1.0, 0.0, 0.0 });
  h.add(stamp(1.5), { 2.0, 0.0, 0.0 });  // 0.0 is now 1.5 s old; 0.5 is exactly kTruthHistorySec old
  EXPECT_FALSE(h.at(stamp(0.2)).has_value());
  expectPose(*h.at(stamp(0.5)), { 1.0, 0.0, 0.0 });
}

TEST(TruthHistory, CapsTheNumberOfSamples)
{
  TruthHistory h;
  const int n = static_cast<int>(kTruthHistoryMax) + 100;
  for (int i = 0; i < n; ++i)
  {
    h.add(stamp(i * 1e-5), { static_cast<double>(i), 0.0, 0.0 });  // all well inside the time window
  }
  EXPECT_FALSE(h.at(stamp(0.0)).has_value());  // the oldest 100 are gone
  expectPose(*h.at(stamp((n - 1) * 1e-5)), { static_cast<double>(n - 1), 0.0, 0.0 });
  expectPose(*h.at(stamp(100 * 1e-5)), { 100.0, 0.0, 0.0 });  // the oldest survivor
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

  /// Transforms broadcast while spinning for `duration`, starting from a clean slate.
  std::vector<geometry_msgs::msg::TransformStamped> broadcastDuring(std::chrono::milliseconds duration)
  {
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
  drift_->onEst(odom(0.0, { 1.0, 0.0, 0.0 }));
  EXPECT_TRUE(broadcastDuring(150ms).empty());  // estimate but no truth
}

TEST_F(OdomWorldDriftTest, PublishesEstimateTimesInverseTruth)
{
  const Pose2 est{ 1.0, 2.0, 0.3 }, truth{ 0.5, -1.0, -0.2 };
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

TEST_F(OdomWorldDriftTest, DifferencesTruthAtTheEstimatesOwnStamp)
{
  drift_->onTruth(odom(0.0, { 0.0, 0.0, 0.0 }));
  drift_->onTruth(odom(0.2, { 2.0, 0.0, 0.0 }));
  drift_->onEst(odom(0.1, { 1.0, 0.0, 0.0 }));  // truth was at x = 1 then, so no drift
  const auto tfs = broadcastDuring(200ms);
  ASSERT_FALSE(tfs.empty());
  EXPECT_NEAR(tfs.back().transform.translation.x, 0.0, 1e-6);
}

TEST_F(OdomWorldDriftTest, WithholdsWhenTheEstimateFallsOutsideTheTruthHistory)
{
  drift_->onTruth(odom(0.0, { 0.0, 0.0, 0.0 }));
  drift_->onTruth(odom(0.1, { 1.0, 0.0, 0.0 }));
  drift_->onEst(odom(0.1 + kEstAheadToleranceSec + 0.1, { 1.0, 0.0, 0.0 }));
  EXPECT_TRUE(broadcastDuring(150ms).empty());
  drift_->onEst(odom(-0.5, { 1.0, 0.0, 0.0 }));
  EXPECT_TRUE(broadcastDuring(150ms).empty());
}

TEST_F(OdomWorldDriftTest, WithholdsOnceTheEstimateGoesStale)
{
  drift_->onTruth(odom(0.0, { 0.0, 0.0, 0.0 }));
  drift_->onTruth(odom(0.1, { 1.0, 0.0, 0.0 }));
  drift_->onEst(odom(0.05, { 1.0, 0.0, 0.0 }));
  EXPECT_FALSE(broadcastDuring(150ms).empty());
  std::this_thread::sleep_for(std::chrono::duration<double>(kEstStaleSec + 0.1));
  EXPECT_TRUE(broadcastDuring(150ms).empty());
  drift_->onEst(odom(0.05, { 1.0, 0.0, 0.0 }));  // a fresh estimate resumes publishing
  EXPECT_FALSE(broadcastDuring(150ms).empty());
}

TEST_F(OdomWorldDriftTest, SimResetDropsTheEstimate)
{
  drift_->onTruth(odom(5.0, { 0.0, 0.0, 0.0 }));
  drift_->onTruth(odom(5.1, { 1.0, 0.0, 0.0 }));
  drift_->onEst(odom(5.05, { 1.0, 0.0, 0.0 }));
  EXPECT_FALSE(broadcastDuring(150ms).empty());
  drift_->onTruth(odom(0.5, { 0.0, 0.0, 0.0 }));  // the sim clock went backwards
  EXPECT_TRUE(broadcastDuring(150ms).empty());
  drift_->onEst(odom(0.5, { 0.0, 0.0, 0.0 }));
  EXPECT_FALSE(broadcastDuring(150ms).empty());
}

TEST_F(OdomWorldDriftTest, StopsBroadcastingOnceDestroyed)
{
  drift_->onTruth(odom(0.0, { 0.0, 0.0, 0.0 }));
  drift_->onEst(odom(0.0, { 1.0, 0.0, 0.0 }));
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
