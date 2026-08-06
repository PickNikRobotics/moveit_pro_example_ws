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

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <cmath>

#include "hangar_sim/slip_aware_odom_logic.hpp"

using slip_aware_odom::computeHoldStamp;
using slip_aware_odom::Pose2;
using slip_aware_odom::RelayParams;
using slip_aware_odom::RelayState;
using slip_aware_odom::updateRelay;
using ::testing::DoubleNear;

namespace
{
constexpr double kTol = 1e-6;
}  // namespace

// -------------------- updateRelay: first sample / continuity --------------------

TEST(UpdateRelay, FirstSampleSeedsOutputAtOriginRegardlessOfSourcePose)
{
  // GIVEN fresh state (out_ default-constructed at the origin, nothing relayed yet)
  RelayState state;
  const RelayParams params;

  // WHEN relaying the first sample, from an arbitrary non-origin source pose
  const auto result = updateRelay(Pose2{ 1.0, 2.0, 0.5 }, /*t=*/10.0, /*yaw_rate=*/0.0, /*source=*/0, params, state);

  // THEN the output stream is seeded at the origin, not the raw source pose: with no prior output
  // to preserve continuity from, the re-anchor (offset = compose(out=identity, invert(p))) makes
  // compose(offset, p) collapse back to identity. This is the documented contract -- the published
  // stream is re-anchored, not absolute -- and is why a single relayed message can never be trusted
  // as an absolute pose.
  EXPECT_THAT(result.pose.x, DoubleNear(0.0, kTol));
  EXPECT_THAT(result.pose.y, DoubleNear(0.0, kTol));
  EXPECT_THAT(result.pose.yaw, DoubleNear(0.0, kTol));
  EXPECT_EQ(state.active, 0);
}

TEST(UpdateRelay, SameSourceContinuesWithoutReanchor)
{
  // GIVEN state anchored by an initial sample from source 0
  RelayState state;
  const RelayParams params;
  updateRelay(Pose2{ 0.0, 0.0, 0.0 }, 10.0, 0.0, 0, params, state);

  // WHEN a second sample from the SAME source arrives, shifted by 2 m
  const auto result = updateRelay(Pose2{ 2.0, 0.0, 0.0 }, 10.1, 0.0, 0, params, state);

  // THEN the output tracks the source's own delta directly (offset is unchanged)
  EXPECT_THAT(result.pose.x, DoubleNear(2.0, kTol));
}

TEST(UpdateRelay, SourceSwitchReanchorsSeamlessly)
{
  // GIVEN a stream anchored at the origin by source 0, then driven to output x=5
  RelayState state;
  const RelayParams params;
  updateRelay(Pose2{ 0.0, 0.0, 0.0 }, 10.0, 0.0, 0, params, state);
  updateRelay(Pose2{ 5.0, 0.0, 0.0 }, 10.02, 0.0, 0, params, state);

  // WHEN source 1 takes over publishing its own pose (a different coordinate origin, e.g. x=0)
  const auto result = updateRelay(Pose2{ 0.0, 0.0, 0.0 }, 10.05, 0.0, /*source=*/1, params, state);

  // THEN the output continues from the last output pose (no jump) -- re-anchored at the switch
  EXPECT_THAT(result.pose.x, DoubleNear(5.0, kTol));
  EXPECT_EQ(state.active, 1);
}

TEST(UpdateRelay, StaleGapPastSourceGapReanchorsEvenOnSameSource)
{
  // GIVEN a stream anchored at the origin by source 0, then driven to output x=5
  RelayState state;
  RelayParams params;
  params.source_gap = 0.2;
  updateRelay(Pose2{ 0.0, 0.0, 0.0 }, 10.0, 0.0, 0, params, state);
  updateRelay(Pose2{ 5.0, 0.0, 0.0 }, 10.02, 0.0, 0, params, state);

  // WHEN the SAME source resumes after a gap longer than source_gap, at a pose that jumped (e.g. the
  // controller reset its own internal odom)
  const auto result = updateRelay(Pose2{ 100.0, 0.0, 0.0 }, 10.02 + params.source_gap + 0.01, 0.0, 0, params, state);

  // THEN the output re-anchors instead of jumping to 100
  EXPECT_THAT(result.pose.x, DoubleNear(5.0, kTol));
}

TEST(UpdateRelay, GapAtExactlySourceGapDoesNotReanchor)
{
  // GIVEN a stream anchored at the origin by source 0, then driven to output x=5
  RelayState state;
  RelayParams params;
  params.source_gap = 0.2;
  updateRelay(Pose2{ 0.0, 0.0, 0.0 }, 10.0, 0.0, 0, params, state);
  updateRelay(Pose2{ 5.0, 0.0, 0.0 }, 10.02, 0.0, 0, params, state);

  // WHEN the same source resumes at EXACTLY the source_gap threshold (not longer than it)
  const auto result = updateRelay(Pose2{ 105.0, 0.0, 0.0 }, 10.02 + params.source_gap, 0.0, 0, params, state);

  // THEN no re-anchor happens (the gap must be strictly greater than source_gap) -- output tracks
  // the source's own delta through the unchanged offset: source moved +100 from its anchor pose
  // (5,0,0), so output moves +100 from its own anchor (5,0,0) too.
  EXPECT_THAT(result.pose.x, DoubleNear(105.0, kTol));
}

// -------------------- updateRelay: yaw-covariance ramp --------------------

TEST(UpdateRelay, NoYawRateKeepsCovarianceAtBase)
{
  // GIVEN fresh state
  RelayState state;
  const RelayParams params;

  // WHEN relaying several samples with zero yaw rate (driving straight)
  auto result = updateRelay(Pose2{}, 10.0, 0.0, 0, params, state);
  result = updateRelay(Pose2{}, 10.1, 0.0, 0, params, state);
  result = updateRelay(Pose2{}, 10.2, 0.0, 0, params, state);

  // THEN the yaw covariance stays at base_cov
  EXPECT_THAT(result.yaw_cov, DoubleNear(params.base_cov, kTol));
}

TEST(UpdateRelay, SustainedSpinRampsCovarianceTowardSpinCov)
{
  // GIVEN fresh state and a spin fast/long enough to fully saturate the leaky integral
  // (|yaw_rate| * dt accumulated over many updates exceeds slip_full)
  RelayState state;
  const RelayParams params;
  double t = 10.0;

  // WHEN relaying many samples at a sustained high yaw rate
  auto result = updateRelay(Pose2{}, t, 0.0, 0, params, state);
  for (int i = 0; i < 200; ++i)
  {
    t += 0.05;
    result = updateRelay(Pose2{}, t, /*yaw_rate=*/5.0, 0, params, state);
  }

  // THEN the covariance saturates at spin_cov (clamped, not overshooting)
  EXPECT_THAT(result.yaw_cov, DoubleNear(params.spin_cov, 1e-3));
}

TEST(UpdateRelay, CovarianceDecaysAfterSpinStops)
{
  // GIVEN state that has ramped up to a high slip covariance from sustained spin
  RelayState state;
  const RelayParams params;
  double t = 10.0;
  auto result = updateRelay(Pose2{}, t, 0.0, 0, params, state);
  for (int i = 0; i < 200; ++i)
  {
    t += 0.05;
    result = updateRelay(Pose2{}, t, 5.0, 0, params, state);
  }
  const double spiking_cov = result.yaw_cov;
  ASSERT_GT(spiking_cov, params.base_cov + 1.0);

  // WHEN the spin stops and several tau-scale gaps of straight driving follow
  for (int i = 0; i < 20; ++i)
  {
    t += params.tau;
    result = updateRelay(Pose2{}, t, 0.0, 0, params, state);
  }

  // THEN the covariance decays back toward base_cov
  EXPECT_THAT(result.yaw_cov, DoubleNear(params.base_cov, 1e-2));
}

TEST(UpdateRelay, MaxDtClampsGrowthButNotDecayOnStaleGap)
{
  // GIVEN state with a moderate slip level
  RelayState state;
  RelayParams params;
  params.max_dt = 0.1;
  double t = 10.0;
  auto result = updateRelay(Pose2{}, t, 0.0, 0, params, state);
  result = updateRelay(Pose2{}, t + 0.05, 5.0, 0, params, state);
  const double slip_before = state.slip;

  // WHEN a stale gap much longer than max_dt arrives (e.g. a handoff silence) with a high yaw rate
  result = updateRelay(Pose2{}, t + 5.05, /*yaw_rate=*/5.0, 0, params, state);

  // THEN the growth contribution is bounded by max_dt (slip cannot spike by yaw_rate * 5.0s), even
  // though the decay term used the full 5.0s elapsed
  const double max_possible_growth = 5.0 * params.max_dt;
  EXPECT_LE(state.slip, slip_before * std::exp(-5.0 / params.tau) + max_possible_growth + kTol);
}

// -------------------- computeHoldStamp --------------------

TEST(ComputeHoldStamp, NothingRelayedYetReturnsNullopt)
{
  // GIVEN no source has been relayed yet (active < 0)
  // WHEN checking whether to hold
  const auto held = computeHoldStamp(/*active=*/-1, /*last_t=*/0.0, /*gap=*/1.0, /*hold_gap=*/0.015);

  // THEN nothing is published
  EXPECT_FALSE(held.has_value());
}

TEST(ComputeHoldStamp, GapBelowThresholdReturnsNullopt)
{
  // GIVEN an active source and a gap shorter than hold_gap
  // WHEN checking whether to hold
  const auto held = computeHoldStamp(/*active=*/0, /*last_t=*/10.0, /*gap=*/0.01, /*hold_gap=*/0.015);

  // THEN no held sample is published yet
  EXPECT_FALSE(held.has_value());
}

TEST(ComputeHoldStamp, GapAtExactlyThresholdHolds)
{
  // GIVEN a gap exactly at hold_gap
  // WHEN checking whether to hold
  const auto held = computeHoldStamp(0, 10.0, /*gap=*/0.015, /*hold_gap=*/0.015);

  // THEN it holds (>= threshold triggers)
  ASSERT_TRUE(held.has_value());
  EXPECT_THAT(held.value(), DoubleNear(10.015, kTol));
}

TEST(ComputeHoldStamp, GapPastThresholdAdvancesStampByFullGap)
{
  // GIVEN a source quiet well past hold_gap
  // WHEN checking whether to hold
  const auto held = computeHoldStamp(0, /*last_t=*/10.0, /*gap=*/0.5, /*hold_gap=*/0.015);

  // THEN the held stamp continues the relayed stream's clock base: last_t + gap
  ASSERT_TRUE(held.has_value());
  EXPECT_THAT(held.value(), DoubleNear(10.5, kTol));
}
