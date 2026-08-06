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

// Unit tests for the pure gate-decision function detail::updateGate. Each test
// encodes a failure mode observed in sim as a regression: run a candidate stream
// through the gate and assert the broadcast map->odom behaves. No ROS, no TF.

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "hangar_sim/amcl_odom_gate_logic.hpp"

namespace
{
using amcl_odom_gate::appendOdomSample;
using amcl_odom_gate::blend;
using amcl_odom_gate::compose;
using amcl_odom_gate::GateParams;
using amcl_odom_gate::interpolateOdom;
using amcl_odom_gate::invert;
using amcl_odom_gate::OdomSample;
using amcl_odom_gate::planarDist;
using amcl_odom_gate::Pose2;
using amcl_odom_gate::yawDist;
using amcl_odom_gate::detail::GateState;
using amcl_odom_gate::detail::updateGate;

constexpr double kDt = 0.033;  // 30 Hz, matching the node's broadcast period.

// Drive `count` updates of the same candidate/spread through the gate and return the
// final broadcast pose. `t` advances by kDt each step so persistence timers elapse.
Pose2 run(const Pose2& candidate, double spread, int count, const GateParams& p, GateState& s, double& t)
{
  Pose2 out = s.held;
  for (int i = 0; i < count; ++i)
  {
    out = updateGate(candidate, spread, t, p, s);
    t += kDt;
  }
  return out;
}
}  // namespace

// SE(2) math: compose(a, invert(a)) is the identity, and compose/invert round-trip a
// pose. Guards the transform algebra the gate's candidate computation relies on.
TEST(AmclOdomGate, ComposeInvertRoundTrips)
{
  const Pose2 a{ 1.5, -2.0, 0.9 };
  const Pose2 b{ -0.3, 0.7, -1.2 };
  // GIVEN a pose a; WHEN composed with its inverse; THEN the result is the identity.
  const Pose2 id = compose(a, invert(a));
  EXPECT_NEAR(id.x, 0.0, 1e-9);
  EXPECT_NEAR(id.y, 0.0, 1e-9);
  EXPECT_NEAR(std::remainder(id.yaw, 2.0 * M_PI), 0.0, 1e-9);
  // AND applying b then removing it via inverse-compose recovers b.
  const Pose2 rt = compose(invert(a), compose(a, b));
  EXPECT_NEAR(rt.x, b.x, 1e-9);
  EXPECT_NEAR(rt.y, b.y, 1e-9);
  EXPECT_NEAR(yawDist(rt, b), 0.0, 1e-9);
}

// blend wraps yaw through the SHORT arc: blending 170 deg and -170 deg lands near
// 180 deg, not near 0. A long-way-around implementation would fail this.
TEST(AmclOdomGate, BlendYawTakesShortArc)
{
  const Pose2 a{ 0.0, 0.0, 170.0 * M_PI / 180.0 };
  const Pose2 b{ 0.0, 0.0, -170.0 * M_PI / 180.0 };
  const Pose2 mid = blend(a, b, 0.5);
  EXPECT_NEAR(yawDist(mid, Pose2{ 0.0, 0.0, M_PI }), 0.0, 1e-6);
}

// interpolateOdom: empty buffer yields nullopt (caller falls back to a live TF lookup).
TEST(AmclOdomGate, InterpolateOdomEmptyIsNullopt)
{
  // GIVEN an empty odom buffer; WHEN interpolating; THEN there is no value.
  const std::vector<OdomSample> buf;
  EXPECT_FALSE(interpolateOdom(buf, 1.0).has_value());
}

// interpolateOdom: a target strictly between two samples is linearly interpolated in x/y/yaw.
TEST(AmclOdomGate, InterpolateOdomLinearBetweenSamples)
{
  // GIVEN two samples 1 s apart moving +2 m in x and +0.4 rad in yaw.
  const std::vector<OdomSample> buf{ { 10.0, { 0.0, 0.0, 0.0 } }, { 11.0, { 2.0, 0.0, 0.4 } } };

  // WHEN interpolating at the midpoint (t = 10.5).
  const auto out = interpolateOdom(buf, 10.5);

  // THEN it is exactly halfway.
  ASSERT_TRUE(out.has_value());
  EXPECT_NEAR(out->x, 1.0, 1e-9);
  EXPECT_NEAR(out->y, 0.0, 1e-9);
  EXPECT_NEAR(out->yaw, 0.2, 1e-9);
}

// interpolateOdom: a target at or past the ends clamps to the nearest sample (no extrapolation) --
// this is what happens at startup before the buffer spans (now - lag).
TEST(AmclOdomGate, InterpolateOdomClampsOutOfRange)
{
  const std::vector<OdomSample> buf{ { 10.0, { 1.0, 2.0, 0.1 } }, { 11.0, { 3.0, 4.0, 0.2 } } };

  // GIVEN a target BEFORE the buffer; THEN it clamps to the front.
  const auto before = interpolateOdom(buf, 9.0);
  ASSERT_TRUE(before.has_value());
  EXPECT_NEAR(before->x, 1.0, 1e-9);
  EXPECT_NEAR(before->y, 2.0, 1e-9);

  // AND a target AFTER the buffer clamps to the back (never extrapolates forward).
  const auto after = interpolateOdom(buf, 99.0);
  ASSERT_TRUE(after.has_value());
  EXPECT_NEAR(after->x, 3.0, 1e-9);
  EXPECT_NEAR(after->y, 4.0, 1e-9);
}

// interpolateOdom: yaw interpolates through the SHORT arc across the +/-pi wrap, not the long way.
TEST(AmclOdomGate, InterpolateOdomYawWrapsShortArc)
{
  const std::vector<OdomSample> buf{ { 0.0, { 0.0, 0.0, 170.0 * M_PI / 180.0 } },
                                     { 1.0, { 0.0, 0.0, -170.0 * M_PI / 180.0 } } };
  const auto mid = interpolateOdom(buf, 0.5);
  ASSERT_TRUE(mid.has_value());
  EXPECT_NEAR(yawDist(*mid, Pose2{ 0.0, 0.0, M_PI }), 0.0, 1e-6);
}

// interpolateOdom: a single-element buffer returns that element for any target (front() == back(),
// exercising both clamp branches against the same sample).
TEST(AmclOdomGate, InterpolateOdomSingleElement)
{
  const std::vector<OdomSample> buf{ { 5.0, { 1.0, 2.0, 0.3 } } };
  for (const double target : { 4.0, 5.0, 6.0 })
  {
    const auto out = interpolateOdom(buf, target);
    ASSERT_TRUE(out.has_value());
    EXPECT_NEAR(out->x, 1.0, 1e-9);
    EXPECT_NEAR(out->y, 2.0, 1e-9);
    EXPECT_NEAR(out->yaw, 0.3, 1e-9);
  }
}

// interpolateOdom with duplicate trailing timestamps returns cleanly (no NaN) via the
// `target >= back().t` clamp -- NOT via the loop's zero-span guard, which is unreachable for any
// input (the loop returns at the first buf[i].t >= target, so buf[i-1].t < target <= buf[i].t
// always implies a positive span; the guard is purely defensive). appendOdomSample forbids
// duplicate stamps in production anyway; this only pins the pure function's clamp behaviour.
TEST(AmclOdomGate, InterpolateOdomDuplicateTrailingStampsClamp)
{
  const std::vector<OdomSample> buf{ { 0.0, { 0.0, 0.0, 0.0 } },
                                     { 1.0, { 1.0, 0.0, 0.0 } },
                                     { 1.0, { 9.0, 0.0, 0.0 } } };
  const auto out = interpolateOdom(buf, 1.0);  // clamps to back() since target == back().t
  ASSERT_TRUE(out.has_value());
  EXPECT_FALSE(std::isnan(out->x));
  EXPECT_NEAR(out->x, 9.0, 1e-9);  // the back sample, via the clamp
}

// appendOdomSample: newer samples append and the buffer stays trimmed to the keep window behind
// the newest, leaving the buffer time-ascending.
TEST(AmclOdomGate, AppendOdomSampleTrimsToWindow)
{
  std::vector<OdomSample> buf;
  // GIVEN samples 0.0..1.5 s appended with a 1.0 s keep window.
  for (int i = 0; i <= 15; ++i)
  {
    appendOdomSample(buf, { 0.1 * i, { 0.1 * i, 0.0, 0.0 } }, 1.0);
  }
  // THEN nothing older than (1.5 - 1.0) survives, and the buffer is strictly ascending.
  ASSERT_FALSE(buf.empty());
  EXPECT_GE(buf.front().t, 0.5 - 1e-9);
  EXPECT_NEAR(buf.back().t, 1.5, 1e-9);
  for (size_t i = 1; i < buf.size(); ++i)
  {
    EXPECT_GT(buf[i].t, buf[i - 1].t);
  }
}

// appendOdomSample: a stale/duplicate stamp (not strictly newer than the back) is ignored, so the
// buffer never goes out of order even if TF replays an old or repeated transform.
TEST(AmclOdomGate, AppendOdomSampleIgnoresNonNewer)
{
  std::vector<OdomSample> buf;
  appendOdomSample(buf, { 10.0, { 1.0, 0.0, 0.0 } }, 5.0);
  appendOdomSample(buf, { 10.0, { 2.0, 0.0, 0.0 } }, 5.0);  // equal stamp -> ignored
  appendOdomSample(buf, { 9.5, { 3.0, 0.0, 0.0 } }, 5.0);   // older stamp -> ignored
  ASSERT_EQ(buf.size(), 1u);
  EXPECT_NEAR(buf.front().pose.x, 1.0, 1e-9);  // still the original sample
}

// appendOdomSample: a sample far older than the newest (a backward clock jump: sim reset / bag loop)
// flushes the stale buffer and re-seeds, rather than freezing forever under the strictly-newer rule.
TEST(AmclOdomGate, AppendOdomSampleFlushesOnBackwardJump)
{
  std::vector<OdomSample> buf;
  // GIVEN a buffer around t=100 with a 1 s window.
  for (int i = 0; i <= 10; ++i)
  {
    appendOdomSample(buf, { 100.0 + 0.1 * i, { 0.1 * i, 0.0, 0.0 } }, 1.0);
  }
  ASSERT_GT(buf.size(), 1u);

  // WHEN the clock jumps back to t=5 (far more than one window older than the back).
  appendOdomSample(buf, { 5.0, { 42.0, 0.0, 0.0 } }, 1.0);

  // THEN the stale pre-jump samples are gone and the buffer re-seeds from the post-jump sample.
  ASSERT_EQ(buf.size(), 1u);
  EXPECT_NEAR(buf.front().t, 5.0, 1e-9);
  EXPECT_NEAR(buf.front().pose.x, 42.0, 1e-9);
}

// GIVEN a fresh gate; WHEN the first candidate arrives; THEN it is adopted as-is
// (nothing to hold onto yet).
TEST(AmclOdomGate, FirstCandidateAdoptedImmediately)
{
  GateParams p;
  GateState s;
  const Pose2 first{ 1.0, 2.0, 0.3 };
  const Pose2 out = updateGate(first, 0.2, 0.0, p, s);
  EXPECT_NEAR(out.x, 1.0, 1e-9);
  EXPECT_NEAR(out.y, 2.0, 1e-9);
  EXPECT_NEAR(out.yaw, 0.3, 1e-9);
}

// Case 3 (normal tracking, no jitter/accuracy loss): small confident innovations are
// followed transparently -- the gate output equals AMCL within a hair.
TEST(AmclOdomGate, SmallInnovationsTrackTransparently)
{
  GateParams p;
  GateState s;
  double t = 0.0;
  run({ 0.0, 0.0, 0.0 }, 0.2, 1, p, s, t);
  // WHEN AMCL drifts slowly (each step well under jump_hold) at low spread
  Pose2 out{ 0.0, 0.0, 0.0 };
  for (int i = 1; i <= 30; ++i)
  {
    const Pose2 c{ 0.01 * i, 0.0, 0.0 };  // 1 cm/step, far below jump_hold
    out = updateGate(c, 0.2, t, p, s);
    t += kDt;
  }
  // THEN the gate has followed AMCL essentially exactly (no held lag, no jitter).
  EXPECT_NEAR(out.x, 0.30, 0.02);
}

// Case 2 (plane/box ambiguity): a candidate that THRASHES (jumps to a different far
// pose every update) never persists, so the gate holds -- the robot coasts on odom.
TEST(AmclOdomGate, ThrashingTeleportsAreHeld)
{
  GateParams p;
  GateState s;
  double t = 0.0;
  const Pose2 good{ 0.0, 0.0, 0.0 };
  run(good, 0.2, 1, p, s, t);  // establish a confident held pose

  // WHEN AMCL teleports to a DIFFERENT far pose each update (ambiguity artifact),
  // with spread also elevated as it does near unmapped/degenerate geometry.
  Pose2 out{ 0.0, 0.0, 0.0 };
  const double far = 5.0;
  for (int i = 0; i < 60; ++i)
  {
    const double ang = 0.7 * i;  // walk the teleport around so it never repeats a spot
    const Pose2 c{ far * std::cos(ang), far * std::sin(ang), 0.0 };
    out = updateGate(c, 2.0, t, p, s);  // spread 2.0 > spread_hold
    t += kDt;
  }
  // THEN the broadcast pose stayed near the last good pose (held), NOT chasing the
  // teleports -- otherwise the map would flip.
  EXPECT_LT(planarDist(out, good), 0.5);
}

// Case 2b (yaw teleport): a heading flip with x/y essentially unchanged must still be
// caught -- position innovation alone would miss it. The flip thrashes (never
// persists), so the gate holds the last good heading.
TEST(AmclOdomGate, YawTeleportWithStablePositionIsHeld)
{
  GateParams p;
  GateState s;
  double t = 0.0;
  const Pose2 good{ 0.0, 0.0, 0.0 };
  run(good, 0.2, 1, p, s, t);

  // WHEN yaw flips by ~+/-160 deg each update while x/y barely move, at LOW spread
  // (position variance stays small -- the spread trigger cannot see a yaw-only flip).
  Pose2 out{ 0.0, 0.0, 0.0 };
  for (int i = 0; i < 60; ++i)
  {
    const double yaw = ((i % 2) == 0) ? 2.8 : -2.8;  // ~160 deg, alternating
    out = updateGate({ 0.02, 0.0, yaw }, 0.2, t, p, s);
    t += kDt;
  }
  // THEN the broadcast heading stayed near the last good heading (0), not chasing flips.
  EXPECT_LT(yawDist(out, good), 0.35);
}

// Case 1 (bad seed / genuinely-lost recovery): a large correction that PERSISTS at
// the same place is accepted after persist_time -- the gate must NOT lock in the
// wrong pose forever (the limitation the first gate hit).
TEST(AmclOdomGate, PersistentCorrectionIsAccepted)
{
  GateParams p;
  GateState s;
  double t = 0.0;
  run({ 0.0, 0.0, 0.0 }, 0.2, 1, p, s, t);  // held starts at origin (a bad seed)

  // WHEN AMCL consistently reports the true pose 8 m away (a real, stable correction),
  // with the cloud converged there (low spread).
  const Pose2 truth{ 8.0, 0.0, 0.0 };
  Pose2 out{ 0.0, 0.0, 0.0 };
  // Enough steps for persist_time to elapse AND the alpha ramp to converge.
  for (int i = 0; i < 120; ++i)
  {
    out = updateGate(truth, 0.2, t, p, s);
    t += kDt;
  }
  // THEN the gate eventually accepts and converges to the true pose (recovers),
  // rather than holding the wrong seed indefinitely.
  EXPECT_LT(planarDist(out, truth), 0.3);
}

// Boundary: a persistent correction is NOT accepted before persist_time has elapsed
// (pins the timer -- proves it waits, and that persist_time is load-bearing).
TEST(AmclOdomGate, PersistentCorrectionHeldUntilTimerElapses)
{
  GateParams p;
  p.persist_time = 1.0;
  GateState s;
  double t = 0.0;
  run({ 0.0, 0.0, 0.0 }, 0.2, 1, p, s, t);

  const Pose2 truth{ 8.0, 0.0, 0.0 };
  // Just UNDER persist_time worth of updates (0.9 s of 0.033 s steps ~= 27).
  Pose2 out{ 0.0, 0.0, 0.0 };
  for (int i = 0; i < 27; ++i)
  {
    out = updateGate(truth, 0.2, t, p, s);
    t += kDt;
  }
  // THEN it is still held near origin -- the correction has not yet proven persistent.
  EXPECT_LT(planarDist(out, Pose2{ 0.0, 0.0, 0.0 }), 0.5);
}

// Regression: a backward clock jump (sim reset / bag loop) must re-anchor the persistence timer.
// Without re-anchoring, provisional_since stays ahead of `now`, so now - provisional_since is
// negative and a valid persistent correction can never be accepted until the clock climbs back
// past the pre-jump timestamp -- the gate would strand itself holding a stale pose.
TEST(AmclOdomGate, BackwardClockJumpReanchorsPersistence)
{
  GateParams p;
  p.persist_time = 1.0;
  GateState s;
  double t = 100.0;  // start with a large clock so the jump below is unambiguously backward
  run({ 0.0, 0.0, 0.0 }, 0.2, 1, p, s, t);

  // GIVEN a true correction that has begun persisting (anchor captured near t = 100), not yet accepted
  const Pose2 truth{ 8.0, 0.0, 0.0 };
  for (int i = 0; i < 10; ++i)
  {
    updateGate(truth, 0.2, t, p, s);
    t += kDt;
  }

  // WHEN the clock jumps backward and the same true pose keeps persisting from there for > persist_time
  t = 0.0;
  Pose2 out{ 0.0, 0.0, 0.0 };
  for (int i = 0; i < 120; ++i)
  {
    out = updateGate(truth, 0.2, t, p, s);
    t += kDt;
  }

  // THEN the gate re-anchored on the jump and accepts after persist_time from the NEW clock, rather
  // than staying stranded waiting for the pre-jump timestamp (which would leave it near origin).
  EXPECT_LT(planarDist(out, truth), 0.3);
}

// Case 1b (confident-WRONG lock): a large candidate that PERSISTS at one pose but with
// a SEVERELY spread cloud is a divergence (e.g. scan ambiguity sliding along a smooth
// surface), not a real correction. Persistence alone would accept it; spread_accept_max
// must reject it so the gate keeps coasting on odom instead of adopting the wrong pose.
TEST(AmclOdomGate, SevereSpreadPersistentPoseNotAccepted)
{
  GateParams p;
  GateState s;
  double t = 0.0;
  run({ 0.0, 0.0, 0.0 }, 0.2, 1, p, s, t);  // established a confident held pose

  // WHEN AMCL sits at the same far pose well past persist_time, but the cloud stays
  // severely spread (above spread_accept_max) the whole time -- the wrong-lock signature.
  const Pose2 wrong{ 8.0, 0.0, 0.0 };
  const double severe_spread = 5.0;  // > spread_accept_max (3.0)
  Pose2 out{ 0.0, 0.0, 0.0 };
  for (int i = 0; i < 120; ++i)
  {
    out = updateGate(wrong, severe_spread, t, p, s);
    t += kDt;
  }
  // THEN the gate never adopts it -- it stays held near the last good pose (coasting on
  // odom), NOT snapping to the persistent-but-wrong pose.
  EXPECT_LT(planarDist(out, Pose2{ 0.0, 0.0, 0.0 }), 0.5);
}

// Case 1c (right case preserved): a genuine recovery whose cloud starts severely spread
// then CONVERGES (spread drops below the cap) must still be accepted once it tightens --
// the cap waits for spread to fall, it does not reject legitimate corrections.
TEST(AmclOdomGate, RecoveryAcceptedOnceSpreadDrops)
{
  GateParams p;
  GateState s;
  double t = 0.0;
  run({ 0.0, 0.0, 0.0 }, 0.2, 1, p, s, t);

  const Pose2 truth{ 8.0, 0.0, 0.0 };
  Pose2 out{ 0.0, 0.0, 0.0 };
  // Phase 1: persistent at truth but spread still severe -> held (not yet accepted).
  for (int i = 0; i < 60; ++i)
  {
    out = updateGate(truth, 5.0, t, p, s);  // severe spread > spread_accept_max
    t += kDt;
  }
  EXPECT_LT(planarDist(out, Pose2{ 0.0, 0.0, 0.0 }), 0.5) << "must not accept while spread severe";

  // Phase 2: cloud converges (spread drops below the cap) while the pose stays put.
  for (int i = 0; i < 120; ++i)
  {
    out = updateGate(truth, 0.2, t, p, s);  // spread now tight
    t += kDt;
  }
  // THEN the now-converged correction is accepted and the gate recovers to truth.
  EXPECT_LT(planarDist(out, truth), 0.3);
}

// Boundary: pin spread_accept_max. A persisted correction just BELOW the cap is accepted;
// just ABOVE it is held. Probes threshold +/- a small epsilon so the constant cannot
// drift without this test failing (per test-design threshold+/-1 rule).
TEST(AmclOdomGate, SpreadAcceptMaxBoundary)
{
  constexpr double kEps = 0.1;

  // GIVEN spread just UNDER spread_accept_max: a persistent correction is accepted.
  {
    GateParams p;  // spread_accept_max = 3.0
    // Push the spread-hysteresis band above the probed spread so this case isolates
    // spread_accept_max alone -- otherwise the 2.9 probe also trips the default spread_hold (1.5),
    // latching a hold that freezes the alpha ramp part-way and leaves only a ~0.04 m margin.
    p.spread_hold = p.spread_accept_max + 1.0;
    p.spread_resume = p.spread_accept_max;
    GateState s;
    double t = 0.0;
    run({ 0.0, 0.0, 0.0 }, 0.2, 1, p, s, t);
    const Pose2 truth{ 8.0, 0.0, 0.0 };
    Pose2 out{ 0.0, 0.0, 0.0 };
    for (int i = 0; i < 120; ++i)
    {
      out = updateGate(truth, p.spread_accept_max - kEps, t, p, s);
      t += kDt;
    }
    EXPECT_LT(planarDist(out, truth), 0.3) << "just below the cap must be accepted";
  }

  // GIVEN spread just OVER spread_accept_max: the same persistent correction is held.
  {
    GateParams p;
    GateState s;
    double t = 0.0;
    run({ 0.0, 0.0, 0.0 }, 0.2, 1, p, s, t);
    const Pose2 truth{ 8.0, 0.0, 0.0 };
    Pose2 out{ 0.0, 0.0, 0.0 };
    for (int i = 0; i < 120; ++i)
    {
      out = updateGate(truth, p.spread_accept_max + kEps, t, p, s);
      t += kDt;
    }
    EXPECT_LT(planarDist(out, Pose2{ 0.0, 0.0, 0.0 }), 0.5) << "just above the cap must be held";
  }
}

// A persistent-but-wrong pose at severe spread must stay bounded near the last good pose
// for the FULL divergence window -- the gate never drifts toward it (regression for the
// organic plane-nose cascade where the gate snapped to a 6.7 m wrong lock after ~20 s).
TEST(AmclOdomGate, SevereSpreadWrongLockStaysBoundedThroughout)
{
  GateParams p;
  GateState s;
  double t = 0.0;
  const Pose2 good{ 0.0, 0.0, 0.0 };
  run(good, 0.2, 1, p, s, t);

  // WHEN AMCL sits at a far wrong pose with severe spread for a long window (~20 s),
  // far longer than persist_time.
  const Pose2 wrong{ 6.7, 0.0, 0.0 };
  double worst = 0.0;
  for (int i = 0; i < 600; ++i)  // 600 * 0.033 s ~= 20 s
  {
    const Pose2 out = updateGate(wrong, 5.0, t, p, s);
    worst = std::max(worst, planarDist(out, good));
    t += kDt;
  }
  // THEN the broadcast pose never crept toward the wrong lock at any point in the window.
  EXPECT_LT(worst, 0.5);
}

// The spread check is part of the persistence requirement, not an instantaneous gate: a
// wrong pose that persists while spread NOISILY dips below the cap on alternating frames
// must never ratchet the held pose toward it (the blend is stateful and never reverts).
TEST(AmclOdomGate, OscillatingSpreadAcrossCapDoesNotRatchet)
{
  GateParams p;
  GateState s;
  double t = 0.0;
  run({ 0.0, 0.0, 0.0 }, 0.2, 1, p, s, t);

  // WHEN AMCL sits at a far wrong pose whose cloud spread flickers ABOVE and BELOW the cap
  // (severe on even frames, just under on odd) -- so the longest continuously-tight run is a
  // single frame, never persist_time.
  const Pose2 wrong{ 8.0, 0.0, 0.0 };
  double worst = 0.0;
  for (int i = 0; i < 300; ++i)
  {
    const double spread = ((i % 2) == 0) ? 5.0 : 2.0;  // 2.0 < spread_accept_max(3.0) < 5.0
    const Pose2 out = updateGate(wrong, spread, t, p, s);
    worst = std::max(worst, planarDist(out, Pose2{ 0.0, 0.0, 0.0 }));
    t += kDt;
  }
  // THEN intermittent sub-cap dips never accumulate persist_time, so the held pose stays put.
  EXPECT_LT(worst, 0.5);
}

// Case 4 (spread hysteresis): high spread alone holds even for a small innovation,
// and it resumes only after spread drops below the lower resume threshold.
TEST(AmclOdomGate, HighSpreadHoldsAndResumesWithHysteresis)
{
  GateParams p;
  GateState s;
  double t = 0.0;
  run({ 0.0, 0.0, 0.0 }, 0.2, 1, p, s, t);

  // WHEN spread is high, a small nearby candidate is held (not followed).
  Pose2 out{ 0.0, 0.0, 0.0 };
  for (int i = 0; i < 20; ++i)
  {
    out = updateGate({ 0.2, 0.0, 0.0 }, 2.0, t, p, s);  // spread 2.0 > spread_hold
    t += kDt;
  }
  EXPECT_LT(planarDist(out, Pose2{ 0.0, 0.0, 0.0 }), 0.15);

  // AND between spread_resume and spread_hold it stays held (hysteresis: no flicker).
  for (int i = 0; i < 20; ++i)
  {
    out = updateGate({ 0.2, 0.0, 0.0 }, 1.0, t, p, s);  // resume(0.6) < 1.0 < hold(1.5)
    t += kDt;
  }
  EXPECT_LT(planarDist(out, Pose2{ 0.0, 0.0, 0.0 }), 0.2);

  // THEN once spread drops below spread_resume it follows AMCL again.
  for (int i = 0; i < 40; ++i)
  {
    out = updateGate({ 0.2, 0.0, 0.0 }, 0.2, t, p, s);
    t += kDt;
  }
  EXPECT_NEAR(out.x, 0.2, 0.03);
}
