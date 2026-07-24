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
using amcl_odom_gate::blend;
using amcl_odom_gate::compose;
using amcl_odom_gate::GateParams;
using amcl_odom_gate::invert;
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
