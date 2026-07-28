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

#pragma once

// Pure decision logic for the degeneracy-aware map->odom gate. No ROS, no TF, so it
// is unit-testable in isolation (see test/test_amcl_odom_gate.cpp). The node
// (src/amcl_odom_gate.cpp) does the ROS I/O and calls detail::updateGate.
//
// A large innovation in AMCL's implied map->odom is neither hard-accepted nor
// hard-rejected: it goes PROVISIONAL and is accepted only if it PERSISTS at the same
// place for persist_time (a valid correction persists; an ambiguity teleport
// thrashes). Small innovations track AMCL transparently. Particle spread is a
// second, independent hold trigger (hysteresis) for the self-reported-lost case.

#include <algorithm>
#include <cmath>
#include <numbers>
#include <optional>
#include <vector>

#include "hangar_sim/se2.hpp"

namespace amcl_odom_gate
{
// SE(2) primitives are shared with slip_aware_odom (se2.hpp); alias them in so the logic below and
// the tests keep referring to Pose2/compose/invert unqualified.
using hangar_sim::se2::compose;
using hangar_sim::se2::invert;
using hangar_sim::se2::Pose2;
// Fraction alpha of the way from a to b, wrapping the yaw through the short arc.
[[nodiscard]] inline Pose2 blend(const Pose2& a, const Pose2& b, double alpha)
{
  const double dyaw = std::remainder(b.yaw - a.yaw, 2.0 * std::numbers::pi);
  return { a.x + alpha * (b.x - a.x), a.y + alpha * (b.y - a.y), a.yaw + alpha * dyaw };
}
[[nodiscard]] inline double planarDist(const Pose2& a, const Pose2& b)
{
  return std::hypot(a.x - b.x, a.y - b.y);
}
// Absolute yaw difference through the short arc [0, pi].
[[nodiscard]] inline double yawDist(const Pose2& a, const Pose2& b)
{
  return std::abs(std::remainder(a.yaw - b.yaw, 2.0 * std::numbers::pi));
}

// A timestamped odom->base sample, used to reconstruct where odom->base was at a past
// instant so a latency-lagged AMCL correction can be composed at the CONTENT time, not now.
struct OdomSample
{
  double t;    // seconds
  Pose2 pose;  // odom->base at t
};

// Append `s` to a time-ascending odom buffer, then drop samples older than keep_window_sec behind
// the newest. Ignores a sample that is not strictly newer than the current back, so the buffer
// stays strictly ascending (which keeps interpolateOdom's span positive). If a sample arrives more
// than one window OLDER than the back, the clock jumped backward (sim reset / bag loop) -- the
// strictly-newer rule would otherwise freeze the buffer stale forever, so flush and re-seed from it.
// Pure -- unit-tested.
inline void appendOdomSample(std::vector<OdomSample>& buf, const OdomSample& s, double keep_window_sec)
{
  if (!buf.empty())
  {
    if (s.t < buf.back().t - keep_window_sec)
    {
      buf.clear();  // backward time jump: discard the now-unreachable pre-jump samples and re-seed.
    }
    else if (s.t <= buf.back().t)
    {
      return;  // not strictly newer (duplicate / near-stale replay): keep the buffer ascending.
    }
  }
  buf.push_back(s);
  const double cutoff = s.t - keep_window_sec;
  while (buf.size() > 1 && buf.front().t < cutoff)
  {
    buf.erase(buf.begin());
  }
}

// Linear interpolation of odom->base at `target` [s] from a time-ordered (ascending t) buffer.
// Clamps to the nearest end if `target` is outside [front, back] (e.g. before the buffer fills).
// nullopt only if the buffer is empty. Pure -- unit-tested in isolation.
[[nodiscard]] inline std::optional<Pose2> interpolateOdom(const std::vector<OdomSample>& buf, double target)
{
  if (buf.empty())
  {
    return std::nullopt;
  }
  if (target <= buf.front().t)
  {
    return buf.front().pose;
  }
  if (target >= buf.back().t)
  {
    return buf.back().pose;
  }
  for (size_t i = 1; i < buf.size(); ++i)
  {
    if (buf[i].t >= target)
    {
      const double span = buf[i].t - buf[i - 1].t;
      // span is always positive here (the front/back clamps above mean buf[i-1].t < target <=
      // buf[i].t at this point), so the guard is defensive against a hand-built zero-span buffer,
      // not something reachable through appendOdomSample (which keeps stamps strictly ascending).
      const double alpha = (span > 1e-9) ? (target - buf[i - 1].t) / span : 0.0;
      return blend(buf[i - 1].pose, buf[i].pose, alpha);
    }
  }
  return buf.back().pose;  // unreachable given the guards above, but keeps the compiler happy.
}

// Tunables for the gate decision.
struct GateParams
{
  double spread_hold = 1.5;               // spread [m] above which AMCL is held as untrustworthy
  double spread_resume = 0.6;             // resume below this (< spread_hold): hysteresis, no flicker
  double jump_hold = 0.5;                 // position innovation [m] above which a candidate goes provisional
  double jump_hold_yaw = 0.35;            // yaw innovation [rad, ~20 deg] above which a candidate goes provisional
  double provisional_tol = 0.3;           // position drift [m] within which successive candidates are the same target
  double provisional_tol_yaw = 0.2;       // yaw drift [rad] within which successive candidates are the same target
  double persist_time = 1.0;              // a provisional target must persist this long [s] to be accepted
  double spread_accept_max = 3.0;         // spread [m] above which a persisted correction is NEVER accepted
                                          // (keep above spread_hold; default is 2x it). Persistence distinguishes
                                          // a real correction from a thrashing ambiguity, but a confident-WRONG
                                          // lock (e.g. plane scan ambiguity) also persists. A legitimate
                                          // correction converges so its spread drops below this; a divergence
                                          // stays wide, so the gate keeps coasting on odom instead of adopting it.
                                          // Guards POSITIONAL trust only -- heading multimodality is caught by
                                          // jump_hold_yaw + persistence, not by (positional) spread.
  double latency_compensation_sec = 0.0;  // AMCL estimate lag [s]. beluga's cloud content trails the
                                          // true pose by ~this long (its stamp under-reports the lag), so
                                          // during motion the correction is stale by speed*lag. Compose the
                                          // mean with odom->base from (cloud stamp - this), not the latest, so
                                          // map->odom is consistent at the CONTENT time and the live odom
                                          // carries it to now. 0 disables (compose at latest); tune to the
                                          // measured lag (relative to the cloud stamp).
  double alpha_slew = 0.05;               // max change in follow fraction per update (smooth ramp)
};

namespace detail
{
// Mutable state carried between updates.
struct GateState
{
  Pose2 held{ 0.0, 0.0, 0.0 };  // current broadcast map->odom
  double alpha = 1.0;           // follow fraction: 1 transparent, 0 held
  bool have_held = false;
  bool spread_holding = false;         // spread hysteresis latch
  Pose2 provisional{ 0.0, 0.0, 0.0 };  // candidate being watched for persistence
  double provisional_since = 0.0;      // time the current provisional target began persisting [s]
  bool have_provisional = false;
};

// Pure decision. Given AMCL's implied candidate map->odom, the particle spread, the
// current time, params, and mutable state, update and return the map->odom to
// broadcast. No ROS, no TF -- unit-tested in test/test_amcl_odom_gate.cpp.
[[nodiscard]] Pose2 updateGate(const Pose2& candidate, double spread, double now, const GateParams& p, GateState& s);
}  // namespace detail
}  // namespace amcl_odom_gate
