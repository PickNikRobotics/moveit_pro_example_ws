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

// Pure re-anchor/stitch and slip-covariance-ramp logic for slip_aware_odom, split out so it is
// unit-testable without ROS (see test/test_slip_aware_odom.cpp). The node (src/slip_aware_odom.cpp)
// does the ROS I/O and calls updateRelay/computeHoldStamp.

#include <algorithm>
#include <cmath>
#include <optional>

#include "hangar_sim/se2.hpp"

namespace slip_aware_odom
{
using hangar_sim::se2::compose;
using hangar_sim::se2::invert;
using hangar_sim::se2::Pose2;

// Tunables for the relay/covariance-ramp decision.
struct RelayParams
{
  double base_cov = 0.03;   // straight-line yaw covariance
  double spin_cov = 10.0;   // fully-slipping yaw covariance: large enough to effectively drop the
                            // wheel yaw so fuse rides the IMU
  double tau = 0.5;         // slip relaxation time constant [s]
  double slip_full = 0.5;   // leaky-integrated |yaw rate| for full distrust
  double max_dt = 0.1;      // clamp dt so a stale gap can't spike the integral
  double source_gap = 0.2;  // a source quiet longer than this counts as switched out [s]
};

// Mutable state carried between relay updates.
struct RelayState
{
  Pose2 out{ 0.0, 0.0, 0.0 };     // continuous output pose
  Pose2 offset{ 0.0, 0.0, 0.0 };  // current source -> output-frame offset
  int active = -1;                // source id currently being forwarded
  double slip = 0.0;
  double last_t = 0.0;
};

struct RelayResult
{
  Pose2 pose;      // odom pose to publish this update
  double yaw_cov;  // pose-yaw / twist-yaw-rate covariance to publish alongside it
};

// Re-anchor `p` (the active source's pose at time `t`, source id `source`) onto the continuous
// output stream, and ramp the yaw covariance from base_cov toward spin_cov as |yaw_rate| accumulates
// (leaky integral, decaying with tau). On a source switch -- or a stale gap longer than
// params.source_gap -- re-anchor so the output continues seamlessly: pick the offset for which
// compose(offset, p) equals the last output pose. Pure -- unit-tested in isolation.
[[nodiscard]] inline RelayResult updateRelay(const Pose2& p, double t, double yaw_rate, int source,
                                             const RelayParams& params, RelayState& s)
{
  if (source != s.active || s.last_t <= 0.0 || t - s.last_t > params.source_gap)
  {
    s.offset = compose(s.out, invert(p));
    s.active = source;
  }
  s.out = compose(s.offset, p);

  // Full elapsed time drives the decay -- a long gap (e.g. a nav <-> whole-body handoff silence)
  // SHOULD relax the integral toward base_cov. The clamped value drives only the growth term, so a
  // stale gap cannot spike |yaw rate| * dt. Under normal high-rate relay the two are equal.
  const double elapsed = (s.last_t > 0.0) ? std::max(t - s.last_t, 0.0) : 0.0;
  const double dt = std::min(elapsed, params.max_dt);
  s.last_t = t;
  s.slip = s.slip * std::exp(-elapsed / params.tau) + std::abs(yaw_rate) * dt;
  const double cov = params.base_cov + (params.spin_cov - params.base_cov) * std::min(1.0, s.slip / params.slip_full);

  return { s.out, cov };
}

// If the active source has been quiet for at least hold_gap [s] (gap = now - last relayed message
// time, on the node clock), return the stamp [s] to publish a held (zero-velocity) sample at:
// last_t advanced by the elapsed gap, so held samples stay on the relayed stream's clock base and
// strictly increase. nullopt if nothing has been relayed yet (active < 0) or the gap hasn't reached
// hold_gap. Pure -- unit-tested in isolation.
[[nodiscard]] inline std::optional<double> computeHoldStamp(int active, double last_t, double gap, double hold_gap)
{
  if (active < 0 || gap < hold_gap)
  {
    return std::nullopt;
  }
  return last_t + gap;
}
}  // namespace slip_aware_odom
