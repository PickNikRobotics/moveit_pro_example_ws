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

namespace amcl_odom_gate
{
// Planar SE(2) pose (x, y, yaw).
struct Pose2
{
  double x, y, yaw;
};

// Trivial SE(2) helpers -- kept inline as they are one-liners the node and tests share.
[[nodiscard]] inline Pose2 invert(const Pose2& p)
{
  const double c = std::cos(p.yaw), s = std::sin(p.yaw);
  return { -c * p.x - s * p.y, s * p.x - c * p.y, -p.yaw };
}
[[nodiscard]] inline Pose2 compose(const Pose2& a, const Pose2& b)
{
  const double c = std::cos(a.yaw), s = std::sin(a.yaw);
  return { a.x + c * b.x - s * b.y, a.y + s * b.x + c * b.y, a.yaw + b.yaw };
}
// Fraction alpha of the way from a to b, wrapping the yaw through the short arc.
[[nodiscard]] inline Pose2 blend(const Pose2& a, const Pose2& b, double alpha)
{
  const double dyaw = std::remainder(b.yaw - a.yaw, 2.0 * M_PI);
  return { a.x + alpha * (b.x - a.x), a.y + alpha * (b.y - a.y), a.yaw + alpha * dyaw };
}
[[nodiscard]] inline double planarDist(const Pose2& a, const Pose2& b)
{
  return std::hypot(a.x - b.x, a.y - b.y);
}
// Absolute yaw difference through the short arc [0, pi].
[[nodiscard]] inline double yawDist(const Pose2& a, const Pose2& b)
{
  return std::abs(std::remainder(a.yaw - b.yaw, 2.0 * M_PI));
}

// Tunables for the gate decision.
struct GateParams
{
  double spread_hold = 1.5;          // spread [m] above which AMCL is held as untrustworthy
  double spread_resume = 0.6;        // resume below this (< spread_hold): hysteresis, no flicker
  double jump_hold = 0.5;            // position innovation [m] above which a candidate goes provisional
  double jump_hold_yaw = 0.35;       // yaw innovation [rad, ~20 deg] above which a candidate goes provisional
  double provisional_tol = 0.3;      // position drift [m] within which successive candidates are the same target
  double provisional_tol_yaw = 0.2;  // yaw drift [rad] within which successive candidates are the same target
  double persist_time = 1.0;         // a provisional target must persist this long [s] to be accepted
  double alpha_slew = 0.05;          // max change in follow fraction per update (smooth ramp)
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
