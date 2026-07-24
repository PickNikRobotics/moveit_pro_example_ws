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

#include "hangar_sim/amcl_odom_gate_logic.hpp"

#include <algorithm>

namespace amcl_odom_gate::detail
{
Pose2 updateGate(const Pose2& candidate, double spread, double now, const GateParams& p, GateState& s)
{
  if (!s.have_held)
  {
    s.held = candidate;
    s.have_held = true;
    s.alpha = 1.0;
    return s.held;
  }

  // Trigger 1: spread hysteresis (filter self-reported uncertainty).
  if (spread > p.spread_hold)
  {
    s.spread_holding = true;
  }
  else if (spread < p.spread_resume)
  {
    s.spread_holding = false;
  }

  // Innovation is gated on BOTH position and yaw: a heading teleport (e.g. a 180 deg
  // flip in a symmetric corridor) can leave x/y nearly fixed, so position alone would
  // miss it and follow the flip transparently.
  const bool large_innov = planarDist(candidate, s.held) > p.jump_hold || yawDist(candidate, s.held) > p.jump_hold_yaw;
  bool follow;  // follow (accept) the candidate this update, vs hold and coast on odom

  if (!large_innov)
  {
    // Small innovation: normal tracking. Transparent unless spread says hold.
    s.have_provisional = false;
    follow = !s.spread_holding;
  }
  else
  {
    // Trigger 2: large innovation -> provisional persistence check (the way to tell a
    // valid correction from a teleport: valid persists at one pose, ambiguity thrashes).
    // "Same target" is measured against the ANCHOR captured when the clock started, not
    // the latest candidate, so a target creeping just under the tolerance each step
    // still eventually resets rather than being accepted after drifting far.
    const bool same_target = s.have_provisional && planarDist(candidate, s.provisional) <= p.provisional_tol &&
                             yawDist(candidate, s.provisional) <= p.provisional_tol_yaw;
    if (!same_target)
    {
      // New or moving jump target: (re)anchor and restart the persistence clock, hold.
      s.provisional = candidate;
      s.provisional_since = now;
      s.have_provisional = true;
      follow = false;
    }
    else
    {
      // Candidate has persisted near the anchor: accept once it has held long enough.
      // Persistence overrides spread here by design -- a correction that stays put for
      // persist_time is trustworthy even if the cloud is still wide (e.g. recovering
      // from a bad initial seed), which the spread trigger alone would keep rejecting.
      follow = (now - s.provisional_since >= p.persist_time);
    }
  }

  // Asymmetric slew: clamp to a hold IMMEDIATELY (react fast the instant a jump or
  // high spread is detected, so the held pose is not dragged toward a bad candidate),
  // but ease back to transparent GRADUALLY on resume (so re-engaging never teleports).
  const double target = follow ? 1.0 : 0.0;
  s.alpha = (target < s.alpha) ? target : std::min(s.alpha + p.alpha_slew, target);
  s.held = blend(s.held, candidate, s.alpha);
  return s.held;
}
}  // namespace amcl_odom_gate::detail
