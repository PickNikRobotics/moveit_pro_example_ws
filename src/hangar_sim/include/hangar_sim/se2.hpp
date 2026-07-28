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

// Shared planar SE(2) pose composition for hangar_sim's odometry/localization nodes. Kept ROS-free
// so the pure gate decision logic can include it without pulling in ROS. Previously copy-pasted in
// amcl_odom_gate_logic.hpp and slip_aware_odom.cpp. (For quaternion->yaw use tf2::getYaw, the
// ecosystem standard -- there is no equally-standard planar-SE(2) compose/invert, tf2::Transform
// being full SE(3), so these two stay here.)

#include <cmath>

namespace hangar_sim::se2
{
// Planar SE(2) pose. Members default to zero so a default-constructed Pose2 is the identity rather
// than an indeterminate pose.
struct Pose2
{
  double x = 0.0;    // translation along world/parent x [m]
  double y = 0.0;    // translation along world/parent y [m]
  double yaw = 0.0;  // heading about z [rad]
};

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
}  // namespace hangar_sim::se2
