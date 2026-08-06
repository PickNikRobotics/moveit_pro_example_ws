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

// Pure /joint_states index-resolution for odom_world_drift, split out so its branches
// (all present / reordered / joint absent / duplicate name) are unit-testable without ROS.
// In an internal namespace: NOT part of any supported API (see .claude/rules/cpp-style.md).

#include <algorithm>
#include <array>
#include <cstddef>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace odom_world_drift::internal
{
// Virtual-rail joints encoding the ground-truth base pose in `world`, in (x, y, yaw) order.
//
// Reading these three values as world->base = Pose2{x, y, yaw} is exact only under the sim's rail
// kinematics: the chain is prismatic-x -> prismatic-y -> revolute-z with the revolute joint
// base-most, so the prismatic values are un-rotated world translations and world->base reduces to
// [R(yaw), (x, y)]. It also assumes ridgeback_base_link IS the revolute joint's child frame with no
// intervening fixed offset -- the predecessor read world->base via a full SE(3) TF lookup and so was
// robust to such an offset; this direct read is not. A URDF change that reorders the chain or inserts
// a carriage offset would silently bias world->base (and thus map->odom). Both hold in the shipped
// hangar_sim URDF; this note is here so a future edit that breaks either fails loudly in review.
inline constexpr std::array<std::string_view, 3> kRailJoints = { "linear_x_joint", "linear_y_joint",
                                                                 "rotational_yaw_joint" };

// Resolve the indices of kRailJoints within a /joint_states name list, in kRailJoints order.
// Returns nullopt if any rail joint is absent -- /joint_states is multi-publisher, so a given
// message may omit them entirely. On a duplicate name the first occurrence wins.
[[nodiscard]] inline std::optional<std::array<std::size_t, kRailJoints.size()>>
resolveRailIndices(const std::vector<std::string>& names)
{
  std::array<std::size_t, kRailJoints.size()> out{};
  for (std::size_t j = 0; j < kRailJoints.size(); ++j)
  {
    const auto it = std::ranges::find(names, kRailJoints[j]);
    if (it == names.end())
    {
      return std::nullopt;
    }
    out[j] = static_cast<std::size_t>(std::distance(names.begin(), it));
  }
  return out;
}
}  // namespace odom_world_drift::internal
