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

#include <string>
#include <vector>

#include "hangar_sim/odom_world_drift_logic.hpp"

using odom_world_drift::internal::resolveRailIndices;
using ::testing::ElementsAre;

// The three rail joints in canonical (x, y, yaw) order, plus decoys the real /joint_states carries.
TEST(ResolveRailIndices, AllPresentInCanonicalOrder)
{
  // GIVEN a name list with the rail joints first, in kRailJoints order
  const std::vector<std::string> names = { "linear_x_joint", "linear_y_joint", "rotational_yaw_joint" };

  // WHEN resolving
  const auto idx = resolveRailIndices(names);

  // THEN each rail joint maps to its own position
  ASSERT_TRUE(idx.has_value());
  EXPECT_THAT(idx.value(), ElementsAre(0u, 1u, 2u));
}

TEST(ResolveRailIndices, ResolvesByNameNotColumnOrder)
{
  // GIVEN the rail joints interleaved with arm joints and reordered vs kRailJoints
  const std::vector<std::string> names = { "shoulder_pan_joint", "rotational_yaw_joint", "elbow_joint",
                                           "linear_x_joint",     "wrist_1_joint",        "linear_y_joint" };

  // WHEN resolving
  const auto idx = resolveRailIndices(names);

  // THEN indices follow kRailJoints order (x, y, yaw), pointing at the actual columns
  ASSERT_TRUE(idx.has_value());
  EXPECT_THAT(idx.value(), ElementsAre(3u, 5u, 1u));
}

TEST(ResolveRailIndices, MissingAnyRailJointReturnsNullopt)
{
  // GIVEN a list missing rotational_yaw_joint (e.g. an arm-only /joint_states publisher)
  const std::vector<std::string> names = { "linear_x_joint", "linear_y_joint", "elbow_joint" };

  // WHEN resolving
  const auto idx = resolveRailIndices(names);

  // THEN the whole resolve fails rather than returning a partial/garbage index
  EXPECT_FALSE(idx.has_value());
}

TEST(ResolveRailIndices, EmptyNameListReturnsNullopt)
{
  // GIVEN an empty name list
  const std::vector<std::string> names = {};

  // WHEN resolving
  const auto idx = resolveRailIndices(names);

  // THEN it fails cleanly
  EXPECT_FALSE(idx.has_value());
}

TEST(ResolveRailIndices, DuplicateNameTakesFirstOccurrence)
{
  // GIVEN linear_x_joint appearing twice
  const std::vector<std::string> names = { "linear_x_joint", "linear_x_joint", "linear_y_joint",
                                           "rotational_yaw_joint" };

  // WHEN resolving
  const auto idx = resolveRailIndices(names);

  // THEN the first occurrence wins (index 0, not 1)
  ASSERT_TRUE(idx.has_value());
  EXPECT_THAT(idx.value(), ElementsAre(0u, 2u, 3u));
}
