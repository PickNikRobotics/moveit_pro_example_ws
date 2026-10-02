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

#include <array>
#include <cmath>
#include <limits>
#include <set>
#include <string>
#include <utility>

#include <vla_sim_behaviors/cube_stack_trial.hpp>

namespace vla_sim_behaviors
{
namespace
{
using ::testing::AllOf;
using ::testing::ElementsAre;
using ::testing::HasSubstr;

constexpr int kIndexCount = 200;
// Leaves room for a one-ulp difference where a compiler fuses the range arithmetic into one multiply-add.
constexpr double kGoldenTolerance = 1e-12;

// The area the shipped vla_sim checkpoint's demonstrations started from.
constexpr double kMinX = 0.42;
constexpr double kMaxX = 0.60;
constexpr double kMinY = -0.16;
constexpr double kMaxY = 0.16;
constexpr double kMinYawDeg = 0.0;
constexpr double kMaxYawDeg = 90.0;
constexpr double kMinCenterSpacing = 0.07;

MATCHER_P(PlacementNear, expected, "")
{
  *result_listener << "which differs by x " << arg.x - expected.x << ", y " << arg.y - expected.y << ", yaw_deg "
                   << arg.yaw_deg - expected.yaw_deg;
  return std::abs(arg.x - expected.x) <= kGoldenTolerance && std::abs(arg.y - expected.y) <= kGoldenTolerance &&
         std::abs(arg.yaw_deg - expected.yaw_deg) <= kGoldenTolerance;
}

[[nodiscard]] double centerDistance(const CubePlacement& a, const CubePlacement& b)
{
  return std::hypot(a.x - b.x, a.y - b.y);
}

TEST(SampleCubeStackTrial, NegativeTrialIndexFails)
{
  // GIVEN a negative trial index
  // WHEN sampling the trial
  const auto trial = sampleCubeStackTrial(-1, 0);

  // THEN it fails with a message stating the index and what it must be
  ASSERT_FALSE(trial.has_value());
  EXPECT_THAT(trial.error(), AllOf(HasSubstr("trial_index is -1"), HasSubstr("0 or greater")));
}

struct GoldenTrial
{
  std::string name;
  int trial_index;
  int seed;
  std::array<CubePlacement, 3> placements;
  std::string prompt;
};

class GoldenTrialTest : public ::testing::TestWithParam<GoldenTrial>
{
};

TEST_P(GoldenTrialTest, MatchesTheReferenceImplementation)
{
  // GIVEN a trial index and a seed
  const GoldenTrial& golden = GetParam();

  // WHEN sampling the trial
  const auto trial = sampleCubeStackTrial(golden.trial_index, golden.seed);

  // THEN it matches the layout and the prompt the reference implementation gives
  ASSERT_TRUE(trial.has_value()) << trial.error();
  EXPECT_THAT(trial->placements, ElementsAre(PlacementNear(golden.placements[0]), PlacementNear(golden.placements[1]),
                                             PlacementNear(golden.placements[2])));
  EXPECT_EQ(trial->prompt, golden.prompt);
}

// The golden values come from a separate Python implementation of std::seed_seq, MT19937, and the same draw
// arithmetic, not from this code. A change of platform, compiler, or standard library that alters a trial fails here.
INSTANTIATE_TEST_SUITE_P(
    SampleCubeStackTrial, GoldenTrialTest,
    ::testing::Values(GoldenTrial{ "TrialZero",
                                   0,
                                   0,
                                   { { { 0.5281189396924197, 0.14991254958771968, 4.440711148879874 },
                                       { 0.5884322025837883, 0.09390988978498296, 9.005780407485755 },
                                       { 0.5321143358207409, -0.11753503561011229, 74.80121565750997 } } },
                                   "stack the blue cube on the green cube" },
                      GoldenTrial{ "TrialOne",
                                   1,
                                   0,
                                   { { { 0.5820171461395504, 0.14201054954803025, 57.398653370885164 },
                                       { 0.45435316182243946, 0.0016000624273574127, 44.74942559437963 },
                                       { 0.5663645384917858, -0.11602451392336896, 78.990721669634 } } },
                                   "stack the green cube on the blue cube" },
                      // Seeds the engine from the same two numbers as trial 1 with seed 0, in the other order. Its own
                      // layout shows that the seed and the index are not interchangeable, and trial 0's prompt shows
                      // that the seed leaves the pair alone.
                      GoldenTrial{ "TrialZeroWithSeedOne",
                                   0,
                                   1,
                                   { { { 0.4918343944378798, 0.09072457894575317, 28.74802111640812 },
                                       { 0.5956163338327466, 0.040878737757964606, 33.77139403718489 },
                                       { 0.5173104755829288, -0.03867272334104406, 84.76452250443823 } } },
                                   "stack the blue cube on the green cube" },
                      GoldenTrial{ "HighestIndexWithLowestSeed",
                                   std::numeric_limits<int>::max(),
                                   std::numeric_limits<int>::min(),
                                   { { { 0.5776177169784368, -0.0333083162606195, 27.102855555117216 },
                                       { 0.4493729193865127, 0.14588365930180966, 34.11512038980131 },
                                       { 0.5315697553038266, -0.10149997202525238, 28.202126892633494 } } },
                                   "stack the green cube on the blue cube" }),
    [](const ::testing::TestParamInfo<GoldenTrial>& info) { return info.param.name; });

TEST(SampleCubeStackTrial, EveryTrialStaysInTheTrainingAreaAndKeepsTheSpacing)
{
  for (int trial_index = 0; trial_index < kIndexCount; ++trial_index)
  {
    // GIVEN each of the first 200 trial indices
    // WHEN sampling the trial
    const auto trial = sampleCubeStackTrial(trial_index, 0);
    ASSERT_TRUE(trial.has_value()) << "trial " << trial_index << ": " << trial.error();

    // THEN every cube lies inside the training area
    for (const CubePlacement& placement : trial->placements)
    {
      EXPECT_GE(placement.x, kMinX) << "trial " << trial_index;
      EXPECT_LE(placement.x, kMaxX) << "trial " << trial_index;
      EXPECT_GE(placement.y, kMinY) << "trial " << trial_index;
      EXPECT_LE(placement.y, kMaxY) << "trial " << trial_index;
      EXPECT_GE(placement.yaw_deg, kMinYawDeg) << "trial " << trial_index;
      EXPECT_LE(placement.yaw_deg, kMaxYawDeg) << "trial " << trial_index;
    }

    // AND every pair of centers is at least the minimum spacing apart
    const auto& [red, green, blue] = trial->placements;
    EXPECT_GE(centerDistance(red, green), kMinCenterSpacing) << "trial " << trial_index;
    EXPECT_GE(centerDistance(red, blue), kMinCenterSpacing) << "trial " << trial_index;
    EXPECT_GE(centerDistance(green, blue), kMinCenterSpacing) << "trial " << trial_index;
  }
}

TEST(SampleCubeStackTrial, AnySixTrialsInARowAskForEachPairOnce)
{
  const std::set<std::pair<std::string, std::string>> all_pairs{ { "red", "green" }, { "red", "blue" },
                                                                 { "green", "red" }, { "green", "blue" },
                                                                 { "blue", "red" },  { "blue", "green" } };

  for (int first_index = 0; first_index < 12; ++first_index)
  {
    // GIVEN six trial indices in a row, starting anywhere in the cycle
    std::set<std::pair<std::string, std::string>> pairs;
    for (int trial_index = first_index; trial_index < first_index + 6; ++trial_index)
    {
      // WHEN sampling each trial
      const auto trial = sampleCubeStackTrial(trial_index, 0);
      ASSERT_TRUE(trial.has_value()) << "trial " << trial_index << ": " << trial.error();

      pairs.emplace(trial->top_color, trial->bottom_color);
    }

    // THEN the six trials ask for every ordered pair of two different cubes
    EXPECT_EQ(pairs, all_pairs) << "six trials from " << first_index;
  }
}
}  // namespace
}  // namespace vla_sim_behaviors
