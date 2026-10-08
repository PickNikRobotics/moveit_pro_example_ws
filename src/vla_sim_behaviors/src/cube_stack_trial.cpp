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

#include <vla_sim_behaviors/cube_stack_trial.hpp>

#include <cstddef>
#include <cstdint>
#include <random>
#include <string_view>

#include <fmt/format.h>

namespace vla_sim_behaviors
{
namespace
{
// The area the shipped checkpoint's demonstrations started from, measured from the 360 layouts that generated its
// dataset, PickNikRobotics/kinova_gen3_cube_stack_sim. Positions are cube centers in mj_world, in meters.
constexpr double kMinX = 0.42;
constexpr double kMaxX = 0.60;
constexpr double kMinY = -0.16;
constexpr double kMaxY = 0.16;
constexpr double kMinYawDeg = 0.0;
constexpr double kMaxYawDeg = 90.0;
constexpr double kMinCenterSpacing = 0.07;

struct CubePair
{
  std::string_view top_color;
  std::string_view bottom_color;
};
// Every ordered pair of two different cubes. Trial i asks for pair i modulo 6.
constexpr std::array<CubePair, 6> kCubePairs{ { { "blue", "green" },
                                                { "green", "blue" },
                                                { "red", "green" },
                                                { "green", "red" },
                                                { "red", "blue" },
                                                { "blue", "red" } } };

constexpr double kTwoToThe26 = 67108864.0;
constexpr double kTwoToThe53 = 9007199254740992.0;

// A double in [0, 1) from 53 random bits of two draws, the same mapping as MT19937's reference genrand_res53.
[[nodiscard]] double drawUnitInterval(std::mt19937& engine)
{
  const auto high_bits = static_cast<std::uint32_t>(engine() >> 5U);
  const auto low_bits = static_cast<std::uint32_t>(engine() >> 6U);
  return (static_cast<double>(high_bits) * kTwoToThe26 + static_cast<double>(low_bits)) / kTwoToThe53;
}

[[nodiscard]] double drawInRange(std::mt19937& engine, const double min, const double max)
{
  return min + drawUnitInterval(engine) * (max - min);
}

[[nodiscard]] bool centersAreSpacedApart(const std::array<CubePlacement, 3>& placements)
{
  for (std::size_t i = 0; i < placements.size(); ++i)
  {
    for (std::size_t j = i + 1; j < placements.size(); ++j)
    {
      const double dx = placements[i].x - placements[j].x;
      const double dy = placements[i].y - placements[j].y;
      if (dx * dx + dy * dy < kMinCenterSpacing * kMinCenterSpacing)
      {
        return false;
      }
    }
  }
  return true;
}
}  // namespace

tl::expected<CubeStackTrial, std::string> sampleCubeStackTrial(const int trial_index, const int seed)
{
  if (trial_index < 0)
  {
    return tl::make_unexpected(fmt::format("trial_index is {}, but it must be 0 or greater.", trial_index));
  }

  CubeStackTrial trial;
  const CubePair& pair = kCubePairs[static_cast<std::size_t>(trial_index) % kCubePairs.size()];
  trial.top_color = pair.top_color;
  trial.bottom_color = pair.bottom_color;
  trial.prompt = fmt::format("stack the {} cube on the {} cube", trial.top_color, trial.bottom_color);

  // The C++ standard fixes how std::seed_seq mixes its values, so the engine starts in the same state on every
  // standard library.
  std::seed_seq seed_sequence{ static_cast<std::uint32_t>(seed), static_cast<std::uint32_t>(trial_index) };
  std::mt19937 engine{ seed_sequence };

  // Every draw below is its own statement, so every compiler makes the draws in the same order and gives the same
  // trial. C++ leaves the order of a function call's arguments unspecified. About half of the layouts drawn from this
  // area keep the spacing, so the loop ends after a few draws.
  do
  {
    for (CubePlacement& placement : trial.placements)
    {
      placement.x = drawInRange(engine, kMinX, kMaxX);
      placement.y = drawInRange(engine, kMinY, kMaxY);
    }
  } while (!centersAreSpacedApart(trial.placements));
  for (CubePlacement& placement : trial.placements)
  {
    placement.yaw_deg = drawInRange(engine, kMinYawDeg, kMaxYawDeg);
  }
  return trial;
}
}  // namespace vla_sim_behaviors
