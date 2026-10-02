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

#include <array>
#include <string>

#include <tl/expected.hpp>

namespace vla_sim_behaviors
{
/** @brief Where one cube starts a trial: its center's x and y, and its rotation about the vertical axis. */
struct CubePlacement
{
  /** @brief x of the cube center in mj_world, in meters. */
  double x = 0.0;
  /** @brief y of the cube center in mj_world, in meters. */
  double y = 0.0;
  /** @brief Rotation of the cube about the vertical axis, in degrees. */
  double yaw_deg = 0.0;
};

/** @brief The starting layout of one cube stacking trial and the cube pair it asks to stack. */
struct CubeStackTrial
{
  /** @brief Placements of the red, green, and blue cubes, in that order. */
  std::array<CubePlacement, 3> placements;
  /** @brief The cube to pick: "red", "green", or "blue". */
  std::string top_color;
  /** @brief The cube to stack it on, never the same as top_color. */
  std::string bottom_color;
  /** @brief "stack the <top_color> cube on the <bottom_color> cube". */
  std::string prompt;
};

/**
 * @brief Gives the cube pair and the layout of trial @p trial_index.
 * @details The pair cycles through the six ordered pairs of two different cubes, so any six trials in a row ask for
 * each pair once. The layout is drawn from the area the shipped checkpoint's demonstrations started from: cube centers
 * with x from 0.42 to 0.60 m and y from -0.16 to 0.16 m, at least 0.07 m apart, and each cube turned by 0 to 90
 * degrees. The draw seeds std::mt19937 from @p seed and @p trial_index through std::seed_seq and maps the engine's raw
 * output with fixed arithmetic, so the same seed and index give the same layout with every standard library. A
 * compiler that fuses a multiply and an add into one instruction, as GCC does on arm64, can change the last bit of a
 * coordinate.
 * @param trial_index The number of the trial, 0 or greater.
 * @param seed Picks the set of layouts the trials draw from. Every value is valid. It does not change the pair.
 * @return The trial, or an error when @p trial_index is negative.
 */
[[nodiscard]] tl::expected<CubeStackTrial, std::string> sampleCubeStackTrial(int trial_index, int seed);
}  // namespace vla_sim_behaviors
