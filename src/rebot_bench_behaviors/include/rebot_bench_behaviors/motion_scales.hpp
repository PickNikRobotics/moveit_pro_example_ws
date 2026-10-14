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

#include <yaml-cpp/yaml.h>
#include <cmath>
#include <stdexcept>
#include <string>

namespace rebot_bench_behaviors
{
/** Planning velocity and acceleration scale factors for one kind of motion leg. */
struct MotionScales
{
  double velocity;
  double acceleration;
};

/**
 * Reads `<leg_kind>: {velocity_scale_factor, acceleration_scale_factor}` from a motion settings YAML file.
 * Throws std::runtime_error when the file, the leg kind or either factor is missing, or a factor is outside (0, 1].
 */
inline MotionScales readMotionScales(const std::string& path, const std::string& leg_kind)
{
  YAML::Node settings;
  try
  {
    settings = YAML::LoadFile(path);
  }
  catch (const YAML::Exception& error)
  {
    throw std::runtime_error("Cannot read motion settings " + path + ": " + error.what());
  }
  const YAML::Node leg = settings[leg_kind];
  if (!leg.IsMap())
    throw std::runtime_error(path + " has no '" + leg_kind + "' motion settings.");
  const auto factor = [&](const std::string& key) {
    const std::string name = path + ": " + leg_kind + "." + key;
    double value = 0.0;
    try
    {
      value = leg[key].as<double>();
    }
    catch (const YAML::Exception&)
    {
      throw std::runtime_error(name + " must be a number.");
    }
    if (!std::isfinite(value) || value <= 0.0 || value > 1.0)
      throw std::runtime_error(name + " must be greater than 0 and at most 1.");
    return value;
  };
  return { factor("velocity_scale_factor"), factor("acceleration_scale_factor") };
}
}  // namespace rebot_bench_behaviors
