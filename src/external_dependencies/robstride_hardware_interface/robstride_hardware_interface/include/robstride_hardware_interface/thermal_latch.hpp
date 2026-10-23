// Copyright 2026 PickNik Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#ifndef ROBSTRIDE_HARDWARE_INTERFACE__THERMAL_LATCH_HPP_
#define ROBSTRIDE_HARDWARE_INTERFACE__THERMAL_LATCH_HPP_

#include <atomic>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <vector>

namespace robstride_hardware_interface
{
// Detection only: a gravity-loaded robot must return to a supported rest pose
// before torque is removed. The runtime owns that policy; this latch never
// emits Stop frames. Clearing additionally requires a runtime-verified rest
// request and fresh, healthy, OFF feedback from every configured motor.
struct ThermalSample
{
  double temperature_c;
  uint64_t feedback_ns;
  bool off;
  bool healthy;
};

class ThermalLatch
{
public:
  void Configure(double cutoff_c, double clear_c = 75.0)
  {
    if (!std::isfinite(cutoff_c) || cutoff_c < 0.0) {
      throw std::invalid_argument("thermal_cutoff_c must be finite and nonnegative (0 disables)");
    }
    if (!std::isfinite(clear_c) || clear_c < 0.0 ||
      (cutoff_c > 0.0 && clear_c >= cutoff_c))
    {
      throw std::invalid_argument(
              "thermal_clear_c must be finite, nonnegative and below thermal_cutoff_c");
    }
    cutoff_c_ = cutoff_c;
    clear_c_ = clear_c;
  }

  bool Observe(double temperature_c)
  {
    if (cutoff_c_ > 0.0 && std::isfinite(temperature_c) && temperature_c >= cutoff_c_) {
      tripped_.store(true, std::memory_order_relaxed);
    }
    return tripped();
  }

  bool tripped() const {return tripped_.load(std::memory_order_relaxed);}

  // Called by the control loop only, after consuming a fresh rest-clear request.
  // Cooling alone is deliberately insufficient, including after failed HOME.
  bool TryClear(const std::vector<ThermalSample> & samples, uint64_t now_ns)
  {
    if (!tripped() || samples.empty()) {
      return false;
    }
    for (const auto & sample : samples) {
      if (!sample.off || !sample.healthy || !std::isfinite(sample.temperature_c) ||
        sample.temperature_c < 0.0 || sample.temperature_c >= clear_c_ ||
        sample.feedback_ns == 0 || sample.feedback_ns > now_ns ||
        now_ns - sample.feedback_ns >= 500'000'000ull)
      {
        return false;
      }
    }
    tripped_.store(false, std::memory_order_relaxed);
    return true;
  }

private:
  double cutoff_c_ = 0.0;
  double clear_c_ = 75.0;
  std::atomic<bool> tripped_{false};
};
}  // namespace robstride_hardware_interface
#endif  // ROBSTRIDE_HARDWARE_INTERFACE__THERMAL_LATCH_HPP_
