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
#include <stdexcept>

namespace robstride_hardware_interface
{
// Detection only: a gravity-loaded robot must return to a supported rest pose
// before torque is removed. The runtime owns that policy; this latch never
// emits Stop frames or clears itself when a motor cools.
class ThermalLatch
{
public:
  void Configure(double cutoff_c)
  {
    if (!std::isfinite(cutoff_c) || cutoff_c < 0.0) {
      throw std::invalid_argument("thermal_cutoff_c must be finite and nonnegative (0 disables)");
    }
    cutoff_c_ = cutoff_c;
  }

  bool Observe(double temperature_c)
  {
    if (cutoff_c_ > 0.0 && std::isfinite(temperature_c) && temperature_c >= cutoff_c_) {
      tripped_.store(true, std::memory_order_relaxed);
    }
    return tripped();
  }

  bool tripped() const {return tripped_.load(std::memory_order_relaxed);}

private:
  double cutoff_c_ = 0.0;
  std::atomic<bool> tripped_{false};
};
}  // namespace robstride_hardware_interface
#endif  // ROBSTRIDE_HARDWARE_INTERFACE__THERMAL_LATCH_HPP_
