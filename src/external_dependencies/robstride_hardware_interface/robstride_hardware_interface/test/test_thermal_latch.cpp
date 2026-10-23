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
#include <gtest/gtest.h>

#include <limits>
#include "robstride_hardware_interface/thermal_latch.hpp"

using robstride_hardware_interface::ThermalLatch;

TEST(ThermalLatch, ExactThresholdAndCoolingStayLatched)
{
  ThermalLatch latch;
  latch.Configure(100.0);
  EXPECT_FALSE(latch.Observe(99.9));
  EXPECT_TRUE(latch.Observe(100.0));
  EXPECT_TRUE(latch.Observe(25.0));
  EXPECT_TRUE(latch.Observe(std::numeric_limits<double>::quiet_NaN()));
  // Reconfiguring or a lifecycle transition must not reset a trip.
  latch.Configure(110.0);
  EXPECT_TRUE(latch.tripped());
}

TEST(ThermalLatch, CustomThresholdAndDisabledDefault)
{
  ThermalLatch latch;
  EXPECT_FALSE(latch.Observe(150.0));
  latch.Configure(60.0, 45.0);
  EXPECT_FALSE(latch.Observe(59.9));
  EXPECT_TRUE(latch.Observe(60.0));
}

TEST(ThermalLatch, InvalidThresholdIsRejected)
{
  ThermalLatch latch;
  EXPECT_THROW(latch.Configure(-1.0), std::invalid_argument);
  EXPECT_THROW(latch.Configure(std::numeric_limits<double>::infinity()), std::invalid_argument);
  EXPECT_THROW(latch.Configure(std::numeric_limits<double>::quiet_NaN()), std::invalid_argument);
}

TEST(ThermalLatch, InvalidSampleDoesNotInventHeat)
{
  ThermalLatch latch;
  latch.Configure(100.0);
  EXPECT_FALSE(latch.Observe(std::numeric_limits<double>::quiet_NaN()));
  EXPECT_FALSE(latch.Observe(std::numeric_limits<double>::infinity()));
  EXPECT_FALSE(latch.tripped());
}

TEST(ThermalLatch, EveryMotorMustBeFreshCoolHealthyAndOff)
{
  constexpr uint64_t now = 2'000'000'000ull;
  const std::vector<robstride_hardware_interface::ThermalSample> cool(7, {74.9, now, true, true});
  for (size_t motor = 0; motor < 7; ++motor) {
    for (int bad = 0; bad < 9; ++bad) {
      ThermalLatch latch;
      latch.Configure(100.0);
      latch.Observe(100.0);
      auto samples = cool;
      auto & sample = samples[motor];
      switch (bad) {
        case 0: sample.temperature_c = 75.0; break;
        case 1: sample.temperature_c = 100.0; break;
        case 2: sample.temperature_c = std::numeric_limits<double>::quiet_NaN(); break;
        case 3: sample.temperature_c = std::numeric_limits<double>::infinity(); break;
        case 4: sample.temperature_c = -1.0; break;
        case 5: sample.feedback_ns = 0; break;
        case 6: sample.feedback_ns = now - 500'000'000ull; break;
        case 7: sample.off = false; break;
        case 8: sample.healthy = false; break;
      }
      EXPECT_FALSE(latch.TryClear(samples, now)) << "motor=" << motor << " bad=" << bad;
      EXPECT_TRUE(latch.tripped());
      EXPECT_TRUE(latch.TryClear(cool, now));
      EXPECT_FALSE(latch.tripped());
    }
  }
}

TEST(ThermalLatch, MissingAndFutureSamplesCannotClear)
{
  ThermalLatch latch;
  latch.Configure(100.0);
  latch.Observe(100.0);
  EXPECT_FALSE(latch.TryClear({}, 100));
  EXPECT_FALSE(latch.TryClear({{25.0, 101, true, true}}, 100));
  EXPECT_TRUE(latch.tripped());
}

TEST(ThermalLatch, ClearThresholdIsConfigurableAndStrict)
{
  ThermalLatch latch;
  latch.Configure(100.0, 60.0);
  latch.Observe(100.0);
  EXPECT_FALSE(latch.TryClear({{60.0, 100, true, true}}, 100));
  EXPECT_TRUE(latch.TryClear({{59.9, 100, true, true}}, 100));
  EXPECT_TRUE(latch.Observe(100.0));  // A subsequent trip still works.
  for (const auto value : {-1.0, 100.0, 110.0, std::numeric_limits<double>::infinity(),
      std::numeric_limits<double>::quiet_NaN()})
  {
    EXPECT_THROW(latch.Configure(100.0, value), std::invalid_argument);
  }
}
