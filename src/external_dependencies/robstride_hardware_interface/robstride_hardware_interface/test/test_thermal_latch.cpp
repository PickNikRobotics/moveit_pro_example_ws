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
  latch.Configure(60.0);
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
