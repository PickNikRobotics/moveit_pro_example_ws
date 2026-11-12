// Copyright 2026 PickNik Inc.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
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

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <string>

#include <gtest/gtest.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <nav2_map_server/map_io.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

TEST(SavedLunarMap, InstalledMapLoadsThroughNav2)
{
  const auto map_file = ament_index_cpp::get_package_share_directory("lunar_sim") + "/maps/lunar_map.yaml";
  nav_msgs::msg::OccupancyGrid map;
  ASSERT_EQ(nav2_map_server::loadMapFromYaml(map_file, map), nav2_map_server::LOAD_MAP_SUCCESS);

  // The shipped map's raster and world placement are an intentional persisted-data contract.
  EXPECT_EQ(map.info.width, 552u);
  EXPECT_EQ(map.info.height, 585u);
  EXPECT_NEAR(map.info.resolution, 0.05, 1e-8);
  EXPECT_DOUBLE_EQ(map.info.origin.position.x, -13.226);
  EXPECT_DOUBLE_EQ(map.info.origin.position.y, -13.579);
  EXPECT_DOUBLE_EQ(map.info.origin.orientation.w, 1.0);
  ASSERT_EQ(map.data.size(), 552u * 585u);
  EXPECT_EQ(std::count(map.data.begin(), map.data.end(), -1), 83058);
  EXPECT_EQ(std::count(map.data.begin(), map.data.end(), 0), 232636);
  EXPECT_EQ(std::count(map.data.begin(), map.data.end(), 100), 7226);

  const auto column = static_cast<std::size_t>(std::floor(-map.info.origin.position.x / map.info.resolution));
  const auto row = static_cast<std::size_t>(std::floor(-map.info.origin.position.y / map.info.resolution));
  ASSERT_LT(column, map.info.width);
  ASSERT_LT(row, map.info.height);
  EXPECT_EQ(map.data.at(row * map.info.width + column), 0);
}
