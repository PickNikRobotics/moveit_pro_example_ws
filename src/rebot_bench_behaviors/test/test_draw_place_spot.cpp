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
#include <gtest/gtest.h>

#include <cmath>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_pro_test_utils/ros_test.hpp>
#include <rebot_bench_behaviors/draw_place_spot.hpp>
#include <vector>

namespace
{
class DrawPlaceSpotTest : public moveit_pro::test_utils::RosTest
{
protected:
  void SetUp() override
  {
    config.blackboard = BT::Blackboard::create();
    for (const auto* port : { "x_range", "y_range", "keep_clear_of", "clearance" })
      config.input_ports[port] = "=";
    config.output_ports["spot"] = "=";
    context = std::make_shared<moveit_pro::behaviors::BehaviorContext>(
        std::make_shared<rclcpp::Node>("draw_place_spot_test"), false);
    red.header.frame_id = "world";
    red.pose.position.x = 0.35;
    red.pose.position.y = 0.14;
    red.pose.position.z = 0.05;
    red.pose.orientation.z = std::sin(0.1);
    red.pose.orientation.w = std::cos(0.1);
    config.blackboard->set("x_range", std::vector<double>{ 0.29, 0.46 });
    config.blackboard->set("y_range", std::vector<double>{ 0.02, 0.26 });
    config.blackboard->set("keep_clear_of", red);
    config.blackboard->set("clearance", 0.11);
  }
  BT::NodeConfiguration config;
  std::shared_ptr<moveit_pro::behaviors::BehaviorContext> context;
  geometry_msgs::msg::PoseStamped red;
};

TEST_F(DrawPlaceSpotTest, DrawsSpreadSpotsInsideTheRegionAndClearOfTheOtherPose)
{
  rebot_bench_behaviors::DrawPlaceSpot behavior("DrawPlaceSpot", config, context);
  double min_x = 1, max_x = 0;
  for (int i = 0; i < 200; ++i)
  {
    ASSERT_EQ(behavior.executeTick(), BT::NodeStatus::SUCCESS);
    const auto spot = config.blackboard->get<geometry_msgs::msg::PoseStamped>("spot");
    EXPECT_EQ(spot.header.frame_id, "world");
    EXPECT_GE(spot.pose.position.x, 0.29);
    EXPECT_LE(spot.pose.position.x, 0.46);
    EXPECT_GE(spot.pose.position.y, 0.02);
    EXPECT_LE(spot.pose.position.y, 0.26);
    EXPECT_EQ(spot.pose.position.z, 0.0);
    EXPECT_GE(std::hypot(spot.pose.position.x - 0.35, spot.pose.position.y - 0.14), 0.11);
    EXPECT_EQ(spot.pose.orientation, red.pose.orientation);
    min_x = std::min(min_x, spot.pose.position.x);
    max_x = std::max(max_x, spot.pose.position.x);
  }
  // Not one fixed spot: the draws cover most of the region.
  EXPECT_LT(min_x, 0.31);
  EXPECT_GT(max_x, 0.44);
}

TEST_F(DrawPlaceSpotTest, FailsWhenTheClearanceCoversTheRegion)
{
  config.blackboard->set("clearance", 0.5);
  rebot_bench_behaviors::DrawPlaceSpot behavior("DrawPlaceSpot", config, context);
  EXPECT_EQ(behavior.executeTick(), BT::NodeStatus::FAILURE);
}

TEST_F(DrawPlaceSpotTest, RejectsAMalformedRange)
{
  config.blackboard->set("x_range", std::vector<double>{ 0.46, 0.29 });
  rebot_bench_behaviors::DrawPlaceSpot behavior("DrawPlaceSpot", config, context);
  EXPECT_EQ(behavior.executeTick(), BT::NodeStatus::FAILURE);
}
}  // namespace
