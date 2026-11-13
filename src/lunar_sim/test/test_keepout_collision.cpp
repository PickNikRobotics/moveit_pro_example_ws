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

#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <nav2_costmap_2d/footprint.hpp>
#include <nav2_costmap_2d/footprint_collision_checker.hpp>
#include <nav2_costmap_2d/costmap_filters/keepout_filter.hpp>
#include <nav2_map_server/map_io.hpp>
#include <pluginlib/class_loader.hpp>
#include <tf2_ros/buffer.h>

using namespace std::chrono_literals;

TEST(KeepoutCollision, GarageRecoveriesAndHazards)
{
  const std::string package = LUNAR_SIM_SOURCE_DIR;
  for (const auto & name : {std::string("local_costmap"), std::string("global_costmap")}) {
    auto node = std::make_shared<nav2_util::LifecycleNode>(
      name, "/" + name,
      rclcpp::NodeOptions().arguments({"--ros-args", "--params-file",
        package + "/params/nav2_params.yaml"}).automatically_declare_parameters_from_overrides(true));
    pluginlib::ClassLoader<nav2_costmap_2d::Layer> loader(
      "nav2_costmap_2d", "nav2_costmap_2d::Layer");
    nav2_costmap_2d::LayeredCostmap layered(
      node->get_parameter("global_frame").as_string(), false, false);
    nav_msgs::msg::OccupancyGrid mask;
    ASSERT_EQ(nav2_map_server::loadMapFromYaml(
      package + "/maps/lunar_keepout_mask.yaml", mask), nav2_map_server::LOAD_MAP_SUCCESS);
    mask.header.frame_id = "map";
    layered.resizeMap(mask.info.width, mask.info.height, mask.info.resolution,
      mask.info.origin.position.x, mask.info.origin.position.y);
    nav2_costmap_2d::Footprint footprint;
    ASSERT_TRUE(nav2_costmap_2d::makeFootprintFromString(
      node->get_parameter("footprint").as_string(), footprint));
    nav2_costmap_2d::padFootprint(footprint, node->get_parameter("footprint_padding").as_double());
    tf2_ros::Buffer tf(node->get_clock());
    tf.setUsingDedicatedThread(true);
    geometry_msgs::msg::TransformStamped identity;
    identity.header.frame_id = "map";
    identity.child_frame_id = "odom";
    identity.transform.rotation.w = 1.0;
    ASSERT_TRUE(tf.setTransform(identity, "test", true));
    auto group = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    std::shared_ptr<nav2_costmap_2d::KeepoutFilter> keepout;
    const auto filter_names = node->get_parameter("filters").as_string_array();
    for (const auto & filter_name : filter_names) {
      auto filter = loader.createSharedInstance(
        node->get_parameter(filter_name + ".plugin").as_string());
      layered.addFilter(filter);
      filter->initialize(&layered, filter_name, &tf, node, group);
      filter->activate();
      if (auto candidate = std::dynamic_pointer_cast<nav2_costmap_2d::KeepoutFilter>(filter)) {
        keepout = candidate;
      }
    }
    layered.setFootprint(footprint);
    ASSERT_NE(keepout, nullptr);
    auto publisher_node = std::make_shared<rclcpp::Node>("keepout_test_publisher");
    auto qos = rclcpp::QoS(1).transient_local().reliable();
    auto info_publisher = publisher_node->create_publisher<nav2_msgs::msg::CostmapFilterInfo>(
      "/keepout_costmap_filter_info", qos);
    auto mask_publisher = publisher_node->create_publisher<nav_msgs::msg::OccupancyGrid>(
      "/keepout_filter_mask", qos);
    nav2_msgs::msg::CostmapFilterInfo info;
    info.type = 0;
    info.base = 0.0;
    info.multiplier = 1.0;
    info.filter_mask_topic = "/keepout_filter_mask";
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node->get_node_base_interface());
    executor.add_node(publisher_node);
    info_publisher->publish(info);
    mask_publisher->publish(mask);
    const auto deadline = std::chrono::steady_clock::now() + 3s;
    while (!keepout->isActive() && std::chrono::steady_clock::now() < deadline) {
      executor.spin_once(10ms);
    }
    ASSERT_TRUE(keepout->isActive());
    layered.updateMap(0.0, 0.0, 0.0);
    auto * costmap = layered.getCostmap();
    nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *> checker(costmap);
    const auto cost = [&](double wx, double wy, double yaw) {
        return checker.footprintCostAtPose(wx + 0.30, wy - 0.50, yaw, footprint);
      };
    const double garage_yaw = 3.4416;
    for (int step = 0; step <= 72; ++step) {
      EXPECT_LT(cost(10.54, -9.5, garage_yaw + step * M_PI / 36.0),
        nav2_costmap_2d::LETHAL_OBSTACLE) << name << " spin step " << step;
    }
    for (double offset : {-0.20, 0.0, 0.20}) {
      for (int step = 0; step <= 10; ++step) {
        const double along = -step * 0.05;
        const double wx = 10.54 + along * std::cos(garage_yaw) - offset * std::sin(garage_yaw);
        const double wy = -9.5 + along * std::sin(garage_yaw) + offset * std::cos(garage_yaw);
        EXPECT_LT(cost(wx, wy, garage_yaw), nav2_costmap_2d::LETHAL_OBSTACLE)
          << name << " backup offset " << offset << " step " << step;
      }
    }
    for (double heading : {0.0, M_PI / 4.0, M_PI / 2.0}) {
      EXPECT_EQ(cost(-0.50, -5.25, heading), nav2_costmap_2d::LETHAL_OBSTACLE) << name;
      EXPECT_EQ(cost(12.45, 0.0, heading), nav2_costmap_2d::LETHAL_OBSTACLE) << name;
      EXPECT_EQ(cost(-3.55, 17.0, heading), nav2_costmap_2d::LETHAL_OBSTACLE) << name;
    }
    unsigned int mx, my;
    ASSERT_TRUE(costmap->worldToMap(-0.50 + 0.30, -5.25 - 0.50, mx, my));
    EXPECT_EQ(costmap->getCost(mx, my), nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
    ASSERT_TRUE(costmap->worldToMap(0.0 + 0.30, -5.25 - 0.50, mx, my));
    EXPECT_GT(costmap->getCost(mx, my), nav2_costmap_2d::FREE_SPACE);
    EXPECT_LT(costmap->getCost(mx, my), nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
    executor.remove_node(node->get_node_base_interface());
    executor.remove_node(publisher_node);
  }
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
