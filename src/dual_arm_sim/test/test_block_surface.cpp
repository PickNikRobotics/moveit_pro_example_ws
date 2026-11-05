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

#include <chrono>
#include <filesystem>
#include <memory>
#include <stdexcept>
#include <string>
#include <tuple>
#include <vector>

#include <behaviortree_cpp/bt_factory.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tinyxml2.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_pro_behavior_interface/behavior_context.hpp>
#include <moveit_pro_behavior_interface/shared_resources_node_loader.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace
{
using sensor_msgs::msg::PointCloud2;

class BlockSurface : public ::testing::Test
{
public:
  static void SetUpTestSuite()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }

  BlockSurface()
  {
    context_ = std::make_shared<moveit_pro::behaviors::BehaviorContext>(
        std::make_shared<rclcpp::Node>("block_surface_test"), false);
    for (const auto* name : { "CoreBehaviorsLoader", "VisionBehaviorsLoader" })
    {
      plugins_.push_back(loader_.createSharedInstance(std::string("moveit_pro::behaviors::") + name));
      plugins_.back()->registerBehaviors(factory_, context_);
    }
    // Only inference and marker publication are substituted. Selection, filtering and centroid use shipped nodes.
    factory_.registerSimpleAction("CapturedSegmentation",
                                  [this](BT::TreeNode& node) {
                                    if (segmentation_failed)
                                    {
                                      return BT::NodeStatus::FAILURE;
                                    }
                                    node.setOutput("point_cloud_vector", clouds);
                                    return BT::NodeStatus::SUCCESS;
                                  },
                                  { BT::OutputPort<std::vector<BT::Any>>("point_cloud_vector") });
    std::ignore = factory_.unregisterBuilder("VisualizePose");
    factory_.registerSimpleAction("VisualizePose",
                                  [this](BT::TreeNode& node) {
                                    visualized = node.getInput<geometry_msgs::msg::PoseStamped>("pose").value();
                                    ++visualizations;
                                    return BT::NodeStatus::SUCCESS;
                                  },
                                  { BT::InputPort<geometry_msgs::msg::PoseStamped>("pose"),
                                    BT::InputPort<double>("marker_lifetime"), BT::InputPort<std::string>("marker_name"),
                                    BT::InputPort<double>("marker_size") });
  }

  [[nodiscard]] BT::Tree makeTree(const std::string& tree_id = "Find with prompt")
  {
    tinyxml2::XMLDocument source;
    if (source.LoadFile(FIND_WITH_PROMPT_XML) != tinyxml2::XML_SUCCESS)
    {
      throw std::runtime_error("Cannot load Find with prompt Objective XML");
    }
    auto* segmentation = source.RootElement()
                             ->FirstChildElement("BehaviorTree")
                             ->FirstChildElement("Control")
                             ->FirstChildElement("SubTree");
    if (!segmentation || std::string(segmentation->Attribute("ID")) != "Segment Point Cloud from Text Prompt Subtree")
    {
      throw std::runtime_error("Find with prompt has no segmentation boundary");
    }
    while (segmentation->FirstAttribute())
    {
      segmentation->DeleteAttribute(segmentation->FirstAttribute()->Name());
    }
    segmentation->SetName("Action");
    segmentation->SetAttribute("ID", "CapturedSegmentation");
    segmentation->SetAttribute("point_cloud_vector", "{point_cloud_vector}");
    tinyxml2::XMLPrinter printer;
    source.Print(&printer);
    factory_.clearRegisteredBehaviorTrees();
    factory_.registerBehaviorTreeFromText(printer.CStr());
    const std::filesystem::path objectives = std::filesystem::path(FIND_WITH_PROMPT_XML).parent_path();
    factory_.registerBehaviorTreeFromFile((objectives / "find_red_block.xml").string());
    factory_.registerBehaviorTreeFromFile((objectives / "find_green_block.xml").string());
    return factory_.createTree(tree_id);
  }

  [[nodiscard]] static PointCloud2 capturedCloud(const std::string& filename)
  {
    pcl::PointCloud<pcl::PointXYZRGB> points;
    const std::filesystem::path path = std::filesystem::path(BLOCK_CLOUD_DATA) / filename;
    if (pcl::io::loadPCDFile(path.string(), points) != 0)
    {
      throw std::runtime_error("Cannot load captured block cloud: " + path.string());
    }
    PointCloud2 cloud;
    pcl::toROSMsg(points, cloud);
    // SendPointCloudToUI captures are already transformed to the world frame.
    cloud.header.frame_id = "world";
    cloud.header.stamp.sec = 42;
    cloud.header.stamp.nanosec = 123;
    return cloud;
  }

  [[nodiscard]] static BT::NodeStatus finish(BT::Tree& tree)
  {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (std::chrono::steady_clock::now() < deadline)
    {
      const BT::NodeStatus status = tree.tickExactlyOnce();
      if (status != BT::NodeStatus::RUNNING)
      {
        return status;
      }
      tree.sleep(std::chrono::milliseconds(1));
    }
    tree.haltTree();
    return BT::NodeStatus::RUNNING;
  }

  std::vector<BT::Any> clouds;
  bool segmentation_failed = false;
  geometry_msgs::msg::PoseStamped visualized;
  int visualizations = 0;

private:
  std::shared_ptr<moveit_pro::behaviors::BehaviorContext> context_;
  pluginlib::ClassLoader<moveit_pro::behaviors::SharedResourcesNodeLoaderBase> loader_{
    "moveit_pro_behavior_interface", "moveit_pro::behaviors::SharedResourcesNodeLoaderBase"
  };
  std::vector<std::shared_ptr<moveit_pro::behaviors::SharedResourcesNodeLoaderBase>> plugins_;
  BT::BehaviorTreeFactory factory_;
};

TEST_F(BlockSurface, EmptyDetectionsFailWithoutPublishingAPose)
{
  // GIVEN successful segmentation with no objects.
  auto tree = makeTree();
  // WHEN the production Find subtree runs.
  EXPECT_EQ(finish(tree), BT::NodeStatus::FAILURE);
  // THEN callers can distinguish exhaustion from a perception error, without a pose.
  EXPECT_EQ(visualizations, 0);
  bool detections_empty = false;
  EXPECT_TRUE(tree.rootBlackboard()->get("detections_empty", detections_empty));
  EXPECT_TRUE(detections_empty);
}

TEST_F(BlockSurface, InsufficientSurfaceFailsWithoutPublishingAPose)
{
  // GIVEN an object mask containing only two finite depth points.
  PointCloud2 cloud = capturedCloud("red3_after_drop.pcd");
  cloud.width = 2;
  cloud.height = 1;
  cloud.row_step = 2 * cloud.point_step;
  cloud.data.resize(cloud.row_step);
  clouds.emplace_back(cloud);
  auto tree = makeTree();
  // WHEN the production Find subtree runs.
  EXPECT_EQ(finish(tree), BT::NodeStatus::FAILURE);
  // THEN a centroid without a measurable surface must not be used for motion or mistaken for exhaustion.
  EXPECT_EQ(visualizations, 0);
  bool detections_empty = true;
  EXPECT_TRUE(tree.rootBlackboard()->get("detections_empty", detections_empty));
  EXPECT_FALSE(detections_empty);
}

TEST_F(BlockSurface, RegressionAmbiguousDroppedBlockIsNotReportedAsExhaustion)
{
  // GIVEN the observed mask of a dropped block whose largest surface covers only 55% of the cloud.
  clouds = { BT::Any(capturedCloud("red3_on_floor.pcd")) };
  auto tree = makeTree();

  // WHEN the real surface estimator rejects that ambiguous mask.
  EXPECT_EQ(finish(tree), BT::NodeStatus::FAILURE);

  // THEN Sort Blocks must see an error, not an empty table or a usable pose.
  EXPECT_FALSE(tree.rootBlackboard()->get<bool>("detections_empty"));
  EXPECT_EQ(visualizations, 0);
}

TEST_F(BlockSurface, RegressionFindWrappersClearExhaustionAndDoNotPublishStalePoses)
{
  for (const auto* tree_id : { "Find Red Block", "Find Green Block" })
  {
    // GIVEN either real color wrapper after a valid detection.
    SCOPED_TRACE(tree_id);
    clouds = { BT::Any(capturedCloud("red3_before_pick.pcd")) };
    auto tree = makeTree(tree_id);
    ASSERT_EQ(finish(tree), BT::NodeStatus::SUCCESS);
    const int previous_visualizations = visualizations;

    // WHEN subsequent segmentation is empty, then fails.
    clouds.clear();
    EXPECT_EQ(finish(tree), BT::NodeStatus::FAILURE);
    EXPECT_TRUE(tree.rootBlackboard()->get<bool>("detections_empty"));
    segmentation_failed = true;
    EXPECT_EQ(finish(tree), BT::NodeStatus::FAILURE);

    // THEN the wrapper preserves the error/empty distinction and never republishes the old pose.
    EXPECT_FALSE(tree.rootBlackboard()->get<bool>("detections_empty"));
    EXPECT_EQ(visualizations, previous_visualizations);
    segmentation_failed = false;
  }
}

TEST_F(BlockSurface, RegressionSegmentationFailureAfterEmptyDetectionsIsNotExhaustion)
{
  // GIVEN a tree whose previous successful segmentation was empty.
  auto tree = makeTree();
  ASSERT_EQ(finish(tree), BT::NodeStatus::FAILURE);
  segmentation_failed = true;

  // WHEN inference or cloud acquisition fails on the next attempt.
  EXPECT_EQ(finish(tree), BT::NodeStatus::FAILURE);

  // THEN the previous exhaustion result is cleared and no pose is published.
  bool detections_empty = true;
  EXPECT_TRUE(tree.rootBlackboard()->get("detections_empty", detections_empty));
  EXPECT_FALSE(detections_empty);
  EXPECT_EQ(visualizations, 0);
}

TEST_F(BlockSurface, RegressionSurfaceStaysOnTheSameBlockBeforeAndAfterADrop)
{
  // GIVEN paired masks of one block before picking and after it lands on another face near the table edge.
  for (const auto* filename : { "red3_before_pick.pcd", "red3_after_drop.pcd" })
  {
    SCOPED_TRACE(filename);
    const PointCloud2 cloud = capturedCloud(filename);
    clouds = { BT::Any(cloud) };
    auto tree = makeTree();
    // WHEN real selection, plane extraction and centroid run in the shipped Objective.
    ASSERT_EQ(finish(tree), BT::NodeStatus::SUCCESS);
    const auto pose = tree.rootBlackboard()->get<geometry_msgs::msg::PoseStamped>("object_centroid");
    // THEN the point lies on the measured top, not the lower sides/table/floor caught by the mask.
    // The independently measured MuJoCo top stays at this world height after the face change.
    EXPECT_NEAR(pose.pose.position.z, 0.804362824, 0.001);
    const bool before_pick = std::string(filename) == "red3_before_pick.pcd";
    EXPECT_NEAR(pose.pose.position.x, before_pick ? 0.850000006 : 0.733832575, 0.001);
    EXPECT_NEAR(pose.pose.position.y, before_pick ? -0.250000002 : -0.257507979, 0.001);
    EXPECT_EQ(pose.header, cloud.header);
    EXPECT_EQ(visualized, pose);
    bool detections_empty = true;
    EXPECT_TRUE(tree.rootBlackboard()->get("detections_empty", detections_empty));
    EXPECT_FALSE(detections_empty);
  }
}
}  // namespace
