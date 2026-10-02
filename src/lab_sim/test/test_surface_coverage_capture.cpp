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

#include <behaviortree_cpp/bt_factory.h>
#include <gtest/gtest.h>
#include <tinyxml2.h>

#include <rcl/time.h>
#include <moveit_pro_behavior_interface/behavior_context.hpp>
#include <moveit_pro_behavior_interface/shared_resources_node_loader.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <array>
#include <cstdlib>
#include <deque>
#include <filesystem>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace
{
using sensor_msgs::msg::PointCloud2;

struct Scenario
{
  builtin_interfaces::msg::Time now;
  std::deque<PointCloud2> captures;
  int requests = 0;
  int halts = 0;
  int errors = 0;
  std::string error_message;
};

PointCloud2 cloudAt(const int sec, const unsigned nanosec, const bool on_table = true)
{
  PointCloud2 cloud;
  cloud.header.frame_id = "wrist_camera_optical_frame";
  cloud.header.stamp.sec = sec;
  cloud.header.stamp.nanosec = nanosec;
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
  modifier.resize(8);
  sensor_msgs::PointCloud2Iterator<float> x(cloud, "x"), y(cloud, "y"), z(cloud, "z");
  // Finite planar patches at the two demo crop centers; an earlier view puts them outside both crops.
  for (const auto& center :
       { std::array<float, 3>{ 0.05f, -0.10f, 0.597f }, std::array<float, 3>{ -0.15f, -0.15f, 0.600f } })
  {
    for (const auto dx : { -0.01f, 0.01f })
    {
      for (const auto dy : { -0.01f, 0.01f })
      {
        *x = center[0] + dx + (on_table ? 0.0f : 2.0f);
        *y = center[1] + dy;
        *z = center[2];
        ++x;
        ++y;
        ++z;
      }
    }
  }
  return cloud;
}

class Capture : public BT::StatefulActionNode
{
public:
  Capture(const std::string& name, const BT::NodeConfig& config, Scenario* scenario)
    : BT::StatefulActionNode(name, config), scenario_(*scenario)
  {
  }

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<std::string>("topic_name"),
             BT::InputPort<double>("message_timeout_sec", 5.0, "Capture timeout."),
             BT::InputPort<double>("publisher_timeout_sec", 5.0, "Publisher timeout."),
             BT::OutputPort<PointCloud2>("message_out") };
  }

private:
  BT::NodeStatus onStart() override
  {
    ++scenario_.requests;
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    if (scenario_.captures.empty())
    {
      return BT::NodeStatus::RUNNING;
    }
    setOutput("message_out", scenario_.captures.front());
    scenario_.captures.pop_front();
    return BT::NodeStatus::SUCCESS;
  }

  void onHalted() override
  {
    ++scenario_.halts;
  }

  Scenario& scenario_;
};

// Keep the capture, crop, bounding box, and built-in control flow; substitute motion and sweep boundaries.
void substituteDemoOperations(tinyxml2::XMLElement* element)
{
  const std::string tag = element->Name();
  const std::string id = element->Attribute("ID") ? element->Attribute("ID") : "";
  if ((tag == "SubTree" && id != "Get Fresh Point Cloud") ||
      (tag == "Action" && id != "GetPointCloud" && id != "CreatePoseStamped" && id != "CropPointsInSphere" &&
       id != "GetOrientedBoundingBoxFromPointCloud"))
  {
    while (element->FirstAttribute())
    {
      element->DeleteAttribute(element->FirstAttribute()->Name());
    }
    element->SetName("Action");
    element->SetAttribute("ID", "Operation");
    element->SetAttribute("operation", id.c_str());
  }
  for (auto* child = element->FirstChildElement(); child; child = child->NextSiblingElement())
  {
    substituteDemoOperations(child);
  }
}

class SurfaceCoverageCapture : public ::testing::Test
{
public:
  SurfaceCoverageCapture()
    : context_(std::make_shared<moveit_pro::behaviors::BehaviorContext>(
          std::make_shared<rclcpp::Node>("surface_capture_test")))
    , loader_("moveit_pro_behavior_interface", "moveit_pro::behaviors::SharedResourcesNodeLoaderBase")
  {
    scenario.now.sec = 100;
    scenario.now.nanosec = 500;
    for (const auto* name : { "CoreBehaviorsLoader", "VisionBehaviorsLoader", "ConverterBehaviorsLoader" })
    {
      plugins_.push_back(loader_.createSharedInstance(std::string("moveit_pro::behaviors::") + name));
      plugins_.back()->registerBehaviors(factory_, context_);
    }
    setClock();
    factory_.unregisterBuilder("GetPointCloud");
    factory_.registerNodeType<Capture>("GetPointCloud", &scenario);
    factory_.registerSimpleAction("Operation",
                                  [this](BT::TreeNode& node) {
                                    const auto operation = node.getInput<std::string>("operation").value();
                                    if (operation == "Move to Waypoint")
                                    {
                                      scenario.now.sec += 10;
                                      setClock();
                                    }
                                    return BT::NodeStatus::SUCCESS;
                                  },
                                  { BT::InputPort<std::string>("operation") });
    factory_.unregisterBuilder("LogMessage");
    factory_.registerSimpleAction("LogMessage",
                                  [this](BT::TreeNode& node) {
                                    if (node.getInput<std::string>("log_level").value() == "error")
                                    {
                                      ++scenario.errors;
                                      scenario.error_message = node.getInput<std::string>("message").value();
                                      EXPECT_NE(scenario.error_message.find("/wrist_camera/points"), std::string::npos);
                                      EXPECT_NE(scenario.error_message.find("publisher"), std::string::npos);
                                      EXPECT_NE(scenario.error_message.find("timestamp clock"), std::string::npos);
                                    }
                                    return BT::NodeStatus::SUCCESS;
                                  },
                                  { BT::InputPort<std::string>("message"),
                                    BT::InputPort<std::string>("log_level", "info", "Log severity.") });
    factory_.registerBehaviorTreeFromFile((objectiveDir() / "get_fresh_point_cloud.xml").string());
  }

  static void SetUpTestSuite()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }

  void setClock()
  {
    const std::scoped_lock lock(context_->node->get_clock()->get_clock_mutex());
    auto* clock = context_->node->get_clock()->get_clock_handle();
    if (rcl_enable_ros_time_override(clock) != RCL_RET_OK ||
        rcl_set_ros_time_override(clock, rclcpp::Time(scenario.now).nanoseconds()) != RCL_RET_OK)
    {
      throw std::runtime_error("Cannot set test capture clock");
    }
  }

  static std::filesystem::path objectiveDir()
  {
    const char* override_dir = std::getenv("LAB_OBJECTIVE_DIR");
    return override_dir ? override_dir : LAB_OBJECTIVE_DIR;
  }

  BT::Tree makeCapture(const unsigned timeout_ms = 1000)
  {
    factory_.registerBehaviorTreeFromText(
        "<root BTCPP_format='4'><BehaviorTree ID='Capture test'><SubTree ID='Get Fresh Point Cloud' "
        "point_cloud='{cloud}' timeout_ms='" +
        std::to_string(timeout_ms) + "'/></BehaviorTree></root>");
    return factory_.createTree("Capture test");
  }

  BT::Tree makeDemo(const std::string& filename)
  {
    tinyxml2::XMLDocument source;
    if (source.LoadFile((objectiveDir() / filename).c_str()) != tinyxml2::XML_SUCCESS)
    {
      throw std::runtime_error("Cannot load surface coverage Objective XML");
    }
    substituteDemoOperations(source.RootElement()->FirstChildElement("BehaviorTree"));
    tinyxml2::XMLPrinter printer;
    source.Print(&printer);
    factory_.registerBehaviorTreeFromText(printer.CStr());
    return factory_.createTree(source.RootElement()->Attribute("main_tree_to_execute"));
  }

  Scenario scenario;

private:
  std::shared_ptr<moveit_pro::behaviors::BehaviorContext> context_;
  pluginlib::ClassLoader<moveit_pro::behaviors::SharedResourcesNodeLoaderBase> loader_;
  std::vector<std::shared_ptr<moveit_pro::behaviors::SharedResourcesNodeLoaderBase>> plugins_;
  BT::BehaviorTreeFactory factory_;
};

TEST_F(SurfaceCoverageCapture, MissingCaptureFailsAndHaltsSubscriber)
{
  // GIVEN a camera that never publishes a capture.
  auto tree = makeCapture(10);
  // WHEN the capture budget expires.
  EXPECT_EQ(tree.tickWhileRunning(), BT::NodeStatus::FAILURE);
  // THEN the subscriber is halted and the camera failure is reported.
  EXPECT_EQ(scenario.halts, 1);
  EXPECT_EQ(scenario.errors, 1);
  EXPECT_NE(scenario.error_message.find("within 10 ms"), std::string::npos);
}

TEST_F(SurfaceCoverageCapture, RepeatedOldCaptureFails)
{
  // GIVEN repeated messages from a capture taken before the request.
  scenario.captures = { cloudAt(99, 999), cloudAt(99, 999), cloudAt(99, 999) };
  auto tree = makeCapture(100);
  // WHEN no newer capture arrives, THEN the Objective fails instead of accepting a replay.
  EXPECT_EQ(tree.tickWhileRunning(), BT::NodeStatus::FAILURE);
  EXPECT_TRUE(scenario.captures.empty());
  EXPECT_EQ(scenario.errors, 1);
  EXPECT_NE(scenario.error_message.find("within 100 ms"), std::string::npos);
}

TEST_F(SurfaceCoverageCapture, RejectsOldAndEqualStampsBeforeAcceptingNewerNanosecond)
{
  // GIVEN captures before and exactly at the request timestamp, then one a nanosecond later.
  scenario.captures = { cloudAt(99, 999999999), cloudAt(100, 499), cloudAt(100, 500), cloudAt(100, 501) };
  auto tree = makeCapture();
  // WHEN the capture sequence runs, THEN only the strictly newer frame completes it.
  EXPECT_EQ(tree.tickWhileRunning(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(tree.rootBlackboard()->get<PointCloud2>("cloud").header.stamp.nanosec, 501u);
  EXPECT_EQ(scenario.requests, 4);
  EXPECT_EQ(scenario.errors, 0);
}

TEST_F(SurfaceCoverageCapture, AcceptsNewSecondWithSmallerNanosecond)
{
  // GIVEN a frame from the next second whose nanosecond field is smaller.
  scenario.now.nanosec = 999999999;
  setClock();
  scenario.captures = { cloudAt(101, 0) };
  auto tree = makeCapture();
  // WHEN captured, THEN second rollover is compared correctly.
  EXPECT_EQ(tree.tickWhileRunning(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(tree.rootBlackboard()->get<PointCloud2>("cloud").header.stamp.sec, 101);
  EXPECT_EQ(scenario.requests, 1);
}

TEST_F(SurfaceCoverageCapture, CancellationHaltsSubscriber)
{
  // GIVEN a capture request waiting for the camera.
  auto tree = makeCapture();
  ASSERT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::RUNNING);
  // WHEN cancelled, THEN its subscription is halted without a timeout error.
  tree.haltTree();
  EXPECT_EQ(scenario.halts, 1);
  EXPECT_EQ(scenario.errors, 0);
}

TEST_F(SurfaceCoverageCapture, RecordDemoWaitsForFreshCloudAtBothViewingPoses)
{
  // GIVEN stale captures at both viewing poses, followed by fresh captures.
  scenario.captures = { cloudAt(100, 999, false), cloudAt(110, 501), cloudAt(110, 501, false), cloudAt(120, 501) };
  auto tree = makeDemo("record_surface_coverage_demo.xml");
  // WHEN the actual capture and geometry Behaviors run, THEN both planar patches yield bounding boxes.
  EXPECT_EQ(tree.tickWhileRunning(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(scenario.requests, 4);
  EXPECT_EQ(tree.rootBlackboard()->get<PointCloud2>("region_cloud").width, 4u);
}

TEST_F(SurfaceCoverageCapture, LiveDemoWaitsForFreshCloudBeforeSweep)
{
  // GIVEN a stale camera frame immediately after the viewing motion.
  scenario.captures = { cloudAt(100, 999, false), cloudAt(110, 501) };
  auto tree = makeDemo("live_surface_coverage_demo.xml");
  // WHEN the capture and geometry Behaviors run, THEN the fresh patch yields a bounding box.
  EXPECT_EQ(tree.tickWhileRunning(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(scenario.requests, 2);
  EXPECT_EQ(tree.rootBlackboard()->get<PointCloud2>("region_cloud").width, 4u);
}

TEST_F(SurfaceCoverageCapture, FreshCaptureOutsideRegionStillFails)
{
  // GIVEN a fresh capture with no points in the requested region.
  scenario.captures = { cloudAt(110, 501, false) };
  auto tree = makeDemo("live_surface_coverage_demo.xml");
  // WHEN bounding the crop, THEN the geometry failure is preserved without reacquiring a cloud.
  EXPECT_EQ(tree.tickWhileRunning(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(scenario.requests, 1);
  EXPECT_EQ(tree.rootBlackboard()->get<PointCloud2>("region_cloud").width, 0u);
}
}  // namespace
