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

#include <unistd.h>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <moveit_msgs/msg/robot_state.hpp>
#include <moveit_pro_test_utils/ros_test.hpp>
#include <rebot_bench_behaviors/is_joint_at_least.hpp>
#include <rebot_bench_behaviors/load_motion_scales.hpp>
#include <rebot_bench_behaviors/report_unreachable_cube.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <stdexcept>
#include <string>

namespace
{
class StackChecksTest : public moveit_pro::test_utils::RosTest
{
protected:
  void SetUp() override
  {
    config.blackboard = BT::Blackboard::create();
    context = std::make_shared<moveit_pro::behaviors::BehaviorContext>(
        std::make_shared<rclcpp::Node>("stack_checks_test"), false);
  }
  void TearDown() override
  {
    std::filesystem::remove_all(prefix);
  }
  std::string write(const std::string& text) const
  {
    std::filesystem::create_directories(settings.parent_path());
    std::ofstream(settings) << text;
    return settings.string();
  }
  // Installs `package` into `prefix`, as colcon would, so ament_index finds its share directory.
  void installPackage() const
  {
    const auto index = prefix / "share/ament_index/resource_index/packages";
    std::filesystem::create_directories(index);
    std::ofstream(index / package);
    const char* installed = std::getenv("AMENT_PREFIX_PATH");
    setenv("AMENT_PREFIX_PATH", (prefix.string() + (installed ? ":" + std::string(installed) : "")).c_str(), 1);
  }
  BT::NodeConfiguration config;
  std::shared_ptr<moveit_pro::behaviors::BehaviorContext> context;
  const std::string package = "rebot_stack_checks_test";
  const std::filesystem::path prefix =
      std::filesystem::temp_directory_path() / ("rebot_stack_checks_" + std::to_string(getpid()));
  const std::filesystem::path settings = prefix / "share" / package / "objectives/motion.yaml";
};

TEST_F(StackChecksTest, ReadsEachLegKindsFactors)
{
  const auto path = write("joint: {velocity_scale_factor: 0.25, acceleration_scale_factor: 0.2}\n"
                          "gripper: {velocity_scale_factor: 0.1, acceleration_scale_factor: 0.05}\n");
  const auto joint = rebot_bench_behaviors::readMotionScales(path, "joint");
  EXPECT_DOUBLE_EQ(joint.velocity, 0.25);
  EXPECT_DOUBLE_EQ(joint.acceleration, 0.2);
  const auto gripper = rebot_bench_behaviors::readMotionScales(path, "gripper");
  EXPECT_DOUBLE_EQ(gripper.velocity, 0.1);
  EXPECT_DOUBLE_EQ(gripper.acceleration, 0.05);
}

TEST_F(StackChecksTest, RejectsMissingOrInvalidSettings)
{
  const auto path = write("joint: {velocity_scale_factor: 0.25}\n"
                          "fast: {velocity_scale_factor: 1.5, acceleration_scale_factor: 0.2}\n"
                          "stopped: {velocity_scale_factor: 0, acceleration_scale_factor: 0.2}\n"
                          "word: {velocity_scale_factor: slow, acceleration_scale_factor: 0.2}\n");
  for (const auto* kind : { "joint", "fast", "stopped", "word", "cartesian" })
    EXPECT_THROW(rebot_bench_behaviors::readMotionScales(path, kind), std::runtime_error) << kind;
  EXPECT_THROW(rebot_bench_behaviors::readMotionScales("/nonexistent/rebot_stack_motion.yaml", "joint"),
               std::runtime_error);
}

TEST_F(StackChecksTest, LoadMotionScalesReadsTheFileFromThePackageShare)
{
  installPackage();
  write("cartesian: {velocity_scale_factor: 0.15, acceleration_scale_factor: 0.1}");
  config.input_ports = { { "package_name", package },
                         { "file_path", "objectives/motion.yaml" },
                         { "leg_kind", "cartesian" } };
  config.output_ports = { { "velocity_scale_factor", "{velocity}" }, { "acceleration_scale_factor", "{acceleration}" } };
  rebot_bench_behaviors::LoadMotionScales behavior("LoadMotionScales", config, context);
  ASSERT_EQ(behavior.executeTick(), BT::NodeStatus::SUCCESS);
  EXPECT_DOUBLE_EQ(config.blackboard->get<double>("velocity"), 0.15);
  EXPECT_DOUBLE_EQ(config.blackboard->get<double>("acceleration"), 0.1);
}

TEST_F(StackChecksTest, LoadMotionScalesRefusesAPathOutsideThePackage)
{
  installPackage();
  const auto absolute_path = write("cartesian: {velocity_scale_factor: 0.15, acceleration_scale_factor: 0.1}");
  const auto sibling = prefix / "share/other_pkg/motion.yaml";
  std::filesystem::create_directories(sibling.parent_path());
  std::filesystem::copy_file(settings, sibling);
  config.output_ports = { { "velocity_scale_factor", "{velocity}" }, { "acceleration_scale_factor", "{acceleration}" } };
  for (const auto& path : { absolute_path, std::string("../other_pkg/motion.yaml"),
                            std::string("objectives/../../other_pkg/motion.yaml") })
  {
    config.input_ports = { { "package_name", package }, { "file_path", path }, { "leg_kind", "cartesian" } };
    rebot_bench_behaviors::LoadMotionScales behavior("LoadMotionScales", config, context);
    EXPECT_EQ(behavior.executeTick(), BT::NodeStatus::FAILURE) << path;
  }
}

TEST_F(StackChecksTest, LoadMotionScalesFailsOnAnUnknownPackage)
{
  config.input_ports = { { "package_name", "no_such_rebot_package" },
                         { "file_path", "objectives/rebot_stack_motion.yaml" },
                         { "leg_kind", "joint" } };
  rebot_bench_behaviors::LoadMotionScales behavior("LoadMotionScales", config, context);
  EXPECT_EQ(behavior.executeTick(), BT::NodeStatus::FAILURE);
}

class JointCheckTest : public StackChecksTest
{
protected:
  BT::NodeStatus check(double measured, const std::string& joint = "joint7")
  {
    sensor_msgs::msg::JointState state;
    state.name = { "gripper_joint1", joint };
    state.position = { 0.04, measured };
    config.blackboard->set("state", state);
    config.input_ports = { { "joint_state", "{state}" }, { "joint_name", "joint7" }, { "minimum", "5.1" } };
    rebot_bench_behaviors::IsJointAtLeast behavior("IsJointAtLeast", config, context);
    return behavior.executeTick();
  }
};

TEST_F(JointCheckTest, AcceptsAnOpenJaw)
{
  EXPECT_EQ(check(5.1), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(check(5.24), BT::NodeStatus::SUCCESS);
}

TEST_F(JointCheckTest, RejectsAShortOpening)
{
  EXPECT_EQ(check(0.085), BT::NodeStatus::FAILURE);
  EXPECT_EQ(check(5.09), BT::NodeStatus::FAILURE);
}

TEST_F(JointCheckTest, RejectsAStateWithoutTheJoint)
{
  EXPECT_EQ(check(5.25, "joint6"), BT::NodeStatus::FAILURE);
}

TEST_F(StackChecksTest, PlannedJointPastItsMaximumIsRefused)
{
  const auto check = [this](double planned) {
    moveit_msgs::msg::RobotState state;
    state.joint_state.name = { "joint1", "joint2" };
    state.joint_state.position = { 0.3, planned };
    config.blackboard->set("goal", state);
    config.input_ports = {
      { "robot_state", "{goal}" }, { "joint_name", "joint2" }, { "maximum", "2.4" }, { "reason", "heat budget" }
    };
    rebot_bench_behaviors::IsPlannedJointAtMost behavior("IsPlannedJointAtMost", config, context);
    return behavior.executeTick();
  };
  EXPECT_EQ(check(2.13), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(check(2.214), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(check(2.36), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(check(2.4), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(check(2.400001), BT::NodeStatus::FAILURE);
}

TEST_F(StackChecksTest, UnreachableCubeNamesTheCubeAndWhereItSits)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.pose.position.x = 0.2771;
  pose.pose.position.y = 0.2534;
  EXPECT_EQ(rebot_bench_behaviors::unreachableCubeMessage("blue cube", pose),
            "blue cube at x=0.277 y=0.253 is outside the arm's reach");
  config.blackboard->set("pose", pose);
  config.input_ports = { { "cube", "blue cube" }, { "pose", "{pose}" } };
  rebot_bench_behaviors::ReportUnreachableCube behavior("ReportUnreachableCube", config, context);
  EXPECT_EQ(behavior.executeTick(), BT::NodeStatus::FAILURE);
}
}  // namespace
