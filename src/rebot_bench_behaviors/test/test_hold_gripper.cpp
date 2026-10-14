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
#include <limits>
#include <moveit_pro_test_utils/ros_test.hpp>
#include <rebot_bench_behaviors/hold_gripper_in_trajectory.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

class HoldGripperTest : public moveit_pro::test_utils::RosTest
{
protected:
  void SetUp() override
  {
    config.blackboard = BT::Blackboard::create();
    config.input_ports = { { "arm_trajectory", "=" }, { "gripper_position", "=" }, { "gripper_state", "=" } };
    config.output_ports = { { "trajectory", "=" } };
    auto context = std::make_shared<moveit_pro::behaviors::BehaviorContext>(
        std::make_shared<rclcpp::Node>("hold_gripper_test"), false);
    behavior = std::make_unique<rebot_bench_behaviors::HoldGripperInTrajectory>("HoldGripper", config, context);
    arm.joint_names = { "joint2", "joint1" };
    arm.header.frame_id = "world";
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = { 0.2, 0.1 };
    point.velocities = { 0.02, 0.01 };
    point.accelerations = { -0.01, -0.02 };
    point.time_from_start.sec = 2;
    arm.points.push_back(point);
    point.positions = { 0.4, 0.3 };
    point.time_from_start.sec = 4;
    arm.points.push_back(point);
  }
  BT::NodeStatus run(double command = 3.0625)
  {
    config.blackboard->set("arm_trajectory", arm);
    config.blackboard->set("gripper_position", command);
    return behavior->executeTick();
  }
  BT::NodeConfiguration config;
  std::unique_ptr<rebot_bench_behaviors::HoldGripperInTrajectory> behavior;
  trajectory_msgs::msg::JointTrajectory arm;
};

TEST_F(HoldGripperTest, PreservesArmMotionAndHoldsCommandAtEveryPoint)
{
  ASSERT_EQ(run(), BT::NodeStatus::SUCCESS);
  const auto result = config.blackboard->get<trajectory_msgs::msg::JointTrajectory>("trajectory");
  EXPECT_EQ(result.header, arm.header);
  EXPECT_EQ(result.joint_names, (std::vector<std::string>{ "joint2", "joint1", "joint7" }));
  ASSERT_EQ(result.points.size(), 2U);
  for (std::size_t i = 0; i < result.points.size(); ++i)
  {
    EXPECT_DOUBLE_EQ(result.points[i].positions.back(), 3.0625);
    auto restored = result.points[i];
    restored.positions.pop_back();
    ASSERT_EQ(restored.velocities.back(), 0.0);
    ASSERT_EQ(restored.accelerations.back(), 0.0);
    restored.velocities.pop_back();
    restored.accelerations.pop_back();
    EXPECT_EQ(restored, arm.points[i]);
  }
  EXPECT_EQ(config.blackboard->get<trajectory_msgs::msg::JointTrajectory>("arm_trajectory"), arm);
}
TEST_F(HoldGripperTest, KeepsOptionalArraysEmpty)
{
  for (auto& point : arm.points)
  {
    point.velocities.clear();
    point.accelerations.clear();
  }
  ASSERT_EQ(run(), BT::NodeStatus::SUCCESS);
  for (const auto& point : config.blackboard->get<trajectory_msgs::msg::JointTrajectory>("trajectory").points)
  {
    EXPECT_TRUE(point.velocities.empty());
    EXPECT_TRUE(point.accelerations.empty());
    EXPECT_TRUE(point.effort.empty());
  }
}
TEST_F(HoldGripperTest, RejectsExistingGripperInsteadOfReplacingItsMotion)
{
  arm.joint_names[0] = "joint7";
  EXPECT_EQ(run(), BT::NodeStatus::FAILURE);
}
TEST_F(HoldGripperTest, RejectsDuplicateJointNames)
{
  arm.joint_names[0] = "joint1";
  EXPECT_EQ(run(), BT::NodeStatus::FAILURE);
}
TEST_F(HoldGripperTest, RejectsTruncatedPositions)
{
  arm.points.back().positions.pop_back();
  EXPECT_EQ(run(), BT::NodeStatus::FAILURE);
}
TEST_F(HoldGripperTest, RejectsMismatchedDerivativeArray)
{
  arm.points.back().velocities.pop_back();
  EXPECT_EQ(run(), BT::NodeStatus::FAILURE);
}
TEST_F(HoldGripperTest, RejectsNonfiniteArmCommand)
{
  arm.points.back().positions[0] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(run(), BT::NodeStatus::FAILURE);
}
TEST_F(HoldGripperTest, RejectsNonfiniteGripperCommand)
{
  EXPECT_EQ(run(std::numeric_limits<double>::infinity()), BT::NodeStatus::FAILURE);
}
TEST_F(HoldGripperTest, HoldsMeasuredJawWhenGivenStateInstead)
{
  sensor_msgs::msg::JointState state;
  state.name = { "gripper_joint1", "joint7" };
  state.position = { 0.03, 4.6 };
  config.blackboard->set("arm_trajectory", arm);
  config.blackboard->set("gripper_state", state);
  ASSERT_EQ(behavior->executeTick(), BT::NodeStatus::SUCCESS);
  for (const auto& point : config.blackboard->get<trajectory_msgs::msg::JointTrajectory>("trajectory").points)
    EXPECT_DOUBLE_EQ(point.positions.back(), 4.6);
}
TEST_F(HoldGripperTest, RejectsBothCommandAndState)
{
  sensor_msgs::msg::JointState state;
  state.name = { "joint7" };
  state.position = { 4.6 };
  config.blackboard->set("gripper_state", state);
  EXPECT_EQ(run(), BT::NodeStatus::FAILURE);
}
TEST_F(HoldGripperTest, RejectsStateWithoutGripperPosition)
{
  sensor_msgs::msg::JointState state;
  state.name = { "joint7" };
  config.blackboard->set("arm_trajectory", arm);
  config.blackboard->set("gripper_state", state);
  EXPECT_EQ(behavior->executeTick(), BT::NodeStatus::FAILURE);
}
TEST_F(HoldGripperTest, RejectsEmptyMotion)
{
  arm.points.clear();
  EXPECT_EQ(run(), BT::NodeStatus::FAILURE);
}
