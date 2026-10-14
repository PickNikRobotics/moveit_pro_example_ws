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

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <vector>

#include <behaviortree_cpp/bt_factory.h>
#include <filesystem>
#include <fstream>
#include <moveit_msgs/msg/robot_state.hpp>
#include <moveit_pro_behavior_interface/behavior_context.hpp>
#include <moveit_pro_behavior_interface/shared_resources_node_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rebot_bench_behaviors/anchor_trajectory_to_command.hpp>
#include <rebot_bench_behaviors/record_rebot_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

namespace rebot_bench_behaviors
{
namespace
{
using ::testing::ElementsAre;
using ::testing::HasSubstr;
using ::testing::IsEmpty;

constexpr double kTolerance = 1e-12;

// joint1 rises from 1.0 to 1.2 and joint2 from 2.0 to 1.9 over 2 s, at rest at both ends, as planned from the
// measured state.
[[nodiscard]] trajectory_msgs::msg::JointTrajectory plannedTrajectory()
{
  trajectory_msgs::msg::JointTrajectory trajectory;
  trajectory.joint_names = { "joint1", "joint2" };
  const auto add = [&](const int seconds, const std::vector<double>& positions, const std::vector<double>& velocities) {
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.time_from_start.sec = seconds;
    point.positions = positions;
    point.velocities = velocities;
    point.accelerations = { 0.0, 0.0 };
    trajectory.points.push_back(point);
  };
  add(0, { 1.0, 2.0 }, { 0.0, 0.0 });
  add(1, { 1.1, 1.95 }, { 0.15, -0.075 });
  add(2, { 1.2, 1.9 }, { 0.0, 0.0 });
  return trajectory;
}

[[nodiscard]] moveit_msgs::msg::RobotState state(const std::vector<std::string>& names,
                                                 const std::vector<double>& positions)
{
  moveit_msgs::msg::RobotState robot_state;
  robot_state.joint_state.name = names;
  robot_state.joint_state.position = positions;
  return robot_state;
}

class AnchorTrajectoryToCommandTest : public ::testing::Test
{
protected:
  AnchorTrajectoryToCommandTest()
    : node_(std::make_shared<rclcpp::Node>("test_anchor_trajectory_to_command"))
    , context_(std::make_shared<moveit_pro::behaviors::BehaviorContext>(node_, false))
  {
    moveit_pro::behaviors::registerBehavior<AnchorTrajectoryToCommand>(factory_, "AnchorTrajectoryToCommand", context_);
  }

  BT::NodeStatus anchor(const moveit_msgs::msg::RobotState& commanded, const std::string& ports = "")
  {
    tree_ = factory_.createTreeFromText(
        R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="Test"><Sequence>
          <Action ID="AnchorTrajectoryToCommand" joint_trajectory="{planned}" commanded_state="{commanded}" measured_state="{measured}" )" +
        ports + R"( anchored_trajectory="{anchored}" end_state="{end_state}" />
          <Script code="committed := end_state" />
        </Sequence></BehaviorTree>
      </root>)");
    tree_.rootBlackboard()->set("planned", planned_);
    tree_.rootBlackboard()->set("measured", measured_);
    tree_.rootBlackboard()->set("commanded", commanded);
    return tree_.tickOnce();
  }

  [[nodiscard]] trajectory_msgs::msg::JointTrajectory anchored()
  {
    return tree_.rootBlackboard()->get<trajectory_msgs::msg::JointTrajectory>("anchored");
  }

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<moveit_pro::behaviors::BehaviorContext> context_;
  BT::BehaviorTreeFactory factory_;
  BT::Tree tree_;
  trajectory_msgs::msg::JointTrajectory planned_ = plannedTrajectory();
  moveit_msgs::msg::RobotState measured_ = state({ "joint1", "joint2", "joint7" }, { 1.0, 2.0, 2.8 });
};

TEST_F(AnchorTrajectoryToCommandTest, StartsAtTheCommandAndHoldsTheJointsTheTrajectoryLeavesOut)
{
  // GIVEN a command 0.05 above the measured start on joint1, 0.04 below it on joint2, and a gripper at 2.8
  const auto commanded = state({ "joint7", "joint1", "joint2" }, { 2.8, 1.05, 1.96 });

  // WHEN anchoring the planned trajectory to it
  ASSERT_EQ(anchor(commanded), BT::NodeStatus::SUCCESS);
  EXPECT_THAT(context_->logger->consumeErrorLogBuffer(), IsEmpty());

  // THEN every point of each planned joint moves by the same offset, and the gripper is added and held
  const auto trajectory = anchored();
  EXPECT_THAT(trajectory.joint_names, ElementsAre("joint1", "joint2", "joint7"));
  const std::vector<std::vector<double>> expected_positions = { { 1.05, 1.96, 2.8 },
                                                                { 1.15, 1.91, 2.8 },
                                                                { 1.25, 1.86, 2.8 } };
  ASSERT_EQ(trajectory.points.size(), expected_positions.size());
  for (std::size_t i = 0; i < expected_positions.size(); ++i)
  {
    for (std::size_t j = 0; j < 3; ++j)
    {
      EXPECT_NEAR(trajectory.points[i].positions[j], expected_positions[i][j], kTolerance);
    }
  }

  // The command for the next leg is the exact endpoint, including held joints.
  const auto endpoint = tree_.rootBlackboard()->get<moveit_msgs::msg::RobotState>("end_state");
  EXPECT_EQ(tree_.rootBlackboard()->get<moveit_msgs::msg::RobotState>("committed"), endpoint);
  EXPECT_FALSE(endpoint.is_diff);
  EXPECT_EQ(endpoint.joint_state.name, trajectory.joint_names);
  EXPECT_EQ(endpoint.joint_state.position, trajectory.points.back().positions);
  EXPECT_EQ(endpoint.joint_state.velocity, trajectory.points.back().velocities);

  // AND velocities are unchanged, with the held joint at rest
  EXPECT_THAT(trajectory.points[1].velocities, ElementsAre(0.15, -0.075, 0.0));
  EXPECT_THAT(trajectory.points[1].accelerations, ElementsAre(0.0, 0.0, 0.0));
}

TEST_F(AnchorTrajectoryToCommandTest, ReplacesStaleCommandsWithMeasuredOnEveryJoint)
{
  measured_ = state({ "joint1", "joint2", "joint7" }, { 1.0, 2.0, 5.25 });
  ASSERT_EQ(anchor(state({ "joint1", "joint2", "joint7" }, { 1.15, 2.0, 0.0 })), BT::NodeStatus::SUCCESS);
  EXPECT_THAT(anchored().points.front().positions, ElementsAre(1.0, 2.0, 5.25));
}

TEST_F(AnchorTrajectoryToCommandTest, MissingEndpointsFallBackToMeasuredIncludingUnplannedJoints)
{
  measured_ = state({ "joint1", "joint2", "joint7" }, { 1.02, 2.03, 5.25 });
  ASSERT_EQ(anchor(state({}, {})), BT::NodeStatus::SUCCESS);
  EXPECT_THAT(anchored().points.front().positions, ElementsAre(1.02, 2.03, 5.25));
}

TEST_F(AnchorTrajectoryToCommandTest, MeasuredJointsKeepTheirMeasuredStartWhateverTheCommand)
{
  // GIVEN a jaw trajectory planned from its measured 2.0 to open
  measured_ = state({ "joint1", "joint2", "joint7" }, { 1.0, 2.0, 2.0 });
  planned_.joint_names = { "joint7" };
  for (std::size_t i = 0; i < planned_.points.size(); ++i)
  {
    planned_.points[i].positions = { 2.0 + 1.625 * static_cast<double>(i) };
    planned_.points[i].velocities = { i == 1 ? 1.625 : 0.0 };
    planned_.points[i].accelerations = { 0.0 };
  }

  // WHEN anchoring it with joint7 measured, to a stale open command and to one 0.05 from the measurement
  for (const double command : { 5.25, 2.05 })
  {
    ASSERT_EQ(anchor(state({ "joint1", "joint2", "joint7" }, { 1.05, 1.96, command }), R"(measured_joints="joint7")"),
              BT::NodeStatus::SUCCESS);

    // THEN the jaw keeps its planned path, and the arm is held at its command
    const auto trajectory = anchored();
    EXPECT_THAT(trajectory.joint_names, ElementsAre("joint7", "joint1", "joint2"));
    EXPECT_THAT(trajectory.points.front().positions, ElementsAre(2.0, 1.05, 1.96));
    EXPECT_THAT(trajectory.points.back().positions, ElementsAre(5.25, 1.05, 1.96));
  }
}

TEST_F(AnchorTrajectoryToCommandTest, MissingMeasuredJointFailsClosed)
{
  measured_ = state({ "joint1" }, { 1.0 });
  EXPECT_EQ(anchor(state({}, {})), BT::NodeStatus::FAILURE);
}

TEST_F(AnchorTrajectoryToCommandTest, FadesTheShiftOutToEndOnThePlan)
{
  // GIVEN a command 0.05 above the measured start on joint1
  const auto commanded = state({ "joint1", "joint2" }, { 1.05, 2.0 });

  // WHEN anchoring with fade_out
  ASSERT_EQ(anchor(commanded, R"(fade_out="true")"), BT::NodeStatus::SUCCESS);
  const auto trajectory = anchored();

  // THEN it starts at the command and ends exactly on the planned last point
  EXPECT_NEAR(trajectory.points.front().positions[0], 1.05, kTolerance);
  EXPECT_NEAR(trajectory.points.back().positions[0], 1.2, kTolerance);

  // AND it is still at rest at both ends, with the fade's velocity added halfway: -30/16 of the shift per duration
  EXPECT_NEAR(trajectory.points.front().velocities[0], 0.0, kTolerance);
  EXPECT_NEAR(trajectory.points.back().velocities[0], 0.0, kTolerance);
  EXPECT_NEAR(trajectory.points[1].positions[0], 1.1 + 0.025, kTolerance);
  EXPECT_NEAR(trajectory.points[1].velocities[0], 0.15 - 1.875 * 0.05 / 2.0, kTolerance);
  EXPECT_NEAR(trajectory.points.front().accelerations[0], 0.0, kTolerance);
  EXPECT_NEAR(trajectory.points[1].accelerations[0], 0.0, kTolerance);
}
TEST_F(AnchorTrajectoryToCommandTest, AlreadyOpenJawStillHasAnExecutableHoldAndEndpoint)
{
  measured_ = state({ "joint1", "joint2", "joint7" }, { 1.0, 2.0, 5.25 });
  planned_.joint_names = { "joint7" };
  planned_.points.resize(2);
  for (auto& point : planned_.points)
  {
    point.positions = { 5.25 };
    point.velocities = { 0.0 };
    point.accelerations = { 0.0 };
  }
  planned_.points[0].time_from_start.sec = 0;
  planned_.points[1].time_from_start.sec = 1;
  ASSERT_EQ(anchor(state({}, {})), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(anchored().points.size(), 2u);
  EXPECT_DOUBLE_EQ(anchored().points.front().positions[0], 5.25);
  EXPECT_DOUBLE_EQ(tree_.rootBlackboard()->get<moveit_msgs::msg::RobotState>("end_state").joint_state.position[0], 5.25);
}

TEST_F(AnchorTrajectoryToCommandTest, RecordsEveryTrajectoryAndRefusesUnwritableAudit)
{
  moveit_pro::behaviors::registerBehavior<RecordRebotTrajectory>(factory_, "RecordRebotTrajectory", context_);
  auto tree = factory_.createTreeFromText(R"(<root BTCPP_format="4"><BehaviorTree ID="Audit">
    <Action ID="RecordRebotTrajectory" trajectory="{trajectory}" file_path="{path}" />
    </BehaviorTree></root>)");
  const auto path = std::filesystem::temp_directory_path() / "rebot-record-trajectory-test.yaml";
  std::filesystem::remove(path);
  tree.rootBlackboard()->set("trajectory", plannedTrajectory());
  tree.rootBlackboard()->set("path", path.string());
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  const auto first_size = std::filesystem::file_size(path);
  tree.haltTree();
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_GT(std::filesystem::file_size(path), first_size);
  std::ifstream file(path);
  const std::string text((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
  EXPECT_NE(text.find("dispatched_at:"), std::string::npos);
  EXPECT_NE(text.find("positions:"), std::string::npos);
  std::filesystem::remove(path);
  tree.haltTree();
  tree.rootBlackboard()->set("path", (path / "missing" / "audit.yaml").string());
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
}

TEST_F(AnchorTrajectoryToCommandTest, ReportsMissingAuditInputsAndEmptyTrajectory)
{
  moveit_pro::behaviors::registerBehavior<RecordRebotTrajectory>(factory_, "RecordRebotTrajectory", context_);
  for (const std::string scenario : { "missing trajectory", "missing path", "empty trajectory" })
  {
    SCOPED_TRACE(scenario);
    auto tree = factory_.createTreeFromText(R"(<root BTCPP_format="4"><BehaviorTree ID="Audit">
      <Action ID="RecordRebotTrajectory" trajectory="{trajectory}" file_path="{path}" />
      </BehaviorTree></root>)");
    if (scenario != "missing trajectory")
      tree.rootBlackboard()->set(
          "trajectory", scenario == "empty trajectory" ? trajectory_msgs::msg::JointTrajectory{} : plannedTrajectory());
    if (scenario != "missing path")
      tree.rootBlackboard()->set("path", std::string("/unused-audit.yaml"));

    EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
    EXPECT_THAT(context_->logger->consumeErrorLogBuffer(),
                HasSubstr("Provide a trajectory with at least one point and a file_path."));
  }
}
}  // namespace
}  // namespace rebot_bench_behaviors

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
