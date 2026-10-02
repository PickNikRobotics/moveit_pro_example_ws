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

#include <cmath>
#include <cstddef>
#include <memory>
#include <numbers>
#include <string>
#include <vector>

#include <behaviortree_cpp/bt_factory.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_pro_behavior_interface/behavior_context.hpp>
#include <moveit_pro_behavior_interface/shared_resources_node_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vla_sim_behaviors/cube_stack_trial.hpp>
#include <vla_sim_behaviors/sample_cube_stack_trial.hpp>

namespace vla_sim_behaviors
{
namespace
{
using ::testing::IsEmpty;

constexpr double kTolerance = 1e-12;

// Trial 1 asks for a different pair than trial 0, so a Behavior that ignored trial_index would fail here.
constexpr int kTrialIndex = 1;
constexpr int kSeed = 3;

// The blackboard holds a vector output as a std::vector<BT::Any>.
[[nodiscard]] std::vector<geometry_msgs::msg::PoseStamped> getPoses(const BT::Blackboard& blackboard,
                                                                    const std::string& key)
{
  std::vector<geometry_msgs::msg::PoseStamped> poses;
  for (const BT::Any& element : blackboard.get<std::vector<BT::Any>>(key))
  {
    poses.push_back(element.cast<geometry_msgs::msg::PoseStamped>());
  }
  return poses;
}

TEST(SampleCubeStackTrialBehavior, WritesTheSampledTrialToItsOutputs)
{
  // GIVEN a tree that samples trial 1 with seed 3
  const auto node = std::make_shared<rclcpp::Node>("test_sample_cube_stack_trial");
  const auto shared_resources = std::make_shared<moveit_pro::behaviors::BehaviorContext>(node, false);
  BT::BehaviorTreeFactory factory;
  moveit_pro::behaviors::registerBehavior<SampleCubeStackTrial>(factory, "SampleCubeStackTrial", shared_resources);
  BT::Tree tree = factory.createTreeFromText(R"(
    <root BTCPP_format="4">
      <BehaviorTree ID="Test">
        <Action ID="SampleCubeStackTrial" trial_index=")" +
                                             std::to_string(kTrialIndex) + R"(" seed=")" + std::to_string(kSeed) +
                                             R"("
                cube_poses="{cube_poses}" top_color="{top_color}" bottom_color="{bottom_color}" prompt="{prompt}" />
      </BehaviorTree>
    </root>)");
  const auto expected = sampleCubeStackTrial(kTrialIndex, kSeed);
  ASSERT_TRUE(expected.has_value()) << expected.error();

  // WHEN ticking it
  const BT::NodeStatus status = tree.tickOnce();

  // THEN the Behavior succeeds without logging an error
  ASSERT_EQ(status, BT::NodeStatus::SUCCESS);
  EXPECT_THAT(shared_resources->logger->consumeErrorLogBuffer(), IsEmpty());

  // AND the pair and prompt are the trial's
  const BT::Blackboard::Ptr blackboard = tree.rootBlackboard();
  EXPECT_EQ(blackboard->get<std::string>("top_color"), expected->top_color);
  EXPECT_EQ(blackboard->get<std::string>("bottom_color"), expected->bottom_color);
  EXPECT_EQ(blackboard->get<std::string>("prompt"), expected->prompt);

  // AND there is one pose per cube, in red, green, blue order
  const std::vector<geometry_msgs::msg::PoseStamped> cube_poses = getPoses(*blackboard, "cube_poses");
  ASSERT_EQ(cube_poses.size(), expected->placements.size());

  // AND each pose is in mj_world at the resting height, turned about z by the cube's yaw only
  for (std::size_t i = 0; i < cube_poses.size(); ++i)
  {
    const geometry_msgs::msg::PoseStamped& pose = cube_poses[i];
    const CubePlacement& placement = expected->placements[i];
    EXPECT_EQ(pose.header.frame_id, "mj_world");
    EXPECT_DOUBLE_EQ(pose.pose.position.x, placement.x);
    EXPECT_DOUBLE_EQ(pose.pose.position.y, placement.y);
    EXPECT_DOUBLE_EQ(pose.pose.position.z, 0.115);

    const auto& orientation = pose.pose.orientation;
    EXPECT_EQ(orientation.x, 0.0);
    EXPECT_EQ(orientation.y, 0.0);
    EXPECT_NEAR(std::hypot(orientation.z, orientation.w), 1.0, kTolerance);
    const double yaw_deg = 2.0 * std::atan2(orientation.z, orientation.w) * 180.0 / std::numbers::pi;
    EXPECT_NEAR(yaw_deg, placement.yaw_deg, 1e-9);
  }
}
}  // namespace
}  // namespace vla_sim_behaviors

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
