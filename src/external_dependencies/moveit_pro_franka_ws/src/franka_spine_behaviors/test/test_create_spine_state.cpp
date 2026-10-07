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

#include <behaviortree_cpp/bt_factory.h>
#include <moveit_msgs/msg/robot_state.hpp>
#include <moveit_pro_behavior_interface/shared_resources_node_loader.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/node.hpp>

#include <string>

namespace
{
// Runs a one-node tree with CreateSpineState at the given position and returns its status.
BT::NodeStatus runCreateSpineState(const std::string& position, moveit_msgs::msg::RobotState& out)
{
  pluginlib::ClassLoader<moveit_pro::behaviors::SharedResourcesNodeLoaderBase> class_loader(
      "moveit_pro_behavior_interface", "moveit_pro::behaviors::SharedResourcesNodeLoaderBase");
  auto node = std::make_shared<rclcpp::Node>("CreateSpineStateTest");
  auto shared_resources = std::make_shared<moveit_pro::behaviors::BehaviorContext>(node);
  BT::BehaviorTreeFactory factory;
  auto loader = class_loader.createUniqueInstance("franka_spine_behaviors::FrankaSpineBehaviorsLoader");
  loader->registerBehaviors(factory, shared_resources);

  const std::string xml = R"(<root BTCPP_format="4" main_tree_to_execute="T"><BehaviorTree ID="T">
      <Action ID="CreateSpineState" position=")" +
                          position + R"(" spine_joint_state="{spine}" /></BehaviorTree></root>)";
  auto tree = factory.createTreeFromText(xml);
  const auto status = tree.tickWhileRunning();
  if (status == BT::NodeStatus::SUCCESS)
  {
    out = tree.rootBlackboard()->get<moveit_msgs::msg::RobotState>("spine");
  }
  return status;
}
}  // namespace

// PlanToJointGoal rejects a diff, so the goal must be a complete state holding only the spine joint.
TEST(CreateSpineState, OutputsACompleteSpineOnlyState)
{
  moveit_msgs::msg::RobotState state;
  ASSERT_EQ(runCreateSpineState("0.3", state), BT::NodeStatus::SUCCESS);
  EXPECT_FALSE(state.is_diff);
  ASSERT_EQ(state.joint_state.name.size(), 1u);
  EXPECT_EQ(state.joint_state.name[0], "franka_spine_vertical_joint");
  ASSERT_EQ(state.joint_state.position.size(), 1u);
  EXPECT_DOUBLE_EQ(state.joint_state.position[0], 0.3);
}

TEST(CreateSpineState, FailsOutsideTheTravel)
{
  moveit_msgs::msg::RobotState state;
  EXPECT_EQ(runCreateSpineState("0.9", state), BT::NodeStatus::FAILURE);
  EXPECT_EQ(runCreateSpineState("-0.1", state), BT::NodeStatus::FAILURE);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
