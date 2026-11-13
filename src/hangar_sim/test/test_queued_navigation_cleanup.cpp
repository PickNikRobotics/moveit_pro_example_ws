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

#include <array>
#include <memory>
#include <stdexcept>
#include <string>
#include <string_view>
#include <tuple>
#include <utility>

#include <behaviortree_cpp/bt_factory.h>

namespace
{
struct ControllerState
{
  bool arm_active = true;
  bool navigation_active = false;
  bool cleanup_fails = false;
  bool cleanup_started = false;
  int cleanup_ticks = 0;
  int navigation_halts = 0;
  std::string waiting_at;
  bool main_throws = false;
  bool switch_pending = false;
  int switch_halts = 0;
};

// Execute the shipped wrapper and built-in control nodes with stubbed controllers and inner subtrees.
class ControllerSwitchFixture final : public BT::StatefulActionNode
{
public:
  ControllerSwitchFixture(const std::string& name, const BT::NodeConfig& config, std::shared_ptr<ControllerState> state)
    : BT::StatefulActionNode(name, config), state_(std::move(state))
  {
  }

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<std::string>("activate_controllers"),
             BT::InputPort<std::string>("deactivate_controllers", ""),
             BT::InputPort<double>("timeout", -1.0, "Controller switch timeout in seconds.") };
  }

  BT::NodeStatus onStart() override
  {
    const std::string activate = getInput<std::string>("activate_controllers").value();
    if (activate == "joint_trajectory_controller")
    {
      state_->cleanup_started = true;
      return BT::NodeStatus::RUNNING;
    }
    if (activate != "platform_velocity_controller_nav2")
    {
      throw std::runtime_error("Unexpected controller activation in queue Objective");
    }
    const std::string deactivate = getInput<std::string>("deactivate_controllers").value();
    EXPECT_NE(deactivate.find("joint_trajectory_controller"), std::string::npos);
    state_->arm_active = false;
    state_->navigation_active = true;
    return state_->switch_pending ? BT::NodeStatus::RUNNING : BT::NodeStatus::SUCCESS;
  }

  BT::NodeStatus onRunning() override
  {
    ++state_->cleanup_ticks;
    if (state_->cleanup_fails)
    {
      return BT::NodeStatus::FAILURE;
    }
    state_->arm_active = true;
    return BT::NodeStatus::SUCCESS;
  }

  void onHalted() override
  {
    ++state_->switch_halts;
  }

private:
  std::shared_ptr<ControllerState> state_;
};

class NavigationWaitFixture final : public BT::StatefulActionNode
{
public:
  NavigationWaitFixture(const std::string& name, const BT::NodeConfig& config, std::shared_ptr<ControllerState> state)
    : BT::StatefulActionNode(name, config), state_(std::move(state))
  {
  }

  static BT::PortsList providedPorts()
  {
    return {};
  }

  BT::NodeStatus onStart() override
  {
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override
  {
    ++state_->navigation_halts;
  }

private:
  std::shared_ptr<ControllerState> state_;
};

class QueuedNavigationCleanupTest : public ::testing::Test
{
public:
  QueuedNavigationCleanupTest()
  {
    factory_.registerBuilder<ControllerSwitchFixture>("SwitchController", [this](const std::string& name,
                                                                                 const BT::NodeConfig& config) {
      return std::make_unique<ControllerSwitchFixture>(name, config, state_);
    });
    factory_.registerBuilder<NavigationWaitFixture>("FixtureWait", [this](const std::string& name,
                                                                          const BT::NodeConfig& config) {
      return std::make_unique<NavigationWaitFixture>(name, config, state_);
    });
    factory_.registerSimpleAction("FixturePhase", [this](BT::TreeNode& node) {
      if (state_->main_throws)
      {
        throw std::runtime_error("Navigation fixture failed");
      }
      state_->waiting_at = node.name();
      return BT::NodeStatus::SUCCESS;
    });
    factory_.registerBehaviorTreeFromFile(HANGAR_QUEUED_OBJECTIVE_PATH);
  }

  BT::Tree makeTree(BT::NodeStatus result, std::string_view stop_at = "")
  {
    const std::array<std::string, 3> phases{ "Collection", "PathApproval", "Execution" };
    std::string navigation = "<Sequence>";
    for (const auto& phase : phases)
    {
      navigation += "<FixturePhase name=\"" + phase + "\"/>";
      if (phase == stop_at)
      {
        navigation += "<FixtureWait/>";
      }
    }
    navigation += result == BT::NodeStatus::SUCCESS ? "<AlwaysSuccess/>" : "<AlwaysFailure/>";
    navigation += "</Sequence>";
    const std::string xml = R"(<root BTCPP_format="4"><BehaviorTree ID="Refine Localization In Place"><AlwaysSuccess/>)"
                            R"(</BehaviorTree><BehaviorTree ID="Navigate Planar Waypoints">)" +
                            navigation +
                            R"(</BehaviorTree><TreeNodesModel><SubTree ID="Refine Localization In Place"/>)"
                            R"(<SubTree ID="Navigate Planar Waypoints"><input_port name="reference_frame"/>)"
                            R"(<input_port name="waypoint_count"/><input_port name="planner_action_name"/>)"
                            R"(<input_port name="controller_action_name"/><input_port name="planner_id"/>)"
                            R"(<input_port name="controller_id"/><input_port name="goal_checker_id"/>)"
                            R"(<input_port name="timeout_sec"/><input_port name="max_pose_age_sec"/>)"
                            R"(</SubTree></TreeNodesModel></root>)";
    factory_.registerBehaviorTreeFromText(xml);
    const auto blackboard = BT::Blackboard::create();
    blackboard->set("waypoint_count", 2);
    blackboard->set("timeout_sec", -1.0);
    return factory_.createTree("Navigate Queued Waypoints", blackboard);
  }

  std::shared_ptr<ControllerState> state_ = std::make_shared<ControllerState>();
  BT::BehaviorTreeFactory factory_;
};

class QueuedNavigationHaltTest : public QueuedNavigationCleanupTest, public ::testing::WithParamInterface<std::string>
{
};

TEST_P(QueuedNavigationHaltTest, HaltRestoresArmControllerBeforeReturning)
{
  // GIVEN the shipped Objective waiting with the arm controller inactive.
  auto tree = makeTree(BT::NodeStatus::SUCCESS, GetParam());
  ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
  ASSERT_EQ(state_->waiting_at, GetParam());
  ASSERT_FALSE(state_->arm_active);
  ASSERT_TRUE(state_->navigation_active);

  // WHEN the Runtime halts the whole tree instead of ticking a failure branch.
  tree.haltTree();

  // THEN navigation is halted and asynchronous controller restoration finishes before halt returns.
  EXPECT_EQ(state_->navigation_halts, 1);
  EXPECT_TRUE(state_->cleanup_started);
  EXPECT_EQ(state_->cleanup_ticks, 1);
  EXPECT_TRUE(state_->arm_active);
  EXPECT_EQ(tree.rootNode()->status(), BT::NodeStatus::IDLE);
}

INSTANTIATE_TEST_SUITE_P(OperatorWaits, QueuedNavigationHaltTest,
                         ::testing::Values("Collection", "PathApproval", "Execution"),
                         [](const ::testing::TestParamInfo<std::string>& info) { return info.param; });

TEST_F(QueuedNavigationCleanupTest, FailureRestoresControllerWithoutReportingSuccess)
{
  // GIVEN navigation fails after deactivating the arm controller.
  auto tree = makeTree(BT::NodeStatus::FAILURE);
  ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
  ASSERT_FALSE(state_->arm_active);

  // WHEN asynchronous restoration finishes.
  const auto result = tree.tickOnce();

  // THEN the arm controller is restored and the original failure remains a failure.
  EXPECT_EQ(result, BT::NodeStatus::FAILURE);
  EXPECT_TRUE(state_->arm_active);
}

TEST_F(QueuedNavigationCleanupTest, SuccessRestoresController)
{
  // GIVEN every navigation leg succeeds.
  auto tree = makeTree(BT::NodeStatus::SUCCESS);
  ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
  ASSERT_FALSE(state_->arm_active);

  // WHEN asynchronous restoration finishes.
  const auto result = tree.tickOnce();

  // THEN the arm controller is restored before the Objective succeeds.
  EXPECT_EQ(result, BT::NodeStatus::SUCCESS);
  EXPECT_TRUE(state_->arm_active);
}

TEST_F(QueuedNavigationCleanupTest, CleanupFailurePreventsSuccess)
{
  // GIVEN navigation succeeds but controller restoration fails.
  state_->cleanup_fails = true;
  auto tree = makeTree(BT::NodeStatus::SUCCESS);
  ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);

  // WHEN the cleanup service reports failure.
  const auto result = tree.tickOnce();

  // THEN the Objective reports failure instead of hiding the inactive controller.
  EXPECT_EQ(result, BT::NodeStatus::FAILURE);
  EXPECT_FALSE(state_->arm_active);
}

TEST_F(QueuedNavigationCleanupTest, ExceptionRestoresControllerBeforeRethrowing)
{
  // GIVEN a navigation exception after the arm controller is deactivated.
  state_->main_throws = true;
  auto tree = makeTree(BT::NodeStatus::SUCCESS);
  ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
  ASSERT_FALSE(state_->arm_active);

  // WHEN asynchronous controller restoration finishes.
  EXPECT_THROW(std::ignore = tree.tickOnce(), BT::NodeExecutionError);

  // THEN the arm controller is restored without suppressing the navigation exception.
  EXPECT_TRUE(state_->arm_active);
}

TEST_F(QueuedNavigationCleanupTest, HaltDuringControllerSwitchRestoresArm)
{
  // GIVEN the main controller switch is still running after deactivating the arm.
  state_->switch_pending = true;
  auto tree = makeTree(BT::NodeStatus::SUCCESS);
  ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
  ASSERT_FALSE(state_->arm_active);
  ASSERT_TRUE(state_->waiting_at.empty());

  // WHEN Stop Objective halts the switch before collection starts.
  tree.haltTree();

  // THEN the running switch is halted and cleanup restores the arm.
  EXPECT_EQ(state_->switch_halts, 1);
  EXPECT_TRUE(state_->arm_active);
  EXPECT_EQ(state_->cleanup_ticks, 1);
}

TEST_F(QueuedNavigationCleanupTest, HaltDuringCleanupFinishesRestoration)
{
  // GIVEN navigation succeeded and asynchronous controller restoration is already running.
  auto tree = makeTree(BT::NodeStatus::SUCCESS);
  ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
  ASSERT_TRUE(state_->cleanup_started);
  ASSERT_FALSE(state_->arm_active);

  // WHEN Stop Objective arrives during the cleanup branch.
  tree.haltTree();

  // THEN cleanup completes before the halted tree becomes idle.
  EXPECT_TRUE(state_->arm_active);
  EXPECT_EQ(state_->cleanup_ticks, 1);
  EXPECT_EQ(tree.rootNode()->status(), BT::NodeStatus::IDLE);
}
}  // namespace
