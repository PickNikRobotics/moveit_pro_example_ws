// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include <chrono>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <gtest/gtest.h>

#include <behaviortree_cpp/basic_types.h>
#include <control_msgs/action/gripper_command.hpp>
#include <moveit_pro_behavior_interface/behavior_context.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <vla_sim_behaviors/send_gripper_command.hpp>

namespace
{
using vla_sim_behaviors::SendGripperCommand;
using GripperCommand = control_msgs::action::GripperCommand;

constexpr auto kActionName = "/test_gripper_cmd";
constexpr double kTargetPosition = 0.6;

/**
 * @brief A gripper server that accepts every goal and never finishes it.
 *
 * Closing onto an object behaves this way for seconds at a time: the jaws stall short of the
 * commanded position and the goal stays live. A Behavior that waited would block for all of it.
 */
class StallingGripperServer
{
public:
  explicit StallingGripperServer(const rclcpp::Node::SharedPtr& node)
  {
    server_ = rclcpp_action::create_server<GripperCommand>(
        node, kActionName,
        [this](const rclcpp_action::GoalUUID&, std::shared_ptr<const GripperCommand::Goal> goal) {
          received_.set_value(goal->command.position);
          return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [](const std::shared_ptr<rclcpp_action::ServerGoalHandle<GripperCommand>>&) {
          return rclcpp_action::CancelResponse::ACCEPT;
        },
        [this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<GripperCommand>>& handle) {
          // Held, never completed, exactly like jaws still compressing an object.
          std::lock_guard lock(mutex_);
          held_ = handle;
        });
  }

  /** @brief Ends the held goal, which rclcpp_action can wedge on if left executing. */
  ~StallingGripperServer()
  {
    // Terminating the goal re-enters the server's own callbacks, so the lock is
    // released first; holding it here throws EDEADLK out of a destructor.
    std::shared_ptr<rclcpp_action::ServerGoalHandle<GripperCommand>> held;
    {
      std::lock_guard lock(mutex_);
      held = held_;
    }
    if (held != nullptr && held->is_executing())
    {
      held->abort(std::make_shared<GripperCommand::Result>());
    }
  }

  /** @brief Commanded position of the first goal the server received. */
  [[nodiscard]] std::future<double> receivedPosition()
  {
    return received_.get_future();
  }

private:
  rclcpp_action::Server<GripperCommand>::SharedPtr server_;
  std::mutex mutex_;
  std::shared_ptr<rclcpp_action::ServerGoalHandle<GripperCommand>> held_;
  std::promise<double> received_;
};

/** @brief Spins @p node on its own thread for the lifetime of the fixture. */
class SendGripperCommandTest : public ::testing::Test
{
public:
  SendGripperCommandTest()
    : node_(std::make_shared<rclcpp::Node>("send_gripper_command_test"))
    , context_(std::make_shared<moveit_pro::behaviors::BehaviorContext>(node_))
  {
    executor_.add_node(node_);
    spin_thread_ = std::thread([this] { executor_.spin(); });
  }

  ~SendGripperCommandTest() override
  {
    stopSpinning();
  }

  /**
   * @brief Stops the executor and joins its thread. Idempotent.
   *
   * A test that owns an action server must call this before the server goes out of scope.
   * Server callbacks run on the executor thread and touch the server's own members, so
   * letting the server die first is a use-after-free: it aborts the whole binary on
   * std::mutex::lock, taking the other tests with it.
   */
  void stopSpinning()
  {
    executor_.cancel();
    if (spin_thread_.joinable())
    {
      spin_thread_.join();
    }
  }

  [[nodiscard]] static BT::NodeConfiguration configFor(const std::string& action_name, double timeout)
  {
    BT::NodeConfiguration config;
    config.blackboard = BT::Blackboard::create();
    config.input_ports["gripper_command_action_name"] = action_name;
    config.input_ports["position"] = std::to_string(kTargetPosition);
    config.input_ports["max_effort"] = "0.0";
    config.input_ports["wait_for_server_timeout"] = std::to_string(timeout);
    return config;
  }

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<moveit_pro::behaviors::BehaviorContext> context_;
  rclcpp::executors::MultiThreadedExecutor executor_;
  std::thread spin_thread_;
};
}  // namespace

TEST_F(SendGripperCommandTest, SucceedsWithoutWaitingForAGoalThatNeverFinishes)
{
  // GIVEN a gripper server that accepts the goal and never completes it
  StallingGripperServer server(node_);
  std::future<double> received = server.receivedPosition();
  SendGripperCommand behavior("send_gripper", configFor(kActionName, 5.0), context_);

  // WHEN the Behavior is ticked
  const BT::NodeStatus status = behavior.tick();

  // THEN it reports success rather than blocking on the unfinished goal
  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);

  // AND the server received the commanded position
  ASSERT_EQ(received.wait_for(std::chrono::seconds(10)), std::future_status::ready)
      << "The goal never reached the action server.";
  EXPECT_DOUBLE_EQ(received.get(), kTargetPosition);

  stopSpinning();  // before `server` is destroyed; see stopSpinning().
}

TEST_F(SendGripperCommandTest, FailsWhenNoActionServerIsAvailable)
{
  // GIVEN no server on the configured action name
  SendGripperCommand behavior("send_gripper", configFor("/absent_gripper_cmd", 0.2), context_);

  // WHEN the Behavior is ticked
  // THEN it fails rather than reporting a command nothing can execute
  EXPECT_EQ(behavior.tick(), BT::NodeStatus::FAILURE);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
