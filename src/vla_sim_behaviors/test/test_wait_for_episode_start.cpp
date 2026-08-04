// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <gtest/gtest.h>

#include <behaviortree_cpp/basic_types.h>
#include <moveit_pro_behavior_interface/behavior_context.hpp>
#include <moveit_studio_internal_msgs/srv/get_active_recording.hpp>
#include <rclcpp/rclcpp.hpp>

#include <vla_sim_behaviors/wait_for_episode_start.hpp>

namespace
{
using vla_sim_behaviors::WaitForEpisodeStart;
using GetActiveRecording = moveit_studio_internal_msgs::srv::GetActiveRecording;

constexpr auto kServiceName = "/test_active_recording";

/** @brief A Trainer stand-in whose reported session state the test controls. */
class FakeTrainer
{
public:
  explicit FakeTrainer(const rclcpp::Node::SharedPtr& node)
  {
    server_ = node->create_service<GetActiveRecording>(kServiceName,
                                                       [this](const std::shared_ptr<GetActiveRecording::Request>,
                                                              std::shared_ptr<GetActiveRecording::Response> response) {
                                                         ++calls_;
                                                         std::lock_guard lock(mutex_);
                                                         response->status.success = true;
                                                         response->session_json = payload_;
                                                       });
  }

  void setState(const std::string& state)
  {
    std::lock_guard lock(mutex_);
    payload_ = R"({"id":"rec-1","state":")" + state + R"(","current_episode":1})";
  }

  /** @brief Serve @p payload verbatim, for session_json a parser cannot use. */
  void setRawPayload(const std::string& payload)
  {
    std::lock_guard lock(mutex_);
    payload_ = payload;
  }

  [[nodiscard]] int calls() const
  {
    return calls_;
  }

private:
  rclcpp::Service<GetActiveRecording>::SharedPtr server_;
  // Written by the test thread, read by the executor's.
  mutable std::mutex mutex_;
  std::string payload_{ R"({"id":"rec-1","state":"preparing","current_episode":1})" };
  std::atomic_int calls_{ 0 };
};

/** @brief Spins @p node on its own thread for the lifetime of the fixture. */
class WaitForEpisodeStartTest : public ::testing::Test
{
public:
  WaitForEpisodeStartTest()
    : node_(std::make_shared<rclcpp::Node>("wait_for_episode_start_test"))
    , context_(std::make_shared<moveit_pro::behaviors::BehaviorContext>(node_))
  {
    executor_.add_node(node_);
    spin_thread_ = std::thread([this] { executor_.spin(); });
  }

  ~WaitForEpisodeStartTest() override
  {
    stopSpinning();
  }

  /**
   * @brief Stops the executor and joins its thread. Idempotent.
   *
   * A test that owns a FakeTrainer must call this before the trainer goes out of scope. Its
   * service callback runs on the executor thread and touches the trainer's own members, so
   * letting the trainer die first is a use-after-free that aborts the whole binary.
   */
  void stopSpinning()
  {
    executor_.cancel();
    if (spin_thread_.joinable())
    {
      spin_thread_.join();
    }
  }

  /**
   * @brief Calls stopSpinning() at scope exit, however the test leaves it.
   *
   * Declare one directly after the FakeTrainer it protects and before the Behavior, so the
   * Behavior tears down against a live executor and the trainer outlives a stopped one.
   */
  class SpinGuard
  {
  public:
    explicit SpinGuard(WaitForEpisodeStartTest& test) : test_(test)
    {
    }
    SpinGuard(const SpinGuard&) = delete;
    SpinGuard& operator=(const SpinGuard&) = delete;
    ~SpinGuard()
    {
      test_.stopSpinning();
    }

  private:
    WaitForEpisodeStartTest& test_;
  };

  [[nodiscard]] static BT::NodeConfiguration configFor(const std::string& service_name, double timeout)
  {
    BT::NodeConfiguration config;
    config.blackboard = BT::Blackboard::create();
    config.input_ports["service_name"] = service_name;
    config.input_ports["timeout"] = std::to_string(timeout);
    return config;
  }

  /** @brief Ticks @p behavior until it leaves RUNNING, or returns RUNNING at @p deadline.
   *
   * executeTick() rather than tick(): the latter is StatefulActionNode's protected
   * hook, and only executeTick() routes the first tick through onStart(). */
  static BT::NodeStatus tickUntilSettled(WaitForEpisodeStart& behavior, std::chrono::seconds deadline)
  {
    const auto give_up = std::chrono::steady_clock::now() + deadline;
    BT::NodeStatus status = behavior.executeTick();
    while (status == BT::NodeStatus::RUNNING && std::chrono::steady_clock::now() < give_up)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      status = behavior.executeTick();
    }
    return status;
  }

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<moveit_pro::behaviors::BehaviorContext> context_;
  rclcpp::executors::MultiThreadedExecutor executor_;
  std::thread spin_thread_;
};
}  // namespace

TEST_F(WaitForEpisodeStartTest, RunsUntilTheSessionReportsRecording)
{
  // GIVEN a session that is still preparing
  FakeTrainer trainer(node_);
  SpinGuard guard(*this);  // see SpinGuard: before `behavior`, after `trainer`.
  WaitForEpisodeStart behavior("wait_for_episode", configFor(kServiceName, 10.0), context_);

  // WHEN it is ticked while the session has not opened an episode
  ASSERT_EQ(behavior.executeTick(), BT::NodeStatus::RUNNING);
  std::this_thread::sleep_for(std::chrono::milliseconds(300));

  // THEN it stays RUNNING rather than releasing the motion early
  EXPECT_EQ(behavior.executeTick(), BT::NodeStatus::RUNNING);

  // WHEN the episode opens
  trainer.setState("recording");

  // THEN it succeeds
  EXPECT_EQ(tickUntilSettled(behavior, std::chrono::seconds(10)), BT::NodeStatus::SUCCESS);
  EXPECT_GT(trainer.calls(), 1) << "The state was read once and never re-polled.";
}

TEST_F(WaitForEpisodeStartTest, FailsWhenNoEpisodeOpensWithinTheTimeout)
{
  // GIVEN a session that never leaves preparing
  FakeTrainer trainer(node_);
  SpinGuard guard(*this);  // see SpinGuard: before `behavior`, after `trainer`.
  WaitForEpisodeStart behavior("wait_for_episode", configFor(kServiceName, 0.5), context_);

  // WHEN it is ticked
  // THEN it fails rather than waiting on a recording that will never start
  EXPECT_EQ(tickUntilSettled(behavior, std::chrono::seconds(20)), BT::NodeStatus::FAILURE);

  // AND says which state it was stuck in, so the operator can tell a slow recorder
  // from a session that never started
  EXPECT_NE(context_->logger->consumeErrorLogBuffer().find("'preparing'"), std::string::npos);
}

TEST_F(WaitForEpisodeStartTest, KeepsWaitingWhenTheSessionCannotBeRead)
{
  // GIVEN a Trainer answering with a payload no parser can use
  FakeTrainer trainer(node_);
  SpinGuard guard(*this);  // see SpinGuard: before `behavior`, after `trainer`.
  trainer.setRawPayload("{not json");
  WaitForEpisodeStart behavior("wait_for_episode", configFor(kServiceName, 10.0), context_);

  // WHEN it is ticked
  ASSERT_EQ(behavior.executeTick(), BT::NodeStatus::RUNNING);
  std::this_thread::sleep_for(std::chrono::milliseconds(300));

  // THEN an unreadable payload is not mistaken for an open episode
  EXPECT_EQ(behavior.executeTick(), BT::NodeStatus::RUNNING);

  // AND it recovers once the Trainer answers properly again
  trainer.setState("recording");
  EXPECT_EQ(tickUntilSettled(behavior, std::chrono::seconds(10)), BT::NodeStatus::SUCCESS);
}

TEST_F(WaitForEpisodeStartTest, HaltsWithoutWaitingOutTheTimeout)
{
  // GIVEN a long wait on a session that has not opened an episode
  FakeTrainer trainer(node_);
  SpinGuard guard(*this);  // see SpinGuard: before `behavior`, after `trainer`.
  WaitForEpisodeStart behavior("wait_for_episode", configFor(kServiceName, 600.0), context_);
  ASSERT_EQ(behavior.executeTick(), BT::NodeStatus::RUNNING);

  // WHEN the tree halts it (haltNode(), since halt() is the protected hook)
  const auto started = std::chrono::steady_clock::now();
  behavior.haltNode();
  const auto elapsed = std::chrono::steady_clock::now() - started;

  // THEN it returns promptly instead of blocking until the timeout expires
  EXPECT_LT(elapsed, std::chrono::seconds(5))
      << "halt() waited " << std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count() << "ms.";
}

TEST_F(WaitForEpisodeStartTest, FailsWhenTheTrainerIsAbsent)
{
  // GIVEN no Trainer on the configured service name
  WaitForEpisodeStart behavior("wait_for_episode", configFor("/absent_active_recording", 10.0), context_);

  // WHEN it is ticked
  // THEN it fails instead of blocking on a service nothing serves
  EXPECT_EQ(tickUntilSettled(behavior, std::chrono::seconds(30)), BT::NodeStatus::FAILURE);

  // AND names the service, since a typo here is the likely cause
  EXPECT_NE(context_->logger->consumeErrorLogBuffer().find("/absent_active_recording"), std::string::npos);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
