// Copyright 2026 PickNik Inc.
// SPDX-License-Identifier: BSD-3-Clause
#include <behaviortree_cpp/bt_factory.h>
#include <gtest/gtest.h>
#include <atomic>
#include <chrono>
#include <moveit_pro_behavior_interface/shared_resources_node_loader.hpp>
#include <moveit_pro_test_utils/ros_test.hpp>
#include <pluginlib/class_loader.hpp>
#include <rebot_bench_behaviors/call_rebot_leader_service.hpp>
#include <thread>

using namespace std::chrono_literals;
using Trigger = std_srvs::srv::Trigger;
class TestClient : public moveit_pro::behaviors::ClientInterfaceBase<Trigger>
{
public:
  void initialize(const std::string& name, std::chrono::duration<double> server_timeout,
                  std::chrono::duration<double> response_timeout) override
  {
    service = name;
    wait = server_timeout;
    timeout = response_timeout;
  }
  bool waitForServiceServer() const override
  {
    return available;
  }
  tl::expected<Trigger::Response, std::string> syncSendRequest(const Trigger::Request&) override
  {
    started = true;
    while (block && !canceled)
    {
      std::this_thread::sleep_for(1ms);
    }
    if (!error.empty())
    {
      return tl::make_unexpected(error);
    }
    return response;
  }
  void cancelRequest() override
  {
    canceled = true;
  }
  Trigger::Response response;
  std::string service, error;
  std::chrono::duration<double> wait{}, timeout{};
  bool available = true;
  std::atomic<bool> block{ false };
  std::atomic<bool> started{ false }, canceled{ false };
};

class LeaderServiceTest : public moveit_pro::test_utils::RosTest
{
protected:
  void SetUp() override
  {
    context = std::make_shared<moveit_pro::behaviors::BehaviorContext>(
        std::make_shared<rclcpp::Node>("leader_service_test"), false);
    config.blackboard = BT::Blackboard::create();
    config.input_ports["service_name"] = "/rebot_leader_bridge/mirror";
  }
  void create()
  {
    auto client = std::make_unique<TestClient>();
    fake = client.get();
    behavior =
        std::make_unique<rebot_bench_behaviors::CallRebotLeaderService>("Mirror", config, context, std::move(client));
  }
  BT::NodeStatus run()
  {
    const auto end = std::chrono::steady_clock::now() + 2s;
    BT::NodeStatus status;
    do
    {
      status = behavior->executeTick();
      std::this_thread::sleep_for(1ms);
    } while (status == BT::NodeStatus::RUNNING && std::chrono::steady_clock::now() < end);
    return status;
  }
  BT::NodeConfiguration config;
  std::shared_ptr<moveit_pro::behaviors::BehaviorContext> context;
  TestClient* fake = nullptr;
  std::unique_ptr<rebot_bench_behaviors::CallRebotLeaderService> behavior;
};
TEST_F(LeaderServiceTest, RefusalReachesObjectiveFailureBufferVerbatim)
{
  create();
  fake->response.message = "Following refused to avoid a jump: wrist roll (joint6): -6.9 deg (-0.120 rad). Move the "
                           "Star Arm, then run Mirror reBot Leader again.";
  EXPECT_EQ(run(), BT::NodeStatus::FAILURE);
  const auto errors = context->logger->consumeErrorLogBuffer();
  EXPECT_NE(errors.find(fake->response.message), std::string::npos);
  EXPECT_EQ(fake->service, "/rebot_leader_bridge/mirror");
  EXPECT_EQ(fake->wait, 3s);
  EXPECT_EQ(fake->timeout, 1s);
}
TEST_F(LeaderServiceTest, SuccessfulHeartbeatDoesNotLogFailure)
{
  create();
  fake->response.success = true;
  EXPECT_EQ(run(), BT::NodeStatus::SUCCESS);
  EXPECT_TRUE(context->logger->consumeErrorLogBuffer().empty());
}
TEST_F(LeaderServiceTest, EmptyRefusalHasOperatorInstructions)
{
  create();
  EXPECT_EQ(run(), BT::NodeStatus::FAILURE);
  EXPECT_NE(context->logger->consumeErrorLogBuffer().find("Check the leader connection"), std::string::npos);
}
TEST_F(LeaderServiceTest, MissingServicePortFailsBeforeCallingClient)
{
  config.input_ports.clear();
  create();
  EXPECT_EQ(run(), BT::NodeStatus::FAILURE);
  EXPECT_TRUE(fake->service.empty());
  EXPECT_NE(context->logger->consumeErrorLogBuffer().find("Configure service_name"), std::string::npos);
}
TEST_F(LeaderServiceTest, RejectsOtherServicesBeforeCallingClient)
{
  config.input_ports["service_name"] = "/unrelated";
  create();
  EXPECT_EQ(run(), BT::NodeStatus::FAILURE);
  EXPECT_TRUE(fake->service.empty());
}
TEST_F(LeaderServiceTest, MissingServerLogsFailure)
{
  create();
  fake->available = false;
  EXPECT_EQ(run(), BT::NodeStatus::FAILURE);
  EXPECT_NE(context->logger->consumeErrorLogBuffer().find("Server not available"), std::string::npos);
}
TEST_F(LeaderServiceTest, ResponseTimeoutLogsFailure)
{
  create();
  fake->error = "response timed out";
  EXPECT_EQ(run(), BT::NodeStatus::FAILURE);
  EXPECT_NE(context->logger->consumeErrorLogBuffer().find("response timed out"), std::string::npos);
}
TEST_F(LeaderServiceTest, HaltCancelsPendingServiceWait)
{
  create();
  fake->block = true;
  ASSERT_EQ(behavior->executeTick(), BT::NodeStatus::RUNNING);
  const auto end = std::chrono::steady_clock::now() + 1s;
  while (!fake->started && std::chrono::steady_clock::now() < end)
  {
    std::this_thread::sleep_for(1ms);
  }
  if (!fake->started)
  {
    fake->block = false;
    FAIL() << "Service did not start";
  }
  behavior->haltNode();
  EXPECT_TRUE(fake->canceled);
}
TEST_F(LeaderServiceTest, ConfiguredPluginLoaderInstantiatesLeaderService)
{
  pluginlib::ClassLoader<moveit_pro::behaviors::SharedResourcesNodeLoaderBase> loader(
      "moveit_pro_behavior_interface", "moveit_pro::behaviors::SharedResourcesNodeLoaderBase");
  auto plugin = loader.createSharedInstance("rebot_bench_behaviors::RebotBenchBehaviorsLoader");
  BT::BehaviorTreeFactory factory;
  plugin->registerBehaviors(factory, context);
  factory.registerSimpleAction("SwitchController", [](BT::TreeNode&) { return BT::NodeStatus::SUCCESS; },
                               { BT::InputPort<std::string>("activate_controllers"),
                                 BT::InputPort<bool>("automatic_deactivation"), BT::InputPort<int>("strictness") });
  factory.registerSimpleAction("WaitForDuration", [](BT::TreeNode&) { return BT::NodeStatus::SUCCESS; },
                               { BT::InputPort<double>("delay_duration") });
  auto tree = factory.createTreeFromFile(MIRROR_OBJECTIVE);
  EXPECT_NE(tree.rootNode(), nullptr);
}
