// Copyright 2026 PickNik Inc.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
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
#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <moveit_pro_behavior_interface/behavior_context.hpp>
#include <moveit_pro_behavior_interface/shared_resources_node_loader.hpp>
#include <moveit_studio_internal_msgs/srv/reset_mujoco_keyframe.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <chrono>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace
{
class MotionFixture : public BT::StatefulActionNode
{
public:
  MotionFixture(const std::string& name, const BT::NodeConfig& config, BT::NodeStatus* status, int* calls)
    : BT::StatefulActionNode(name, config), status_(*status), calls_(*calls)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {};
  }

private:
  BT::NodeStatus onStart() override
  {
    ++calls_;
    return status_;
  }
  BT::NodeStatus onRunning() override
  {
    return status_;
  }
  void onHalted() override
  {
  }
  BT::NodeStatus& status_;
  int& calls_;
};

class ControllerRestoration : public ::testing::Test
{
public:
  using List = controller_manager_msgs::srv::ListControllers;
  using Switch = controller_manager_msgs::srv::SwitchController;
  using Reset = moveit_studio_internal_msgs::srv::ResetMujocoKeyframe;

  static void SetUpTestSuite()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }

  void SetUp() override
  {
    node = std::make_shared<rclcpp::Node>("controller_restoration_test");
    services = std::make_shared<rclcpp::Node>("controller_restoration_fixture");
    context = std::make_shared<moveit_pro::behaviors::BehaviorContext>(
        node, std::make_unique<moveit_pro::behavior::LoggerBase>(), false, false);
    states = { { "joint_trajectory_admittance_controller", "inactive" },
               { "base_jgvc", "inactive" },
               { "body_jtc", "active" },
               { "base_pid", "active" } };
    list = services->create_service<List>(
        "/controller_manager/list_controllers",
        [&](const List::Request::SharedPtr, const List::Response::SharedPtr out) {
          const auto lock = std::scoped_lock(state_mutex);
          for (const auto& [name, state] : states)
          {
            controller_manager_msgs::msg::ControllerState controller;
            controller.name = name;
            controller.state = state;
            controller.required_command_interfaces = { name == "base_jgvc" || name == "base_pid" ? "base/velocity" :
                                                                                                   "arm/position" };
            if (state == "active")
            {
              controller.claimed_interfaces = controller.required_command_interfaces;
            }
            out->controller.push_back(controller);
          }
        });
    switching = services->create_service<Switch>(
        "/controller_manager/switch_controller",
        [&](const Switch::Request::SharedPtr request, const Switch::Response::SharedPtr out) {
          const auto lock = std::scoped_lock(state_mutex);
          strictnesses.push_back(request->strictness);
          const bool invalid = !unactivatable.empty() &&
                               std::find(request->activate_controllers.begin(), request->activate_controllers.end(),
                                         unactivatable) != request->activate_controllers.end();
          // ros2_control rejects invalid requests in STRICT but can drop an unactivatable controller in BEST_EFFORT.
          out->ok = !(invalid && request->strictness == Switch::Request::STRICT);
          if (out->ok)
          {
            for (const auto& name : request->deactivate_controllers)
            {
              states[name] = "inactive";
            }
            for (const auto& name : request->activate_controllers)
            {
              if (name != unactivatable)
              {
                states[name] = "active";
              }
            }
          }
        });
    reset = services->create_service<Reset>("/mujoco_system/reset_keyframe",
                                            [&](const Reset::Request::SharedPtr, const Reset::Response::SharedPtr out) {
                                              const auto lock = std::scoped_lock(state_mutex);
                                              ++resets;
                                              out->success = reset_succeeds;
                                              out->message = reset_succeeds ? "reset" : "fixture reset failed";
                                            });
    loader = std::make_unique<pluginlib::ClassLoader<moveit_pro::behaviors::SharedResourcesNodeLoaderBase>>(
        "moveit_pro_behavior_interface", "moveit_pro::behaviors::SharedResourcesNodeLoaderBase");
    for (const auto* name : { "CoreBehaviorsLoader", "MujocoBehaviorsLoader" })
    {
      plugins.push_back(loader->createSharedInstance(std::string("moveit_pro::behaviors::") + name));
      plugins.back()->registerBehaviors(factory, context);
    }
    factory.registerNodeType<MotionFixture>("MotionFixture", &motion_status, &motions);
    factory.registerBehaviorTreeFromText(R"(
      <root BTCPP_format="4"><BehaviorTree ID="Move to Joint State"><MotionFixture/></BehaviorTree>
      <TreeNodesModel><SubTree ID="Move to Joint State">
      <input_port name="target_joint_state" default=""/><input_port name="joint_group_name" default="whole_body"/>
      <input_port name="controller_names" default=""/><input_port name="controller_action_server" default=""/>
      <input_port name="execution_pipeline" default="jtc"/>
      </SubTree></TreeNodesModel></root>)");
    executor.add_node(node);
    executor.add_node(services);
    spin = std::thread([this] { executor.spin(); });
  }

  void TearDown() override
  {
    executor.cancel();
    spin.join();
    factory.clearRegisteredBehaviorTrees();
    node.reset();
    services.reset();
  }

  BT::NodeStatus finish(BT::Tree& tree)
  {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    BT::NodeStatus status = BT::NodeStatus::RUNNING;
    while (status == BT::NodeStatus::RUNNING && std::chrono::steady_clock::now() < deadline)
    {
      status = tree.tickExactlyOnce();
      tree.sleep(std::chrono::milliseconds(1));
    }
    return status;
  }

  struct Snapshot
  {
    std::map<std::string, std::string> states;
    std::vector<int> strictnesses;
    int resets;
  };

  Snapshot snapshot()
  {
    const auto lock = std::scoped_lock(state_mutex);
    return { states, strictnesses, resets };
  }

  BT::Tree wholeBody()
  {
    factory.registerBehaviorTreeFromFile(WHOLE_BODY_XML);
    return factory.createTree("Whole Body Move to Joint State");
  }

  BT::Tree navigationReset()
  {
    factory.registerBehaviorTreeFromFile(NAV_RESET_XML);
    return factory.createTree("Reset MuJoCo Sim For Navigation");
  }

  rclcpp::Node::SharedPtr node;
  rclcpp::Node::SharedPtr services;
  std::shared_ptr<moveit_pro::behaviors::BehaviorContext> context;
  std::unique_ptr<pluginlib::ClassLoader<moveit_pro::behaviors::SharedResourcesNodeLoaderBase>> loader;
  std::vector<std::shared_ptr<moveit_pro::behaviors::SharedResourcesNodeLoaderBase>> plugins;
  BT::BehaviorTreeFactory factory;
  rclcpp::executors::MultiThreadedExecutor executor;
  std::thread spin;
  rclcpp::Service<List>::SharedPtr list;
  rclcpp::Service<Switch>::SharedPtr switching;
  rclcpp::Service<Reset>::SharedPtr reset;
  // Guards what service callbacks write; tests read it through snapshot().
  std::mutex state_mutex;
  std::map<std::string, std::string> states;
  std::vector<int> strictnesses;
  int resets = 0;
  // Callbacks only read these; tests write them only while no service request is in flight.
  std::string unactivatable;
  bool reset_succeeds = true;
  BT::NodeStatus motion_status = BT::NodeStatus::SUCCESS;
  int motions = 0;
};

TEST_F(ControllerRestoration, RegressionSuccessfulMotionCannotHidePartialRestore)
{
  // GIVEN one default controller cannot activate.
  unactivatable = "joint_trajectory_admittance_controller";
  auto tree = wholeBody();
  // WHEN the successful motion attempts its restore.
  EXPECT_EQ(finish(tree), BT::NodeStatus::FAILURE);
  // THEN strict restoration exposes the failure instead of dropping the controller.
  const auto result = snapshot();
  EXPECT_EQ(result.strictnesses, (std::vector<int>{ Switch::Request::STRICT }));
  EXPECT_EQ(result.states.at(unactivatable), "inactive");
}

TEST_F(ControllerRestoration, SuccessfulMotionRestoresBothDefaults)
{
  // GIVEN a successful motion and activatable defaults.
  auto tree = wholeBody();
  // WHEN the restore completes.
  EXPECT_EQ(finish(tree), BT::NodeStatus::SUCCESS);
  // THEN the defaults own control and repeating the restore is idempotent.
  const auto result = snapshot();
  EXPECT_EQ(result.states.at("joint_trajectory_admittance_controller"), "active");
  EXPECT_EQ(result.states.at("base_jgvc"), "active");
  EXPECT_EQ(result.states.at("body_jtc"), "inactive");
  EXPECT_EQ(result.states.at("base_pid"), "inactive");
  EXPECT_EQ(finish(tree), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(snapshot().strictnesses.size(), 1U);
}

TEST_F(ControllerRestoration, FailedMotionRestoresAndStillReportsFailure)
{
  // GIVEN a failed motion with activatable defaults.
  motion_status = BT::NodeStatus::FAILURE;
  auto tree = wholeBody();
  // WHEN the failure branch restores control.
  EXPECT_EQ(finish(tree), BT::NodeStatus::FAILURE);
  // THEN both defaults activate without hiding the motion failure.
  const auto result = snapshot();
  EXPECT_EQ(result.states.at("joint_trajectory_admittance_controller"), "active");
  EXPECT_EQ(result.states.at("base_jgvc"), "active");
  EXPECT_EQ(result.strictnesses, (std::vector<int>{ Switch::Request::STRICT }));
}

TEST_F(ControllerRestoration, HaltDoesNotTickTheTrailingRestore)
{
  // GIVEN a running motion boundary; this fixture does not model controller activation or stopping.
  motion_status = BT::NodeStatus::RUNNING;
  auto tree = wholeBody();
  ASSERT_EQ(tree.tickExactlyOnce(), BT::NodeStatus::RUNNING);
  // WHEN the shipped tree is halted.
  tree.haltTree();
  // THEN no trailing restore request is sent; actual controller stop remains a runtime check.
  EXPECT_TRUE(snapshot().strictnesses.empty());
  EXPECT_EQ(motions, 1);
}

TEST_F(ControllerRestoration, RegressionResetFailureRestoresControllersButFails)
{
  // GIVEN a reset service failure after deactivation.
  auto tree = navigationReset();
  reset_succeeds = false;
  // WHEN the reset reports failure.
  EXPECT_EQ(finish(tree), BT::NodeStatus::FAILURE);
  // THEN entry controllers are restored and a later successful reset has no stale failure.
  const auto result = snapshot();
  EXPECT_EQ(result.resets, 1);
  EXPECT_EQ(result.states.at("body_jtc"), "active");
  EXPECT_EQ(result.states.at("base_pid"), "active");
  EXPECT_EQ(result.strictnesses, (std::vector<int>{ Switch::Request::STRICT, Switch::Request::STRICT }));
  reset_succeeds = true;
  EXPECT_EQ(finish(tree), BT::NodeStatus::SUCCESS);
}

TEST_F(ControllerRestoration, ResetRestoreFailureCannotReportSuccess)
{
  // GIVEN the reset can complete but an entry controller cannot reactivate.
  unactivatable = "body_jtc";
  auto tree = navigationReset();
  // WHEN restoration fails.
  EXPECT_EQ(finish(tree), BT::NodeStatus::FAILURE);
  // THEN a completed reset cannot turn the failed restore into success.
  EXPECT_EQ(snapshot().resets, 1);
}
}  // namespace
