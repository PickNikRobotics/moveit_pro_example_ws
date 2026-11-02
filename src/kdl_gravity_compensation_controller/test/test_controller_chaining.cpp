// Copyright 2026 PickNik Inc.
// SPDX-License-Identifier: BSD-3-Clause

#include <algorithm>
#include <chrono>
#include <future>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include <controller_interface/controller_interface.hpp>
#include <controller_manager/controller_manager.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/rclcpp.hpp>

#include "planar_robot.hpp"

using controller_interface::CallbackReturn;
using controller_interface::interface_configuration_type;
using controller_interface::InterfaceConfiguration;
using controller_interface::return_type;
using lifecycle_msgs::msg::State;
using namespace gravity_test;
using namespace std::chrono_literals;

namespace
{
const std::string kGravity = "gravity_compensation_controller";
const std::string kPlugin = "kdl_gravity_compensation_controller/GravityCompensationController";
const rclcpp::Duration kPeriod = rclcpp::Duration::from_seconds(0.01);
const int kStrict = controller_manager_msgs::srv::SwitchController::Request::STRICT;
const std::vector<double> kSixOffsets{ 0.01, 0.02, 0.03, 0.04, 0.05, 0.06 };

std::string mock_robot(std::string robot, const std::vector<std::string>& joints)
{
  // Declare hardware in reverse joint order, unlike KDL's chain. The mock
  // mirrors commands into state; it has no transport, dynamics or devices.
  std::string control = R"(
  <ros2_control name="FakeArm" type="system">
    <hardware><plugin>mock_components/GenericSystem</plugin></hardware>)";
  for (auto joint = joints.rbegin(); joint != joints.rend(); ++joint)
  {
    control += R"(
    <joint name=")" +
               *joint + R"(">
      <command_interface name="effort"/>
      <command_interface name="position"/>
      <state_interface name="position"><param name="initial_value">0.0</param></state_interface>
      <state_interface name="effort"><param name="initial_value">0.0</param></state_interface>
    </joint>)";
  }
  control += R"(
  </ros2_control>
)";
  robot.insert(robot.rfind("</robot>"), control);
  return robot;
}

// Only exposes the real manager's resource registry for assertions. All loading,
// lifecycle, claim arbitration, chaining and scheduling use ControllerManager.
class TestManager : public controller_manager::ControllerManager
{
public:
  using ControllerManager::ControllerManager;
  hardware_interface::ResourceManager& resources()
  {
    return *resource_manager_;
  }
};

// Test-only upstream producer. No plugin, topic transport or production control
// algorithm. Inputs change between manager cycles, never concurrently with update.
class EffortProducer : public controller_interface::ControllerInterface
{
public:
  explicit EffortProducer(std::vector<std::string> names,
                          controller_interface::ControllerInterfaceBaseSharedPtr downstream = nullptr)
    : names_(std::move(names)), downstream_(std::move(downstream)), values(names_.size(), 0.0)
  {
  }
  CallbackReturn on_init() override
  {
    return CallbackReturn::SUCCESS;
  }
  InterfaceConfiguration command_interface_configuration() const override
  {
    return { interface_configuration_type::INDIVIDUAL, names_ };
  }
  InterfaceConfiguration state_interface_configuration() const override
  {
    return { interface_configuration_type::NONE, {} };
  }
  CallbackReturn on_activate(const rclcpp_lifecycle::State&) override
  {
    // Controller manager makes references available and sets chained mode
    // before activation; the downstream lifecycle may still be inactive within
    // this same switch request. Both must be active before the first update.
    downstream_chained_on_activation = !downstream_ || downstream_->is_in_chained_mode();
    return downstream_chained_on_activation ? CallbackReturn::SUCCESS : CallbackReturn::ERROR;
  }
  return_type update(const rclcpp::Time&, const rclcpp::Duration&) override
  {
    ++updates;
    if (downstream_ &&
        (downstream_->get_lifecycle_id() != State::PRIMARY_STATE_ACTIVE || !downstream_->is_in_chained_mode()))
    {
      return return_type::ERROR;
    }
    for (size_t i = 0; i < values.size(); ++i)
    {
      if (!command_interfaces_[i].set_value(values[i]))
      {
        return return_type::ERROR;
      }
    }
    return return_type::OK;
  }

private:
  std::vector<std::string> names_;
  controller_interface::ControllerInterfaceBaseSharedPtr downstream_;

public:
  std::vector<double> values;
  size_t updates = 0;
  bool downstream_chained_on_activation = false;
};

class ChainTest : public ::testing::Test
{
protected:
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
    auto params = kPlanarParams;
    params.emplace_back("gains", std::vector<double>{ 0.5, 1.75 });
    params.emplace_back("offsets", std::vector<double>{ 0.2, -0.3 });
    start(kPlanarArm, { "j1", "j2" }, params);
  }
  void start(const std::string& urdf, const std::vector<std::string>& joints,
             const std::vector<rclcpp::Parameter>& params)
  {
    joints_ = joints;
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    auto options = controller_manager::get_cm_node_options();
    options.parameter_overrides({ rclcpp::Parameter("update_rate", 100) });
    manager_ = std::make_shared<TestManager>(executor_, mock_robot(urdf, joints), true, "test_controller_manager", "",
                                             options);
    ASSERT_TRUE(manager_->is_resource_manager_initialized());
    time_ = rclcpp::Time(0, 0, manager_->get_trigger_clock()->get_clock_type());
    gravity_ = manager_->load_controller(kGravity, kPlugin);
    ASSERT_NE(gravity_, nullptr);  // Exercises the installed chainable plugin manifest.
    ASSERT_TRUE(gravity_->get_node()->set_parameters_atomically(params).successful);
    ASSERT_EQ(manager_->configure_controller(kGravity), return_type::OK);
    // These position commands supply fake measured poses, like a sensor fixture.
    // They are separate from the effort interfaces owned by gravity.
    for (const auto& joint : joints)
    {
      positions_.emplace_back(manager_->resources().claim_command_interface(joint + "/position"));
      ASSERT_TRUE(positions_.back().set_value(0.0));
    }
  }
  void TearDown() override
  {
    // Release test fixture loans before the resource manager is destroyed.
    positions_.clear();
    gravity_.reset();
    manager_.reset();
    executor_.reset();
  }
  std::shared_ptr<EffortProducer> add_producer(const std::string& name = "upstream",
                                               const std::vector<std::string>& interfaces = { kGravity + "/j2/effort",
                                                                                              kGravity + "/j1/effort" },
                                               bool downstream = true)
  {
    auto producer = std::make_shared<EffortProducer>(interfaces, downstream ? gravity_ : nullptr);
    EXPECT_NE(manager_->add_controller(producer, name, "test_only/EffortProducer"), nullptr);
    EXPECT_EQ(manager_->configure_controller(name), return_type::OK);
    return producer;
  }
  return_type tick()
  {
    time_ = time_ + kPeriod;
    manager_->read(time_, kPeriod);
    const auto result = manager_->update(time_, kPeriod);
    manager_->write(time_, kPeriod);
    return result;
  }
  template <typename Operation>
  return_type run_with_updates(Operation operation)
  {
    auto future = std::async(std::launch::async, std::move(operation));
    const auto deadline = std::chrono::steady_clock::now() + 3s;
    while (future.wait_for(0ms) != std::future_status::ready && std::chrono::steady_clock::now() < deadline)
    {
      // Pending switches and non-RT controller-list swaps require a running
      // update loop, including reconfiguration after the first activation.
      tick();
      std::this_thread::sleep_for(1ms);
    }
    EXPECT_EQ(future.wait_for(0ms), std::future_status::ready);
    return future.get();
  }
  return_type switch_controllers(const std::vector<std::string>& start, const std::vector<std::string>& stop)
  {
    return run_with_updates(
        [&] { return manager_->switch_controller(start, stop, kStrict, true, rclcpp::Duration::from_seconds(2.0)); });
  }
  void expect_output(double q1, double q2, double u1 = 0.0, double u2 = 0.0)
  {
    const auto g = holding_torque(q1, q2);
    expect_raw_output({ u1 + 0.5 * g[0] + 0.2, u2 + 1.75 * g[1] - 0.3 });
  }
  void expect_raw_output(const std::vector<double>& expected)
  {
    // GenericSystem echoes commands on read. Observe through hardware state,
    // not a sum helper or the controller's internal buffers.
    manager_->read(time_, kPeriod);
    ASSERT_EQ(expected.size(), joints_.size());
    for (size_t i = 0; i < joints_.size(); ++i)
    {
      auto state = manager_->resources().claim_state_interface(joints_[i] + "/effort");
      const auto value = state.get_optional();
      ASSERT_TRUE(value);
      EXPECT_NEAR(*value, expected[i], 1e-9) << "hardware " << joints_[i];
    }
  }
  void expect_claims(const std::string& name, std::vector<std::string> expected)
  {
    const auto controllers = manager_->get_loaded_controllers();
    const auto it =
        std::find_if(controllers.begin(), controllers.end(), [&](const auto& c) { return c.info.name == name; });
    ASSERT_NE(it, controllers.end());
    auto actual = it->info.claimed_interfaces;
    std::sort(actual.begin(), actual.end());
    std::sort(expected.begin(), expected.end());
    EXPECT_EQ(actual, expected);
  }

  std::shared_ptr<rclcpp::Executor> executor_;
  std::shared_ptr<TestManager> manager_;
  controller_interface::ControllerInterfaceBaseSharedPtr gravity_;
  std::vector<hardware_interface::LoanedCommandInterface> positions_;
  std::vector<std::string> joints_;
  rclcpp::Time time_;
};

class SixJointChainTest : public ChainTest
{
protected:
  void SetUp() override
  {
    start(kSixJointArm, kSixJoints,
          { rclcpp::Parameter("joints", kSixJoints), rclcpp::Parameter("root_link", "base"),
            rclcpp::Parameter("tip_link", "tool"), rclcpp::Parameter("gains", std::vector<double>(6, 1.0)),
            rclcpp::Parameter("offsets", kSixOffsets) });
  }
  static std::vector<std::string> references(const std::vector<size_t>& joints)
  {
    std::vector<std::string> names;
    for (auto i : joints)
    {
      names.push_back(kGravity + "/" + kSixJoints[i] + "/effort");
    }
    return names;
  }
  void expect_six(const std::vector<double>& q, const std::vector<double>& u)
  {
    const auto gravity = six_joint_holding_torque(q);
    std::vector<double> expected(6);
    for (size_t i = 0; i < 6; ++i)
    {
      expected[i] = gravity[i] + kSixOffsets[i] + u[i];
    }
    expect_raw_output(expected);
  }
};
}  // namespace

TEST_F(ChainTest, StandalonePluginClaimsHardwareAndProducesGravityWithoutAnyUpstream)
{
  EXPECT_EQ(manager_->resources().get_controller_reference_interface_names(kGravity),
            (std::vector<std::string>{ kGravity + "/j1/effort", kGravity + "/j2/effort" }));
  EXPECT_FALSE(manager_->resources().command_interface_is_claimed("j1/effort"));
  ASSERT_EQ(switch_controllers({ kGravity }, {}), return_type::OK);
  EXPECT_FALSE(gravity_->is_in_chained_mode());
  expect_claims(kGravity, { "j1/effort", "j2/effort" });
  for (size_t i = 0; i < 10; ++i)
  {
    const double q1 = static_cast<double>(i) * 0.05;
    const double q2 = -q1 * 2.0;
    ASSERT_TRUE(positions_[0].set_value(q1));
    ASSERT_TRUE(positions_[1].set_value(q2));
    ASSERT_EQ(tick(), return_type::OK);
    expect_output(q1, q2);
  }
  ASSERT_EQ(switch_controllers({}, { kGravity }), return_type::OK);
  expect_raw_output({ 0.0, 0.0 });
  EXPECT_FALSE(manager_->resources().command_interface_is_claimed("j1/effort"));
  ASSERT_EQ(switch_controllers({ kGravity }, {}), return_type::OK);
  ASSERT_EQ(tick(), return_type::OK);
  expect_output(0.45, -0.9);
}

TEST_F(ChainTest, ChainedClaimsAreDisjointAndVaryingUpstreamReachesHardwareInTheSameCycle)
{
  const auto producer = add_producer();
  ASSERT_EQ(switch_controllers({ "upstream", kGravity }, {}), return_type::OK);
  ASSERT_TRUE(producer->downstream_chained_on_activation);
  ASSERT_TRUE(gravity_->is_in_chained_mode());
  expect_claims(kGravity, { "j1/effort", "j2/effort" });
  expect_claims("upstream", { kGravity + "/j1/effort", kGravity + "/j2/effort" });
  for (const auto& name : { "j1/effort", "j2/effort" })
  {
    EXPECT_TRUE(manager_->resources().command_interface_is_claimed(name));
    EXPECT_THROW(manager_->resources().claim_command_interface(name), std::runtime_error);
  }
  EXPECT_TRUE(manager_->resources().command_interface_is_claimed(kGravity + "/j1/effort"));
  const auto controllers = manager_->get_loaded_controllers();
  ASSERT_EQ(controllers.size(), 2u);
  EXPECT_EQ(controllers[0].info.name, "upstream");
  EXPECT_EQ(controllers[1].info.name, kGravity);
  for (size_t i = 0; i < 20; ++i)
  {
    const double t = static_cast<double>(i);
    const double u1 = 2.0 - 0.2 * t;
    const double u2 = -1.0 + 0.5 * t;
    producer->values = { u2, u1 };  // Its claim order is j2, j1, unlike gravity.
    ASSERT_TRUE(positions_[0].set_value(0.03 * t));
    ASSERT_TRUE(positions_[1].set_value(-0.07 * t));
    const auto previous_updates = producer->updates;
    ASSERT_EQ(tick(), return_type::OK);
    EXPECT_EQ(producer->updates, previous_updates + 1);
    expect_output(0.03 * t, -0.07 * t, u1, u2);
  }
}

TEST_F(ChainTest, ManagerSwitchesBetweenStandaloneAndChainedWithoutStaleEffort)
{
  const auto producer = add_producer();
  producer->values = { -2.0, 4.0 };
  ASSERT_EQ(switch_controllers({ kGravity }, {}), return_type::OK);
  ASSERT_EQ(tick(), return_type::OK);
  expect_output(0.0, 0.0);
  ASSERT_EQ(switch_controllers({ "upstream" }, {}), return_type::OK);
  ASSERT_TRUE(producer->downstream_chained_on_activation);
  ASSERT_EQ(tick(), return_type::OK);
  expect_output(0.0, 0.0, 4.0, -2.0);
  ASSERT_EQ(switch_controllers({}, { "upstream" }), return_type::OK);
  EXPECT_EQ(gravity_->get_lifecycle_id(), State::PRIMARY_STATE_ACTIVE);
  EXPECT_FALSE(gravity_->is_in_chained_mode());
  EXPECT_FALSE(manager_->resources().command_interface_is_claimed(kGravity + "/j1/effort"));
  ASSERT_EQ(tick(), return_type::OK);
  expect_output(0.0, 0.0);
  producer->values = { 0.5, -1.0 };
  ASSERT_EQ(switch_controllers({ "upstream" }, {}), return_type::OK);
  ASSERT_EQ(tick(), return_type::OK);
  expect_output(0.0, 0.0, -1.0, 0.5);
  ASSERT_EQ(switch_controllers({}, { kGravity, "upstream" }), return_type::OK);
  expect_raw_output({ 0.0, 0.0 });
  ASSERT_EQ(manager_->cleanup_controller("upstream"), return_type::OK);
  ASSERT_EQ(manager_->cleanup_controller(kGravity), return_type::OK);
  ASSERT_EQ(run_with_updates([&] { return manager_->configure_controller(kGravity); }), return_type::OK);
  ASSERT_EQ(switch_controllers({ kGravity }, {}), return_type::OK);
  ASSERT_EQ(tick(), return_type::OK);
  expect_output(0.0, 0.0);
}

TEST_F(ChainTest, ManagerEnforcesChainActivationDependenciesAndExclusiveHardwareClaims)
{
  const auto producer = add_producer();
  const auto competitor = add_producer("hardware_competitor", { "j1/effort", "j2/effort" }, false);
  EXPECT_EQ(switch_controllers({ "upstream" }, {}), return_type::ERROR);
  EXPECT_EQ(producer->get_lifecycle_id(), State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(switch_controllers({ kGravity, "upstream" }, {}), return_type::OK);
  EXPECT_EQ(switch_controllers({ "hardware_competitor" }, {}), return_type::ERROR);
  EXPECT_EQ(competitor->get_lifecycle_id(), State::PRIMARY_STATE_INACTIVE);
  EXPECT_EQ(switch_controllers({}, { kGravity }), return_type::ERROR);
  EXPECT_EQ(gravity_->get_lifecycle_id(), State::PRIMARY_STATE_ACTIVE);
  EXPECT_EQ(producer->get_lifecycle_id(), State::PRIMARY_STATE_ACTIVE);
  ASSERT_EQ(tick(), return_type::OK);
  expect_output(0.0, 0.0);
}

TEST_F(ChainTest, UnclaimedJointReferenceIsZeroWhenOnlyOneJointHasAnUpstreamProducer)
{
  const auto producer = add_producer("upstream", { kGravity + "/j2/effort" });
  producer->values = { 1.25 };
  ASSERT_EQ(switch_controllers({ kGravity, "upstream" }, {}), return_type::OK);
  ASSERT_EQ(tick(), return_type::OK);
  EXPECT_FALSE(manager_->resources().command_interface_is_claimed(kGravity + "/j1/effort"));
  expect_output(0.0, 0.0, 0.0, 1.25);
  ASSERT_EQ(switch_controllers({}, { "upstream" }), return_type::OK);
  ASSERT_EQ(tick(), return_type::OK);
  expect_output(0.0, 0.0);
}

TEST_F(ChainTest, SwappingToAPartialProducerInOneSwitchLeavesNoStaleEffortOnReleasedJoints)
{
  const auto full = add_producer("full");
  full->values = { -2.0, 4.0 };
  const auto partial = add_producer("partial", { kGravity + "/j2/effort" });
  partial->values = { 1.25 };
  ASSERT_EQ(switch_controllers({ kGravity, "full" }, {}), return_type::OK);
  ASSERT_EQ(tick(), return_type::OK);
  expect_output(0.0, 0.0, 4.0, -2.0);
  ASSERT_EQ(switch_controllers({ "partial" }, { "full" }), return_type::OK);
  EXPECT_EQ(gravity_->get_lifecycle_id(), State::PRIMARY_STATE_ACTIVE);
  EXPECT_TRUE(gravity_->is_in_chained_mode());
  EXPECT_FALSE(manager_->resources().command_interface_is_claimed(kGravity + "/j1/effort"));
  for (size_t i = 0; i < 3; ++i)
  {
    ASSERT_EQ(tick(), return_type::OK);
    expect_output(0.0, 0.0, 0.0, 1.25);
  }
}

TEST_F(SixJointChainTest, SwappingSixJointProducerForThreeJointProducerLeavesReleasedJointsGravityOnly)
{
  // Claim orders differ from the chain and the reverse-declared hardware, so
  // each input must be mapped by name.
  const auto six = add_producer("six", references({ 5, 2, 0, 4, 1, 3 }));
  six->values = { 6.0, 3.0, 1.0, 5.0, 2.0, 4.0 };
  const auto three = add_producer("three", references({ 2, 0, 1 }));
  ASSERT_EQ(switch_controllers({ kGravity, "six" }, {}), return_type::OK);
  const std::vector<double> q{ 0.1, -0.2, 0.3, -0.15, 0.25, -0.35 };
  for (size_t i = 0; i < 6; ++i)
  {
    ASSERT_TRUE(positions_[i].set_value(q[i]));
  }
  ASSERT_EQ(tick(), return_type::OK);
  expect_six(q, { 1.0, 2.0, 3.0, 4.0, 5.0, 6.0 });

  ASSERT_EQ(switch_controllers({ "three" }, { "six" }), return_type::OK);
  EXPECT_EQ(gravity_->get_lifecycle_id(), State::PRIMARY_STATE_ACTIVE);
  EXPECT_TRUE(gravity_->is_in_chained_mode());
  expect_claims("three", references({ 0, 1, 2 }));
  for (const auto& released : references({ 3, 4, 5 }))
  {
    EXPECT_FALSE(manager_->resources().command_interface_is_claimed(released));
  }
  for (size_t tick_count = 0; tick_count < 5; ++tick_count)
  {
    const double t = static_cast<double>(tick_count);
    std::vector<double> moved(6);
    for (size_t i = 0; i < 6; ++i)
    {
      moved[i] = q[i] + 0.04 * t * (i % 2 == 0 ? 1.0 : -1.0);
      ASSERT_TRUE(positions_[i].set_value(moved[i]));
    }
    const double u1 = -0.5 + 0.1 * t;
    const double u2 = 0.75 - 0.2 * t;
    const double u3 = 1.5 + 0.3 * t;
    three->values = { u3, u1, u2 };
    ASSERT_EQ(tick(), return_type::OK);
    expect_six(moved, { u1, u2, u3, 0.0, 0.0, 0.0 });
  }
}

TEST_F(ChainTest, InvalidUpstreamDeactivatesTheChainAndZerosHardwareRatherThanPublishingAHealthySum)
{
  const auto producer = add_producer();
  producer->values = { 1.0, -2.0 };
  ASSERT_EQ(switch_controllers({ kGravity, "upstream" }, {}), return_type::OK);
  ASSERT_EQ(tick(), return_type::OK);
  expect_output(0.0, 0.0, -2.0, 1.0);
  // NaN is "no command" and contributes zero; infinity is invalid.
  producer->values[0] = std::numeric_limits<double>::quiet_NaN();
  ASSERT_EQ(tick(), return_type::OK);
  expect_output(0.0, 0.0, -2.0, 0.0);
  producer->values[0] = std::numeric_limits<double>::infinity();
  EXPECT_EQ(tick(), return_type::ERROR);
  EXPECT_EQ(gravity_->get_lifecycle_id(), State::PRIMARY_STATE_INACTIVE);
  EXPECT_EQ(producer->get_lifecycle_id(), State::PRIMARY_STATE_INACTIVE);
  expect_raw_output({ 0.0, 0.0 });
  EXPECT_FALSE(manager_->resources().command_interface_is_claimed("j1/effort"));
  ASSERT_EQ(switch_controllers({ kGravity }, {}), return_type::OK);
  ASSERT_EQ(tick(), return_type::OK);
  EXPECT_FALSE(gravity_->is_in_chained_mode());
  expect_output(0.0, 0.0);
}
