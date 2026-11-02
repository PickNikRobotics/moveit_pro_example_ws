// Copyright 2026 PickNik Inc.
// SPDX-License-Identifier: BSD-3-Clause

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <future>
#include <limits>
#include <memory>
#include <shared_mutex>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include <hardware_interface/handle.hpp>
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/rclcpp.hpp>

#include "kdl_gravity_compensation_controller/gravity_compensation_controller.hpp"
#include "planar_robot.hpp"

using controller_interface::return_type;
using kdl_gravity_compensation_controller::GravityCompensationController;
using lifecycle_msgs::msg::State;
using namespace gravity_test;

namespace
{
const rclcpp::Duration kPeriod = rclcpp::Duration::from_seconds(0.01);
const double kNaN = std::numeric_limits<double>::quiet_NaN();
const double kInfinity = std::numeric_limits<double>::infinity();

// Hold a handle lock in another thread to deterministically exercise the
// standard non-blocking handle's missing-data / rejected-write result.
class BusyHandle
{
public:
  explicit BusyHandle(hardware_interface::Handle& handle)
  {
    auto release = release_.get_future();
    thread_ = std::thread([this, &handle, release = std::move(release)]() mutable {
      std::unique_lock lock(handle.get_mutex());
      locked_.set_value();
      release.wait();
    });
    locked_.get_future().wait();
  }
  ~BusyHandle()
  {
    release_.set_value();
    thread_.join();
  }

private:
  std::promise<void> locked_;
  std::promise<void> release_;
  std::thread thread_;
};

class ControllerTest : public ::testing::Test
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
    for (size_t i = 0; i < 2; ++i)
    {
      const auto joint = "j" + std::to_string(i + 1);
      efforts_[i] = std::make_shared<hardware_interface::CommandInterface>(joint, "effort", "double", "0.0");
      positions_[i] = std::make_shared<hardware_interface::StateInterface>(joint, "position", "double", "0.0");
    }
  }
  void init(const std::vector<rclcpp::Parameter>& overrides)
  {
    controller_interface::ControllerInterfaceParams params;
    params.controller_name = "gravity_compensation_controller";
    params.robot_description = kPlanarArm;
    params.update_rate = 100;
    params.controller_manager_update_rate = 100;
    params.node_options.parameter_overrides(overrides);
    ASSERT_EQ(controller_.init(params), return_type::OK);
  }
  uint8_t configure(const std::vector<rclcpp::Parameter>& overrides = kPlanarParams)
  {
    init(overrides);
    const auto state = controller_.configure().id();
    if (state == State::PRIMARY_STATE_INACTIVE)
    {
      references_ = controller_.export_reference_interfaces();
    }
    return state;
  }
  void assign(const std::vector<size_t>& commands = { 0, 1 }, const std::vector<size_t>& states = { 0, 1 })
  {
    std::vector<hardware_interface::LoanedCommandInterface> efforts;
    std::vector<hardware_interface::LoanedStateInterface> positions;
    for (auto i : commands)
    {
      efforts.emplace_back(efforts_[i], nullptr);
    }
    for (auto i : states)
    {
      positions.emplace_back(positions_[i], nullptr);
    }
    controller_.assign_interfaces(std::move(efforts), std::move(positions));
  }
  void activate(bool chained = false)
  {
    assign();
    ASSERT_TRUE(controller_.set_chained_mode(chained));
    ASSERT_EQ(controller_.get_node()->activate().id(), State::PRIMARY_STATE_ACTIVE);
  }
  return_type update()
  {
    return controller_.update(rclcpp::Time(0), kPeriod);
  }
  void expect_output(const std::array<double, 2>& expected)
  {
    for (size_t i = 0; i < 2; ++i)
    {
      const auto value = efforts_[i]->get_optional();
      ASSERT_TRUE(value);
      EXPECT_NEAR(*value, expected[i], 1e-9) << "joint " << i;
    }
  }
  void expect_references_cleared()
  {
    for (const auto& reference : references_)
    {
      const auto value = reference->get_optional();
      ASSERT_TRUE(value);
      EXPECT_TRUE(std::isnan(*value)) << reference->get_name();
    }
  }

  // Handles outlive the controller's loans.
  std::array<hardware_interface::CommandInterface::SharedPtr, 2> efforts_;
  std::array<hardware_interface::StateInterface::SharedPtr, 2> positions_;
  GravityCompensationController controller_;
  std::vector<hardware_interface::CommandInterface::SharedPtr> references_;
};
}  // namespace

TEST_F(ControllerTest, ExportsNamedOrderedNaNEffortReferencesAndOnlyClaimsHardwareEffort)
{
  ASSERT_EQ(configure(), State::PRIMARY_STATE_INACTIVE);
  EXPECT_TRUE(controller_.is_chainable());
  EXPECT_FALSE(controller_.is_in_chained_mode());
  EXPECT_TRUE(controller_.export_state_interfaces().empty());
  EXPECT_EQ(controller_.command_interface_configuration().type,
            controller_interface::interface_configuration_type::INDIVIDUAL);
  EXPECT_EQ(controller_.command_interface_configuration().names,
            (std::vector<std::string>{ "j1/effort", "j2/effort" }));
  EXPECT_EQ(controller_.state_interface_configuration().names,
            (std::vector<std::string>{ "j1/position", "j2/position" }));
  ASSERT_EQ(references_.size(), 2u);
  for (size_t i = 0; i < 2; ++i)
  {
    EXPECT_EQ(references_[i]->get_name(), "gravity_compensation_controller/j" + std::to_string(i + 1) + "/effort");
    EXPECT_EQ(references_[i]->get_prefix_name(), "gravity_compensation_controller/j" + std::to_string(i + 1));
    EXPECT_EQ(references_[i]->get_interface_name(), "effort");
  }
  expect_references_cleared();
  // Re-export must not change storage or duplicate interfaces.
  EXPECT_EQ(controller_.export_reference_interfaces(), references_);
}

TEST_F(ControllerTest, StandaloneWritesGainTimesGravityPlusOffsetAndZerosOnDeactivate)
{
  auto params = kPlanarParams;
  params.emplace_back("gains", std::vector<double>{ 2.0, 1.0 });
  params.emplace_back("offsets", std::vector<double>{ 0.0, 0.5 });
  ASSERT_EQ(configure(params), State::PRIMARY_STATE_INACTIVE);
  activate();
  ASSERT_TRUE(positions_[0]->set_value(0.2));
  ASSERT_TRUE(positions_[1]->set_value(0.5));
  // Standalone ignores references, even if someone retained a handle.
  ASSERT_TRUE(references_[0]->set_value(17.0));
  ASSERT_EQ(update(), return_type::OK);
  const auto expected = holding_torque(0.2, 0.5);
  expect_output({ 2.0 * expected[0], expected[1] + 0.5 });
  expect_references_cleared();
  ASSERT_EQ(controller_.get_node()->deactivate().id(), State::PRIMARY_STATE_INACTIVE);
  expect_output({ 0.0, 0.0 });
}

TEST_F(ControllerTest, ChainedEffortIsAddedOnceWithoutGravityGainsOnUpstreamAndWithReorderedLoans)
{
  auto params = kPlanarParams;
  params.emplace_back("gains", std::vector<double>{ 0.0, 1.75 });
  params.emplace_back("offsets", std::vector<double>{ 0.25, -0.5 });
  ASSERT_EQ(configure(params), State::PRIMARY_STATE_INACTIVE);
  ASSERT_TRUE(controller_.set_chained_mode(true));
  assign({ 1, 0 }, { 0, 1 });
  ASSERT_EQ(controller_.get_node()->activate().id(), State::PRIMARY_STATE_ACTIVE);
  for (size_t tick = 0; tick < 12; ++tick)
  {
    const double t = static_cast<double>(tick);
    ASSERT_TRUE(positions_[0]->set_value(0.02 * t));
    ASSERT_TRUE(positions_[1]->set_value(-0.05 * t));
    const double u1 = 2.0 - 0.5 * t;
    const double u2 = -1.0 + 0.3 * t;
    ASSERT_TRUE(references_[0]->set_value(u1));
    ASSERT_TRUE(references_[1]->set_value(u2));
    ASSERT_EQ(update(), return_type::OK);
    const auto gravity = holding_torque(0.02 * t, -0.05 * t);
    expect_output({ u1 + 0.25, u2 + 1.75 * gravity[1] - 0.5 });
    expect_references_cleared();
    // A second write of the same reference must not accumulate gravity.
    ASSERT_TRUE(references_[0]->set_value(u1));
    ASSERT_TRUE(references_[1]->set_value(u2));
    ASSERT_EQ(update(), return_type::OK);
    expect_output({ u1 + 0.25, u2 + 1.75 * gravity[1] - 0.5 });
  }
}

TEST_F(ControllerTest, MissedWriteAndNaNReferenceContributeZeroUpstreamEffort)
{
  ASSERT_EQ(configure(), State::PRIMARY_STATE_INACTIVE);
  activate(true);
  ASSERT_TRUE(positions_[0]->set_value(0.3));
  ASSERT_TRUE(positions_[1]->set_value(-0.2));
  const auto gravity = holding_torque(0.3, -0.2);
  ASSERT_TRUE(references_[0]->set_value(5.0));
  ASSERT_TRUE(references_[1]->set_value(-4.0));
  ASSERT_EQ(update(), return_type::OK);
  expect_output({ 5.0 + gravity[0], -4.0 + gravity[1] });
  // The producer skipped this cycle: its last value must not be reused.
  ASSERT_EQ(update(), return_type::OK);
  expect_output(gravity);
  ASSERT_TRUE(references_[0]->set_value(kNaN));
  ASSERT_TRUE(references_[1]->set_value(-4.0));
  ASSERT_EQ(update(), return_type::OK);
  expect_output({ gravity[0], -4.0 + gravity[1] });
  expect_references_cleared();
}

TEST_F(ControllerTest, PositionLoansAreMappedIndependentlyOfCommandLoans)
{
  ASSERT_EQ(configure(), State::PRIMARY_STATE_INACTIVE);
  assign({ 0, 1 }, { 1, 0 });
  ASSERT_EQ(controller_.get_node()->activate().id(), State::PRIMARY_STATE_ACTIVE);
  ASSERT_TRUE(positions_[0]->set_value(0.3));
  ASSERT_TRUE(positions_[1]->set_value(-0.9));
  ASSERT_EQ(update(), return_type::OK);
  expect_output(holding_torque(0.3, -0.9));
}

TEST_F(ControllerTest, ActivationAndModeChangesDiscardPreviousReferences)
{
  ASSERT_EQ(configure(), State::PRIMARY_STATE_INACTIVE);
  ASSERT_TRUE(references_[0]->set_value(30.0));
  activate(true);
  expect_references_cleared();
  ASSERT_EQ(update(), return_type::OK);
  expect_output(holding_torque(0.0, 0.0));
  ASSERT_TRUE(references_[0]->set_value(4.0));
  EXPECT_FALSE(controller_.set_chained_mode(false));  // Standard API refuses active mode switches.
  EXPECT_TRUE(controller_.is_in_chained_mode());
  ASSERT_EQ(update(), return_type::OK);
  auto expected = holding_torque(0.0, 0.0);
  expected[0] += 4.0;
  expect_output(expected);
  ASSERT_TRUE(references_[0]->set_value(4.0));
  ASSERT_EQ(controller_.get_node()->deactivate().id(), State::PRIMARY_STATE_INACTIVE);
  expect_references_cleared();
  ASSERT_TRUE(references_[0]->set_value(8.0));
  ASSERT_TRUE(controller_.set_chained_mode(false));
  expect_references_cleared();
  ASSERT_TRUE(references_[0]->set_value(9.0));
  assign();
  ASSERT_EQ(controller_.get_node()->activate().id(), State::PRIMARY_STATE_ACTIVE);
  ASSERT_EQ(update(), return_type::OK);
  expect_output(holding_torque(0.0, 0.0));
}

TEST_F(ControllerTest, CleanupReconfigureExportsFreshReferences)
{
  ASSERT_EQ(configure(), State::PRIMARY_STATE_INACTIVE);
  activate(true);
  ASSERT_TRUE(references_[0]->set_value(8.0));
  ASSERT_EQ(controller_.get_node()->deactivate().id(), State::PRIMARY_STATE_INACTIVE);
  controller_.release_interfaces();
  ASSERT_EQ(controller_.get_node()->cleanup().id(), State::PRIMARY_STATE_UNCONFIGURED);
  ASSERT_EQ(controller_.configure().id(), State::PRIMARY_STATE_INACTIVE);
  const auto fresh = controller_.export_reference_interfaces();
  ASSERT_EQ(fresh.size(), 2u);
  EXPECT_NE(fresh[0], references_[0]);
  EXPECT_EQ(fresh[0]->get_name(), references_[0]->get_name());
  EXPECT_TRUE(std::isnan(*fresh[0]->get_optional()));
  // Obsolete handles cannot affect a new configuration.
  ASSERT_TRUE(references_[0]->set_value(90.0));
  references_ = fresh;
  activate();
  ASSERT_EQ(update(), return_type::OK);
  expect_output(holding_torque(0.0, 0.0));
}

TEST_F(ControllerTest, MissingOrDuplicateHardwareInterfacesRefuseActivation)
{
  ASSERT_EQ(configure(), State::PRIMARY_STATE_INACTIVE);
  for (const auto& commands : std::vector<std::vector<size_t>>{ { 0 }, { 0, 0 } })
  {
    assign(commands);
    EXPECT_EQ(controller_.get_node()->activate().id(), State::PRIMARY_STATE_INACTIVE);
  }
  for (const auto& states : std::vector<std::vector<size_t>>{ { 0 }, { 1, 1 } })
  {
    assign({ 0, 1 }, states);
    EXPECT_EQ(controller_.get_node()->activate().id(), State::PRIMARY_STATE_INACTIVE);
  }
}

TEST_F(ControllerTest, NonDoubleInterfacesRefuseActivation)
{
  ASSERT_EQ(configure(), State::PRIMARY_STATE_INACTIVE);
  efforts_[1] = std::make_shared<hardware_interface::CommandInterface>("j2", "effort", "float32", "0.0");
  assign();
  EXPECT_EQ(controller_.get_node()->activate().id(), State::PRIMARY_STATE_INACTIVE);
  controller_.release_interfaces();
  efforts_[1] = std::make_shared<hardware_interface::CommandInterface>("j2", "effort", "double", "0.0");
  positions_[1] = std::make_shared<hardware_interface::StateInterface>("j2", "position", "float32", "0.0");
  assign();
  EXPECT_EQ(controller_.get_node()->activate().id(), State::PRIMARY_STATE_INACTIVE);
}

TEST_F(ControllerTest, InvalidPositionRefusesActivationAndInvalidUpdatesZeroEveryCommand)
{
  ASSERT_EQ(configure(), State::PRIMARY_STATE_INACTIVE);
  assign();
  ASSERT_TRUE(positions_[1]->set_value(kNaN));
  EXPECT_EQ(controller_.get_node()->activate().id(), State::PRIMARY_STATE_INACTIVE);
  ASSERT_TRUE(positions_[1]->set_value(0.0));
  activate(true);
  for (double invalid : { kNaN, kInfinity, -kInfinity })
  {
    ASSERT_TRUE(positions_[1]->set_value(0.0));
    ASSERT_EQ(update(), return_type::OK);
    ASSERT_TRUE(positions_[1]->set_value(invalid));
    ASSERT_TRUE(references_[0]->set_value(1.0));
    EXPECT_EQ(update(), return_type::ERROR);
    expect_output({ 0.0, 0.0 });
    // A failed update still consumes every reference.
    expect_references_cleared();
  }
  ASSERT_TRUE(positions_[1]->set_value(0.0));
  for (double invalid : { kInfinity, -kInfinity })
  {
    ASSERT_EQ(update(), return_type::OK);
    ASSERT_TRUE(references_[0]->set_value(invalid));
    EXPECT_EQ(update(), return_type::ERROR);
    expect_output({ 0.0, 0.0 });
  }
}

TEST_F(ControllerTest, UnreadablePositionAndReferenceReturnErrorNotLastHealthyTorque)
{
  ASSERT_EQ(configure(), State::PRIMARY_STATE_INACTIVE);
  activate(true);
  ASSERT_EQ(update(), return_type::OK);
  {
    BusyHandle busy(*positions_[1]);
    EXPECT_EQ(update(), return_type::ERROR);
    expect_output({ 0.0, 0.0 });
  }
  ASSERT_EQ(update(), return_type::OK);
  {
    BusyHandle busy(*references_[1]);
    EXPECT_EQ(update(), return_type::ERROR);
    expect_output({ 0.0, 0.0 });
  }
}

TEST_F(ControllerTest, RejectedHardwareWriteReturnsErrorAndBestEffortZerosOtherJoints)
{
  ASSERT_EQ(configure(), State::PRIMARY_STATE_INACTIVE);
  activate(true);
  ASSERT_EQ(update(), return_type::OK);
  {
    BusyHandle busy(*efforts_[1]);
    EXPECT_EQ(update(), return_type::ERROR);
    EXPECT_EQ(efforts_[0]->get_optional(), 0.0);
  }
  // The busy command could not be zeroed. Failure is not reported as success.
  EXPECT_NE(efforts_[1]->get_optional(), 0.0);
}

TEST_F(ControllerTest, MissingDataDuringActivationOrStandaloneResetDoesNotSucceed)
{
  ASSERT_EQ(configure(), State::PRIMARY_STATE_INACTIVE);
  assign();
  {
    BusyHandle busy(*positions_[0]);
    EXPECT_EQ(controller_.get_node()->activate().id(), State::PRIMARY_STATE_INACTIVE);
  }
  {
    BusyHandle busy(*references_[0]);
    EXPECT_FALSE(controller_.set_chained_mode(true));
    EXPECT_FALSE(controller_.is_in_chained_mode());
    EXPECT_EQ(controller_.get_node()->activate().id(), State::PRIMARY_STATE_INACTIVE);
  }
  activate();
  ASSERT_EQ(update(), return_type::OK);
  {
    BusyHandle busy(*references_[0]);
    EXPECT_EQ(update(), return_type::ERROR);
    expect_output({ 0.0, 0.0 });
  }
}

TEST_F(ControllerTest, ShutdownZerosHardwareAndReferences)
{
  ASSERT_EQ(configure(), State::PRIMARY_STATE_INACTIVE);
  activate(true);
  ASSERT_TRUE(references_[0]->set_value(2.0));
  ASSERT_EQ(update(), return_type::OK);
  ASSERT_TRUE(references_[0]->set_value(2.0));
  ASSERT_EQ(controller_.get_node()->shutdown().id(), State::PRIMARY_STATE_FINALIZED);
  expect_output({ 0.0, 0.0 });
  expect_references_cleared();
}

TEST_F(ControllerTest, DiagnosticReportsMeasuredPositionsAndSummedCommands)
{
  ASSERT_EQ(configure(), State::PRIMARY_STATE_INACTIVE);
  activate(true);
  const auto observer = std::make_shared<rclcpp::Node>("effort_observer");
  std::shared_ptr<sensor_msgs::msg::JointState> received;
  const auto subscription = observer->create_subscription<sensor_msgs::msg::JointState>(
      "/gravity_compensation_controller/commanded_effort", rclcpp::SystemDefaultsQoS(),
      [&](sensor_msgs::msg::JointState::SharedPtr message) { received = std::move(message); });
  ASSERT_TRUE(positions_[0]->set_value(0.2));
  ASSERT_TRUE(positions_[1]->set_value(-0.4));
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
  const rclcpp::Time stamp(1, 230000000, RCL_ROS_TIME);
  while (!received && std::chrono::steady_clock::now() < deadline)
  {
    ASSERT_TRUE(references_[0]->set_value(3.0));
    ASSERT_TRUE(references_[1]->set_value(-2.0));
    ASSERT_EQ(controller_.update(stamp, kPeriod), return_type::OK);
    rclcpp::spin_some(observer);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  ASSERT_NE(received, nullptr);
  EXPECT_EQ(received->name, (std::vector<std::string>{ "j1", "j2" }));
  EXPECT_EQ(received->position, (std::vector<double>{ 0.2, -0.4 }));
  ASSERT_EQ(received->effort.size(), 2u);
  const auto gravity = holding_torque(0.2, -0.4);
  EXPECT_NEAR(received->effort[0], 3.0 + gravity[0], 1e-9);
  EXPECT_NEAR(received->effort[1], -2.0 + gravity[1], 1e-9);
  EXPECT_EQ(rclcpp::Time(received->header.stamp), stamp);
}

TEST_F(ControllerTest, FiniteInputsThatOverflowAreRejectedBeforeAnyCommandIsWritten)
{
  auto params = kPlanarParams;
  params.emplace_back("gains", std::vector<double>{ 0.0, std::numeric_limits<double>::max() });
  ASSERT_EQ(configure(params), State::PRIMARY_STATE_INACTIVE);
  activate(true);
  ASSERT_TRUE(references_[0]->set_value(1.0));
  EXPECT_EQ(update(), return_type::ERROR);
  expect_output({ 0.0, 0.0 });
}

TEST_F(ControllerTest, GainsAndOffsetsApplyLiveOnlyToGravityAndRejectInvalidTransactions)
{
  ASSERT_EQ(configure(), State::PRIMARY_STATE_INACTIVE);
  activate(true);
  const auto write_and_update = [&] {
    EXPECT_TRUE(references_[0]->set_value(2.0));
    EXPECT_TRUE(references_[1]->set_value(-3.0));
    return update();
  };
  const auto node = controller_.get_node();
  ASSERT_TRUE(node->set_parameters_atomically({ rclcpp::Parameter("gains", std::vector<double>{ 0.0, 1.75 }),
                                                rclcpp::Parameter("offsets", std::vector<double>{ 0.25, 0.0 }) })
                  .successful);
  const auto expected = holding_torque(0.0, 0.0);
  for (const auto& bad :
       std::vector<rclcpp::Parameter>{ rclcpp::Parameter("gains", std::vector<double>{ 1.0 }),
                                       rclcpp::Parameter("gains", std::vector<double>{ kNaN, 1.0 }),
                                       rclcpp::Parameter("gains", std::vector<double>{ 1.0, kInfinity }),
                                       rclcpp::Parameter("offsets", std::vector<double>{ 0.0, -kInfinity }),
                                       rclcpp::Parameter("offsets", "not a double array") })
  {
    EXPECT_FALSE(node->set_parameter(bad).successful);
    ASSERT_EQ(write_and_update(), return_type::OK);
    expect_output({ 2.25, -3.0 + 1.75 * expected[1] });
  }
  EXPECT_FALSE(node->set_parameters_atomically({ rclcpp::Parameter("gains", std::vector<double>{ 0.0, 0.0 }),
                                                 rclcpp::Parameter("offsets", std::vector<double>{ kNaN, 0.0 }) })
                   .successful);
  ASSERT_EQ(write_and_update(), return_type::OK);
  expect_output({ 2.25, -3.0 + 1.75 * expected[1] });
}

TEST_F(ControllerTest, LaterParameterVetoCannotChangeControllerTrim)
{
  init(kPlanarParams);
  // Register first: validation callbacks are called in reverse registration order.
  const auto veto = controller_.get_node()->add_on_set_parameters_callback([](const auto&) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = false;
    result.reason = "test transaction veto";
    return result;
  });
  ASSERT_EQ(controller_.configure().id(), State::PRIMARY_STATE_INACTIVE);
  references_ = controller_.export_reference_interfaces();
  activate();
  EXPECT_FALSE(
      controller_.get_node()->set_parameter(rclcpp::Parameter("gains", std::vector<double>{ 0.0, 0.0 })).successful);
  ASSERT_EQ(update(), return_type::OK);
  expect_output(holding_torque(0.0, 0.0));
}

class InvalidConfigurationTest : public ControllerTest, public ::testing::WithParamInterface<rclcpp::Parameter>
{
};

TEST_P(InvalidConfigurationTest, RefusesInvalidModelOrParameters)
{
  auto params = kPlanarParams;
  const auto invalid = GetParam();
  params.erase(std::remove_if(params.begin(), params.end(),
                              [&](const auto& param) { return param.get_name() == invalid.get_name(); }),
               params.end());
  params.push_back(invalid);
  EXPECT_NE(configure(params), State::PRIMARY_STATE_INACTIVE);
}

INSTANTIATE_TEST_SUITE_P(Invalid, InvalidConfigurationTest,
                         ::testing::Values(rclcpp::Parameter("joints", std::vector<std::string>{}),
                                           rclcpp::Parameter("joints", std::vector<std::string>{ "j2", "j1" }),
                                           rclcpp::Parameter("joints", std::vector<std::string>{ "j1", "j1" }),
                                           rclcpp::Parameter("gains", std::vector<double>{ 1.0 }),
                                           rclcpp::Parameter("gains", std::vector<double>{ kNaN, 1.0 }),
                                           rclcpp::Parameter("offsets", std::vector<double>{ 0.0, kInfinity }),
                                           rclcpp::Parameter("gravity_vector", std::vector<double>{ 0.0, 0.0 }),
                                           rclcpp::Parameter("gravity_vector", std::vector<double>{ 0.0, 0.0, kNaN }),
                                           rclcpp::Parameter("payload_com", std::vector<double>{ kInfinity, 0.0, 0.0 }),
                                           rclcpp::Parameter("payload_mass", kNaN),
                                           rclcpp::Parameter("payload_mass", -0.1),
                                           rclcpp::Parameter("tip_link", "base"),
                                           rclcpp::Parameter("tip_link", "nowhere")));
