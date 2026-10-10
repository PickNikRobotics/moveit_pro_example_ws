// Copyright 2026 PickNik Inc.
// SPDX-License-Identifier: BSD-3-Clause

#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include <hardware_interface/handle.hpp>
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/rclcpp.hpp>

#include "kdl_gravity_compensation_controller/gravity_compensation_controller.hpp"
#include "kdl_gravity_compensation_controller/gravity_model.hpp"

using kdl_gravity_compensation_controller::GravityCompensationController;
using kdl_gravity_compensation_controller::GravityModel;
using lifecycle_msgs::msg::State;

namespace
{

constexpr double kG = 9.81;
const KDL::Vector kDown(0.0, 0.0, -kG);

// Two links pitching about +y: link1 is 1 m long with 2 kg at 0.5 m, link2
// 1 kg at 0.25 m, and the tool frame 0.5 m along link2. With q = 0 both
// links point along +x, and a positive angle swings them down.
const char* kPlanarArm = R"(
<robot name="planar">
  <link name="base"/>
  <link name="link1">
    <inertial><origin xyz="0.5 0 0"/><mass value="2.0"/>
      <inertia ixx="0" ixy="0" ixz="0" iyy="0" iyz="0" izz="0"/></inertial>
  </link>
  <link name="link2">
    <inertial><origin xyz="0.25 0 0"/><mass value="1.0"/>
      <inertia ixx="0" ixy="0" ixz="0" iyy="0" iyz="0" izz="0"/></inertial>
  </link>
  <link name="tool"/>
  <joint name="j1" type="revolute">
    <parent link="base"/><child link="link1"/><axis xyz="0 1 0"/>
    <limit lower="-3" upper="3" effort="1" velocity="1"/>
  </joint>
  <joint name="j2" type="revolute">
    <parent link="link1"/><child link="link2"/><origin xyz="1 0 0"/><axis xyz="0 1 0"/>
    <limit lower="-3" upper="3" effort="1" velocity="1"/>
  </joint>
  <joint name="tool_joint" type="fixed">
    <parent link="link2"/><child link="tool"/><origin xyz="0.5 0 0"/>
  </joint>
</robot>)";

// Hand-derived holding torques. Gravity pulls a mass at horizontal reach r
// with moment +m g r about +y, so the joint must supply -m g r.
std::vector<double> PlanarHoldingTorque(double q1, double q2, double payload)
{
  const double c1 = std::cos(q1);
  const double c12 = std::cos(q1 + q2);
  const double tau2 = -kG * (1.0 * 0.25 * c12 + payload * 0.5 * c12);
  const double tau1 = -kG * (2.0 * 0.5 * c1 + 1.0 * (c1 + 0.25 * c12) + payload * (c1 + 0.5 * c12));
  return { tau1, tau2 };
}

std::vector<double> Torques(GravityModel& model, const std::vector<double>& q)
{
  KDL::JntArray jq(q.size());
  KDL::JntArray tau(q.size());
  for (size_t i = 0; i < q.size(); ++i)
  {
    jq(i) = q[i];
  }
  EXPECT_TRUE(model.Compute(jq, tau));
  return std::vector<double>(tau.data.data(), tau.data.data() + tau.rows());
}

}  // namespace

TEST(GravityModel, PlanarArmMatchesHandComputedTorques)
{
  GravityModel model(kPlanarArm, "base", "tool", kDown, 0.0, KDL::Vector::Zero());
  EXPECT_EQ(model.joint_names(), (std::vector<std::string>{ "j1", "j2" }));
  for (const auto& q :
       std::vector<std::vector<double>>{ { 0.0, 0.0 }, { M_PI / 2, 0.0 }, { M_PI / 4, -M_PI / 4 }, { -0.3, 1.1 } })
  {
    const auto expected = PlanarHoldingTorque(q[0], q[1], 0.0);
    const auto tau = Torques(model, q);
    EXPECT_NEAR(tau[0], expected[0], 1e-9) << q[0] << ", " << q[1];
    EXPECT_NEAR(tau[1], expected[1], 1e-9) << q[0] << ", " << q[1];
  }
  // Fully horizontal: -9.81 * (2 * 0.5 + 1 * 1.25) and -9.81 * 0.25.
  const auto tau = Torques(model, { 0.0, 0.0 });
  EXPECT_NEAR(tau[0], -22.0725, 1e-9);
  EXPECT_NEAR(tau[1], -2.4525, 1e-9);
}

TEST(GravityModel, PlanarArmPayloadMatchesHandComputedTorques)
{
  GravityModel model(kPlanarArm, "base", "tool", kDown, 0.4, KDL::Vector::Zero());
  const auto expected = PlanarHoldingTorque(0.2, 0.5, 0.4);
  const auto tau = Torques(model, { 0.2, 0.5 });
  EXPECT_NEAR(tau[0], expected[0], 1e-9);
  EXPECT_NEAR(tau[1], expected[1], 1e-9);
}

TEST(GravityModel, PayloadComIsInTheTipFrame)
{
  // 0.1 m further out along the tool's x axis is 0.6 m along link2.
  GravityModel model(kPlanarArm, "base", "tool", kDown, 0.4, KDL::Vector(0.1, 0.0, 0.0));
  GravityModel bare(kPlanarArm, "base", "tool", kDown, 0.0, KDL::Vector::Zero());
  const auto tau = Torques(model, { 0.0, 0.0 });
  const auto tau_bare = Torques(bare, { 0.0, 0.0 });
  EXPECT_NEAR(tau[1] - tau_bare[1], -kG * 0.4 * 0.6, 1e-9);
  EXPECT_NEAR(tau[0] - tau_bare[0], -kG * 0.4 * 1.6, 1e-9);
}

TEST(GravityModel, ZeroGravityGivesZeroTorque)
{
  GravityModel model(kPlanarArm, "base", "tool", KDL::Vector::Zero(), 0.4, KDL::Vector::Zero());
  for (double t : Torques(model, { 0.3, -1.2 }))
  {
    EXPECT_EQ(t, 0.0);
  }
}

TEST(GravityModel, RejectsAMissingChain)
{
  EXPECT_THROW(GravityModel(kPlanarArm, "base", "nowhere", kDown, 0.0, KDL::Vector::Zero()), std::runtime_error);
}

namespace
{

// The controller on the planar arm, driven through loaned interfaces the way
// the controller manager drives it.
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

  uint8_t Configure(const std::vector<rclcpp::Parameter>& overrides)
  {
    controller_interface::ControllerInterfaceParams params;
    params.controller_name = "gravity_compensation_controller";
    params.robot_description = kPlanarArm;
    params.update_rate = 100;
    params.controller_manager_update_rate = 100;
    params.node_options.parameter_overrides(overrides);
    EXPECT_EQ(controller_.init(params), controller_interface::return_type::OK);
    return controller_.configure().id();
  }

  void Activate()
  {
    std::vector<hardware_interface::LoanedCommandInterface> commands;
    std::vector<hardware_interface::LoanedStateInterface> states;
    for (size_t i = 0; i < 2; ++i)
    {
      const std::string joint = "j" + std::to_string(i + 1);
      effort_[i] = std::make_shared<hardware_interface::CommandInterface>(joint, "effort", &effort_value_[i]);
      position_[i] = std::make_shared<hardware_interface::StateInterface>(joint, "position", &position_value_[i]);
      commands.emplace_back(effort_[i], nullptr);
      states.emplace_back(position_[i], nullptr);
    }
    controller_.assign_interfaces(std::move(commands), std::move(states));
    ASSERT_EQ(controller_.get_node()->activate().id(), State::PRIMARY_STATE_ACTIVE);
  }

  GravityCompensationController controller_;
  double effort_value_[2] = { 0.0, 0.0 };
  double position_value_[2] = { 0.0, 0.0 };
  hardware_interface::CommandInterface::SharedPtr effort_[2];
  hardware_interface::StateInterface::SharedPtr position_[2];
};

const std::vector<rclcpp::Parameter> kPlanarParams = {
  rclcpp::Parameter("joints", std::vector<std::string>{ "j1", "j2" }),
  rclcpp::Parameter("root_link", "base"),
  rclcpp::Parameter("tip_link", "tool"),
};

}  // namespace

TEST_F(ControllerTest, WritesGainTimesGravityPlusOffsetAndZeroOnDeactivate)
{
  auto params = kPlanarParams;
  params.emplace_back("gains", std::vector<double>{ 2.0, 1.0 });
  params.emplace_back("offsets", std::vector<double>{ 0.0, 0.5 });
  ASSERT_EQ(Configure(params), State::PRIMARY_STATE_INACTIVE);
  EXPECT_EQ(controller_.command_interface_configuration().names,
            (std::vector<std::string>{ "j1/effort", "j2/effort" }));
  EXPECT_EQ(controller_.state_interface_configuration().names,
            (std::vector<std::string>{ "j1/position", "j2/position" }));
  Activate();

  position_value_[0] = 0.2;
  position_value_[1] = 0.5;
  ASSERT_EQ(controller_.update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
            controller_interface::return_type::OK);
  const auto expected = PlanarHoldingTorque(0.2, 0.5, 0.0);
  EXPECT_NEAR(effort_value_[0], 2.0 * expected[0], 1e-9);
  EXPECT_NEAR(effort_value_[1], expected[1] + 0.5, 1e-9);

  ASSERT_EQ(controller_.get_node()->deactivate().id(), State::PRIMARY_STATE_INACTIVE);
  EXPECT_EQ(effort_value_[0], 0.0);
  EXPECT_EQ(effort_value_[1], 0.0);
}

TEST_F(ControllerTest, RejectsJointsThatAreNotTheChain)
{
  auto params = kPlanarParams;
  params[0] = rclcpp::Parameter("joints", std::vector<std::string>{ "j2", "j1" });
  EXPECT_NE(Configure(params), State::PRIMARY_STATE_INACTIVE);
}

TEST_F(ControllerTest, RejectsAGainPerJointMismatch)
{
  auto params = kPlanarParams;
  params.emplace_back("gains", std::vector<double>{ 1.0 });
  EXPECT_NE(Configure(params), State::PRIMARY_STATE_INACTIVE);
}
