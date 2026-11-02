// Copyright 2026 PickNik Inc.
// SPDX-License-Identifier: BSD-3-Clause

#include <cmath>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>

#include "kdl_gravity_compensation_controller/gravity_model.hpp"
#include "planar_robot.hpp"

using kdl_gravity_compensation_controller::GravityModel;
using namespace gravity_test;

namespace
{
const KDL::Vector kDown(0.0, 0.0, -kG);

std::vector<double> torques(GravityModel& model, const std::vector<double>& q)
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
    const auto expected = holding_torque(q[0], q[1]);
    const auto tau = torques(model, q);
    EXPECT_NEAR(tau[0], expected[0], 1e-9);
    EXPECT_NEAR(tau[1], expected[1], 1e-9);
  }
  const auto tau = torques(model, { 0.0, 0.0 });
  EXPECT_NEAR(tau[0], -22.0725, 1e-9);
  EXPECT_NEAR(tau[1], -2.4525, 1e-9);
}

TEST(GravityModel, PlanarArmPayloadMatchesHandComputedTorques)
{
  GravityModel model(kPlanarArm, "base", "tool", kDown, 0.4, KDL::Vector::Zero());
  const auto expected = holding_torque(0.2, 0.5, 0.4);
  const auto tau = torques(model, { 0.2, 0.5 });
  EXPECT_NEAR(tau[0], expected[0], 1e-9);
  EXPECT_NEAR(tau[1], expected[1], 1e-9);
}

TEST(GravityModel, PayloadComIsInTheTipFrame)
{
  GravityModel model(kPlanarArm, "base", "tool", kDown, 0.4, KDL::Vector(0.1, 0.0, 0.0));
  GravityModel bare(kPlanarArm, "base", "tool", kDown, 0.0, KDL::Vector::Zero());
  const auto tau = torques(model, { 0.0, 0.0 });
  const auto tau_bare = torques(bare, { 0.0, 0.0 });
  EXPECT_NEAR(tau[1] - tau_bare[1], -kG * 0.4 * 0.6, 1e-9);
  EXPECT_NEAR(tau[0] - tau_bare[0], -kG * 0.4 * 1.6, 1e-9);
}

TEST(GravityModel, GravityVectorSelectsMountOrientation)
{
  GravityModel inverted(kPlanarArm, "base", "tool", -kDown, 0.0, KDL::Vector::Zero());
  const auto expected = holding_torque(0.2, 0.5);
  const auto tau = torques(inverted, { 0.2, 0.5 });
  EXPECT_NEAR(tau[0], -expected[0], 1e-9);
  EXPECT_NEAR(tau[1], -expected[1], 1e-9);
}

TEST(GravityModel, ZeroGravityGivesZeroTorque)
{
  GravityModel model(kPlanarArm, "base", "tool", KDL::Vector::Zero(), 0.4, KDL::Vector::Zero());
  for (double t : torques(model, { 0.3, -1.2 }))
  {
    EXPECT_EQ(t, 0.0);
  }
}

TEST(GravityModel, RejectsInvalidDescriptionAndMissingOrFixedOnlyChains)
{
  EXPECT_THROW(GravityModel("invalid", "base", "tool", kDown, 0.0, KDL::Vector::Zero()), std::runtime_error);
  EXPECT_THROW(GravityModel(kPlanarArm, "base", "nowhere", kDown, 0.0, KDL::Vector::Zero()), std::runtime_error);
  EXPECT_THROW(GravityModel(kPlanarArm, "base", "base", kDown, 0.0, KDL::Vector::Zero()), std::runtime_error);
  EXPECT_THROW(GravityModel(kPlanarArm, "link2", "tool", kDown, 0.0, KDL::Vector::Zero()), std::runtime_error);
}

TEST(GravityModel, WrongSizeReportsSolverError)
{
  GravityModel model(kPlanarArm, "base", "tool", kDown, 0.0, KDL::Vector::Zero());
  KDL::JntArray q(1);
  KDL::JntArray tau(2);
  EXPECT_FALSE(model.Compute(q, tau));
}
