// Copyright 2026 PickNik Inc.
// SPDX-License-Identifier: BSD-3-Clause
//
// The KDL gravity model on this arm's own description, as configured for
// gravity_compensation_controller in config/control/rebot.ros2_control.yaml.

#include <cstdio>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include "kdl_gravity_compensation_controller/gravity_model.hpp"

using kdl_gravity_compensation_controller::GravityModel;

namespace
{

const KDL::Vector kDown(0.0, 0.0, -9.81);
const std::vector<std::string> kJoints = { "joint1", "joint2", "joint3", "joint4", "joint5", "joint6" };

std::string RebotUrdf()
{
  std::unique_ptr<FILE, decltype(&pclose)> pipe(popen("xacro " REBOT_XACRO " hardware_interface:=real", "r"), &pclose);
  if (!pipe)
  {
    throw std::runtime_error("could not run xacro");
  }
  std::string urdf;
  char buffer[4096];
  size_t n = 0;
  while ((n = fread(buffer, 1, sizeof(buffer), pipe.get())) > 0)
  {
    urdf.append(buffer, n);
  }
  return urdf;
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

// Holding torque of a point mass at the gripper_end origin, independently of
// the dynamics solver: -J_v^T * (m * g).
std::vector<double> PointMassTorque(const std::string& urdf, const std::vector<double>& q, double mass)
{
  KDL::Tree tree;
  EXPECT_TRUE(kdl_parser::treeFromString(urdf, tree));
  KDL::Chain chain;
  EXPECT_TRUE(tree.getChain("base_link", "gripper_end", chain));
  KDL::JntArray jq(q.size());
  for (size_t i = 0; i < q.size(); ++i)
  {
    jq(i) = q[i];
  }
  KDL::Jacobian jac(q.size());
  KDL::ChainJntToJacSolver(chain).JntToJac(jq, jac);
  std::vector<double> tau(q.size());
  for (size_t i = 0; i < q.size(); ++i)
  {
    tau[i] = -KDL::dot(jac.getColumn(i).vel, mass * kDown);
  }
  return tau;
}

}  // namespace

TEST(RebotGravity, ChainIsJoint1To6)
{
  GravityModel model(RebotUrdf(), "base_link", "gripper_end", kDown, 0.0, KDL::Vector::Zero());
  EXPECT_EQ(model.joint_names(), kJoints);
}

TEST(RebotGravity, ZeroGravityGivesZeroTorque)
{
  GravityModel model(RebotUrdf(), "base_link", "gripper_end", KDL::Vector::Zero(), 0.5, KDL::Vector::Zero());
  for (double t : Torques(model, { 0.3, 1.2, 1.5, -0.4, 0.6, 0.2 }))
  {
    EXPECT_EQ(t, 0.0);
  }
}

TEST(RebotGravity, PayloadAddsTheTipPointMassTorque)
{
  const std::string urdf = RebotUrdf();
  const double mass = 0.5;
  GravityModel bare(urdf, "base_link", "gripper_end", kDown, 0.0, KDL::Vector::Zero());
  GravityModel loaded(urdf, "base_link", "gripper_end", kDown, mass, KDL::Vector::Zero());
  for (const auto& q :
       std::vector<std::vector<double>>{ { 0.0, 1.0, 1.0, 0.0, 0.0, 0.0 }, { 0.4, 1.6, 2.0, -0.5, 0.7, 0.3 } })
  {
    const auto tau_bare = Torques(bare, q);
    const auto tau_loaded = Torques(loaded, q);
    const auto expected = PointMassTorque(urdf, q, mass);
    for (size_t i = 0; i < q.size(); ++i)
    {
      EXPECT_NEAR(tau_loaded[i] - tau_bare[i], expected[i], 1e-9) << kJoints[i];
    }
    // The tip sits outboard of joint2-joint4 on the same side as the links
    // they carry, so a payload adds to the load each already holds.
    for (size_t i = 1; i <= 3; ++i)
    {
      EXPECT_NE(tau_bare[i], 0.0) << kJoints[i];
      EXPECT_GT((tau_loaded[i] - tau_bare[i]) * tau_bare[i], 0.0) << kJoints[i];
    }
  }
}
