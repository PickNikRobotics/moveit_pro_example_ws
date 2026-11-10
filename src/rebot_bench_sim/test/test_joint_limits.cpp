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
#include <srdfdom/model.h>
#include <urdf_parser/urdf_parser.h>
#include <yaml-cpp/yaml.h>
#include <moveit_pro_base/robot_model/robot_model.hpp>
#include <moveit_pro_base/robot_state/robot_state.hpp>

#include <fstream>
#include <iterator>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>

namespace
{
std::string readFile(const std::string& path)
{
  std::ifstream stream(path);
  if (!stream)
    throw std::runtime_error("Cannot read " + path);
  return { std::istreambuf_iterator<char>(stream), std::istreambuf_iterator<char>() };
}
}  // namespace

TEST(JointLimits, HomeMarginsAcceptEndpointsAndRejectOvershoot)
{
  auto urdf = urdf::parseURDF(readFile(TEST_URDF));
  ASSERT_TRUE(urdf);
  auto srdf = std::make_shared<srdf::Model>();
  ASSERT_TRUE(srdf->initString(*urdf, readFile(std::string(PACKAGE_DIR) + "/config/moveit/rebot.srdf")));
  auto model = std::make_shared<moveit_pro::base::RobotModel>(urdf, srdf);
  moveit_pro::base::RobotState state(model);
  state.setToDefaultValues();
  const auto planning = YAML::LoadFile(std::string(PACKAGE_DIR) + "/config/moveit/joint_limits.yaml")["joint_limits"];
  const std::map<std::string, double> lower_bounds{ { "joint2", -0.02 }, { "joint3", -0.01 }, { "joint7", 0.0 } };

  for (const auto& [name, lower] : lower_bounds)
  {
    SCOPED_TRACE(name);
    auto* joint = model->getJointModel(name);
    ASSERT_NE(joint, nullptr);
    ASSERT_EQ(joint->getVariableBounds().size(), 1U);
    auto bounds = joint->getVariableBounds().front();
    ASSERT_TRUE(bounds.position_bounded);
    EXPECT_DOUBLE_EQ(bounds.min_position, lower);
    ASSERT_TRUE(planning[name]["has_position_limits"].as<bool>());
    EXPECT_DOUBLE_EQ(planning[name]["min_position"].as<double>(), lower);
    EXPECT_DOUBLE_EQ(planning[name]["max_position"].as<double>(), bounds.max_position);

    // Apply the normalized position override to MoveIt's consumer as the
    // planning configuration does, then test actual RobotState bound behavior.
    bounds.min_position = planning[name]["min_position"].as<double>();
    bounds.max_position = planning[name]["max_position"].as<double>();
    joint->setVariableBounds(name, bounds);
    for (double position : { 0.0, lower, lower + 1e-6, bounds.max_position })
    {
      state.setVariablePosition(name, position);
      EXPECT_TRUE(state.satisfiesBounds(joint, 0.0)) << position;
    }
    for (double position : { lower - 1e-6, bounds.max_position + 1e-6 })
    {
      state.setVariablePosition(name, position);
      EXPECT_FALSE(state.satisfiesBounds(joint, 0.0)) << position;
    }
    state.setToDefaultValues();
  }

  for (const auto& name : { "gripper_joint1", "gripper_joint2" })
  {
    const auto* jaw = model->getJointModel(name);
    ASSERT_NE(jaw, nullptr);
    ASSERT_EQ(jaw->getMimic(), model->getJointModel("joint7"));
    EXPECT_DOUBLE_EQ(jaw->getMimicFactor(), 0.008);
    EXPECT_DOUBLE_EQ(jaw->getMimicOffset(), 0.0);
    EXPECT_DOUBLE_EQ(jaw->getVariableBounds().front().min_position, 0.0);
    state.setVariablePosition("joint7", 0.0);
    EXPECT_DOUBLE_EQ(state.getVariablePosition(name), 0.0);
    EXPECT_TRUE(state.satisfiesBounds(jaw, 0.0));
    state.setVariablePosition("joint7", -1e-6);
    EXPECT_LT(state.getVariablePosition(name), 0.0);
    EXPECT_FALSE(state.satisfiesBounds(jaw, 0.0));
  }
}
