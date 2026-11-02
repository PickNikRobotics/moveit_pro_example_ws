// Copyright 2026 PickNik Inc.
// SPDX-License-Identifier: BSD-3-Clause

#pragma once

#include <array>
#include <cmath>
#include <string>
#include <vector>

#include <rclcpp/parameter.hpp>

namespace gravity_test
{
inline constexpr double kG = 9.81;
// Two links pitching about +y: link1 is 1 m long with 2 kg at 0.5 m, link2
// 1 kg at 0.25 m, and the tool frame 0.5 m along link2. At zero both links
// point along +x; positive joint angles swing them down.
inline const std::string kPlanarArm = R"(
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
    <limit lower="-3" upper="3" effort="100" velocity="1"/>
  </joint>
  <joint name="j2" type="revolute">
    <parent link="link1"/><child link="link2"/><origin xyz="1 0 0"/><axis xyz="0 1 0"/>
    <limit lower="-3" upper="3" effort="100" velocity="1"/>
  </joint>
  <joint name="tool_joint" type="fixed">
    <parent link="link2"/><child link="tool"/><origin xyz="0.5 0 0"/>
  </joint>
</robot>)";

// Independent, hand-derived holding torques: the moment due to gravity is
// +m*g*horizontal_reach about +y, so the actuator supplies its negative.
inline std::array<double, 2> holding_torque(double q1, double q2, double payload = 0.0)
{
  const double c1 = std::cos(q1);
  const double c12 = std::cos(q1 + q2);
  return { -kG * (2.0 * 0.5 * c1 + 1.0 * (c1 + 0.25 * c12) + payload * (c1 + 0.5 * c12)),
           -kG * (1.0 * 0.25 * c12 + payload * 0.5 * c12) };
}

// Six links pitching about +y, joint1..joint6 in series. Link i is
// kSixLengths[i] long with kSixMasses[i] at its midpoint along +x.
inline const std::vector<std::string> kSixJoints{ "joint1", "joint2", "joint3", "joint4", "joint5", "joint6" };
inline constexpr std::array<double, 6> kSixLengths{ 0.4, 0.35, 0.3, 0.25, 0.2, 0.15 };
inline constexpr std::array<double, 6> kSixMasses{ 3.0, 2.5, 2.0, 1.5, 1.0, 0.5 };

inline std::string six_joint_arm()
{
  std::string robot = R"(<robot name="six"><link name="base"/>)";
  std::string parent = "base";
  for (size_t i = 0; i < 6; ++i)
  {
    const std::string link = "link" + std::to_string(i + 1);
    const double origin = i == 0 ? 0.0 : kSixLengths[i - 1];
    robot += "<link name=\"" + link + "\"><inertial><origin xyz=\"" + std::to_string(kSixLengths[i] / 2) +
             " 0 0\"/><mass value=\"" + std::to_string(kSixMasses[i]) +
             "\"/><inertia ixx=\"0\" ixy=\"0\" ixz=\"0\" iyy=\"0\" iyz=\"0\" izz=\"0\"/></inertial></link>";
    robot += "<joint name=\"" + kSixJoints[i] + "\" type=\"revolute\"><parent link=\"" + parent + "\"/><child link=\"" +
             link + "\"/><origin xyz=\"" + std::to_string(origin) +
             " 0 0\"/><axis xyz=\"0 1 0\"/><limit lower=\"-3\" upper=\"3\" effort=\"100\" velocity=\"1\"/></joint>";
    parent = link;
  }
  robot += R"(<link name="tool"/><joint name="tool_joint" type="fixed"><parent link="link6"/>)"
           R"(<child link="tool"/><origin xyz=")" +
           std::to_string(kSixLengths[5]) + R"( 0 0"/></joint></robot>)";
  return robot;
}
inline const std::string kSixJointArm = six_joint_arm();

// Hand-derived as for the planar arm: joint k supports every link i >= k at
// that link's horizontal reach from joint k.
inline std::vector<double> six_joint_holding_torque(const std::vector<double>& q)
{
  std::array<double, 6> joint_x{};
  std::array<double, 6> com_x{};
  double x = 0.0;
  double angle = 0.0;
  for (size_t i = 0; i < 6; ++i)
  {
    angle += q[i];
    joint_x[i] = x;
    com_x[i] = x + 0.5 * kSixLengths[i] * std::cos(angle);
    x += kSixLengths[i] * std::cos(angle);
  }
  std::vector<double> tau(6, 0.0);
  for (size_t k = 0; k < 6; ++k)
  {
    for (size_t i = k; i < 6; ++i)
    {
      tau[k] -= kG * kSixMasses[i] * (com_x[i] - joint_x[k]);
    }
  }
  return tau;
}

inline const std::vector<rclcpp::Parameter> kPlanarParams = {
  rclcpp::Parameter("joints", std::vector<std::string>{ "j1", "j2" }),
  rclcpp::Parameter("root_link", "base"),
  rclcpp::Parameter("tip_link", "tool"),
};
}  // namespace gravity_test
