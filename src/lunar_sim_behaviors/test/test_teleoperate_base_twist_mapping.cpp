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

#include <lunar_sim_behaviors/teleoperate_base.hpp>

namespace lunar_sim_behaviors
{
namespace
{

moveit_pro_controllers_msgs::msg::VelocityForceCommand makeCommand(double linear_x, double angular_z)
{
  moveit_pro_controllers_msgs::msg::VelocityForceCommand command;
  command.twist.linear.x = linear_x;
  command.twist.angular.z = angular_z;
  command.velocity_controlled_axes.x = true;
  command.velocity_controlled_axes.rz = true;
  return command;
}

TEST(TeleoperateBaseTwistMapping, PassesThroughWithinLimits)
{
  const auto twist = clampTwistCommand(makeCommand(0.3, -1.0), 0.8, 2.0);
  EXPECT_DOUBLE_EQ(twist.linear.x, 0.3);
  EXPECT_DOUBLE_EQ(twist.angular.z, -1.0);
}

TEST(TeleoperateBaseTwistMapping, ClampsLinearVelocityToMax)
{
  const auto twist = clampTwistCommand(makeCommand(5.0, 0.0), 0.8, 2.0);
  EXPECT_DOUBLE_EQ(twist.linear.x, 0.8);
}

TEST(TeleoperateBaseTwistMapping, ClampsLinearVelocityToMin)
{
  const auto twist = clampTwistCommand(makeCommand(-5.0, 0.0), 0.8, 2.0);
  EXPECT_DOUBLE_EQ(twist.linear.x, -0.8);
}

TEST(TeleoperateBaseTwistMapping, ClampsAngularVelocityToLimits)
{
  const auto positive = clampTwistCommand(makeCommand(0.0, 10.0), 0.8, 2.0);
  EXPECT_DOUBLE_EQ(positive.angular.z, 2.0);

  const auto negative = clampTwistCommand(makeCommand(0.0, -10.0), 0.8, 2.0);
  EXPECT_DOUBLE_EQ(negative.angular.z, -2.0);
}

TEST(TeleoperateBaseTwistMapping, DropsAllOtherAxes)
{
  moveit_pro_controllers_msgs::msg::VelocityForceCommand command = makeCommand(0.1, 0.1);
  command.twist.linear.y = 1.0;
  command.twist.linear.z = 1.0;
  command.twist.angular.x = 1.0;
  command.twist.angular.y = 1.0;

  const auto twist = clampTwistCommand(command, 0.8, 2.0);
  EXPECT_DOUBLE_EQ(twist.linear.y, 0.0);
  EXPECT_DOUBLE_EQ(twist.linear.z, 0.0);
  EXPECT_DOUBLE_EQ(twist.angular.x, 0.0);
  EXPECT_DOUBLE_EQ(twist.angular.y, 0.0);
}

TEST(TeleoperateBaseTwistMapping, DisabledAxisFlagZeroesComponent)
{
  auto command = makeCommand(0.5, 1.5);
  command.velocity_controlled_axes.x = false;
  const auto linear_disabled = clampTwistCommand(command, 0.8, 2.0);
  EXPECT_DOUBLE_EQ(linear_disabled.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(linear_disabled.angular.z, 1.5);

  command = makeCommand(0.5, 1.5);
  command.velocity_controlled_axes.rz = false;
  const auto angular_disabled = clampTwistCommand(command, 0.8, 2.0);
  EXPECT_DOUBLE_EQ(angular_disabled.linear.x, 0.5);
  EXPECT_DOUBLE_EQ(angular_disabled.angular.z, 0.0);
}

}  // namespace
}  // namespace lunar_sim_behaviors
