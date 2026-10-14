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
#include <rebot_bench_behaviors/hold_gripper_in_trajectory.hpp>

#include <algorithm>
#include <cmath>
#include <moveit_pro_behavior_interface/metadata_fields.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <set>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

namespace rebot_bench_behaviors
{
BT::PortsList HoldGripperInTrajectory::providedPorts()
{
  return { BT::InputPort<trajectory_msgs::msg::JointTrajectory>("arm_trajectory"),
           BT::InputPort<double>("gripper_position",
                                 "Commanded joint7 position, radians, to hold throughout this motion."),
           BT::InputPort<sensor_msgs::msg::JointState>(
               "gripper_state", "Measured state containing joint7, held instead of gripper_position when a gripper "
                                "goal did not finish. Provide exactly one of the two."),
           BT::OutputPort<trajectory_msgs::msg::JointTrajectory>("trajectory") };
}

BT::KeyValueVector HoldGripperInTrajectory::metadata()
{
  return { { moveit_pro::behaviors::kSubcategoryMetadataKey, "reBot Bench" },
           { moveit_pro::behaviors::kDescriptionMetadataKey,
             "Adds the commanded gripper position to every arm trajectory point. Validate the resulting seven-joint "
             "trajectory before execution. The command must match the preceding gripper goal, or be the measured "
             "gripper state after a goal that did not finish." } };
}

BT::NodeStatus HoldGripperInTrajectory::tick()
{
  const auto fail = [this](const std::string& message) {
    getBehaviorContext()->logger->publishFailureMessage(name(), message);
    return BT::NodeStatus::FAILURE;
  };
  const auto arm_trajectory = getInput<trajectory_msgs::msg::JointTrajectory>("arm_trajectory");
  if (!arm_trajectory)
    return fail("Provide an arm trajectory: " + arm_trajectory.error());
  auto trajectory = arm_trajectory.value();
  const auto command = getInput<double>("gripper_position");
  const auto state = getInput<sensor_msgs::msg::JointState>("gripper_state");
  if (command.has_value() == state.has_value())
    return fail("Provide exactly one of the preceding gripper command and the measured gripper state.");
  double gripper_position = 0.0;
  if (command)
  {
    gripper_position = command.value();
  }
  else
  {
    const auto joint7 = std::ranges::find(state->name, "joint7");
    const auto index = static_cast<std::size_t>(joint7 - state->name.begin());
    if (joint7 == state->name.end() || index >= state->position.size())
      return fail("The measured gripper state has no joint7 position.");
    gripper_position = state->position[index];
  }
  if (!std::isfinite(gripper_position))
    return fail("The gripper command must be finite.");
  const auto joint_count = trajectory.joint_names.size();
  const std::set<std::string> names(trajectory.joint_names.begin(), trajectory.joint_names.end());
  if (joint_count == 0 || names.size() != joint_count || names.contains("joint7") || trajectory.points.empty())
    return fail(
        "Provide a nonempty arm trajectory with unique joint names and no joint7; this Behavior only adds a hold.");
  const auto valid = [](const auto& values, std::size_t size, bool optional) {
    return (values.size() == size || (optional && values.empty())) &&
           std::ranges::all_of(values, [](double value) { return std::isfinite(value); });
  };
  for (auto& point : trajectory.points)
  {
    if (!valid(point.positions, joint_count, false) || !valid(point.velocities, joint_count, true) ||
        !valid(point.accelerations, joint_count, true) || !valid(point.effort, joint_count, true))
      return fail(
          "Every trajectory point needs one finite position per joint and matching optional derivative/effort arrays.");
    point.positions.push_back(gripper_position);
    if (!point.velocities.empty())
      point.velocities.push_back(0.0);
    if (!point.accelerations.empty())
      point.accelerations.push_back(0.0);
    if (!point.effort.empty())
      point.effort.push_back(0.0);
  }
  trajectory.joint_names.push_back("joint7");
  setOutput("trajectory", trajectory);
  return BT::NodeStatus::SUCCESS;
}
}  // namespace rebot_bench_behaviors
