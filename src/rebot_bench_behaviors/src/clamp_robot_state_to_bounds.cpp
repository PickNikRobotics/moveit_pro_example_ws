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
#include <rebot_bench_behaviors/clamp_robot_state_to_bounds.hpp>

#include <algorithm>
#include <cmath>
#include <moveit_msgs/msg/robot_state.hpp>
#include <moveit_pro_base/robot_model/robot_model.hpp>
#include <moveit_pro_behavior_interface/get_required_ports.hpp>
#include <moveit_pro_behavior_interface/metadata_fields.hpp>

namespace rebot_bench_behaviors
{
tl::expected<void, std::string> clampToBounds(const moveit_pro::base::RobotModel& model,
                                              sensor_msgs::msg::JointState& joints)
{
  if (joints.position.size() != joints.name.size())
    return tl::make_unexpected("The robot state needs one position per joint.");
  for (std::size_t i = 0; i < joints.name.size(); ++i)
  {
    const auto* joint = model.getJointModel(joints.name[i]);
    if (joint == nullptr || joint->getVariableBounds().size() != 1 || !std::isfinite(joints.position[i]))
      return tl::make_unexpected("Cannot bound " + joints.name[i] + ".");
    const auto& bounds = joint->getVariableBounds().front();
    if (bounds.position_bounded)
      joints.position[i] = std::clamp(joints.position[i], bounds.min_position, bounds.max_position);
  }
  return {};
}

BT::PortsList ClampRobotStateToBounds::providedPorts()
{
  return { BT::InputPort<moveit_msgs::msg::RobotState>("robot_state", "State to bound, such as the measured arm."),
           BT::OutputPort<moveit_msgs::msg::RobotState>("bounded_state", "The state within the position bounds.") };
}

BT::KeyValueVector ClampRobotStateToBounds::metadata()
{
  return { { moveit_pro::behaviors::kSubcategoryMetadataKey, "reBot Bench" },
           { moveit_pro::behaviors::kDescriptionMetadataKey,
             "Clamps each joint of a robot state into the robot model's position bounds, so a joint that sagged past "
             "a bound can still be the command a plan is anchored to." } };
}

BT::NodeStatus ClampRobotStateToBounds::tick()
{
  const auto fail = [this](const std::string& message) {
    getBehaviorContext()->logger->publishFailureMessage(name(), message);
    return BT::NodeStatus::FAILURE;
  };
  const auto input = moveit_pro::behaviors::getRequiredInputs(getInput<moveit_msgs::msg::RobotState>("robot_state"));
  if (!input)
    return fail("Provide the robot state to bound: " + input.error());
  const auto& model = getBehaviorContext()->robot_model;
  if (!model)
    return fail("No robot model is loaded.");
  auto [state] = input.value();
  if (const auto bounded = clampToBounds(*model, state.joint_state); !bounded)
    return fail(bounded.error());
  setOutput("bounded_state", state);
  return BT::NodeStatus::SUCCESS;
}
}  // namespace rebot_bench_behaviors
