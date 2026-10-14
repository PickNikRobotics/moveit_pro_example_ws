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
#include <rebot_bench_behaviors/is_joint_at_least.hpp>

#include <algorithm>
#include <cmath>
#include <moveit_pro_behavior_interface/get_required_ports.hpp>
#include <moveit_pro_behavior_interface/metadata_fields.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <string>

namespace rebot_bench_behaviors
{
BT::PortsList IsJointAtLeast::providedPorts()
{
  return { BT::InputPort<sensor_msgs::msg::JointState>("joint_state", "Measured joint state."),
           BT::InputPort<std::string>("joint_name", "Joint to check."),
           BT::InputPort<double>("minimum", "Lowest acceptable measured position, radians.") };
}

BT::KeyValueVector IsJointAtLeast::metadata()
{
  return { { moveit_pro::behaviors::kSubcategoryMetadataKey, "reBot Bench" },
           { moveit_pro::behaviors::kDescriptionMetadataKey,
             "Succeeds only when the joint's measured position is at or above the minimum. Use it after a move "
             "whose controller may report success without reaching the goal, such as a canceled gripper opening." } };
}

BT::NodeStatus IsJointAtLeast::tick()
{
  const auto fail = [this](const std::string& message) {
    getBehaviorContext()->logger->publishFailureMessage(name(), message);
    return BT::NodeStatus::FAILURE;
  };
  const auto inputs =
      moveit_pro::behaviors::getRequiredInputs(getInput<sensor_msgs::msg::JointState>("joint_state"),
                                               getInput<std::string>("joint_name"), getInput<double>("minimum"));
  if (!inputs)
    return fail("Provide the measured joint state, joint name and minimum: " + inputs.error());
  const auto& [state, joint_name, minimum] = inputs.value();
  const auto joint = std::ranges::find(state.name, joint_name);
  const auto index = static_cast<std::size_t>(joint - state.name.begin());
  if (joint == state.name.end() || index >= state.position.size() || !std::isfinite(state.position[index]))
    return fail("The measured joint state has no position for " + joint_name + ".");
  if (state.position[index] < minimum)
    return fail(joint_name + " measured " + std::to_string(state.position[index]) + " rad, below the required " +
                std::to_string(minimum) + " rad.");
  return BT::NodeStatus::SUCCESS;
}
}  // namespace rebot_bench_behaviors
