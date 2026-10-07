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

#include <franka_spine_behaviors/create_spine_state.hpp>

#include <moveit_msgs/msg/robot_state.hpp>
#include <moveit_pro_behavior_interface/get_required_ports.hpp>
#include <moveit_pro_behavior_interface/metadata_fields.hpp>

#include <string>

namespace
{
constexpr auto kPortPosition = "position";
constexpr auto kPortSpineJointName = "spine_joint_name";
constexpr auto kPortMinSpineValue = "min_spine_value";
constexpr auto kPortMaxSpineValue = "max_spine_value";
constexpr auto kPortSpineJointState = "spine_joint_state";
}  // namespace

namespace franka_spine_behaviors
{
CreateSpineState::CreateSpineState(const std::string& name, const BT::NodeConfiguration& config,
                                   const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
  : moveit_pro::behaviors::SharedResourcesNode<BT::SyncActionNode>(name, config, shared_resources)
{
}

BT::PortsList CreateSpineState::providedPorts()
{
  return {
    BT::InputPort<double>(kPortPosition, "{position}", "Absolute spine joint value (meters)."),
    BT::InputPort<std::string>(kPortSpineJointName, "franka_spine_vertical_joint", "The prismatic spine joint."),
    BT::InputPort<double>(kPortMinSpineValue, 0.0, "Lower end of the spine travel (meters)."),
    BT::InputPort<double>(kPortMaxSpineValue, 0.85, "Upper end of the spine travel (meters)."),
    BT::OutputPort<moveit_msgs::msg::RobotState>(kPortSpineJointState, "{spine_joint_state}",
                                                 "Spine-only goal for \"Move to Joint State\" on the spine group."),
  };
}

BT::KeyValueVector CreateSpineState::metadata()
{
  return { { moveit_pro::behaviors::kSubcategoryMetadataKey, "Motion - Plan" },
           { moveit_pro::behaviors::kDescriptionMetadataKey,
             "Creates a spine-only joint goal at an absolute spine joint value. Fails if the value is outside the "
             "spine travel. Does not move the robot." } };
}

BT::NodeStatus CreateSpineState::tick()
{
  const auto ports = moveit_pro::behaviors::getRequiredInputs(
      getInput<double>(kPortPosition), getInput<std::string>(kPortSpineJointName),
      getInput<double>(kPortMinSpineValue), getInput<double>(kPortMaxSpineValue));
  if (!ports.has_value())
  {
    getBehaviorContext()->logger->publishFailureMessage(name(), "Missing required input: " + ports.error());
    return BT::NodeStatus::FAILURE;
  }
  const auto& [position, joint_name, min_value, max_value] = ports.value();
  if (position < min_value || position > max_value)
  {
    getBehaviorContext()->logger->publishFailureMessage(
        name(), "Spine position " + std::to_string(position) + " m is outside the travel [" +
                    std::to_string(min_value) + ", " + std::to_string(max_value) + "] m.");
    return BT::NodeStatus::FAILURE;
  }

  moveit_msgs::msg::RobotState spine_state;
  // A complete state holding only the spine joint, as PlanToJointGoal requires.
  spine_state.is_diff = false;
  spine_state.joint_state.name = { joint_name };
  spine_state.joint_state.position = { position };
  setOutput(kPortSpineJointState, spine_state);
  return BT::NodeStatus::SUCCESS;
}
}  // namespace franka_spine_behaviors
