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

#include <franka_behaviors/get_spine_state_for_pose_height.hpp>
#include <franka_behaviors/spine_height.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/msg/robot_state.hpp>
#include <moveit_pro_behavior_interface/get_required_ports.hpp>
#include <moveit_pro_behavior_interface/metadata_fields.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>

#include <algorithm>
#include <iterator>
#include <string>

namespace
{
constexpr auto kPortTargetPose = "target_pose";
constexpr auto kPortPlanningScene = "planning_scene";
constexpr auto kPortSpineJointName = "spine_joint_name";
constexpr auto kPortSpineParentFrame = "spine_parent_frame";
constexpr auto kPortSpineLinkName = "spine_link_name";
constexpr auto kPortHeightBias = "height_bias";
constexpr auto kPortMinSpineValue = "min_spine_value";
constexpr auto kPortMaxSpineValue = "max_spine_value";
constexpr auto kPortSpineJointState = "spine_joint_state";
constexpr auto kPortTargetSpineValue = "target_spine_value";
}  // namespace

namespace franka_behaviors
{
GetSpineStateForPoseHeight::GetSpineStateForPoseHeight(
    const std::string& name, const BT::NodeConfiguration& config,
    const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
  : moveit_pro::behaviors::SharedResourcesNode<BT::SyncActionNode>(name, config, shared_resources)
{
}

BT::PortsList GetSpineStateForPoseHeight::providedPorts()
{
  return {
    BT::InputPort<geometry_msgs::msg::PoseStamped>(kPortTargetPose, "{target_pose}",
                                                   "Pose whose height the spine link should reach."),
    BT::InputPort<moveit_msgs::msg::PlanningScene>(kPortPlanningScene, "{planning_scene}",
                                                   "Current planning scene, from GetCurrentPlanningScene. "
                                                   "Gives the current spine joint value."),
    BT::InputPort<std::string>(kPortSpineJointName, "franka_spine_vertical_joint", "The prismatic spine joint."),
    BT::InputPort<std::string>(kPortSpineParentFrame, "franka_spine",
                               "Parent link of the spine joint. The joint axis is +Z in this frame."),
    BT::InputPort<std::string>(kPortSpineLinkName, "franka_spine_mounting_point",
                               "Child link of the spine joint, whose height is matched to the pose."),
    BT::InputPort<double>(kPortHeightBias, 0.0,
                          "Added to the pose height before the computation (meters). Positive puts the spine link "
                          "above the pose."),
    BT::InputPort<double>(kPortMinSpineValue, 0.0, "Lower end of the spine travel (meters)."),
    BT::InputPort<double>(kPortMaxSpineValue, 0.85, "Upper end of the spine travel (meters)."),
    BT::OutputPort<moveit_msgs::msg::RobotState>(kPortSpineJointState, "{spine_joint_state}",
                                                 "Spine-only goal for \"Move to Joint State\" on the spine group."),
    BT::OutputPort<double>(kPortTargetSpineValue, "{target_spine_value}",
                           "The clamped spine joint value of that goal (meters)."),
  };
}

BT::KeyValueVector GetSpineStateForPoseHeight::metadata()
{
  return { { moveit_pro::behaviors::kSubcategoryMetadataKey, "Motion - Plan" },
           { moveit_pro::behaviors::kDescriptionMetadataKey,
             "Computes a spine-only joint goal that puts the spine link at the height of a target pose, clamped to "
             "the spine travel. Does not move the robot." } };
}

BT::NodeStatus GetSpineStateForPoseHeight::tick()
{
  const auto ports = moveit_pro::behaviors::getRequiredInputs(
      getInput<geometry_msgs::msg::PoseStamped>(kPortTargetPose),
      getInput<moveit_msgs::msg::PlanningScene>(kPortPlanningScene), getInput<std::string>(kPortSpineJointName),
      getInput<std::string>(kPortSpineParentFrame), getInput<std::string>(kPortSpineLinkName),
      getInput<double>(kPortHeightBias), getInput<double>(kPortMinSpineValue), getInput<double>(kPortMaxSpineValue));
  if (!ports.has_value())
  {
    getBehaviorContext()->logger->publishFailureMessage(name(), "Missing required input: " + ports.error());
    return BT::NodeStatus::FAILURE;
  }
  const auto& [target_pose, planning_scene, joint_name, parent_frame, link_name, bias, min_value, max_value] =
      ports.value();
  if (min_value > max_value)
  {
    getBehaviorContext()->logger->publishFailureMessage(name(), "min_spine_value is above max_spine_value.");
    return BT::NodeStatus::FAILURE;
  }

  const auto& joint_state = planning_scene.robot_state.joint_state;
  const auto it = std::find(joint_state.name.begin(), joint_state.name.end(), joint_name);
  const auto index = static_cast<std::size_t>(std::distance(joint_state.name.begin(), it));
  if (it == joint_state.name.end() || index >= joint_state.position.size())
  {
    getBehaviorContext()->logger->publishFailureMessage(name(), "The planning scene has no position for joint '" +
                                                                    joint_name + "'.");
    return BT::NodeStatus::FAILURE;
  }

  geometry_msgs::msg::PoseStamped pose_in_parent;
  double link_height = 0.0;
  try
  {
    const auto& tf_buffer = getBehaviorContext()->transform_buffer_ptr;
    tf2::doTransform(target_pose, pose_in_parent,
                     tf_buffer->lookupTransform(parent_frame, target_pose.header.frame_id, tf2::TimePointZero));
    link_height = tf_buffer->lookupTransform(parent_frame, link_name, tf2::TimePointZero).transform.translation.z;
  }
  catch (const std::exception& e)
  {
    getBehaviorContext()->logger->publishFailureMessage(name(), std::string("Transform lookup failed: ") + e.what());
    return BT::NodeStatus::FAILURE;
  }

  const double value = spineValueForHeight(joint_state.position[index], link_height,
                                           pose_in_parent.pose.position.z + bias, min_value, max_value);

  moveit_msgs::msg::RobotState spine_state;
  // A complete state holding only the spine joint, as PlanToJointGoal requires.
  spine_state.is_diff = false;
  spine_state.joint_state.name = { joint_name };
  spine_state.joint_state.position = { value };
  setOutput(kPortSpineJointState, spine_state);
  setOutput(kPortTargetSpineValue, value);
  return BT::NodeStatus::SUCCESS;
}
}  // namespace franka_behaviors
