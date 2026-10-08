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

#include <vla_sim_behaviors/sample_cube_stack_trial.hpp>

#include <cmath>
#include <numbers>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_pro_behavior_interface/get_required_ports.hpp>
#include <moveit_pro_behavior_interface/metadata_fields.hpp>
#include <vla_sim_behaviors/cube_stack_trial.hpp>

namespace
{
inline constexpr auto kDescriptionSampleCubeStackTrial = R"(
                <p>
                    Turns a trial index into that trial's cube layout and the cube pair to stack, for evaluating the
                    cube-stacking policy in vla_sim. The same trial_index and seed always give the same layout and
                    pair.
                </p>
                <p>
                    It draws the layout from the area the shipped policy was trained on, and a different seed gives a
                    different set of layouts. The pair cycles through the six ways to stack one of the three cubes on
                    another, so any six trials in a row ask for each pair once. Fails if trial_index is negative.
                </p>
            )";

constexpr auto kPortIDTrialIndex = "trial_index";
constexpr auto kPortIDSeed = "seed";
constexpr auto kPortIDCubePoses = "cube_poses";
constexpr auto kPortIDTopColor = "top_color";
constexpr auto kPortIDBottomColor = "bottom_color";
constexpr auto kPortIDPrompt = "prompt";

// MuJoCo's world frame, which SetMujocoFreeBodyPoses takes its poses in.
constexpr auto kCubePoseFrameId = "mj_world";
// Height of a cube center in mj_world, in meters, when the cube rests on the vla_sim table.
constexpr double kCubeCenterHeight = 0.115;

[[nodiscard]] geometry_msgs::msg::PoseStamped toPoseStamped(const vla_sim_behaviors::CubePlacement& placement)
{
  const double yaw_rad = placement.yaw_deg * std::numbers::pi / 180.0;
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = kCubePoseFrameId;
  pose.pose.position.x = placement.x;
  pose.pose.position.y = placement.y;
  pose.pose.position.z = kCubeCenterHeight;
  pose.pose.orientation.x = 0.0;
  pose.pose.orientation.y = 0.0;
  pose.pose.orientation.z = std::sin(yaw_rad / 2.0);
  pose.pose.orientation.w = std::cos(yaw_rad / 2.0);
  return pose;
}
}  // namespace

namespace vla_sim_behaviors
{
SampleCubeStackTrial::SampleCubeStackTrial(
    const std::string& name, const BT::NodeConfiguration& config,
    const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
  : SharedResourcesNode<BT::SyncActionNode>(name, config, shared_resources)
{
}

BT::PortsList SampleCubeStackTrial::providedPorts()
{
  return {
    BT::InputPort<int>(kPortIDTrialIndex, "{trial_index}", "The number of the trial to sample, 0 or greater."),
    BT::InputPort<int>(kPortIDSeed, 0,
                       "Picks the set of layouts the trials draw from. Give every Objective of an evaluation the "
                       "same seed, so they agree on each trial's layout."),
    BT::OutputPort<std::vector<geometry_msgs::msg::PoseStamped>>(
        kPortIDCubePoses, "{cube_poses}",
        "Poses of the red, green, and blue cubes, in that order, in mj_world. Pass them to SetMujocoFreeBodyPoses "
        "with body_names 'cube_red;cube_green;cube_blue'."),
    BT::OutputPort<std::string>(kPortIDTopColor, "{top_color}", "The cube to pick: red, green, or blue."),
    BT::OutputPort<std::string>(kPortIDBottomColor, "{bottom_color}",
                                "The cube to stack top_color on: red, green, or blue, never the same as top_color."),
    BT::OutputPort<std::string>(kPortIDPrompt, "{prompt}",
                                "The instruction for the policy, 'stack the <top_color> cube on the <bottom_color> "
                                "cube'."),
  };
}

BT::KeyValueVector SampleCubeStackTrial::metadata()
{
  return { { moveit_pro::behaviors::kSubcategoryMetadataKey, "Simulation - MuJoCo" },
           { moveit_pro::behaviors::kDescriptionMetadataKey, kDescriptionSampleCubeStackTrial } };
}

BT::NodeStatus SampleCubeStackTrial::tick()
{
  const auto ports =
      moveit_pro::behaviors::getRequiredInputs(getInput<int>(kPortIDTrialIndex), getInput<int>(kPortIDSeed));
  if (!ports.has_value())
  {
    getBehaviorContext()->logger->publishFailureMessage(
        name(), "Failed to get required values from input data ports: " + ports.error());
    return BT::NodeStatus::FAILURE;
  }
  const auto& [trial_index, seed] = ports.value();

  const auto trial = sampleCubeStackTrial(trial_index, seed);
  if (!trial.has_value())
  {
    getBehaviorContext()->logger->publishFailureMessage(name(), trial.error());
    return BT::NodeStatus::FAILURE;
  }

  std::vector<geometry_msgs::msg::PoseStamped> cube_poses;
  cube_poses.reserve(trial->placements.size());
  for (const CubePlacement& placement : trial->placements)
  {
    cube_poses.push_back(toPoseStamped(placement));
  }

  setOutput(kPortIDCubePoses, cube_poses);
  setOutput(kPortIDTopColor, trial->top_color);
  setOutput(kPortIDBottomColor, trial->bottom_color);
  setOutput(kPortIDPrompt, trial->prompt);
  return BT::NodeStatus::SUCCESS;
}
}  // namespace vla_sim_behaviors
