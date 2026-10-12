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

#include <rebot_bench_behaviors/anchor_trajectory_to_command.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <map>
#include <string>
#include <vector>

#include <fmt/format.h>
#include <moveit_msgs/msg/robot_state.hpp>
#include <moveit_pro_behavior_interface/get_required_ports.hpp>
#include <moveit_pro_behavior_interface/metadata_fields.hpp>
#include <rclcpp/duration.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

namespace
{
inline constexpr auto kDescriptionAnchorTrajectoryToCommand = R"(
                <p>
                    Moves a trajectory planned from the measured joint state so that it starts at the controller's
                    position command, for an arm whose joints settle short of their command under load.
                </p>
                <p>
                    A controller that adopts the first point of each trajectory as its new command would otherwise
                    step a loaded joint by the difference between its command and its measured position before it
                    moves. Every point of each planned joint is offset by that difference, so the command stays
                    continuous and the arm, still settling short by the same amount, ends where the plan does. Joints
                    that commanded_state names but the trajectory leaves out are added and held at their commanded
                    positions, so a goal for part of the robot does not let the controller fill them from the measured
                    state. Check the planned trajectory for collisions, not the anchored one: the planned one is where
                    the loaded arm goes.
                </p>
                <p>
                    measured_joints start from measured_state even when commanded_state names them, such as a jaw
                    planned from its measured position: they are neither shifted nor held at their command.
                </p>
                <p>
                    fade_out ramps the offsets to zero, smoothly in velocity and acceleration, so that the trajectory
                    ends exactly at its planned last point, for a move to a pose where the arm is not loaded.
                    Uses measured state when a command is absent or differs from measurement by more than 0.10,
                    including held joints.
                </p>
            )";

constexpr auto kPortIDJointTrajectory = "joint_trajectory";
constexpr auto kPortIDCommandedState = "commanded_state";
constexpr auto kPortIDFadeOut = "fade_out";
constexpr auto kPortIDMeasuredJoints = "measured_joints";
constexpr auto kPortIDAnchoredTrajectory = "anchored_trajectory";

[[nodiscard]] std::map<std::string, double> positionsByName(const sensor_msgs::msg::JointState& joint_state)
{
  std::map<std::string, double> positions;
  for (std::size_t i = 0; i < joint_state.name.size() && i < joint_state.position.size(); ++i)
  {
    positions[joint_state.name[i]] = joint_state.position[i];
  }
  return positions;
}

// Share of the offset still applied at normalized time s, and its first and second derivatives with respect to s: a
// quintic that falls from 1 to 0 with zero slope and curvature at both ends.
struct Fade
{
  double weight;
  double rate;
  double curvature;
};

[[nodiscard]] Fade fadeAt(const double s)
{
  return { .weight = 1.0 - s * s * s * (10.0 - 15.0 * s + 6.0 * s * s),
           .rate = -30.0 * s * s * (1.0 - s) * (1.0 - s),
           .curvature = -60.0 * s * (1.0 - s) * (1.0 - 2.0 * s) };
}
}  // namespace

namespace rebot_bench_behaviors
{
AnchorTrajectoryToCommand::AnchorTrajectoryToCommand(
    const std::string& name, const BT::NodeConfiguration& config,
    const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
  : SharedResourcesNode<BT::SyncActionNode>(name, config, shared_resources)
{
}

BT::PortsList AnchorTrajectoryToCommand::providedPorts()
{
  return {
    BT::InputPort<trajectory_msgs::msg::JointTrajectory>(kPortIDJointTrajectory, "{joint_trajectory_msg}",
                                                         "Trajectory planned from the measured joint state."),
    BT::InputPort<moveit_msgs::msg::RobotState>("measured_state", "Current measured positions for every controlled "
                                                                  "joint, including held joints."),
    BT::InputPort<moveit_msgs::msg::RobotState>(kPortIDCommandedState, "{commanded_state}",
                                                "The controller's position command when the trajectory starts, such "
                                                "as the last point of the trajectory executed before it. Joints it "
                                                "names that the trajectory leaves out are held at these positions."),
    BT::InputPort<bool>(kPortIDFadeOut, false,
                        "If true, ramp the offsets to zero so the trajectory ends exactly at its planned last point."),
    BT::InputPort<std::vector<std::string>>(kPortIDMeasuredJoints,
                                            "Optional. Joints that start from measured_state even when "
                                            "commanded_state names them; plan them from that measurement."),
    BT::OutputPort<trajectory_msgs::msg::JointTrajectory>(kPortIDAnchoredTrajectory, "{anchored_trajectory}",
                                                          "The trajectory to execute."),
    BT::OutputPort<moveit_msgs::msg::RobotState>("end_state", "{end_state}",
                                                 "Exact last commanded point, including held joints."),
  };
}

BT::KeyValueVector AnchorTrajectoryToCommand::metadata()
{
  return { { moveit_pro::behaviors::kSubcategoryMetadataKey, "Motion - Planning" },
           { moveit_pro::behaviors::kDescriptionMetadataKey, kDescriptionAnchorTrajectoryToCommand } };
}

BT::NodeStatus AnchorTrajectoryToCommand::tick()
{
  const auto fail = [this](const std::string& message) {
    getBehaviorContext()->logger->publishFailureMessage(name(), message);
    return BT::NodeStatus::FAILURE;
  };

  const auto ports =
      moveit_pro::behaviors::getRequiredInputs(getInput<trajectory_msgs::msg::JointTrajectory>(kPortIDJointTrajectory),
                                               getInput<moveit_msgs::msg::RobotState>("measured_state"),
                                               getInput<bool>(kPortIDFadeOut));
  if (!ports.has_value())
  {
    return fail("Failed to get required values from input data ports: " + ports.error());
  }
  const auto& [planned, measured_state, fade_out] = ports.value();
  const auto command_input = getInput<moveit_msgs::msg::RobotState>(kPortIDCommandedState);
  const auto commanded_state = command_input ? command_input.value() : moveit_msgs::msg::RobotState{};

  const std::size_t joint_count = planned.joint_names.size();
  if (planned.points.empty() || std::ranges::any_of(planned.points, [joint_count](const auto& point) {
        return point.positions.size() != joint_count ||
               (!point.velocities.empty() && point.velocities.size() != joint_count) ||
               (!point.accelerations.empty() && point.accelerations.size() != joint_count);
      }))
  {
    return fail("The trajectory must have at least one point, and one position, and velocity and acceleration if "
                "any, per joint at every point.");
  }

  std::map<std::string, double> commanded = positionsByName(commanded_state.joint_state);
  const auto measured_joints =
      getInput<std::vector<std::string>>(kPortIDMeasuredJoints).value_or(std::vector<std::string>{});
  for (const auto& joint : measured_joints)
  {
    commanded.erase(joint);
  }
  const auto measured = positionsByName(measured_state.joint_state);
  // A missing endpoint after startup/recovery is not a zero command. Never trust an
  // endpoint farther than 0.10 from measurement.
  const auto safe_start = [&](const std::string& joint, const double candidate) {
    const double actual = measured.at(joint);
    if (!std::isfinite(candidate) || std::abs(candidate - actual) > 0.10)
    {
      getBehaviorContext()->logger->publishWarnMessage(
          name(),
          fmt::format("Replacing stale start for '{}' ({:.4f}) with measured {:.4f}.", joint, candidate, actual));
      return actual;
    }
    return candidate;
  };
  for (const auto& joint : planned.joint_names)
  {
    if (!measured.contains(joint) || !std::isfinite(measured.at(joint)))
    {
      return fail(fmt::format("No finite measured position for '{}'.", joint));
    }
  }
  for (const auto& [joint, position] : commanded)
  {
    if (!measured.contains(joint) || !std::isfinite(measured.at(joint)))
    {
      return fail(fmt::format("No finite measured position for held joint '{}'.", joint));
    }
  }
  std::vector<double> shift(joint_count, 0.0);
  for (std::size_t j = 0; j < joint_count; ++j)
  {
    const auto& joint = planned.joint_names[j];
    const double candidate = commanded.contains(joint) ? commanded.at(joint) : measured.at(joint);
    shift[j] = safe_start(joint, candidate) - planned.points.front().positions[j];
  }

  std::vector<std::string> held_joints;
  std::vector<double> held_positions;
  for (const auto& [joint, position] : measured)
  {
    if (!std::isfinite(position))
    {
      return fail(fmt::format("No finite measured position for '{}'.", joint));
    }
    if (std::ranges::find(planned.joint_names, joint) == planned.joint_names.end())
    {
      held_joints.push_back(joint);
      held_positions.push_back(safe_start(joint, commanded.contains(joint) ? commanded.at(joint) : position));
    }
  }

  trajectory_msgs::msg::JointTrajectory anchored = planned;
  anchored.joint_names.insert(anchored.joint_names.end(), held_joints.begin(), held_joints.end());
  const double duration = rclcpp::Duration(planned.points.back().time_from_start).seconds();
  // A trajectory without duration has nowhere to fade, so it keeps its full offset.
  const bool fading = fade_out && duration > 0.0;
  for (auto& point : anchored.points)
  {
    const Fade fade = fading ?
                          fadeAt(std::clamp(rclcpp::Duration(point.time_from_start).seconds() / duration, 0.0, 1.0)) :
                          Fade{ .weight = 1.0, .rate = 0.0, .curvature = 0.0 };
    for (std::size_t j = 0; j < joint_count; ++j)
    {
      point.positions[j] += fade.weight * shift[j];
      if (fading && !point.velocities.empty())
      {
        point.velocities[j] += fade.rate * shift[j] / duration;
      }
      if (fading && !point.accelerations.empty())
      {
        point.accelerations[j] += fade.curvature * shift[j] / (duration * duration);
      }
    }
    point.positions.insert(point.positions.end(), held_positions.begin(), held_positions.end());
    if (!point.velocities.empty())
    {
      point.velocities.resize(anchored.joint_names.size(), 0.0);
    }
    if (!point.accelerations.empty())
    {
      point.accelerations.resize(anchored.joint_names.size(), 0.0);
    }
  }

  moveit_msgs::msg::RobotState end_state = commanded_state;
  end_state.joint_state.name = anchored.joint_names;
  end_state.joint_state.position = anchored.points.back().positions;
  end_state.joint_state.velocity = anchored.points.back().velocities;
  setOutput("end_state", end_state);
  setOutput(kPortIDAnchoredTrajectory, anchored);
  return BT::NodeStatus::SUCCESS;
}
}  // namespace rebot_bench_behaviors
