// Copyright 2026 PickNik Inc.
// SPDX-License-Identifier: BSD-3-Clause

#include <factory_sim/is_bracket_pick_start_current.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <map>
#include <set>
#include <string>

#include <fmt/format.h>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_pro_behavior_interface/get_required_ports.hpp>
#include <moveit_task_constructor_msgs/msg/solution.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tl/expected.hpp>

namespace
{
constexpr std::array kArmJoints{ "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6" };

[[nodiscard]] tl::expected<void, std::string> checkStart(const moveit_task_constructor_msgs::msg::Solution& solution,
                                                         const moveit_msgs::msg::PlanningScene& scene, double tolerance)
{
  if (!std::isfinite(tolerance) || tolerance < 0.0)
    return tl::make_unexpected("Bracket pick position_tolerance must be finite and nonnegative, in radians.");
  if (scene.is_diff || scene.robot_state.is_diff)
    return tl::make_unexpected("Bracket pick handoff requires a full planning scene. Fetch the current scene.");
  const trajectory_msgs::msg::JointTrajectory* trajectory = nullptr;
  for (const auto& segment : solution.sub_trajectory)
  {
    if (!segment.trajectory.multi_dof_joint_trajectory.points.empty())
      return tl::make_unexpected("Bracket pick handoff does not support multi-DOF trajectories.");
    if (!trajectory && !segment.trajectory.joint_trajectory.points.empty())
      trajectory = &segment.trajectory.joint_trajectory;
  }
  if (!trajectory || trajectory->joint_names.size() != kArmJoints.size())
    return tl::make_unexpected("Bracket pick solution must contain a trajectory naming all six Fanuc arm joints.");
  const std::set<std::string> names(trajectory->joint_names.begin(), trajectory->joint_names.end());
  for (const auto* name : kArmJoints)
    if (!names.contains(name))
      return tl::make_unexpected(
          fmt::format("Bracket pick solution does not name arm joint '{}'. Replan the pick.", name));
  const auto& positions = trajectory->points.front().positions;
  if (positions.size() != trajectory->joint_names.size())
    return tl::make_unexpected("Bracket pick start positions must match its joint names. Replan the pick.");
  const auto& actual = scene.robot_state.joint_state;
  if (actual.name.size() != actual.position.size())
    return tl::make_unexpected("Bracket pick state joint names and positions differ in size. Fetch the current scene.");
  std::map<std::string, double> by_name;
  for (std::size_t i = 0; i < actual.name.size(); ++i)
    if (!std::isfinite(actual.position[i]) || !by_name.emplace(actual.name[i], actual.position[i]).second)
      return tl::make_unexpected(
          "Bracket pick state contains duplicate joints or nonfinite positions. Fetch the current scene.");
  for (std::size_t i = 0; i < positions.size(); ++i)
  {
    const auto current = by_name.find(trajectory->joint_names[i]);
    if (current == by_name.end() || !std::isfinite(positions[i]))
      return tl::make_unexpected(
          fmt::format("Bracket pick joint '{}' is missing from the state or has an invalid planned position.",
                      trajectory->joint_names[i]));
    const double error = std::abs(current->second - positions[i]);
    if (error > tolerance)
      return tl::make_unexpected(fmt::format("Bracket pick joint '{}' start differs by {:.6f} rad; tolerance is {:.6f} "
                                             "rad. Replan from the current state.",
                                             trajectory->joint_names[i], error, tolerance));
  }
  return {};
}
}  // namespace

namespace factory_sim
{
BT::PortsList IsBracketPickStartCurrent::providedPorts()
{
  return { BT::InputPort<moveit_task_constructor_msgs::msg::Solution>("solution", "Prepared bracket pick solution."),
           BT::InputPort<moveit_msgs::msg::PlanningScene>("planning_scene", "Fresh full scene at the stopped handoff."),
           BT::InputPort<sensor_msgs::msg::JointState>("joint_state", "Fresh measured positions and velocities."),
           BT::InputPort<double>("current_time", "ROS-clock time when the feedback was received, in seconds."),
           BT::InputPort<double>("maximum_state_age_s", 0.2, "Maximum feedback age, in seconds."),
           BT::InputPort<double>("velocity_tolerance", 0.02, "Maximum absolute arm velocity, in radians per second."),
           BT::InputPort<double>("position_tolerance", 0.02,
                                 "Maximum start-position error for each Fanuc arm joint, in radians."),
           BT::OutputPort<std::string>("reason", "Reason for rejecting the candidate; empty on success.") };
}

BT::KeyValueVector IsBracketPickStartCurrent::metadata()
{
  return { { "subcategory", "Motion - Planning" },
           { "description",
             "Checks fresh stopped Fanuc feedback and current-scene positions against a prepared pick start." } };
}

BT::NodeStatus IsBracketPickStartCurrent::tick()
{
  const auto inputs = moveit_pro::behaviors::getRequiredInputs(
      getInput<moveit_task_constructor_msgs::msg::Solution>("solution"),
      getInput<moveit_msgs::msg::PlanningScene>("planning_scene"), getInput<double>("position_tolerance"),
      getInput<sensor_msgs::msg::JointState>("joint_state"), getInput<double>("current_time"),
      getInput<double>("maximum_state_age_s"), getInput<double>("velocity_tolerance"));
  std::string reason;
  if (!inputs)
    reason = "Bracket pick handoff input ports: " + inputs.error();
  else
  {
    const auto& [solution, scene, tolerance, feedback, now, maximum_age, velocity_tolerance] = inputs.value();
    if (const auto result = checkStart(solution, scene, tolerance); !result)
      reason = result.error();
    else if (!std::isfinite(now) || !std::isfinite(maximum_age) || maximum_age <= 0.0 ||
             !std::isfinite(velocity_tolerance) || velocity_tolerance < 0.0)
      reason = "Bracket pick feedback time and tolerances must be finite and in range.";
    else
    {
      const double stamp = feedback.header.stamp.sec + feedback.header.stamp.nanosec * 1e-9;
      if (feedback.header.stamp.nanosec >= 1000000000u || stamp <= 0.0 || now < stamp || now - stamp > maximum_age)
        reason = "Bracket pick feedback is unstamped, stale, or ahead of the ROS clock.";
      else if (feedback.name.size() != feedback.velocity.size())
        reason = "Bracket pick feedback must contain a velocity for each named joint.";
      else
      {
        moveit_msgs::msg::PlanningScene measured;
        measured.robot_state.joint_state = feedback;
        if (const auto result = checkStart(solution, measured, tolerance); !result)
          reason = result.error();
        else
          for (std::size_t i = 0; i < feedback.name.size(); ++i)
            if (std::find(kArmJoints.begin(), kArmJoints.end(), feedback.name[i]) != kArmJoints.end() &&
                (!std::isfinite(feedback.velocity[i]) || std::abs(feedback.velocity[i]) > velocity_tolerance))
            {
              reason = fmt::format("Bracket pick joint '{}' has not settled below {:.6f} rad/s.", feedback.name[i],
                                   velocity_tolerance);
              break;
            }
      }
    }
  }
  if (!setOutput("reason", reason))
    throw BT::RuntimeError("Bracket pick handoff cannot write the reason output port.");
  return reason.empty() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}
}  // namespace factory_sim
