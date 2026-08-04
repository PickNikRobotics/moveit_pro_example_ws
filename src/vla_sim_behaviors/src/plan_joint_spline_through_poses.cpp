// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include <vla_sim_behaviors/plan_joint_spline_through_poses.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include <fmt/format.h>
#include <moveit_pro_base/robot_model/joint_model_group.hpp>
#include <moveit_pro_base/robot_state/robot_state.hpp>
#include <moveit_pro_behavior_interface/get_required_ports.hpp>
#include <moveit_pro_behavior_interface/metadata_fields.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

namespace
{
inline constexpr auto kDescriptionPlanJointSplineThroughPoses = R"(
                <p>
                    Plans a Cartesian <code>path</code> as a single joint-space trajectory:
                    each waypoint is solved for inverse kinematics once, warm-started from the
                    previous solution, and the resulting joint knots are joined by a clamped
                    cubic spline. The arm eases from rest, passes through every waypoint exactly
                    without stopping, and arrives at the last one at rest.
                </p>
                <p>
                    Interpolating in joint space between same-branch solutions holds a redundant
                    arm's posture steady across the whole segment, which is what a demonstration
                    needs.
                </p>
                <p>
                    The duration is the slower of covering the path at
                    <code>cartesian_speed</code> and holding every joint under its model velocity
                    limit scaled by <code>joint_velocity_scale</code>, so the tip holds a roughly
                    constant speed instead of a constant joint rate.
                </p>
                <p>
                    Feed <code>path</code> from <code>ComputeTopDownKeyposes</code>,
                    <code>seed_joint_state</code> from <code>GetJointState</code>, and the
                    output to <code>ExecuteTrajectory</code>.
                </p>
            )";

constexpr auto kPortIDPath = "path";
constexpr auto kPortIDSeedJointState = "seed_joint_state";
constexpr auto kPortIDPlanningGroupName = "planning_group_name";
constexpr auto kPortIDTipLink = "tip_link";
constexpr auto kPortIDTipOffset = "tip_offset";
constexpr auto kPortIDCartesianSpeed = "cartesian_speed";
constexpr auto kPortIDJointVelocityScale = "joint_velocity_scale";
constexpr auto kPortIDSamplingRate = "sampling_rate";
constexpr auto kPortIDJointTrajectory = "joint_trajectory_msg";

// Peak-to-mean speed ratio of a smootherstep profile. It sizes each segment's nominal duration,
// which in turn sets how much of the spline parameter that segment gets; the spline's true peak
// is measured afterwards.
constexpr double kNominalPeakSpeedRatio = 1.875;

// A trajectory long enough to be worth executing, and a ceiling above which the segment fails
// rather than leaving the arm to creep for minutes.
constexpr double kMinimumDuration = 0.2;
constexpr double kMaximumDuration = 60.0;

// Enough to resolve the fastest segment the speed budgets allow, and a ceiling that keeps a
// mistyped rate from reserving a trajectory too large to allocate.
constexpr unsigned int kMaximumSamplingRate = 1000;
}  // namespace

namespace vla_sim_behaviors
{
JointSpline::JointSpline(const std::vector<double>& parameters, const std::vector<Eigen::VectorXd>& knots)
  : parameters_(parameters), knots_(knots)
{
  const std::size_t count = knots.size();
  if (count < 2)
  {
    throw std::invalid_argument(fmt::format("A spline needs at least 2 knots, got {}.", count));
  }
  if (parameters.size() != count)
  {
    throw std::invalid_argument(
        fmt::format("Got {} knots but {} parameters; they must correspond.", count, parameters.size()));
  }
  const Eigen::Index width = knots.front().size();
  for (std::size_t i = 0; i < count; ++i)
  {
    if (knots[i].size() != width)
    {
      throw std::invalid_argument(
          fmt::format("Knot {} has width {}, but knot 0 has width {}.", i, knots[i].size(), width));
    }
    if (i > 0 && !(parameters[i] > parameters[i - 1]))
    {
      throw std::invalid_argument(fmt::format("Parameters must strictly increase, but parameter {} is {} and {} is {}.",
                                              i - 1, parameters[i - 1], i, parameters[i]));
    }
  }

  // Second derivatives at the knots, from the standard tridiagonal moment formulation with
  // both ends clamped to zero first derivative.
  Eigen::MatrixXd system = Eigen::MatrixXd::Zero(count, count);
  Eigen::MatrixXd right_hand_side = Eigen::MatrixXd::Zero(count, width);
  const auto span = [&](std::size_t i) { return parameters_[i + 1] - parameters_[i]; };
  const auto slope = [&](std::size_t i) -> Eigen::RowVectorXd {
    return (knots_[i + 1] - knots_[i]).transpose() / span(i);
  };

  for (std::size_t i = 1; i + 1 < count; ++i)
  {
    system(i, i - 1) = span(i - 1);
    system(i, i) = 2.0 * (span(i - 1) + span(i));
    system(i, i + 1) = span(i);
    right_hand_side.row(i) = 6.0 * (slope(i) - slope(i - 1));
  }
  system(0, 0) = 2.0 * span(0);
  system(0, 1) = span(0);
  right_hand_side.row(0) = 6.0 * slope(0);
  system(count - 1, count - 1) = 2.0 * span(count - 2);
  system(count - 1, count - 2) = span(count - 2);
  right_hand_side.row(count - 1) = -6.0 * slope(count - 2);

  const Eigen::MatrixXd moments = system.colPivHouseholderQr().solve(right_hand_side);
  moments_.reserve(count);
  for (std::size_t i = 0; i < count; ++i)
  {
    moments_.push_back(moments.row(i).transpose());
  }
}

JointSpline::Segment JointSpline::locate(double s) const
{
  // Saturating rather than extrapolating: a cubic continued past its last knot diverges fast.
  s = std::clamp(s, parameters_.front(), parameters_.back());
  const auto upper = std::upper_bound(parameters_.begin(), parameters_.end(), s);
  const auto found = static_cast<std::size_t>(std::distance(parameters_.begin(), upper));
  const std::size_t index = std::clamp<std::size_t>(found == 0 ? 0 : found - 1, 0, parameters_.size() - 2);
  const double width = parameters_[index + 1] - parameters_[index];
  return { index, width, (parameters_[index + 1] - s) / width, (s - parameters_[index]) / width };
}

Eigen::VectorXd JointSpline::position(double s) const
{
  const auto [i, h, a, b] = locate(s);
  return a * knots_[i] + b * knots_[i + 1] +
         ((a * a * a - a) * moments_[i] + (b * b * b - b) * moments_[i + 1]) * (h * h / 6.0);
}

Eigen::VectorXd JointSpline::velocity(double s) const
{
  const auto [i, h, a, b] = locate(s);
  return (knots_[i + 1] - knots_[i]) / h +
         ((1.0 - 3.0 * a * a) * moments_[i] + (3.0 * b * b - 1.0) * moments_[i + 1]) * (h / 6.0);
}

Eigen::VectorXd JointSpline::acceleration(double s) const
{
  const auto [i, h, a, b] = locate(s);
  return a * moments_[i] + b * moments_[i + 1];
}

Eigen::VectorXd JointSpline::peakSpeed() const
{
  Eigen::VectorXd peak = Eigen::VectorXd::Zero(knots_.front().size());
  for (std::size_t i = 0; i + 1 < parameters_.size(); ++i)
  {
    // Speed is quadratic within a segment, so its extremes are at the ends or where the
    // acceleration crosses zero.
    std::vector<double> candidates = { parameters_[i], parameters_[i + 1] };
    for (Eigen::Index j = 0; j < peak.size(); ++j)
    {
      const double denominator = moments_[i + 1][j] - moments_[i][j];
      if (std::abs(denominator) > std::numeric_limits<double>::epsilon())
      {
        const double a = moments_[i + 1][j] / denominator;
        if (a >= 0.0 && a <= 1.0)
        {
          candidates.push_back(parameters_[i + 1] - a * (parameters_[i + 1] - parameters_[i]));
        }
      }
    }
    for (const double s : candidates)
    {
      peak = peak.cwiseMax(velocity(s).cwiseAbs());
    }
  }
  return peak;
}

double segmentDuration(double cartesian_length, const Eigen::VectorXd& joint_delta, double cartesian_speed,
                       const Eigen::VectorXd& joint_velocity_cap)
{
  const double cartesian = cartesian_speed > 0.0 ? cartesian_length / cartesian_speed : 0.0;
  double joint = 0.0;
  for (Eigen::Index j = 0; j < joint_delta.size() && j < joint_velocity_cap.size(); ++j)
  {
    if (joint_velocity_cap[j] > 0.0)
    {
      joint = std::max(joint, kNominalPeakSpeedRatio * std::abs(joint_delta[j]) / joint_velocity_cap[j]);
    }
  }
  return std::max(cartesian, joint);
}

std::vector<double> splineKnotParameters(const std::vector<double>& segment_durations)
{
  std::vector<double> parameters{ 0.0 };
  parameters.reserve(segment_durations.size() + 1);
  for (const double duration : segment_durations)
  {
    // Every segment gets a share, keeping the knot parameters strictly increasing. A fixed
    // floor cannot do that: any increment below half an ulp of the running total rounds
    // straight back off it, and one ulp grows with the total.
    const double next = parameters.back() + std::max(duration, 0.0);
    parameters.push_back(
        next > parameters.back() ? next : std::nextafter(parameters.back(), std::numeric_limits<double>::infinity()));
  }
  // Scaled, then made strictly increasing again: JointSpline validates these values, and
  // dividing by the total can round a one-ulp gap back shut when the quotients land just
  // above a power of two, where the spacing is coarser than it was before the division.
  const double total = parameters.back();
  std::vector<double> normalized{ 0.0 };
  normalized.reserve(parameters.size());
  for (std::size_t i = 1; i < parameters.size(); ++i)
  {
    const double scaled = parameters[i] / total;
    normalized.push_back(scaled > normalized.back() ?
                             scaled :
                             std::nextafter(normalized.back(), std::numeric_limits<double>::infinity()));
  }
  return normalized;
}

double splineDuration(const JointSpline& spline, double cartesian_length, double cartesian_speed,
                      const Eigen::VectorXd& joint_velocity_cap)
{
  const double cartesian = cartesian_speed > 0.0 ? cartesian_length / cartesian_speed : 0.0;
  const Eigen::VectorXd peak = spline.peakSpeed();
  double joint = 0.0;
  for (Eigen::Index j = 0; j < peak.size() && j < joint_velocity_cap.size(); ++j)
  {
    if (joint_velocity_cap[j] > 0.0)
    {
      joint = std::max(joint, peak[j] / joint_velocity_cap[j]);
    }
  }
  // Floored, not capped: a duration over the ceiling means the speed budgets cannot be met, and
  // the caller rejects it rather than running the trajectory over the joint limits.
  return std::max({ cartesian, joint, kMinimumDuration });
}

PlanJointSplineThroughPoses::PlanJointSplineThroughPoses(
    const std::string& name, const BT::NodeConfiguration& config,
    const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
  : SharedResourcesNode<BT::SyncActionNode>(name, config, shared_resources)
{
}

BT::PortsList PlanJointSplineThroughPoses::providedPorts()
{
  return {
    BT::InputPort<std::vector<geometry_msgs::msg::PoseStamped>>(kPortIDPath, "{path}",
                                                                "Waypoints to pass through, in order, in the robot "
                                                                "model frame. They are solved for inverse kinematics "
                                                                "as given, so a producer working in another frame has "
                                                                "to transform them first."),
    BT::InputPort<sensor_msgs::msg::JointState>(kPortIDSeedJointState, "{seed_joint_state}",
                                                "Joint positions the trajectory starts from, from "
                                                "GetJointState. Also seeds the first inverse kinematics "
                                                "solve, which fixes the branch the whole path stays on."),
    BT::InputPort<std::string>(kPortIDPlanningGroupName, "manipulator", "SRDF joint group to plan for."),
    BT::InputPort<std::string>(kPortIDTipLink, "grasp_link", "Link the waypoints are solved for."),
    BT::InputPort<std::vector<double>>(kPortIDTipOffset, "0;0;0",
                                       "Translation from tip_link to the frame the waypoints actually position, "
                                       "in the tip_link frame, semicolon separated."),
    BT::InputPort<double>(kPortIDCartesianSpeed, "0.065", "Tip speed budget along the path, in meters per second."),
    BT::InputPort<double>(kPortIDJointVelocityScale, "0.5",
                          "Fraction of each joint's model velocity limit the trajectory may reach. Binds only "
                          "where the path is short enough that cartesian_speed would exceed it."),
    BT::InputPort<unsigned int>(kPortIDSamplingRate, "100", "Output trajectory sampling rate, in Hz."),
    BT::OutputPort<trajectory_msgs::msg::JointTrajectory>(kPortIDJointTrajectory, "{joint_trajectory_msg}",
                                                          "Timed trajectory for ExecuteTrajectory."),
  };
}

BT::KeyValueVector PlanJointSplineThroughPoses::metadata()
{
  return { { moveit_pro::behaviors::kSubcategoryMetadataKey, "Cube Stacking" },
           { moveit_pro::behaviors::kDescriptionMetadataKey, kDescriptionPlanJointSplineThroughPoses } };
}

BT::NodeStatus PlanJointSplineThroughPoses::tick()
{
  const auto ports = moveit_pro::behaviors::getRequiredInputs(
      getInput<std::vector<geometry_msgs::msg::PoseStamped>>(kPortIDPath),
      getInput<sensor_msgs::msg::JointState>(kPortIDSeedJointState), getInput<std::string>(kPortIDPlanningGroupName),
      getInput<std::string>(kPortIDTipLink), getInput<std::vector<double>>(kPortIDTipOffset),
      getInput<double>(kPortIDCartesianSpeed), getInput<double>(kPortIDJointVelocityScale),
      getInput<unsigned int>(kPortIDSamplingRate));
  if (!ports.has_value())
  {
    getBehaviorContext()->logger->publishFailureMessage(
        name(), "Failed to get required values from input data ports: " + ports.error());
    return BT::NodeStatus::FAILURE;
  }
  const auto& [path, seed_joint_state, planning_group_name, tip_link, tip_offset, cartesian_speed, joint_velocity_scale,
               sampling_rate] = ports.value();

  if (path.empty())
  {
    getBehaviorContext()->logger->publishFailureMessage(name(), "path is empty, so there is nothing to plan.");
    return BT::NodeStatus::FAILURE;
  }
  if (tip_offset.size() != 3)
  {
    getBehaviorContext()->logger->publishFailureMessage(name(), fmt::format("tip_offset needs 3 values, got {}.",
                                                                            tip_offset.size()));
    return BT::NodeStatus::FAILURE;
  }
  if (sampling_rate == 0 || sampling_rate > kMaximumSamplingRate)
  {
    getBehaviorContext()->logger->publishFailureMessage(
        name(), fmt::format("sampling_rate must be between 1 and {} Hz, got {}.", kMaximumSamplingRate, sampling_rate));
    return BT::NodeStatus::FAILURE;
  }
  if (!(joint_velocity_scale > 0.0 && joint_velocity_scale <= 1.0))
  {
    getBehaviorContext()->logger->publishFailureMessage(
        name(), fmt::format("joint_velocity_scale must be in (0, 1], got {}. A scale of 0 or NaN drops the joint "
                            "speed cap silently, and above 1 asks for more than the model allows.",
                            joint_velocity_scale));
    return BT::NodeStatus::FAILURE;
  }

  const auto& robot_model = getBehaviorContext()->robot_model;
  const auto* joint_group = robot_model ? robot_model->getJointModelGroup(planning_group_name) : nullptr;
  if (joint_group == nullptr)
  {
    getBehaviorContext()->logger->publishFailureMessage(
        name(), fmt::format("No planning group '{}' in the robot model.", planning_group_name));
    return BT::NodeStatus::FAILURE;
  }

  moveit_pro::base::RobotState state(robot_model);
  state.setToDefaultValues();
  for (std::size_t i = 0; i < seed_joint_state.name.size() && i < seed_joint_state.position.size(); ++i)
  {
    if (robot_model->hasJointModel(seed_joint_state.name[i]))
    {
      state.setJointPositions(seed_joint_state.name[i], { seed_joint_state.position[i] });
    }
  }
  state.update();

  // The waypoints position a frame offset from tip_link, so the link itself has to arrive
  // the same offset short of each one.
  const Eigen::Translation3d offset(-tip_offset[0], -tip_offset[1], -tip_offset[2]);

  std::vector<Eigen::VectorXd> knots;
  std::vector<Eigen::Vector3d> tip_positions;
  {
    std::vector<double> positions;
    state.copyJointGroupPositions(joint_group, positions);
    knots.push_back(Eigen::Map<Eigen::VectorXd>(positions.data(), static_cast<Eigen::Index>(positions.size())));
    tip_positions.push_back((state.getGlobalLinkTransform(tip_link) * offset.inverse()).translation());
  }

  for (std::size_t i = 0; i < path.size(); ++i)
  {
    Eigen::Isometry3d waypoint;
    tf2::fromMsg(path[i].pose, waypoint);
    if (!state.setFromIK(joint_group, waypoint * offset, tip_link))
    {
      getBehaviorContext()->logger->publishFailureMessage(
          name(), fmt::format("Inverse kinematics failed for waypoint {} of {}.", i + 1, path.size()));
      return BT::NodeStatus::FAILURE;
    }
    std::vector<double> positions;
    state.copyJointGroupPositions(joint_group, positions);
    knots.push_back(Eigen::Map<Eigen::VectorXd>(positions.data(), static_cast<Eigen::Index>(positions.size())));
    tip_positions.push_back(waypoint.translation());
  }

  Eigen::VectorXd velocity_cap(knots.front().size());
  const auto& bounds = joint_group->getActiveJointModelsBounds();
  for (Eigen::Index j = 0; j < velocity_cap.size(); ++j)
  {
    const auto index = static_cast<std::size_t>(j);
    const bool bounded = index < bounds.size() && !bounds[index]->empty() && bounds[index]->front().velocity_bounded;
    velocity_cap[j] =
        bounded ? bounds[index]->front().max_velocity * joint_velocity_scale : std::numeric_limits<double>::infinity();
  }

  std::vector<double> segment_durations;
  double cartesian_length = 0.0;
  segment_durations.reserve(knots.size() - 1);
  for (std::size_t i = 0; i + 1 < knots.size(); ++i)
  {
    const double length = (tip_positions[i + 1] - tip_positions[i]).norm();
    cartesian_length += length;
    segment_durations.push_back(segmentDuration(length, knots[i + 1] - knots[i], cartesian_speed, velocity_cap));
  }

  std::unique_ptr<JointSpline> spline;
  try
  {
    spline = std::make_unique<JointSpline>(splineKnotParameters(segment_durations), knots);
  }
  catch (const std::invalid_argument& exception)
  {
    getBehaviorContext()->logger->publishFailureMessage(name(), fmt::format("Could not fit a spline through the "
                                                                            "inverse kinematics solutions: {}",
                                                                            exception.what()));
    return BT::NodeStatus::FAILURE;
  }

  const double duration = splineDuration(*spline, cartesian_length, cartesian_speed, velocity_cap);
  if (duration > kMaximumDuration)
  {
    getBehaviorContext()->logger->publishFailureMessage(
        name(), fmt::format("This path needs {:.1f}s to stay under the speed budgets, over the {:g}s ceiling. "
                            "Shorten the segment, or raise cartesian_speed or joint_velocity_scale.",
                            duration, kMaximumDuration));
    return BT::NodeStatus::FAILURE;
  }
  const auto steps = static_cast<std::size_t>(std::ceil(duration * sampling_rate));

  trajectory_msgs::msg::JointTrajectory trajectory;
  trajectory.header = path.front().header;
  trajectory.joint_names = joint_group->getVariableNames();
  trajectory.points.reserve(steps + 1);
  for (std::size_t step = 0; step <= steps; ++step)
  {
    const double s = static_cast<double>(step) / static_cast<double>(steps);
    const Eigen::VectorXd position = spline->position(s);
    const Eigen::VectorXd velocity = spline->velocity(s) / duration;
    const Eigen::VectorXd acceleration = spline->acceleration(s) / (duration * duration);

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions.assign(position.data(), position.data() + position.size());
    point.velocities.assign(velocity.data(), velocity.data() + velocity.size());
    point.accelerations.assign(acceleration.data(), acceleration.data() + acceleration.size());
    point.time_from_start = rclcpp::Duration::from_seconds(s * duration);
    trajectory.points.push_back(std::move(point));
  }
  setOutput(kPortIDJointTrajectory, trajectory);

  return BT::NodeStatus::SUCCESS;
}
}  // namespace vla_sim_behaviors
