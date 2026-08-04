// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include <vla_sim_behaviors/compute_top_down_keyposes.hpp>

#include <algorithm>
#include <cmath>

#include <fmt/format.h>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <moveit_pro_base/robot_state/robot_state.hpp>
#include <moveit_pro_behavior_interface/get_required_ports.hpp>
#include <moveit_pro_behavior_interface/metadata_fields.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

namespace
{
inline constexpr auto kDescriptionComputeTopDownKeyposes = R"(
                <p>
                    Builds the Cartesian path for one segment of the cube stacking oracle:
                    a top-down waypoint at each of <code>heights</code> above <code>aim_pose</code>,
                    all sharing one grasp orientation.
                </p>
                <p>
                    The orientation points the tip's approach axis straight down, spun to whichever
                    of the cube's four symmetry-equivalent yaws costs the least arm motion from
                    <code>seed_joint_state</code>, scored by inverse kinematics. Set
                    <code>held_object_offset</code> to the grasped object's transform in the tip
                    frame to aim the object at the waypoints instead of the tip, which is what
                    places a carried cube on a target cube.
                </p>
                <p>
                    Wire <code>reuse_orientation</code> from an earlier segment's
                    <code>orientation</code> once the gripper has closed, so the whole carry holds
                    the yaw the object was grasped at.
                </p>
                <p>
                    Feed the scene from <code>GetCurrentPlanningScene</code> into
                    <code>GetJointState</code>, whose <code>joint_state</code> output is this
                    Behavior's <code>seed_joint_state</code>. Send <code>path</code> on to
                    <code>PlanJointSplineThroughPoses</code>.
                </p>
            )";

constexpr auto kPortIDAimPose = "aim_pose";
constexpr auto kPortIDSeedJointState = "seed_joint_state";
constexpr auto kPortIDPlanningGroupName = "planning_group_name";
constexpr auto kPortIDTipLink = "tip_link";
constexpr auto kPortIDHeldObjectOffset = "held_object_offset";
constexpr auto kPortIDHeights = "heights";
constexpr auto kPortIDReuseOrientation = "reuse_orientation";
constexpr auto kPortIDPath = "path";
constexpr auto kPortIDOrientation = "orientation";
}  // namespace

namespace vla_sim_behaviors
{
Eigen::Quaterniond topDownGraspOrientation(double yaw)
{
  return Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                            Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()));
}

double yawOf(const Eigen::Quaterniond& orientation)
{
  const Eigen::Matrix3d rotation = orientation.normalized().toRotationMatrix();
  return std::atan2(rotation(1, 0), rotation(0, 0));
}

std::optional<double> chooseTopDownYawByCost(double cube_yaw, const std::function<std::optional<double>(double)>& cost)
{
  std::optional<double> best_yaw;
  std::optional<double> best_cost;
  for (int quarter_turns = 0; quarter_turns < 4; ++quarter_turns)
  {
    const double yaw = cube_yaw + quarter_turns * (M_PI / 2.0);
    const std::optional<double> candidate = cost(yaw);
    if (candidate.has_value() && (!best_cost.has_value() || candidate.value() < best_cost.value()))
    {
      best_cost = candidate;
      best_yaw = yaw;
    }
  }
  return best_yaw;
}

double jointDistanceCost(const std::vector<double>& from, const std::vector<double>& to)
{
  double cost = 0.0;
  for (std::size_t i = 0; i < std::min(from.size(), to.size()); ++i)
  {
    const double difference = to[i] - from[i];
    cost += difference * difference;
  }
  return cost;
}

std::optional<double>
reachableChainCost(const std::vector<double>& seed_positions, const std::vector<Eigen::Isometry3d>& keyposes,
                   const std::function<std::optional<std::vector<double>>(const Eigen::Isometry3d&)>& solve)
{
  std::optional<double> cost;
  for (const auto& keypose : keyposes)
  {
    const std::optional<std::vector<double>> solution = solve(keypose);
    if (!solution.has_value())
    {
      return std::nullopt;
    }
    if (!cost.has_value())
    {
      cost = jointDistanceCost(seed_positions, solution.value());
    }
  }
  return cost;
}

Eigen::Isometry3d topDownKeypose(const Eigen::Isometry3d& aim_pose, const Eigen::Quaterniond& orientation,
                                 const Eigen::Vector3d& held_object_offset, double height)
{
  // The offset is rigid in the tip frame, so rotating the wrist from the grasp orientation to
  // this one carries the object with it.
  const Eigen::Vector3d tip_to_object = orientation * held_object_offset;
  Eigen::Isometry3d keypose(orientation);
  keypose.translation() = aim_pose.translation() + Eigen::Vector3d(0.0, 0.0, height) - tip_to_object;
  return keypose;
}

std::vector<Eigen::Isometry3d> computeTopDownKeyposes(const Eigen::Isometry3d& aim_pose,
                                                      const Eigen::Quaterniond& orientation,
                                                      const Eigen::Vector3d& held_object_offset,
                                                      const std::vector<double>& heights)
{
  std::vector<Eigen::Isometry3d> keyposes;
  keyposes.reserve(heights.size());
  for (const double height : heights)
  {
    keyposes.push_back(topDownKeypose(aim_pose, orientation, held_object_offset, height));
  }
  return keyposes;
}

ComputeTopDownKeyposes::ComputeTopDownKeyposes(
    const std::string& name, const BT::NodeConfiguration& config,
    const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
  : SharedResourcesNode<BT::SyncActionNode>(name, config, shared_resources)
{
}

BT::PortsList ComputeTopDownKeyposes::providedPorts()
{
  return {
    BT::InputPort<geometry_msgs::msg::PoseStamped>(kPortIDAimPose, "{aim_pose}",
                                                   "Pose the waypoints are stacked above, and whose yaw the grasp "
                                                   "aligns to. The cube to grasp or to stack onto; for a straight "
                                                   "retract, the current tip pose."),
    BT::InputPort<sensor_msgs::msg::JointState>(kPortIDSeedJointState, "{seed_joint_state}",
                                                "Arm's current joint positions, from GetJointState. Both the IK "
                                                "seed and the pose the grasp candidates are scored against."),
    BT::InputPort<std::string>(kPortIDPlanningGroupName, "manipulator",
                               "SRDF joint group the grasp candidates are solved for."),
    BT::InputPort<std::string>(kPortIDTipLink, "grasp_link", "Link the waypoints position."),
    BT::InputPort<geometry_msgs::msg::TransformStamped>(kPortIDHeldObjectOffset, "",
                                                        "Grasped object's transform in the frame the waypoints "
                                                        "position: tip_link shifted by the planner's tip_offset. "
                                                        "Read it with GetLatestTransform using that frame as the "
                                                        "target. Leave unwired to position the frame itself."),
    BT::InputPort<std::vector<double>>(kPortIDHeights,
                                       "Heights above aim_pose, in meters, one waypoint each, semicolon separated."),
    BT::InputPort<geometry_msgs::msg::Quaternion>(kPortIDReuseOrientation, "",
                                                  "Grasp orientation to reuse instead of choosing one. Wire this "
                                                  "from an earlier segment's orientation for any move that carries "
                                                  "a grasped object."),
    BT::OutputPort<std::vector<geometry_msgs::msg::PoseStamped>>(kPortIDPath, "{path}",
                                                                 "Cartesian path for PlanJointSplineThroughPoses, in "
                                                                 "the frame of aim_pose."),
    BT::OutputPort<geometry_msgs::msg::Quaternion>(kPortIDOrientation, "{orientation}",
                                                   "The grasp orientation these waypoints share."),
  };
}

BT::KeyValueVector ComputeTopDownKeyposes::metadata()
{
  return { { moveit_pro::behaviors::kSubcategoryMetadataKey, "Cube Stacking" },
           { moveit_pro::behaviors::kDescriptionMetadataKey, kDescriptionComputeTopDownKeyposes } };
}

BT::NodeStatus ComputeTopDownKeyposes::tick()
{
  const auto ports = moveit_pro::behaviors::getRequiredInputs(
      getInput<geometry_msgs::msg::PoseStamped>(kPortIDAimPose),
      getInput<sensor_msgs::msg::JointState>(kPortIDSeedJointState), getInput<std::vector<double>>(kPortIDHeights),
      getInput<std::string>(kPortIDPlanningGroupName), getInput<std::string>(kPortIDTipLink));
  if (!ports.has_value())
  {
    getBehaviorContext()->logger->publishFailureMessage(
        name(), "Failed to get required values from input data ports: " + ports.error());
    return BT::NodeStatus::FAILURE;
  }
  const auto& [aim_pose_msg, seed_joint_state, heights, planning_group_name, tip_link] = ports.value();

  if (heights.empty())
  {
    getBehaviorContext()->logger->publishFailureMessage(name(), "heights is empty, so the path would have no "
                                                                "waypoints.");
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

  moveit_pro::base::RobotState seed_state(robot_model);
  seed_state.setToDefaultValues();
  for (std::size_t i = 0; i < seed_joint_state.name.size() && i < seed_joint_state.position.size(); ++i)
  {
    if (robot_model->hasJointModel(seed_joint_state.name[i]))
    {
      seed_state.setJointPositions(seed_joint_state.name[i], { seed_joint_state.position[i] });
    }
  }
  seed_state.update();
  std::vector<double> seed_positions;
  seed_state.copyJointGroupPositions(joint_group, seed_positions);

  Eigen::Vector3d held_object_offset = Eigen::Vector3d::Zero();
  if (const auto offset = getInput<geometry_msgs::msg::TransformStamped>(kPortIDHeldObjectOffset); offset.has_value())
  {
    held_object_offset = tf2::transformToEigen(offset.value()).translation();
  }

  Eigen::Isometry3d aim_pose;
  tf2::fromMsg(aim_pose_msg.pose, aim_pose);

  // Named locals, since a lambda cannot capture a structured binding.
  const std::string ik_tip_link = tip_link;
  const std::vector<double>& keypose_heights = heights;
  const auto cost_of = [&](double yaw) -> std::optional<double> {
    const Eigen::Quaterniond orientation = topDownGraspOrientation(yaw);
    // One state carried down the chain, so each solve warm-starts from the last and the whole
    // segment stays on one IK branch, as PlanJointSplineThroughPoses will solve it.
    moveit_pro::base::RobotState candidate(seed_state);
    const auto solve = [&](const Eigen::Isometry3d& keypose) -> std::optional<std::vector<double>> {
      if (!candidate.setFromIK(joint_group, keypose, ik_tip_link))
      {
        return std::nullopt;
      }
      std::vector<double> solution;
      candidate.copyJointGroupPositions(joint_group, solution);
      return solution;
    };
    return reachableChainCost(
        seed_positions, computeTopDownKeyposes(aim_pose, orientation, held_object_offset, keypose_heights), solve);
  };

  Eigen::Quaterniond orientation;
  if (const auto reused = getInput<geometry_msgs::msg::Quaternion>(kPortIDReuseOrientation); reused.has_value())
  {
    tf2::fromMsg(reused.value(), orientation);
  }
  else
  {
    const double cube_yaw = yawOf(Eigen::Quaterniond(aim_pose.rotation()));
    const std::optional<double> yaw = chooseTopDownYawByCost(cube_yaw, cost_of);
    if (!yaw.has_value())
    {
      getBehaviorContext()->logger->publishFailureMessage(
          name(), "No top-down grasp of this pose is reachable: every one of the four symmetry-equivalent "
                  "yaws has a waypoint inverse kinematics cannot solve.");
      return BT::NodeStatus::FAILURE;
    }
    orientation = topDownGraspOrientation(yaw.value());
  }

  const std::vector<Eigen::Isometry3d> keyposes =
      computeTopDownKeyposes(aim_pose, orientation, held_object_offset, heights);

  std::vector<geometry_msgs::msg::PoseStamped> path;
  path.reserve(keyposes.size());
  for (const auto& keypose : keyposes)
  {
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header = aim_pose_msg.header;
    pose_msg.pose = tf2::toMsg(keypose);
    path.push_back(pose_msg);
  }
  setOutput(kPortIDPath, path);
  setOutput(kPortIDOrientation, tf2::toMsg(orientation));

  return BT::NodeStatus::SUCCESS;
}
}  // namespace vla_sim_behaviors
