// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include <kinova_vla_test_sim_behaviors/compute_top_down_keyposes.hpp>

#include <cmath>

#include <fmt/format.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <moveit_pro_behavior_interface/get_required_ports.hpp>
#include <moveit_pro_behavior_interface/metadata_fields.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

namespace
{
inline constexpr auto kDescriptionComputeTopDownKeyposes = R"(
                <p>
                    Builds the Cartesian path for one segment of the color-cube stacking oracle:
                    a top-down waypoint at each of <code>heights</code> above <code>aim_pose</code>,
                    all sharing one grasp orientation.
                </p>
                <p>
                    The orientation points the tip's approach axis straight down, spun to whichever
                    of the cube's four symmetry-equivalent yaws is nearest the yaw of
                    <code>reference_pose</code> (the current tip pose). Set
                    <code>held_object_offset</code> to the grasped object's transform in the tip
                    frame to aim the object at the waypoints instead of the tip, which is what
                    places a carried cube on a target cube.
                </p>
                <p>
                    Feed the output <code>path</code> to <code>PlanCartesianPath</code>.
                </p>
            )";

constexpr auto kPortIDAimPose = "aim_pose";
constexpr auto kPortIDReferencePose = "reference_pose";
constexpr auto kPortIDHeldObjectOffset = "held_object_offset";
constexpr auto kPortIDHeights = "heights";
constexpr auto kPortIDPath = "path";
}  // namespace

namespace kinova_vla_test_sim_behaviors
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

double chooseTopDownYaw(double cube_yaw, double reference_yaw)
{
  const double quarter_turn = M_PI / 2.0;
  const double turns = std::round((reference_yaw - cube_yaw) / quarter_turn);
  return cube_yaw + turns * quarter_turn;
}

std::vector<Eigen::Isometry3d> computeTopDownKeyposes(const Eigen::Isometry3d& aim_pose, double reference_yaw,
                                                      const Eigen::Vector3d& held_object_offset,
                                                      const std::vector<double>& heights)
{
  const double yaw = chooseTopDownYaw(yawOf(Eigen::Quaterniond(aim_pose.rotation())), reference_yaw);
  const Eigen::Quaterniond orientation = topDownGraspOrientation(yaw);
  // The offset is rigid in the tip frame, so rotating the wrist from the grasp orientation to
  // this one carries the object with it.
  const Eigen::Vector3d tip_to_object = orientation * held_object_offset;

  std::vector<Eigen::Isometry3d> keyposes;
  keyposes.reserve(heights.size());
  for (const double height : heights)
  {
    Eigen::Isometry3d keypose(orientation);
    keypose.translation() = aim_pose.translation() + Eigen::Vector3d(0.0, 0.0, height) - tip_to_object;
    keyposes.push_back(keypose);
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
    BT::InputPort<geometry_msgs::msg::PoseStamped>(kPortIDReferencePose, "{reference_pose}",
                                                   "Current tip pose. Only its yaw is read, to pick the nearest of "
                                                   "the cube's four equivalent grasp yaws, so it must be in the same "
                                                   "frame as aim_pose."),
    BT::InputPort<geometry_msgs::msg::TransformStamped>(kPortIDHeldObjectOffset, "",
                                                        "Grasped object's transform in the tip frame, as "
                                                        "GetLatestTransform reports it with the tip as target frame. "
                                                        "Leave unwired to position the tip itself."),
    BT::InputPort<std::vector<double>>(kPortIDHeights,
                                       "Heights above aim_pose, in meters, one waypoint each, semicolon separated."),
    BT::OutputPort<std::vector<geometry_msgs::msg::PoseStamped>>(kPortIDPath, "{path}",
                                                                 "Cartesian path for PlanCartesianPath, in the frame "
                                                                 "of aim_pose."),
  };
}

BT::KeyValueVector ComputeTopDownKeyposes::metadata()
{
  return { { moveit_pro::behaviors::kSubcategoryMetadataKey, "Color-Cube Stacking" },
           { moveit_pro::behaviors::kDescriptionMetadataKey, kDescriptionComputeTopDownKeyposes } };
}

BT::NodeStatus ComputeTopDownKeyposes::tick()
{
  const auto ports =
      moveit_pro::behaviors::getRequiredInputs(getInput<geometry_msgs::msg::PoseStamped>(kPortIDAimPose),
                                               getInput<geometry_msgs::msg::PoseStamped>(kPortIDReferencePose),
                                               getInput<std::vector<double>>(kPortIDHeights));
  if (!ports.has_value())
  {
    getBehaviorContext()->logger->publishFailureMessage(
        name(), "Failed to get required values from input data ports: " + ports.error());
    return BT::NodeStatus::FAILURE;
  }
  const auto& [aim_pose_msg, reference_pose_msg, heights] = ports.value();

  if (heights.empty())
  {
    getBehaviorContext()->logger->publishFailureMessage(name(), "heights is empty, so the path would have no "
                                                                "waypoints.");
    return BT::NodeStatus::FAILURE;
  }
  if (aim_pose_msg.header.frame_id != reference_pose_msg.header.frame_id)
  {
    getBehaviorContext()->logger->publishFailureMessage(
        name(), fmt::format("aim_pose is in frame '{}' but reference_pose is in frame '{}'; their yaws are not "
                            "comparable.",
                            aim_pose_msg.header.frame_id, reference_pose_msg.header.frame_id));
    return BT::NodeStatus::FAILURE;
  }

  Eigen::Vector3d held_object_offset = Eigen::Vector3d::Zero();
  if (const auto offset = getInput<geometry_msgs::msg::TransformStamped>(kPortIDHeldObjectOffset); offset.has_value())
  {
    held_object_offset = tf2::transformToEigen(offset.value()).translation();
  }

  Eigen::Isometry3d aim_pose;
  tf2::fromMsg(aim_pose_msg.pose, aim_pose);
  Eigen::Isometry3d reference_pose;
  tf2::fromMsg(reference_pose_msg.pose, reference_pose);

  const std::vector<Eigen::Isometry3d> keyposes = computeTopDownKeyposes(
      aim_pose, yawOf(Eigen::Quaterniond(reference_pose.rotation())), held_object_offset, heights);

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

  return BT::NodeStatus::SUCCESS;
}
}  // namespace kinova_vla_test_sim_behaviors
