#include "calibration_behaviors/calibrate_camera_pose.hpp"

#include <vector>

#include "fmt/format.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_eigen/tf2_eigen.hpp"

#include "moveit_studio_behavior_interface/get_required_ports.hpp"

namespace
{
constexpr auto kPortIDDetectedPoses = "detected_poses";
constexpr auto kPortIDCalibrationToolFrame = "calibration_tool_frame";
constexpr auto kPortIDBaseFrame = "base_link";
constexpr auto kPortIDOpticalFrame = "camera_optical_frame";
constexpr auto kPortIDComputedPose = "computed_pose";
// DEBUG PORTS
constexpr auto kPortIDDebugBaseCalibPose = "debug_base_calibration_pose";
constexpr auto kPortIDDebugCameraCalibPose = "debug_camera_calibration_pose";
}  // namespace

namespace calibration_behaviors
{
CalibrateCameraPose::CalibrateCameraPose(
    const std::string& name, const BT::NodeConfiguration& config,
    const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources)
  : moveit_studio::behaviors::SharedResourcesNode<BT::SyncActionNode>(name, config, shared_resources)
{
}

BT::PortsList CalibrateCameraPose::providedPorts()
{
  return BT::PortsList(
      { BT::InputPort<std::vector<geometry_msgs::msg::PoseStamped>>(kPortIDDetectedPoses, "{detected_poses}",
                                                                    "Vector of the detected poses in the correct order "
                                                                    "(right, left, "
                                                                    "bottom)."),
        BT::InputPort<std::string>(kPortIDCalibrationToolFrame, "calibration_tool_link",
                                   "Frame of the calibration tool. Must be a "
                                   "part of the robot kinematic chain."),
        BT::InputPort<std::string>(kPortIDBaseFrame, "base_link",
                                   "Frame the output computed pose should be represented in. Must be a "
                                   "part of the robot kinematic chain."),
        BT::InputPort<std::string>(kPortIDOpticalFrame, "camera_optical_frame",
                                   "Current optical frame of the camera for debug purposes."),
        BT::OutputPort<geometry_msgs::msg::PoseStamped>(kPortIDComputedPose, "{computed_pose}",
                                                        "Computed pose of the camera."),
        BT::OutputPort<geometry_msgs::msg::PoseStamped>(kPortIDDebugBaseCalibPose, "{base_to_calib_pose}",
                                                        "Pose of the calibration tool represented in the base frame."),
        BT::OutputPort<geometry_msgs::msg::PoseStamped>(kPortIDDebugCameraCalibPose, "{camera_to_calib_pose}",
                                                        "Pose of the calibration tool represented in the camera "
                                                        "frame.") });
}

BT::KeyValueVector CalibrateCameraPose::metadata()
{
  return { { "description", "Calibrate scene camera extrinsics" } };
}

BT::NodeStatus CalibrateCameraPose::tick()
{
  const auto ports = moveit_studio::behaviors::getRequiredInputs(
      getInput<std::vector<geometry_msgs::msg::PoseStamped>>(kPortIDDetectedPoses),
      getInput<std::string>(kPortIDCalibrationToolFrame), getInput<std::string>(kPortIDBaseFrame),
      getInput<std::string>(kPortIDOpticalFrame));

  if (!ports.has_value())
  {
    shared_resources_->logger->publishFailureMessage(name(), fmt::format("Missing input port: {}", ports.error()));
    return BT::NodeStatus::FAILURE;
  }

  const auto& [detected_poses, calibration_tool_frame, base_frame, optical_frame] = ports.value();

  if (detected_poses.size() != 3)
  {
    shared_resources_->logger->publishFailureMessage(name(),
                                                     fmt::format("`detected_poses` input port should have size of "
                                                                 "3, but has size of {}",
                                                                 detected_poses.size()));
    return BT::NodeStatus::FAILURE;
  }

  // Convert points to Eigen for easier computation.
  Eigen::Vector3d point1(detected_poses[0].pose.position.x, detected_poses[0].pose.position.y,
                         detected_poses[0].pose.position.z);
  Eigen::Vector3d point2(detected_poses[1].pose.position.x, detected_poses[1].pose.position.y,
                         detected_poses[1].pose.position.z);
  Eigen::Vector3d point3(detected_poses[2].pose.position.x, detected_poses[2].pose.position.y,
                         detected_poses[2].pose.position.z);

  // Compute center point of the 3 poses.
  Eigen::Vector3d center_point = (point1 + point2 + point3) / 3.0;

  // Compute the vectors of the triangle sides.
  Eigen::Vector3d v1 = point2 - point1;
  Eigen::Vector3d v2 = point3 - point1;

  // Compute x axis as the vector between point1 & point2.
  Eigen::Vector3d x_axis(v1.normalized());

  // Compute z axis as the triangle normal.
  Eigen::Vector3d z_axis(v1.cross(v2).normalized());

  // Compute the y axis as the cross product between the z & x axes.
  Eigen::Vector3d y_axis(z_axis.cross(x_axis).normalized());

  // Convert the pose into an Eigen::Isometry3d for computations & to get the
  // quaternion orientation.
  Eigen::Isometry3d camera_to_calibration_tool = Eigen::Isometry3d::Identity();
  camera_to_calibration_tool.linear().col(0) = x_axis;
  camera_to_calibration_tool.linear().col(1) = y_axis;
  camera_to_calibration_tool.linear().col(2) = z_axis;
  camera_to_calibration_tool.translation() = center_point;

  if (auto determinant = camera_to_calibration_tool.rotation().determinant();
      abs(determinant) - 1 > std::numeric_limits<double>::epsilon())
  {
    shared_resources_->logger->publishFailureMessage(
        fmt::format("Rotation matrix of computed pose is not normalized. Determinant={}", determinant));
    return BT::NodeStatus::FAILURE;
  }

  // Get the transform from the base frame to the calibration tool.
  if (!shared_resources_->transform_buffer_ptr->canTransform(base_frame, calibration_tool_frame, tf2::TimePointZero))
  {
    shared_resources_->logger->publishFailureMessage(
        fmt::format("Cannot transform between '{}' and '{}'", calibration_tool_frame, base_frame));
    return BT::NodeStatus::FAILURE;
  }
  geometry_msgs::msg::TransformStamped base_to_calibration_tool_tf =
      shared_resources_->transform_buffer_ptr->lookupTransform(base_frame, calibration_tool_frame, tf2::TimePointZero);
  Eigen::Isometry3d base_to_calibration_tool = tf2::transformToEigen(base_to_calibration_tool_tf);

  // Compute the pose of the camera optical frame in the base frame.
  Eigen::Isometry3d base_to_camera = base_to_calibration_tool * camera_to_calibration_tool.inverse();

  // Create the PoseStamped object to return.
  geometry_msgs::msg::PoseStamped computed_pose;
  computed_pose.header.frame_id = base_frame;
  computed_pose.pose = tf2::toMsg(base_to_camera);

  setOutput(kPortIDComputedPose, computed_pose);

  // DEBUG output
  geometry_msgs::msg::PoseStamped camera_calibration_pose;
  camera_calibration_pose.header.frame_id = optical_frame;
  camera_calibration_pose.pose = tf2::toMsg(camera_to_calibration_tool);
  setOutput(kPortIDDebugCameraCalibPose, camera_calibration_pose);

  geometry_msgs::msg::PoseStamped base_calibration_pose;
  base_calibration_pose.header.frame_id = base_frame;
  base_calibration_pose.pose = tf2::toMsg(base_to_calibration_tool);
  setOutput(kPortIDDebugBaseCalibPose, base_calibration_pose);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace calibration_behaviors
