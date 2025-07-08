#include <detect_aruco_tag/detect_aruco_tag.hpp>

#include "cv_bridge/cv_bridge.h"
#include "eigen3/Eigen/Dense"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "opencv2/aruco.hpp"
#include "opencv2/core/eigen.hpp"
#include "opencv2/opencv.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "spdlog/spdlog.h"

#include "moveit_studio_behavior_interface/get_required_ports.hpp"

namespace detect_aruco_tag
{
DetectArucoTag::DetectArucoTag(const std::string& name, const BT::NodeConfiguration& config,
                               const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources)
  : moveit_studio::behaviors::SharedResourcesNode<BT::SyncActionNode>(name, config, shared_resources)
{
}

BT::PortsList DetectArucoTag::providedPorts()
{
  return BT::PortsList(
      { BT::InputPort<sensor_msgs::msg::Image>("input_image", "{image}", "Input image message."),
        BT::InputPort<sensor_msgs::msg::CameraInfo>("camera_info", "{camera_info}", "Input camera parameters message."),
        BT::InputPort<double>("tag_size", "{tag_size}",
                              "Size length of the Aruco tag in meters. Used to calculate the pose of the tag."),
        BT::OutputPort<geometry_msgs::msg::PoseStamped>("pose_detection", "{detected_pose}",
                                                        "Output pose of the detected Aruco tag.") });
}

BT::KeyValueVector DetectArucoTag::metadata()
{
  return { { "description", "Run Aruco tag detection on a provided image and output the detected pose" },
           { "subcategory", "User Created Behaviors" } };
}

BT::NodeStatus DetectArucoTag::tick()
{
  const auto ports = moveit_studio::behaviors::getRequiredInputs(getInput<sensor_msgs::msg::Image>("input_image"),
                                                                 getInput<sensor_msgs::msg::CameraInfo>("camera_info"),
                                                                 getInput<double>("tag_size"));
  // Check that all required input data ports were set
  if (!ports.has_value())
  {
    spdlog::error("Missing input port: {}", ports.error());
    shared_resources_->logger->publishFailureMessage(name(), fmt::format("Missing input port: {}.", ports.error()));
    return BT::NodeStatus::FAILURE;
  }

  const auto [image_msg, camera_info_msg, tag_size] = ports.value();

  // Convert ROS image message to OpenCV image
  cv::Mat cv_image;
  try
  {
    cv_image = cv_bridge::toCvCopy(image_msg)->image;
  }
  catch (const std::exception& e)
  {
    shared_resources_->logger->publishFailureMessage(name(), "Image conversion failed.");
    spdlog::error("Image conversion failed: {}", e.what());
    return BT::NodeStatus::FAILURE;
  }

  // Detect Aruco tag
  cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);
  std::vector<int> ids;
  std::vector<std::vector<cv::Point2f> > corners;
  cv::aruco::detectMarkers(cv_image, dictionary, corners, ids);

  if (ids.empty())
  {
    shared_resources_->logger->publishWarnMessage(name(), "No Aruco tag detected.");
    spdlog::warn("No Aruco tag detected.");
    return BT::NodeStatus::FAILURE;
  }

  // Estimate pose (assuming camera parameters are known and set)
  cv::Mat cameraMatrix(3, 3, CV_64F, const_cast<double*>(camera_info_msg.k.data()));
  cv::Mat distCoeffs(static_cast<int>(camera_info_msg.d.size()), 1, CV_64F,
                     const_cast<double*>(camera_info_msg.d.data()));
  std::vector<cv::Vec3d> rvecs, tvecs;
  cv::aruco::estimatePoseSingleMarkers(corners, tag_size, cameraMatrix, distCoeffs, rvecs, tvecs);

  if (rvecs.empty() || tvecs.empty())
  {
    shared_resources_->logger->publishFailureMessage(name(), "Pose estimation failed.");
    spdlog::error("Pose estimation failed.");
    return BT::NodeStatus::FAILURE;
  }

  // Fill output pose message
  geometry_msgs::msg::PoseStamped pose_msg;
  pose_msg.header = image_msg.header;
  pose_msg.pose.position.x = tvecs[0][0];
  pose_msg.pose.position.y = tvecs[0][1];
  pose_msg.pose.position.z = tvecs[0][2];
  cv::Mat rot;
  cv::Rodrigues(rvecs[0], rot);
  Eigen::Matrix3d eig_rot;
  cv::cv2eigen(rot, eig_rot);
  Eigen::Quaterniond q(eig_rot);
  pose_msg.pose.orientation.x = q.x();
  pose_msg.pose.orientation.y = q.y();
  pose_msg.pose.orientation.z = q.z();
  pose_msg.pose.orientation.w = q.w();

  // Output the detected pose
  setOutput("pose_detection", pose_msg);
  shared_resources_->logger->publishInfoMessage(name(), "Aruco tag detected and pose output.");
  spdlog::info("Aruco tag detected and pose output.");

  return BT::NodeStatus::SUCCESS;
}
}  // namespace detect_aruco_tag
