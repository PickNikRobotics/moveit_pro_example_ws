// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include <cmath>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <fmt/format.h>
#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <lab_sim_behaviors/compute_tray_place_positions_using_apriltags.hpp>
#include <moveit_pro_behavior_interface/get_required_ports.hpp>
#include <moveit_pro_behavior_interface/metadata_fields.hpp>
#include <moveit_studio_vision/image_utils.hpp>
#include <moveit_studio_vision_msgs/msg/object_detection_array.hpp>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

namespace
{
inline constexpr auto kDescriptionComputeTrayPlacePositionsUsingAprilTags = R"(
                <p>
                    Computes a grid of place positions within a tray detected by its AprilTag.
                    Finds the tray tag by ID, uses its 3D pose plus a known offset to define the
                    tray center, then generates a grid of positions with the specified number of
                    columns per row and row spacing. Rows fill from near the tag toward
                    the far end of the tray (top-to-bottom). Positions are transformed to the world frame with a
                    fixed gripper-down orientation. If an input image is provided, annotates it
                    with green circles matching the bottle diameter at each position.
                </p>
            )";

constexpr auto kPortIDDetections = "detections";
constexpr auto kPortIDCameraInfo = "camera_info";
constexpr auto kPortIDInputImage = "input_image";
constexpr auto kPortIDTrayApriltagId = "tray_apriltag_id";
constexpr auto kPortIDNumRows = "num_rows";
constexpr auto kPortIDColumnsPerRow = "columns_per_row";
constexpr auto kPortIDRowSpacing = "row_spacing";
constexpr auto kPortIDColumnSpacing = "column_spacing";
constexpr auto kPortIDTagToTrayCenter = "tag_to_tray_center";
constexpr auto kPortIDBottleDiameter = "bottle_diameter";
constexpr auto kPortIDVisualizationTopic = "visualization_topic";
constexpr auto kPortIDPlacePositions = "place_positions";
constexpr auto kDefaultVisualizationTopic = "/apriltag_detections";
constexpr auto kMarkerThickness = 2;
}  // namespace

namespace lab_sim_behaviors
{
ComputeTrayPlacePositionsUsingAprilTags::ComputeTrayPlacePositionsUsingAprilTags(
    const std::string& name, const BT::NodeConfiguration& config,
    const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources,
    std::unique_ptr<moveit_pro::behaviors::ROSPublisherHandle> ros_publisher_interface)
  : SharedResourcesNode<BT::SyncActionNode>(name, config, shared_resources)
  , ros_publisher_interface_{ std::move(ros_publisher_interface) }
{
}

BT::PortsList ComputeTrayPlacePositionsUsingAprilTags::providedPorts()
{
  return {
    BT::InputPort<moveit_studio_vision_msgs::msg::ObjectDetectionArray>(kPortIDDetections, "{detections}",
                                                                        "Array of AprilTag detections from "
                                                                        "DetectAprilTags. Must include the tray tag."),
    BT::InputPort<sensor_msgs::msg::CameraInfo>(kPortIDCameraInfo, "{camera_info}",
                                                "Camera intrinsics used to project 3D positions to pixel coordinates "
                                                "for visualization."),
    BT::InputPort<sensor_msgs::msg::Image>(
        kPortIDInputImage, "", "Optional camera image to annotate with green circles at each computed place position."),
    BT::InputPort<int>(kPortIDTrayApriltagId, 1,
                       "AprilTag ID assigned to the tray. Used to find the tray among all detections."),
    BT::InputPort<int>(kPortIDNumRows, 3, "Number of rows of place positions along the tray's Y axis."),
    BT::InputPort<int>(kPortIDColumnsPerRow, 2, "Number of columns per row along the tray's X axis."),
    BT::InputPort<double>(kPortIDRowSpacing, 0.055, "Distance in meters between rows along the tray's Y axis."),
    BT::InputPort<double>(kPortIDColumnSpacing, 0.05, "Distance in meters between columns along the tray's X axis."),
    BT::InputPort<double>(kPortIDTagToTrayCenter, 0.15,
                          "Distance in meters from the tray tag to the tray center along the tray's +Y axis."),
    BT::InputPort<double>(kPortIDBottleDiameter, 0.04,
                          "Bottle diameter in meters. Used to size the visualization circles on the annotated image."),
    BT::InputPort<std::string>(kPortIDVisualizationTopic, kDefaultVisualizationTopic,
                               "ROS image topic to publish the annotated image to. Only used when input_image is "
                               "provided."),
    BT::OutputPort<std::vector<geometry_msgs::msg::PoseStamped>>(kPortIDPlacePositions, "{place_positions}",
                                                                 "Computed world-frame poses for placing objects in "
                                                                 "the tray, with gripper-down orientation."),
  };
}

BT::KeyValueVector ComputeTrayPlacePositionsUsingAprilTags::metadata()
{
  return { { moveit_pro::behaviors::kSubcategoryMetadataKey, "Vision" },
           { moveit_pro::behaviors::kDescriptionMetadataKey, kDescriptionComputeTrayPlacePositionsUsingAprilTags } };
}

BT::NodeStatus ComputeTrayPlacePositionsUsingAprilTags::tick()
{
  const auto ports = moveit_pro::behaviors::getRequiredInputs(
      getInput<moveit_studio_vision_msgs::msg::ObjectDetectionArray>(kPortIDDetections),
      getInput<sensor_msgs::msg::CameraInfo>(kPortIDCameraInfo));

  if (!ports.has_value())
  {
    shared_resources_->logger->publishFailureMessage(name(), "Failed to get required values from input data ports: " +
                                                                 ports.error());
    return BT::NodeStatus::FAILURE;
  }

  const auto& [detections_array, camera_info] = ports.value();

  const int tray_apriltag_id = getInput<int>(kPortIDTrayApriltagId).value();
  const int num_rows = getInput<int>(kPortIDNumRows).value();
  const int columns_per_row = getInput<int>(kPortIDColumnsPerRow).value();
  const double row_spacing = getInput<double>(kPortIDRowSpacing).value();
  const double column_spacing = getInput<double>(kPortIDColumnSpacing).value();
  const double tag_to_tray_center = getInput<double>(kPortIDTagToTrayCenter).value();
  const double bottle_diameter = getInput<double>(kPortIDBottleDiameter).value();

  // Find the tray AprilTag by matching tray_apriltag_id
  const auto tray_it =
      std::find_if(detections_array.detections.begin(), detections_array.detections.end(),
                   [tray_apriltag_id](const auto& detection) { return detection.id == tray_apriltag_id; });

  if (tray_it == detections_array.detections.end())
  {
    shared_resources_->logger->publishFailureMessage(
        name(), fmt::format("Tray AprilTag with ID {} not found in detections.", tray_apriltag_id));
    return BT::NodeStatus::FAILURE;
  }

  const auto& tray_pose = tray_it->pose.pose;
  const std::string& camera_frame = tray_it->pose.header.frame_id;

  // Get the tray tag's 3D pose in camera frame
  const Eigen::Quaterniond tag_rotation(tray_pose.orientation.w, tray_pose.orientation.x, tray_pose.orientation.y,
                                        tray_pose.orientation.z);
  const Eigen::Vector3d tag_position(tray_pose.position.x, tray_pose.position.y, tray_pose.position.z);

  // Define tray local axes in camera frame
  const Eigen::Vector3d tray_x = tag_rotation * Eigen::Vector3d::UnitX();
  const Eigen::Vector3d tray_y = tag_rotation * Eigen::Vector3d::UnitY();

  // Compute tray center from tag position. The tag's +Y points away from the tray
  // interior, so we move in the -Y direction to reach the tray center.
  const Eigen::Vector3d tray_center = tag_position - tray_y * tag_to_tray_center;

  // Generate grid positions. Rows fill from near the tag (-Y) toward the far end (+Y).
  // Within each row, columns are centered around X=0.
  const int num_positions = num_rows * columns_per_row;
  std::vector<Eigen::Vector3d> positions_camera;
  positions_camera.reserve(num_positions);

  for (int row = 0; row < num_rows; ++row)
  {
    // Row 0 is closest to the tag (-Y), last row is at the far end (+Y)
    const double row_offset = (row - (num_rows - 1) / 2.0) * row_spacing;

    for (int col = 0; col < columns_per_row; ++col)
    {
      const double col_offset = (col - (columns_per_row - 1) / 2.0) * column_spacing;
      positions_camera.push_back(tray_center + tray_y * row_offset + tray_x * col_offset);
    }
  }

  // Look up camera_frame -> "world" transform
  if (!shared_resources_->transform_buffer_ptr->canTransform("world", camera_frame, tf2::TimePointZero))
  {
    shared_resources_->logger->publishFailureMessage(name(), fmt::format("Cannot transform between '{}' and 'world'.",
                                                                         camera_frame));
    return BT::NodeStatus::FAILURE;
  }
  const geometry_msgs::msg::TransformStamped camera_to_world_tf =
      shared_resources_->transform_buffer_ptr->lookupTransform("world", camera_frame, tf2::TimePointZero);
  const Eigen::Isometry3d camera_to_world = tf2::transformToEigen(camera_to_world_tf);

  // Transform all positions to world frame with gripper-down orientation (x=1, y=0, z=0, w=0)
  std::vector<geometry_msgs::msg::PoseStamped> place_positions;
  place_positions.reserve(num_positions);
  for (const auto& pos_camera : positions_camera)
  {
    const Eigen::Vector3d pos_world = camera_to_world * pos_camera;

    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "world";
    pose.pose.position.x = pos_world.x();
    pose.pose.position.y = pos_world.y();
    pose.pose.position.z = pos_world.z();
    pose.pose.orientation.x = 1.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 0.0;
    place_positions.push_back(pose);
  }

  // Visualization: annotate the camera image with place position markers
  const auto maybe_image = getInput<sensor_msgs::msg::Image>(kPortIDInputImage);
  if (!maybe_image.has_value())
  {
    shared_resources_->logger->publishInfoMessage(name(), "No input_image provided; skipping visualization.");
  }
  else
  {
    try
    {
      const auto viz_topic = getInput<std::string>(kPortIDVisualizationTopic).value();
      ros_publisher_interface_->init(viz_topic, shared_resources_);

      const double fx = camera_info.k[0];
      const double fy = camera_info.k[4];
      const double cx = camera_info.k[2];
      const double cy = camera_info.k[5];

      const cv_bridge::CvImagePtr cv_image_ptr =
          cv_bridge::toCvCopy(maybe_image.value(), sensor_msgs::image_encodings::RGB8);
      cv::Mat& cv_image = cv_image_ptr->image;

      const cv::Scalar green(0, 255, 0);

      for (std::size_t i = 0; i < positions_camera.size(); ++i)
      {
        const auto& p = positions_camera[i];
        if (p.z() <= 0.0)
        {
          continue;
        }

        const int u = static_cast<int>(fx * p.x() / p.z() + cx);
        const int v = static_cast<int>(fy * p.y() / p.z() + cy);

        // Skip positions that project outside the image bounds
        if (u < 0 || u >= cv_image.cols || v < 0 || v >= cv_image.rows)
        {
          continue;
        }

        // Compute pixel radius from bottle diameter using pinhole projection
        const double radius_3d = bottle_diameter / 2.0;
        const int pixel_radius = std::max(1, static_cast<int>(fx * radius_3d / p.z()));

        // Draw a green circle matching the bottle diameter and a cross at each place position
        cv::circle(cv_image, cv::Point(u, v), pixel_radius, green, kMarkerThickness);
        cv::line(cv_image, cv::Point(u - pixel_radius, v), cv::Point(u + pixel_radius, v), green, kMarkerThickness);
        cv::line(cv_image, cv::Point(u, v - pixel_radius), cv::Point(u, v + pixel_radius), green, kMarkerThickness);

        const std::string label = fmt::format("P{}", i);
        const cv::Rect bbox(u - pixel_radius, v - pixel_radius, pixel_radius * 2, pixel_radius * 2);
        moveit_studio::vision::overlayText(label, bbox, green, cv_image);
      }

      sensor_msgs::msg::Image annotated_image;
      cv_image_ptr->toImageMsg(annotated_image);
      annotated_image.header = maybe_image.value().header;

      ros_publisher_interface_->publish_image(annotated_image);
    }
    catch (const cv_bridge::Exception& e)
    {
      shared_resources_->logger->publishFailureMessage(name(), fmt::format("cv_bridge error during visualization: {}",
                                                                           e.what()));
    }
  }

  setOutput(kPortIDPlacePositions, place_positions);

  return BT::NodeStatus::SUCCESS;
}
}  // namespace lab_sim_behaviors
