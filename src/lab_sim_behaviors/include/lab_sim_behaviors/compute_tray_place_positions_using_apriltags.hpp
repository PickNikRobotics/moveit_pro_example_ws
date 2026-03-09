// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <behaviortree_cpp/action_node.h>
#include <moveit_pro_behavior/behaviors/visualization/ros_publisher_handle.hpp>
#include <moveit_pro_behavior_interface/behavior_context.hpp>
#include <moveit_pro_behavior_interface/shared_resources_node.hpp>

namespace lab_sim_behaviors
{
/**
 * @brief Computes a grid of place positions within a tray detected via its AprilTag.
 *
 * @details Given an array of AprilTag detections and camera intrinsics, this Behavior finds the
 * tray's AprilTag by ID, uses its 3D pose plus a known offset to locate the tray center, then
 * generates a grid of place positions (multiple columns per row). Rows fill from the far end
 * of the tray toward the tag (top-to-bottom). Positions are transformed from the camera frame
 * to the world frame with a fixed gripper-down orientation.
 *
 * If an input image is provided, the Behavior annotates it with green circles sized to match
 * the bottle diameter at each computed place position and publishes the result.
 *
 * | Data Port Name          | Port Type | Object Type                                         |
 * | ----------------------- | --------- | --------------------------------------------------- |
 * | detections              | input     | moveit_studio_vision_msgs::msg::ObjectDetectionArray |
 * | camera_info             | input     | sensor_msgs::msg::CameraInfo                        |
 * | input_image             | input     | sensor_msgs::msg::Image (optional)                  |
 * | tray_apriltag_id        | input     | int                                                 |
 * | num_rows                | input     | int                                                 |
 * | columns_per_row         | input     | int                                                 |
 * | row_spacing             | input     | double                                              |
 * | column_spacing          | input     | double                                              |
 * | tag_to_tray_center      | input     | double                                              |
 * | bottle_diameter         | input     | double                                              |
 * | visualization_topic     | input     | std::string                                         |
 * | place_positions         | output    | std::vector<geometry_msgs::msg::PoseStamped>        |
 */
class ComputeTrayPlacePositionsUsingAprilTags final
  : public moveit_pro::behaviors::SharedResourcesNode<BT::SyncActionNode>
{
public:
  ComputeTrayPlacePositionsUsingAprilTags(
      const std::string& name, const BT::NodeConfiguration& config,
      const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources,
      std::unique_ptr<moveit_pro::behaviors::ROSPublisherHandle> ros_publisher_interface =
          std::make_unique<moveit_pro::behaviors::ROSPublisherHandle>());

  static BT::PortsList providedPorts();

  static BT::KeyValueVector metadata();

  BT::NodeStatus tick() override;

private:
  std::unique_ptr<moveit_pro::behaviors::ROSPublisherHandle> ros_publisher_interface_;
};
}  // namespace lab_sim_behaviors
