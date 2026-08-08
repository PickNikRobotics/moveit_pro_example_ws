// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <memory>
#include <string>
#include <vector>

#include <behaviortree_cpp/action_node.h>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <moveit_studio_vision_msgs/msg/mask2_d.hpp>

#include <moveit_pro_behavior_interface/behavior_context.hpp>
#include <moveit_pro_behavior_interface/shared_resources_node.hpp>

namespace lab_sim_behaviors
{
/**
 * @brief Pure geometry: rasterize a normalized 2D region into a single Mask2D.
 *
 * @details GetRegionFromUser returns normalized [0..1] image coordinates; this scales them to pixels
 * by @p image_width / @p image_height, then fills the (convex) polygon into a mono8 image sized to the
 * polygon's bounding box, with the box's top-left corner stored in `Mask2D.x/y` — the layout
 * `GetMasks3DFromMasks2D` expects. The image header frame is inherited from @p region so the mask
 * lines up with the camera image it was drawn on. Fails (returns an error string) when the region has
 * fewer than three points, the image dimensions are non-positive, or the bounding box has zero area.
 *
 * @param region The normalized [0..1] polygon (e.g. the `region` output of GetRegionFromUser).
 * @param image_width Camera image width in pixels, used to denormalize the region's x coordinates.
 * @param image_height Camera image height in pixels, used to denormalize the region's y coordinates.
 * @note The image the region was drawn on and the camera_info supplying these dimensions must share
 * the same resolution, otherwise the mask is rasterized at the wrong scale.
 * @return The rasterized mask, or an error string.
 */
[[nodiscard]] tl::expected<moveit_studio_vision_msgs::msg::Mask2D, std::string>
regionToMask2D(const geometry_msgs::msg::PolygonStamped& region, int image_width, int image_height);

/**
 * @brief Converts a user-selected 2D region into a Mask2D that the 3D-lifting Behaviors consume.
 *
 * @details Thin BT wrapper over regionToMask2D(): reads the `region` polygon (see GetRegionFromUser),
 * rasterizes it, and writes a single-element `masks2d` vector for `GetMasks3DFromMasks2D` (which
 * projects it onto the camera point cloud to recover the 3D region).
 *
 * | Data Port Name | Port Type | Object Type                                       |
 * | -------------- | --------- | ------------------------------------------------- |
 * | region         | input     | geometry_msgs::msg::PolygonStamped                |
 * | camera_info    | input     | sensor_msgs::msg::CameraInfo                      |
 * | masks2d        | output    | std::vector<moveit_studio_vision_msgs::msg::Mask2D> |
 */
class GetMask2DFromRegion final : public moveit_pro::behaviors::SharedResourcesNode<BT::SyncActionNode>
{
public:
  GetMask2DFromRegion(const std::string& name, const BT::NodeConfiguration& config,
                      const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources);

  static BT::PortsList providedPorts();

  static BT::KeyValueVector metadata();

  BT::NodeStatus tick() override;
};
}  // namespace lab_sim_behaviors
