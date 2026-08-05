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
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <moveit_pro_behavior_interface/behavior_context.hpp>
#include <moveit_pro_behavior_interface/shared_resources_node.hpp>

namespace lab_sim_behaviors
{
/**
 * @brief Parameters that shape the raster produced by generateRasterPath().
 */
struct CoveragePathParams
{
  double line_spacing;   ///< Distance between adjacent raster lines, in meters.
  double point_spacing;  ///< Sampling distance along each raster line, in meters.
  double standoff;       ///< Tool height above the region's top face, along the region +Z, in meters.
  double margin;         ///< Inset from the region edges on all sides, in meters.
};

/**
 * @brief Pure geometry: a boustrophedon (serpentine) raster of tool poses over the top face of an
 * oriented box.
 *
 * @details The region is described by @p region_pose (the box center + orientation) and
 * @p region_dimensions (`[x, y, z]` full extents in the pose's frame). Poses are laid out in the
 * pose's local X-Y plane at `z = dz/2 + standoff` (a standoff above the top face), stepping lines
 * along Y by `line_spacing` and sampling along X by `point_spacing`, reversing X each line so the
 * path is continuous. Each pose keeps the region's X-Y orientation but points the tool +Z into the
 * surface (rotated 180° about the region X axis). Header frame is inherited from @p region_pose.
 *
 * Returns an empty vector when the usable area (after @p margin) is non-positive, a spacing is
 * non-positive, or the region/spacing combination would exceed the internal pose cap.
 *
 * @param region_pose Center pose + orientation of the region.
 * @param region_dimensions Full extents `[x, y, z]` in the region frame (must have 3 entries).
 * @param params Raster shaping parameters.
 * @return The ordered tool poses, or empty on degenerate input.
 */
[[nodiscard]] std::vector<geometry_msgs::msg::PoseStamped>
generateRasterPath(const geometry_msgs::msg::PoseStamped& region_pose, const std::vector<double>& region_dimensions,
                   const CoveragePathParams& params);

/**
 * @brief Generates a coverage (raster) path of tool poses over a rectangular region.
 *
 * @details Thin BT wrapper over generateRasterPath(): reads the region and parameters from ports,
 * computes the serpentine raster, and writes it to `coverage_path` for a Cartesian planner
 * (e.g. `PlanCartesianPath`) to follow. Intended to turn a user-selected region (see
 * `GetRegionFromUser`, projected to 3D) into a wipe/inspection sweep.
 *
 * | Data Port Name    | Port Type | Object Type                                  |
 * | ----------------- | --------- | -------------------------------------------- |
 * | region_pose       | input     | geometry_msgs::msg::PoseStamped              |
 * | region_dimensions | input     | std::vector<double>                          |
 * | line_spacing      | input     | double                                       |
 * | point_spacing     | input     | double                                       |
 * | standoff          | input     | double                                       |
 * | margin            | input     | double                                       |
 * | coverage_path     | output    | std::vector<geometry_msgs::msg::PoseStamped> |
 */
class GenerateSurfaceCoveragePath final : public moveit_pro::behaviors::SharedResourcesNode<BT::SyncActionNode>
{
public:
  GenerateSurfaceCoveragePath(const std::string& name, const BT::NodeConfiguration& config,
                              const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources);

  static BT::PortsList providedPorts();

  static BT::KeyValueVector metadata();

  BT::NodeStatus tick() override;
};
}  // namespace lab_sim_behaviors
