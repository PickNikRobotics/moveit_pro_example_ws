// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <behaviortree_cpp/action_node.h>
#include <hangar_sim_behaviors/localization_gates.hpp>
#include <moveit_pro_behavior_interface/behavior_context.hpp>
#include <moveit_pro_behavior_interface/shared_resources_node.hpp>
#include <nav_msgs/msg/map_meta_data.hpp>

#include <memory>
#include <string>

namespace hangar_sim_behaviors
{
/**
 * @brief Measure how well a candidate pose explains the live scan against the map.
 *
 * @details This is the acceptance test for in-place localization refinement, and it exists because
 * confidence cannot be one. A particle filter's covariance falls as its particle set depletes, so it
 * shrinks whether the surviving pose is right or wrong: on meta_ws hardware two stationary
 * refinements from different clicks were measured settling 3 m apart while *both* grew more
 * confident, and the best sigma threshold available caught only four failures in five. This
 * Behavior measures something else entirely -- whether the scan, projected from the candidate pose,
 * lands on the obstacles the map says are there.
 *
 * It reads beluga's own likelihood field one step before beluga turns it into likelihoods: for each
 * beam the filter would have used, the distance from that beam's endpoint to the nearest occupied
 * cell. Same obstacle mask, same wavefront, same truncation, same beam decimation -- see
 * localization_gates.hpp, where each mirror is pinned to the beluga source it mirrors. A residual
 * computed over a *different* field, or over beams the filter never scored, is not a gate on the
 * filter; it is a second opinion that happens to be nearby.
 *
 * The threshold is a port, not a constant, because it depends on how well the map still matches the
 * space. Re-measure it at a known-good pose after every map rebuild; the outputs make that a
 * ten-second check.
 *
 * | Data Port Name        | Port Type | Object Type                     |
 * | --------------------- |-----------|---------------------------------|
 * | pose                  | Input     | geometry_msgs::msg::PoseStamped |
 * | scan_topic            | Input     | std::string                     |
 * | map_topic             | Input     | std::string                     |
 * | min_range             | Input     | double                          |
 * | max_range             | Input     | double                          |
 * | max_beams             | Input     | int                             |
 * | max_obstacle_distance | Input     | double                          |
 * | inlier_distance       | Input     | double                          |
 * | min_inlier_fraction   | Input     | double                          |
 * | timeout               | Input     | double                          |
 * | inlier_fraction       | Output    | double                          |
 * | median_residual       | Output    | double                          |
 * | beams_used            | Output    | int                             |
 */
class ScanMatchResidual final : public moveit_pro::behaviors::SharedResourcesNode<BT::SyncActionNode>
{
public:
  ScanMatchResidual(const std::string& name, const BT::NodeConfiguration& config,
                    const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources);

  static BT::PortsList providedPorts();

  static BT::KeyValueVector metadata();

  BT::NodeStatus tick() override;

private:
  /**
   * @brief Cached so a repeated measurement does not redo the wavefront over a million cells.
   *
   * The map is latched and rarely changes, but it does change when a new one is deployed, so the
   * cache is keyed on the grid's own metadata and on the truncation distance rather than assumed
   * permanent.
   */
  localization::DistanceField field_;
  nav_msgs::msg::MapMetaData cached_map_info_;
  std::string cached_map_topic_;
  double cached_max_obstacle_distance_ = 0.0;
};

}  // namespace hangar_sim_behaviors
