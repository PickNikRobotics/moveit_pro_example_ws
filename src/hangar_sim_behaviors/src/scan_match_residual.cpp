// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include <hangar_sim_behaviors/scan_match_residual.hpp>
#include <hangar_sim_behaviors/wait_for_one_message.hpp>

#include <fmt/format.h>

#include <moveit_pro_behavior_interface/behavior_subcategories.hpp>
#include <moveit_pro_behavior_interface/get_required_ports.hpp>
#include <moveit_pro_behavior_interface/metadata_fields.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2/utils.hpp>

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

namespace
{
constexpr auto kPortPose = "pose";
constexpr auto kPortScanTopic = "scan_topic";
constexpr auto kPortMapTopic = "map_topic";
constexpr auto kPortMinRange = "min_range";
constexpr auto kPortMaxRange = "max_range";
constexpr auto kPortMaxBeams = "max_beams";
constexpr auto kPortMaxObstacleDistance = "max_obstacle_distance";
constexpr auto kPortInlierDistance = "inlier_distance";
constexpr auto kPortMinInlierFraction = "min_inlier_fraction";
constexpr auto kPortTimeout = "timeout";
constexpr auto kPortInlierFraction = "inlier_fraction";
constexpr auto kPortMedianResidual = "median_residual";
constexpr auto kPortBeamsUsed = "beams_used";

constexpr auto kDefaultScanTopic = "/scan_merged";
constexpr auto kDefaultMapTopic = "/map";
constexpr auto kMapFrame = "map";
constexpr double kDefaultTimeout = 5.0;
/// Negative means "measure and report, let the tree judge" -- the house style for a threshold that
/// should be tunable from XML without a rebuild.
constexpr double kNoThreshold = -1.0;

inline constexpr auto kDescription = R"(
                <p>Measures how well a candidate pose explains the live scan against the map, and reports it as the fraction of beams whose endpoint lands within <code>inlier_distance</code> of an occupied cell.</p>
                <p>This is the acceptance test for in-place localization refinement, and it exists because confidence cannot be one. A particle filter's covariance falls as its particle set depletes, so it shrinks whether the surviving pose is right or wrong; two stationary refinements from different clicks have been measured settling 3 m apart while <em>both</em> grew more confident. This Behavior measures something else: whether the scan actually fits the map from here.</p>
                <p>The field it reads is beluga's own likelihood field, one step before beluga turns distances into likelihoods, and the beams it scores are the beams beluga's decimation would have picked. The port defaults mirror the <code>amcl:</code> block of <code>nav2_params.yaml</code>; if that block changes, change these with it or the gate stops measuring the filter.</p>
                <p>Re-measure <code>min_inlier_fraction</code> at a known-good pose after every map rebuild. It depends on how well the map still matches the space, and even a correct pose falls short of 100% because people, pallets and parked equipment are not in the map.</p>
                <p>Leave <code>min_inlier_fraction</code> negative to always report and let the tree judge with a Precondition. Set it to gate here instead, in which case a measurement below it exits with FAILURE.</p>
                <p>Exits with FAILURE if no map or no scan arrives before the timeout, if the pose is not in the map frame, or if no beam survives the range filters.</p>
            )";
}  // namespace

namespace hangar_sim_behaviors
{
ScanMatchResidual::ScanMatchResidual(const std::string& name, const BT::NodeConfiguration& config,
                                     const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
  : SharedResourcesNode(name, config, shared_resources)
{
}

BT::PortsList ScanMatchResidual::providedPorts()
{
  return {
    BT::InputPort<geometry_msgs::msg::PoseStamped>(kPortPose, "{refined_pose}",
                                                   "Candidate pose to test. Must already be in the map frame."),
    BT::InputPort<std::string>(kPortScanTopic, kDefaultScanTopic,
                               "Merged 360-degree scan. dual_laser_merger publishes it in "
                               "ridgeback_base_link, which is also amcl's base_frame_id, so no laser-to-base "
                               "offset is composed; a scan published in its own frame would need one."),
    BT::InputPort<std::string>(kPortMapTopic, kDefaultMapTopic,
                               "Latched occupancy grid. The distance field built from it is cached and rebuilt "
                               "only when the map itself changes."),
    BT::InputPort<double>(kPortMinRange, localization::kLaserMinRange,
                          "Shortest usable return in metres. Mirror amcl's laser_min_range."),
    BT::InputPort<double>(kPortMaxRange, localization::kLaserMaxRange,
                          "Longest usable return in metres. Mirror amcl's laser_max_range so the residual sees "
                          "the beams the filter saw."),
    BT::InputPort<int>(kPortMaxBeams, localization::kMaxBeams,
                       "Beams to score, spread evenly over the scan. Mirror amcl's max_beams. Raising it "
                       "measures the same quantity with finer granularity, but then the number is no longer "
                       "the one the filter acted on."),
    BT::InputPort<double>(kPortMaxObstacleDistance, localization::kMaxObstacleDistance,
                          "Distance at which the field saturates, in metres. Mirror amcl's "
                          "laser_likelihood_max_dist."),
    BT::InputPort<double>(kPortInlierDistance, localization::kInlierDistance,
                          "A beam counts as an inlier if its endpoint lands this close to an occupied cell, in "
                          "metres."),
    BT::InputPort<double>(kPortMinInlierFraction, kNoThreshold,
                          "If non-negative, exit with FAILURE below this inlier fraction. Leave negative to "
                          "report only and gate in the tree."),
    BT::InputPort<double>(kPortTimeout, kDefaultTimeout, "Seconds to wait for the map and for a scan."),
    BT::OutputPort<double>(kPortInlierFraction, "{inlier_fraction}",
                           "Fraction of used beams landing within inlier_distance of an obstacle. This is the "
                           "gate variable."),
    BT::OutputPort<double>(kPortMedianResidual, "{median_residual}",
                           "Median endpoint-to-obstacle distance in metres. Diagnostic only: it separates a "
                           "true pose from an alias by far less than the fraction does, so do not gate on it."),
    BT::OutputPort<int>(kPortBeamsUsed, "{beams_used}",
                        "Beams that survived the range filters, out of max_beams selected. A much smaller "
                        "number means the scan is mostly no-returns, and the fraction is coarser than it looks."),
  };
}

BT::KeyValueVector ScanMatchResidual::metadata()
{
  return { { moveit_pro::behaviors::kSubcategoryMetadataKey,
             std::string(moveit_pro::behaviors::toString(moveit_pro::behaviors::Subcategory::Navigation)) },
           { moveit_pro::behaviors::kDescriptionMetadataKey, kDescription } };
}

BT::NodeStatus ScanMatchResidual::tick()
{
  const auto ports = moveit_pro::behaviors::getRequiredInputs(
      getInput<geometry_msgs::msg::PoseStamped>(kPortPose), getInput<std::string>(kPortScanTopic),
      getInput<std::string>(kPortMapTopic), getInput<double>(kPortMinRange), getInput<double>(kPortMaxRange),
      getInput<int>(kPortMaxBeams), getInput<double>(kPortMaxObstacleDistance), getInput<double>(kPortInlierDistance),
      getInput<double>(kPortMinInlierFraction), getInput<double>(kPortTimeout));
  if (!ports.has_value())
  {
    getBehaviorContext()->logger->publishFailureMessage(name(),
                                                        "ScanMatchResidual: missing required input: " + ports.error());
    return BT::NodeStatus::FAILURE;
  }
  const auto& [pose, scan_topic, map_topic, min_range, max_range, max_beams, max_obstacle_distance, inlier_distance,
               min_inlier_fraction, timeout] = ports.value();

  // The residual projects the scan from the pose straight into map cells, so a pose expressed in
  // any other frame would be silently scored in the wrong place. Refuse rather than guess: an
  // unnoticed frame error here reads as a localization failure and sends someone hunting the map.
  if (pose.header.frame_id != kMapFrame)
  {
    getBehaviorContext()->logger->publishFailureMessage(
        name(), fmt::format("ScanMatchResidual: pose is in frame '{}', but the residual is only meaningful for a "
                            "pose in '{}'. Transform it first.",
                            pose.header.frame_id, kMapFrame));
    return BT::NodeStatus::FAILURE;
  }

  const auto node = getBehaviorContext()->node;

  nav_msgs::msg::OccupancyGrid map;
  if (!waitForOneMessage(node, map_topic, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(), timeout, map))
  {
    getBehaviorContext()->logger->publishFailureMessage(
        name(),
        fmt::format("ScanMatchResidual: no map on '{}' within {:.1f} s. Is map_server up and activated?", map_topic,
                    timeout));
    return BT::NodeStatus::FAILURE;
  }

  const bool map_changed = !field_.valid() || map_topic != cached_map_topic_ ||
                           max_obstacle_distance != cached_max_obstacle_distance_ ||
                           map.info.width != cached_map_info_.width || map.info.height != cached_map_info_.height ||
                           map.info.resolution != cached_map_info_.resolution ||
                           map.info.origin.position.x != cached_map_info_.origin.position.x ||
                           map.info.origin.position.y != cached_map_info_.origin.position.y;
  if (map_changed)
  {
    const localization::GridInfo info{ static_cast<int>(map.info.width),
                                       static_cast<int>(map.info.height),
                                       map.info.resolution,
                                       map.info.origin.position.x,
                                       map.info.origin.position.y,
                                       tf2::getYaw(map.info.origin.orientation) };
    field_ = localization::buildDistanceField(info, map.data, max_obstacle_distance);
    if (!field_.valid())
    {
      getBehaviorContext()->logger->publishFailureMessage(
          name(), fmt::format("ScanMatchResidual: map on '{}' is malformed -- a {} x {} grid carrying {} cells.",
                              map_topic, map.info.width, map.info.height, map.data.size()));
      return BT::NodeStatus::FAILURE;
    }
    cached_map_info_ = map.info;
    cached_map_topic_ = map_topic;
    cached_max_obstacle_distance_ = max_obstacle_distance;
  }

  sensor_msgs::msg::LaserScan scan;
  if (!waitForOneMessage(node, scan_topic, rclcpp::SensorDataQoS(), timeout, scan))
  {
    getBehaviorContext()->logger->publishFailureMessage(
        name(), fmt::format("ScanMatchResidual: no scan on '{}' within {:.1f} s.", scan_topic, timeout));
    return BT::NodeStatus::FAILURE;
  }

  const localization::ScanGeometry geometry{ scan.angle_min, scan.angle_increment, scan.range_min, scan.range_max };
  const double yaw = tf2::getYaw(pose.pose.orientation);
  const auto stats = localization::computeScanResidual(field_, geometry, scan.ranges, max_beams, min_range, max_range,
                                                       inlier_distance, pose.pose.position.x, pose.pose.position.y,
                                                       yaw);

  setOutput(kPortInlierFraction, stats.inlier_fraction);
  setOutput(kPortMedianResidual, stats.median_residual);
  setOutput(kPortBeamsUsed, stats.beams_used);

  if (stats.beams_used == 0)
  {
    getBehaviorContext()->logger->publishFailureMessage(
        name(), fmt::format("ScanMatchResidual: none of the {} beams selected on '{}' survived the range filters "
                            "({:.2f} m to {:.2f} m). Nothing could be measured.",
                            stats.beams_selected, scan_topic, std::max(static_cast<double>(scan.range_min), min_range),
                            std::min(static_cast<double>(scan.range_max), max_range)));
    return BT::NodeStatus::FAILURE;
  }

  getBehaviorContext()->logger->publishInfoMessage(
      name(), fmt::format("Scan match at ({:.3f}, {:.3f}) yaw {:.1f} deg: {:.1f}% of {} usable beams (of {} scored) "
                          "within {:.2f} m of the map, median residual {:.3f} m.",
                          pose.pose.position.x, pose.pose.position.y, yaw * 180.0 / M_PI, stats.inlier_fraction * 100.0,
                          stats.beams_used, stats.beams_selected, inlier_distance, stats.median_residual));

  if (min_inlier_fraction >= 0.0 && stats.inlier_fraction < min_inlier_fraction)
  {
    getBehaviorContext()->logger->publishFailureMessage(
        name(), fmt::format("Pose rejected: only {:.1f}% of beams fit the map, below the required {:.1f}%. This pose "
                            "does not explain the scan.",
                            stats.inlier_fraction * 100.0, min_inlier_fraction * 100.0));
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::SUCCESS;
}

}  // namespace hangar_sim_behaviors
