// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include <lab_sim_behaviors/generate_surface_coverage_path.hpp>

#include <algorithm>
#include <cmath>
#include <numbers>
#include <vector>

#include <Eigen/Geometry>
#include <tf2_eigen/tf2_eigen.hpp>

#include <moveit_pro_behavior_interface/get_required_ports.hpp>

namespace lab_sim_behaviors
{
namespace
{
constexpr auto kPortIDRegionPose = "region_pose";
constexpr auto kPortIDRegionDimensions = "region_dimensions";
constexpr auto kPortIDLineSpacing = "line_spacing";
constexpr auto kPortIDPointSpacing = "point_spacing";
constexpr auto kPortIDStandoff = "standoff";
constexpr auto kPortIDMargin = "margin";
constexpr auto kPortIDCoveragePath = "coverage_path";

// Upper bound on generated poses, so a huge region or tiny spacing cannot overwhelm the planner.
constexpr long long kMaxCoveragePoses = 100000;

inline constexpr auto kDescriptionGenerateSurfaceCoveragePath = R"(
                <p>
                    Generates a serpentine (boustrophedon) coverage path of tool poses over the top
                    face of a rectangular region, for a Cartesian planner to follow. Turns a
                    user-selected region (see GetRegionFromUser, projected to 3D) into a
                    wipe/inspection sweep. The tool points into the surface at a configurable
                    standoff, with configurable line and point spacing.
                </p>
            )";

/**
 * @brief Sample one region axis from -half_extent to +half_extent, always including both edges.
 *
 * Spreads the samples evenly rather than stepping by exactly @p spacing and clamping the last one to
 * the edge: when the extent is a near-multiple of the spacing, that clamp leaves the final pair a
 * fraction of a millimeter apart and Cartesian planning rejects the path. The even step is never
 * larger than the requested spacing, so coverage is unchanged.
 */
std::vector<double> axisSamples(double half_extent, double spacing)
{
  const double extent = 2.0 * half_extent;
  // An axis narrower than one spacing gets a single pass down its middle; sampling both edges would
  // put two waypoints a sliver apart, the same degenerate pair the even step exists to avoid.
  if (extent <= spacing)
  {
    return { 0.0 };
  }

  const int intervals = static_cast<int>(std::ceil(extent / spacing));
  const double step = extent / intervals;
  std::vector<double> samples;
  samples.reserve(static_cast<std::size_t>(intervals) + 1);
  for (int i = 0; i <= intervals; ++i)
  {
    samples.push_back(-half_extent + i * step);
  }
  samples.back() = half_extent;  // Exact edge, free of accumulated rounding.
  return samples;
}
}  // namespace

std::vector<geometry_msgs::msg::PoseStamped> generateRasterPath(const geometry_msgs::msg::PoseStamped& region_pose,
                                                                const std::vector<double>& region_dimensions,
                                                                const CoveragePathParams& params)
{
  std::vector<geometry_msgs::msg::PoseStamped> path;
  if (region_dimensions.size() < 3)
  {
    return path;
  }
  // Reject non-finite or out-of-range inputs up front: NaN/inf slips past every `<= 0` test below and
  // reaches the narrowing cast as UB; a negative margin expands past the region and a negative standoff
  // drives poses into the surface.
  if (!std::isfinite(params.line_spacing) || !std::isfinite(params.point_spacing) || !std::isfinite(params.standoff) ||
      !std::isfinite(params.margin) || !std::isfinite(region_dimensions[0]) || !std::isfinite(region_dimensions[1]) ||
      !std::isfinite(region_dimensions[2]))
  {
    return path;
  }
  if (params.line_spacing <= 0.0 || params.point_spacing <= 0.0 || params.margin < 0.0 || params.standoff < 0.0 ||
      region_dimensions[0] <= 0.0 || region_dimensions[1] <= 0.0 || region_dimensions[2] <= 0.0)
  {
    return path;
  }

  const double half_x = region_dimensions[0] / 2.0 - params.margin;
  const double half_y = region_dimensions[1] / 2.0 - params.margin;
  if (half_x <= 0.0 || half_y <= 0.0)
  {
    return path;
  }

  Eigen::Isometry3d region_transform;
  tf2::fromMsg(region_pose.pose, region_transform);

  // Assumes region +Z is the outward surface normal (see GetOrientedBoundingBoxFromPointCloud
  // reference_pose). Point the tool +Z into the surface: 180 deg about region X maps tool +Z to region -Z.
  const Eigen::Matrix3d tool_rotation =
      region_transform.rotation() * Eigen::AngleAxisd(std::numbers::pi, Eigen::Vector3d::UnitX()).toRotationMatrix();
  const Eigen::Quaterniond tool_orientation(tool_rotation);

  // Standoff above the region's top face is the only clearance; no collision object backs the sweep.
  const double local_z = region_dimensions[2] / 2.0 + params.standoff;

  // Compute the counts in double and cap before narrowing: a tiny spacing would otherwise overflow
  // int during the cast, which is undefined behavior. ceil (not floor) so the far edge is always
  // sampled even when the spacing does not evenly divide the extent.
  const double lines_d = std::ceil((2.0 * half_y) / params.line_spacing) + 1.0;
  const double points_d = std::ceil((2.0 * half_x) / params.point_spacing) + 1.0;
  if (lines_d * points_d > static_cast<double>(kMaxCoveragePoses))
  {
    return path;
  }
  const std::vector<double> y_samples = axisSamples(half_y, params.line_spacing);
  const std::vector<double> x_samples = axisSamples(half_x, params.point_spacing);
  const std::size_t num_points = x_samples.size();
  path.reserve(y_samples.size() * num_points);

  for (std::size_t line = 0; line < y_samples.size(); ++line)
  {
    const double y = y_samples[line];
    const bool reverse = (line % 2) == 1;  // serpentine: alternate direction each line.
    for (std::size_t point = 0; point < num_points; ++point)
    {
      const double x = x_samples[reverse ? (num_points - 1 - point) : point];

      geometry_msgs::msg::PoseStamped pose;
      pose.header = region_pose.header;
      const Eigen::Vector3d world_position = region_transform * Eigen::Vector3d(x, y, local_z);
      pose.pose.position.x = world_position.x();
      pose.pose.position.y = world_position.y();
      pose.pose.position.z = world_position.z();
      pose.pose.orientation = tf2::toMsg(tool_orientation);
      path.push_back(pose);
    }
  }
  return path;
}

GenerateSurfaceCoveragePath::GenerateSurfaceCoveragePath(
    const std::string& name, const BT::NodeConfiguration& config,
    const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
  : moveit_pro::behaviors::SharedResourcesNode<BT::SyncActionNode>(name, config, shared_resources)
{
}

BT::PortsList GenerateSurfaceCoveragePath::providedPorts()
{
  return {
    BT::InputPort<geometry_msgs::msg::PoseStamped>(kPortIDRegionPose, "{region_pose}",
                                                   "Center pose and orientation of the region to cover (e.g. the "
                                                   "box_center_pose output of "
                                                   "GetOrientedBoundingBoxFromPointCloud)."),
    BT::InputPort<std::vector<double>>(kPortIDRegionDimensions, "{region_dimensions}",
                                       "Full extents [x, y, z] of the region, in meters, in the region_pose frame."),
    BT::InputPort<double>(kPortIDLineSpacing, 0.05, "Distance between adjacent raster lines, in meters."),
    BT::InputPort<double>(kPortIDPointSpacing, 0.02, "Sampling distance along each raster line, in meters."),
    BT::InputPort<double>(kPortIDStandoff, 0.10,
                          "Tool height above the region's top face, along the region +Z, in meters."),
    BT::InputPort<double>(kPortIDMargin, 0.0, "Inset from the region edges on all sides, in meters."),
    BT::OutputPort<std::vector<geometry_msgs::msg::PoseStamped>>(
        kPortIDCoveragePath, "{coverage_path}", "Ordered tool poses covering the region, for a Cartesian planner."),
  };
}

BT::KeyValueVector GenerateSurfaceCoveragePath::metadata()
{
  return { { "subcategory", "Motion - Planning" }, { "description", kDescriptionGenerateSurfaceCoveragePath } };
}

BT::NodeStatus GenerateSurfaceCoveragePath::tick()
{
  const auto ports = moveit_pro::behaviors::getRequiredInputs(
      getInput<geometry_msgs::msg::PoseStamped>(kPortIDRegionPose),
      getInput<std::vector<double>>(kPortIDRegionDimensions), getInput<double>(kPortIDLineSpacing),
      getInput<double>(kPortIDPointSpacing), getInput<double>(kPortIDStandoff), getInput<double>(kPortIDMargin));
  if (!ports.has_value())
  {
    getBehaviorContext()->logger->publishFailureMessage(
        name(), "Failed to get required values from input data ports: " + ports.error());
    return BT::NodeStatus::FAILURE;
  }
  const auto& [region_pose, region_dimensions, line_spacing, point_spacing, standoff, margin] = ports.value();

  const CoveragePathParams params{
    .line_spacing = line_spacing, .point_spacing = point_spacing, .standoff = standoff, .margin = margin
  };

  const std::vector<geometry_msgs::msg::PoseStamped> path = generateRasterPath(region_pose, region_dimensions, params);
  if (path.empty())
  {
    getBehaviorContext()->logger->publishFailureMessage(
        name(), "Coverage path is empty: check region_dimensions (need 3 entries and a positive area after margin), "
                "that line_spacing / point_spacing are positive, and that the region/spacing does not exceed the "
                "pose cap.");
    return BT::NodeStatus::FAILURE;
  }

  setOutput(kPortIDCoveragePath, path);
  return BT::NodeStatus::SUCCESS;
}
}  // namespace lab_sim_behaviors
