// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

/**
 * @file
 * @brief Pure geometry behind the in-place localization refinement gate. No ROS, no BT.
 *
 * The refinement loop (see ScanMatchResidual and the "Refine Localization In Place" Objective)
 * accepts or rejects a refined pose on numbers computed here. Keeping them free of ROS is what lets
 * test_localization_gates.cpp hold the acceptance contract: a change that breaks the discrimination
 * fails a test instead of surprising someone in front of a robot.
 *
 * Everything in here mirrors *beluga_amcl*, which is what hangar_sim runs
 * (`localization_launch.py` loads `beluga_amcl::AmclNode`). beluga is not nav2_amcl, and the two
 * differ in three places that matter to a metric gate. They are called out at each site below:
 *   - beam decimation is `beluga::views::take_evenly`, not nav2's integer index step;
 *   - cell lookup floors in the grid's own frame, where nav2's MAP_GXWX rounds to the nearest cell
 *     centre -- a systematic half-cell (0.025 m here) offset if you copy the wrong one;
 *   - a cell is an obstacle at exactly 100, not at ">= occupied_thresh".
 */
namespace hangar_sim_behaviors::localization
{
/**
 * @name Filter mirror
 *
 * These defaults exist so the gate reads the same beams, over the same field, that the filter
 * weighted its particles against. Each one mirrors a key in `hangar_sim/params/nav2_params.yaml`
 * under `amcl:`; if that file changes, these follow it, or the gate is measuring something else.
 * @{
 */
/// Beams the filter scores per update. Mirrors amcl's `max_beams`.
inline constexpr int kMaxBeams = 60;
/// Longest return the filter trusts, metres. Mirrors amcl's `laser_max_range`.
inline constexpr double kLaserMaxRange = 25.0;
/// Shortest return the filter trusts, metres. Mirrors amcl's `laser_min_range`.
inline constexpr double kLaserMinRange = 0.0;
/// Distance at which the likelihood field saturates, metres. Mirrors `laser_likelihood_max_dist`.
inline constexpr double kMaxObstacleDistance = 2.0;
/// Cell value beluga counts as an obstacle. beluga's trinary traits test equality, not a threshold.
inline constexpr std::int8_t kOccupiedValue = 100;
/** @} */

/**
 * @name Acceptance contract
 *
 * MEASURED on hangar_map with this robot's merged scan, using this same scoring code against the
 * grid map_server actually publishes: the true pose scores 87.0%, a pose 0.10 m / 1 deg off scores
 * 69.6%, and the strongest alias found anywhere on the map scores 34.8%. That is a 52-point
 * separation, against the ~21 points meta_ws measured on theirs.
 *
 * Two things about that measurement are worth carrying. Only 23 of the 60 selected beams survive
 * the range filters on this robot, so the fraction moves in steps of about 4.3 points -- the gate
 * is coarse here in a way it was not on meta_ws's denser scan. And every sample came from ONE
 * stationary pose, so "the true pose" is the only true pose measured; a driven multi-pose campaign
 * is still owed. Re-measure with `calibrate_scan_match_gate` whenever the map is rebuilt.
 * @{
 */
/// A beam is an inlier if its endpoint lands this close to an occupied cell, metres.
inline constexpr double kInlierDistance = 0.15;
/**
 * @brief Accept a refined pose only at or above this inlier fraction.
 *
 * 0.60, not the 0.80 meta_ws uses. 0.80 sits only 7 points under the true pose here AND above the
 * 69.6% a 0.10 m error scores, while the refinement itself returns about 6.5 cm -- so 0.80 would
 * reject the refinements this loop legitimately produces. 0.60 still clears the strongest alias on
 * the map by 25 points.
 */
inline constexpr double kMinInlierFraction = 0.60;
/** @} */

/**
 * @name Seed geometry
 *
 * A forced no-motion update cannot move a particle: at zero odometry delta beluga's
 * OmnidirectionalDriveModel derives every noise sigma from the translation and rotation deltas, so
 * all three are identically zero and the sampler returns each state unchanged. The loop therefore
 * *selects* the best pose the seed already drew and can never produce another one. That makes the
 * seed spread both the resolution of the answer and the bound on how far it can land from the click.
 * @{
 */
/// Radius of the click-seeded particle scatter, metres.
inline constexpr double kSeedXyStdDev = 0.5;
/// Heading spread of the seed, radians (~15 degrees).
inline constexpr double kSeedYawStdDev = 0.26;
/// How far, in seed standard deviations, a refinement can possibly move. ~4 sigma at 2000 particles.
inline constexpr double kSeedReachSigmas = 4.0;
/// Reject a refinement that ends further than this from the click, metres.
inline constexpr double kDriftLimit = 1.2;
/// Reject a refinement whose heading ends further than this from the click, radians.
inline constexpr double kYawDriftLimit = 0.52;
/** @} */

/**
 * @brief The furthest a refinement seeded with this spread can possibly land from the click.
 *
 * A drift limit above this can never trip, and one far below it rejects refinements that merely
 * started from a sloppy click. test_localization_gates.cpp holds kDriftLimit inside it.
 */
double seedReachRadius(double xy_std_dev);

/// The pose and size of an occupancy grid, as `nav_msgs/MapMetaData` carries it.
struct GridInfo
{
  int width = 0;
  int height = 0;
  double resolution = 0.0;
  double origin_x = 0.0;
  double origin_y = 0.0;
  /// Yaw of the grid's origin in the map frame. Zero for every map this workspace ships.
  double origin_yaw = 0.0;
};

/**
 * @brief Distance from each cell to the nearest occupied cell, in metres, saturated at max_distance.
 *
 * This is the field beluga builds in `LikelihoodFieldModelBase::make_likelihood_field()`, read one
 * step before it is turned into likelihoods -- same obstacle mask, same propagation, same
 * truncation. Reading it as a metric distance is what makes the gate a fit-to-map test rather than a
 * confidence test.
 */
struct DistanceField
{
  GridInfo info;
  double max_distance = kMaxObstacleDistance;
  /// Row-major, indexed [row * width + column], as an OccupancyGrid's data is.
  std::vector<float> distance;

  /// True once a map has been folded in and the field can be sampled.
  bool valid() const;

  /**
   * @brief Distance from (x, y) in the map frame to the nearest occupied cell.
   *
   * Cell lookup mirrors beluga's `BaseRegularGrid::cell_near`, which takes the point into the grid's
   * own frame and *floors*. nav2_amcl's MAP_GXWX rounds to the nearest cell centre instead; copying
   * that here would bias every endpoint by half a cell (0.025 m at this map's resolution), which is
   * a sixth of the inlier band.
   *
   * Points off the map return max_distance: a pose that throws beams off the map is a bad pose, not
   * an unmeasurable one. beluga makes the same call, scoring an off-grid hit at its floor
   * probability rather than skipping it.
   */
  float at(double x, double y) const;
};

/**
 * @brief Build the distance field from an OccupancyGrid's data.
 *
 * Ports beluga's `nearest_obstacle_distance_map`: a Dijkstra expansion outward from every obstacle
 * cell over the 4-neighbourhood, each cell taking its exact distance to the *nearest obstacle cell
 * found so far along the wavefront*, truncated at max_distance. That truncation also stops the
 * expansion, so cells past max_distance are never visited -- the same shortcut beluga takes, and the
 * reason this is not simply an exact Euclidean transform.
 *
 * @param occupancy Row-major grid data. Cells equal to kOccupiedValue are obstacles; everything
 *                  else, unknown (-1) included, is free. That is beluga's trinary interpretation,
 *                  and unknown counting as free matches `model_unknown_space: false`.
 * @return An empty (invalid) field if the dimensions and data length disagree.
 */
DistanceField buildDistanceField(const GridInfo& info, const std::vector<std::int8_t>& occupancy,
                                 double max_distance = kMaxObstacleDistance);

/**
 * @brief The beam indices beluga would have scored, for a scan of @p range_count returns.
 *
 * Ports `beluga::views::take_evenly`: exactly @p max_beams indices spread over the whole scan,
 * index(k) = ceil(k * (range_count - 1) / (max_beams - 1)), so the last return is always included.
 *
 * This is NOT nav2_amcl's rule. nav2 walks a constant integer step of (range_count - 1) /
 * (max_beams - 1), which both overshoots the requested count and, when the division truncates,
 * leaves the tail of the scan unscored. The two pick different beams from the same scan, so a gate
 * that mirrors the wrong one is scoring beams the filter never looked at.
 */
std::vector<std::size_t> takeEvenlyIndices(std::size_t range_count, std::size_t max_beams);

/// The parts of a LaserScan the residual needs, so the maths stays free of sensor_msgs.
struct ScanGeometry
{
  double angle_min = 0.0;
  double angle_increment = 0.0;
  double range_min = 0.0;
  double range_max = 0.0;
};

struct ResidualStats
{
  /// Fraction of used beams whose endpoint is within inlier_distance of an occupied cell.
  double inlier_fraction = 0.0;
  /// Median endpoint-to-obstacle distance in metres. Diagnostic only -- the gate is the fraction.
  double median_residual = kMaxObstacleDistance;
  /// Beams that survived the range and finiteness filters. Zero means nothing was measured.
  int beams_used = 0;
  /// Beams the decimation selected, before the range filters. beams_used cannot exceed it.
  int beams_selected = 0;
};

/**
 * @brief Score a candidate pose against the map by projecting the scan from it.
 *
 * Beam selection and rejection mirror `beluga_ros::LaserScan` and `beluga::BaseLaserScan`: decimate
 * with takeEvenlyIndices, then drop NaN returns and returns outside
 * [max(scan.range_min, min_range), min(scan.range_max, max_range)]. Note both bounds are inclusive,
 * which is beluga's rule and not nav2's -- nav2 drops a return sitting exactly on the max.
 *
 * @param x,y,yaw The candidate pose in the map frame. The scan is assumed to be expressed in the
 *                robot's own frame at that pose. On this robot `dual_laser_merger` publishes
 *                /scan_merged with `target_frame: ridgeback_base_link`, which is also amcl's
 *                `base_frame_id`, so the laser-to-base transform is the identity and there is
 *                nothing to compose. A scan published in its own frame would need one.
 * @return beams_used == 0 with a zero inlier fraction when nothing could be measured.
 */
ResidualStats computeScanResidual(const DistanceField& field, const ScanGeometry& geometry,
                                  const std::vector<float>& ranges, int max_beams, double min_range, double max_range,
                                  double inlier_distance, double x, double y, double yaw);

}  // namespace hangar_sim_behaviors::localization
