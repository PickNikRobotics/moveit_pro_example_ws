// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include <gtest/gtest.h>

#include <hangar_sim_behaviors/localization_gates.hpp>

#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

/**
 * @file
 * @brief Holds the acceptance contract for the in-place localization refinement gate.
 *
 * Three groups of tests, and the third is the one worth reading:
 *  - the beluga mirrors (beam decimation, cell lookup, obstacle test), each pinned against the
 *    nav2_amcl rule it is deliberately NOT;
 *  - the residual itself, on a synthetic room;
 *  - one deliberately-failing-by-design case: a perfectly periodic corridor, where a pose one bay
 *    along scores exactly as well as the true one. That is a real limit of any fit-to-map test, and
 *    it is why the click and the drift limit exist. If someone ever "fixes" it, they have almost
 *    certainly widened the search and given back the wrong-aisle failure the click was designed out.
 */

namespace
{
using hangar_sim_behaviors::localization::buildDistanceField;
using hangar_sim_behaviors::localization::computeScanResidual;
using hangar_sim_behaviors::localization::GridInfo;
using hangar_sim_behaviors::localization::kMinInlierFraction;
using hangar_sim_behaviors::localization::ScanGeometry;
using hangar_sim_behaviors::localization::takeEvenlyIndices;

constexpr double kResolution = 0.05;
constexpr std::int8_t kFree = 0;
constexpr std::int8_t kOccupied = 100;
constexpr std::int8_t kUnknown = -1;

/// A blank grid with an origin at (0, 0), matching how map_server lays a map out.
struct TestGrid
{
  GridInfo info;
  std::vector<std::int8_t> data;

  TestGrid(int width, int height, double resolution = kResolution)
    : info{ width, height, resolution, 0.0, 0.0, 0.0 }
    , data(static_cast<std::size_t>(width) * static_cast<std::size_t>(height), kFree)
  {
  }

  void set(int column, int row, std::int8_t value)
  {
    data[static_cast<std::size_t>(row) * static_cast<std::size_t>(info.width) + static_cast<std::size_t>(column)] =
        value;
  }

  /// Draw the four walls of a rectangular room, inclusive of the given cell bounds.
  void addRoom(int min_column, int min_row, int max_column, int max_row)
  {
    for (int column = min_column; column <= max_column; ++column)
    {
      set(column, min_row, kOccupied);
      set(column, max_row, kOccupied);
    }
    for (int row = min_row; row <= max_row; ++row)
    {
      set(min_column, row, kOccupied);
      set(max_column, row, kOccupied);
    }
  }

  /// Is the cell containing this map-frame point an obstacle? Uses beluga's flooring convention.
  bool occupiedAt(double x, double y) const
  {
    const int column = static_cast<int>(std::floor(x / info.resolution));
    const int row = static_cast<int>(std::floor(y / info.resolution));
    if (column < 0 || column >= info.width || row < 0 || row >= info.height)
    {
      return true;  // Treat the outside of the grid as a wall so a ray always terminates.
    }
    return data[static_cast<std::size_t>(row) * static_cast<std::size_t>(info.width) + static_cast<std::size_t>(column)] ==
           kOccupied;
  }
};

/**
 * @brief Synthesize the scan a perfect lidar at (x, y, yaw) would see in this grid.
 *
 * Deliberately crude -- a fixed-step march, not a DDA -- because its only job is to put endpoints on
 * walls so the residual has something honest to score. A no-return beam comes back as infinity, the
 * way dual_laser_merger reports one with `use_inf: true`.
 */
std::vector<float> raycast(const TestGrid& grid, double x, double y, double yaw, const ScanGeometry& geometry,
                           std::size_t beam_count)
{
  constexpr double kMarchStep = 0.01;
  std::vector<float> ranges;
  ranges.reserve(beam_count);
  for (std::size_t beam = 0; beam < beam_count; ++beam)
  {
    const double bearing = yaw + geometry.angle_min + static_cast<double>(beam) * geometry.angle_increment;
    const double step_x = std::cos(bearing) * kMarchStep;
    const double step_y = std::sin(bearing) * kMarchStep;
    double probe_x = x;
    double probe_y = y;
    double range = 0.0;
    bool hit = false;
    while (range < geometry.range_max)
    {
      probe_x += step_x;
      probe_y += step_y;
      range += kMarchStep;
      if (grid.occupiedAt(probe_x, probe_y))
      {
        hit = true;
        break;
      }
    }
    ranges.push_back(hit ? static_cast<float>(range) : std::numeric_limits<float>::infinity());
  }
  return ranges;
}

ScanGeometry fullCircleGeometry(std::size_t beam_count, double range_max)
{
  ScanGeometry geometry;
  geometry.angle_min = -M_PI;
  geometry.angle_increment = 2.0 * M_PI / static_cast<double>(beam_count);
  geometry.range_min = 0.05;
  geometry.range_max = range_max;
  return geometry;
}
}  // namespace

// ---------------------------------------------------------------------------
// Mirror 1: beam decimation is beluga's take_evenly, not nav2's integer step.
// ---------------------------------------------------------------------------

TEST(BeamSelection, ReturnsExactlyMaxBeamsAndAlwaysScoresBothEnds)
{
  // The real numbers: dual_laser_merger emits 722 bins over 2*pi at 0.0087 rad, and amcl's
  // max_beams is 60.
  const auto indices = takeEvenlyIndices(722, 60);
  ASSERT_EQ(indices.size(), 60U);
  EXPECT_EQ(indices.front(), 0U);
  EXPECT_EQ(indices.back(), 721U) << "beluga always includes the final return; a gate that stops short "
                                     "leaves a wedge of the scan unscored.";
  for (std::size_t i = 1; i < indices.size(); ++i)
  {
    EXPECT_GT(indices[i], indices[i - 1]) << "indices must be strictly increasing so no beam is scored twice";
  }
}

TEST(BeamSelection, DiffersFromNav2IndexStepRule)
{
  // nav2_amcl walks a constant integer step of (range_count - 1) / (max_beams - 1), truncated.
  // For 722 returns and 60 beams that is 12, giving 61 beams that stop at index 720.
  const std::size_t nav2_step = (722 - 1) / (60 - 1);
  ASSERT_EQ(nav2_step, 12U);
  std::vector<std::size_t> nav2_indices;
  for (std::size_t i = 0; i < 722; i += nav2_step)
  {
    nav2_indices.push_back(i);
  }
  EXPECT_EQ(nav2_indices.size(), 61U);
  EXPECT_EQ(nav2_indices.back(), 720U);

  const auto beluga_indices = takeEvenlyIndices(722, 60);
  EXPECT_NE(beluga_indices, nav2_indices) << "The two rules pick different beams from the same scan. Mirroring the "
                                             "wrong one scores beams this filter never looked at.";
}

TEST(BeamSelection, HandsBackEverythingWhenAskedForMoreBeamsThanExist)
{
  const auto indices = takeEvenlyIndices(5, 60);
  ASSERT_EQ(indices.size(), 5U);
  EXPECT_EQ(indices.back(), 4U);
}

TEST(BeamSelection, EmptyScanSelectsNothing)
{
  EXPECT_TRUE(takeEvenlyIndices(0, 60).empty());
  EXPECT_TRUE(takeEvenlyIndices(722, 0).empty());
}

// ---------------------------------------------------------------------------
// Mirror 2: the distance field is beluga's, including what counts as an obstacle.
// ---------------------------------------------------------------------------

TEST(DistanceField, MeasuresDistanceFromASingleObstacle)
{
  TestGrid grid(40, 40);
  grid.set(20, 20, kOccupied);
  const auto field = buildDistanceField(grid.info, grid.data, 2.0);
  ASSERT_TRUE(field.valid());

  // Inside the obstacle cell itself.
  EXPECT_NEAR(field.at(20.5 * kResolution, 20.5 * kResolution), 0.0, 1e-6);
  // One cell away along x is exactly one resolution.
  EXPECT_NEAR(field.at(21.5 * kResolution, 20.5 * kResolution), kResolution, 1e-5);
  // The diagonal neighbour is resolution * sqrt(2).
  EXPECT_NEAR(field.at(21.5 * kResolution, 21.5 * kResolution), kResolution * std::sqrt(2.0), 1e-5);
}

TEST(DistanceField, SaturatesAtMaxObstacleDistance)
{
  TestGrid grid(200, 200);
  grid.set(0, 0, kOccupied);
  const auto field = buildDistanceField(grid.info, grid.data, 2.0);
  ASSERT_TRUE(field.valid());
  // Far corner is metres away; the field is truncated, and so is the wavefront that built it.
  EXPECT_NEAR(field.at(150.5 * kResolution, 150.5 * kResolution), 2.0, 1e-6);
}

TEST(DistanceField, OffTheMapIsMaximallyBad)
{
  TestGrid grid(40, 40);
  grid.set(20, 20, kOccupied);
  const auto field = buildDistanceField(grid.info, grid.data, 2.0);
  ASSERT_TRUE(field.valid());
  EXPECT_NEAR(field.at(-1.0, -1.0), 2.0, 1e-6) << "A pose that throws beams off the map is a bad pose, not an "
                                                  "unmeasurable one.";
  EXPECT_NEAR(field.at(1000.0, 1000.0), 2.0, 1e-6);
}

TEST(DistanceField, UnknownCellsCountAsFree)
{
  // beluga runs with model_unknown_space false, so unknown space is not an obstacle and does not
  // attract beams. A gate that treated -1 as occupied would flatter every pose near a map edge.
  TestGrid grid(40, 40);
  grid.set(20, 20, kUnknown);
  const auto field = buildDistanceField(grid.info, grid.data, 2.0);
  ASSERT_TRUE(field.valid());
  EXPECT_NEAR(field.at(20.5 * kResolution, 20.5 * kResolution), 2.0, 1e-6);
}

TEST(DistanceField, ObstacleTestIsEqualityNotThreshold)
{
  // beluga's trinary ValueTraits test `value == 100`. nav2's map loader tests
  // `value >= occupied_thresh`, which for a 0.65 threshold would call 90 an obstacle. On a
  // `mode: trinary` map -- which hangar_map.yaml asks for -- map_server has already collapsed every
  // cell to 0 / 100 / -1, so the two agree. This test pins which rule we mirror, so a future map in
  // `scale` mode fails here rather than silently scoring against a field the filter does not use.
  TestGrid grid(40, 40);
  grid.set(20, 20, static_cast<std::int8_t>(90));
  const auto field = buildDistanceField(grid.info, grid.data, 2.0);
  ASSERT_TRUE(field.valid());
  EXPECT_NEAR(field.at(20.5 * kResolution, 20.5 * kResolution), 2.0, 1e-6);
}

TEST(DistanceField, CellLookupFloorsRatherThanRoundingToTheNearestCentre)
{
  // beluga's cell_near floors local/resolution; nav2's MAP_GXWX rounds to the nearest cell centre.
  // A point 0.6 of a cell past a cell's lower edge belongs to that cell under beluga and to the NEXT
  // one under nav2. Copying nav2's rule would bias every endpoint by half a cell -- 0.025 m here,
  // a sixth of the 0.15 m inlier band.
  TestGrid grid(40, 40);
  grid.set(20, 20, kOccupied);
  const auto field = buildDistanceField(grid.info, grid.data, 2.0);
  ASSERT_TRUE(field.valid());

  const double x = 20.6 * kResolution;
  const double y = 20.6 * kResolution;
  EXPECT_NEAR(field.at(x, y), 0.0, 1e-6) << "flooring puts this point inside cell 20, which is the obstacle";

  const int nav2_column = static_cast<int>(std::floor(x / kResolution + 0.5));
  EXPECT_EQ(nav2_column, 21) << "and nav2's rounding would have put it in cell 21, which is not";
}

TEST(DistanceField, RejectsAMalformedGrid)
{
  GridInfo info{ 10, 10, kResolution, 0.0, 0.0, 0.0 };
  const std::vector<std::int8_t> too_short(50, kFree);
  EXPECT_FALSE(buildDistanceField(info, too_short, 2.0).valid());

  const GridInfo zero_resolution{ 10, 10, 0.0, 0.0, 0.0, 0.0 };
  EXPECT_FALSE(buildDistanceField(zero_resolution, std::vector<std::int8_t>(100, kFree), 2.0).valid());
}

// ---------------------------------------------------------------------------
// The residual, on a synthetic room.
// ---------------------------------------------------------------------------

TEST(ScanResidual, TruePoseFitsTheMapAlmostPerfectly)
{
  TestGrid grid(200, 200);
  grid.addRoom(10, 10, 189, 189);
  const auto field = buildDistanceField(grid.info, grid.data, 2.0);
  ASSERT_TRUE(field.valid());

  constexpr std::size_t kBeamCount = 722;
  const auto geometry = fullCircleGeometry(kBeamCount, 25.0);
  const double x = 100.5 * kResolution;
  const double y = 100.5 * kResolution;
  const auto ranges = raycast(grid, x, y, 0.0, geometry, kBeamCount);

  const auto stats = computeScanResidual(field, geometry, ranges, 60, 0.0, 25.0, 0.15, x, y, 0.0);
  EXPECT_EQ(stats.beams_selected, 60);
  EXPECT_GT(stats.beams_used, 50);
  EXPECT_GT(stats.inlier_fraction, 0.99);
  EXPECT_LT(stats.median_residual, 0.05);
}

TEST(ScanResidual, ADisplacedPoseFitsTheMapWorse)
{
  TestGrid grid(200, 200);
  grid.addRoom(10, 10, 189, 189);
  const auto field = buildDistanceField(grid.info, grid.data, 2.0);
  ASSERT_TRUE(field.valid());

  constexpr std::size_t kBeamCount = 722;
  const auto geometry = fullCircleGeometry(kBeamCount, 25.0);
  const double x = 100.5 * kResolution;
  const double y = 100.5 * kResolution;
  const auto ranges = raycast(grid, x, y, 0.0, geometry, kBeamCount);

  const auto truth = computeScanResidual(field, geometry, ranges, 60, 0.0, 25.0, 0.15, x, y, 0.0);
  // Half a metre out: ten times the inlier band, and a distance a careless click easily produces.
  const auto displaced = computeScanResidual(field, geometry, ranges, 60, 0.0, 25.0, 0.15, x + 0.5, y, 0.0);

  EXPECT_LT(displaced.inlier_fraction, truth.inlier_fraction);
  EXPECT_LT(displaced.inlier_fraction, kMinInlierFraction) << "a 0.5 m error must not clear the shipped threshold";
}

TEST(ScanResidual, ARotatedPoseFitsTheMapWorse)
{
  TestGrid grid(200, 200);
  grid.addRoom(10, 10, 189, 189);
  const auto field = buildDistanceField(grid.info, grid.data, 2.0);
  ASSERT_TRUE(field.valid());

  constexpr std::size_t kBeamCount = 722;
  const auto geometry = fullCircleGeometry(kBeamCount, 25.0);
  const double x = 60.5 * kResolution;
  const double y = 100.5 * kResolution;
  const auto ranges = raycast(grid, x, y, 0.0, geometry, kBeamCount);

  const auto truth = computeScanResidual(field, geometry, ranges, 60, 0.0, 25.0, 0.15, x, y, 0.0);
  const auto rotated = computeScanResidual(field, geometry, ranges, 60, 0.0, 25.0, 0.15, x, y, 0.15);
  EXPECT_LT(rotated.inlier_fraction, truth.inlier_fraction);
}

TEST(ScanResidual, DropsNonFiniteAndOutOfRangeReturns)
{
  TestGrid grid(200, 200);
  grid.addRoom(10, 10, 189, 189);
  const auto field = buildDistanceField(grid.info, grid.data, 2.0);
  ASSERT_TRUE(field.valid());

  const auto geometry = fullCircleGeometry(60, 25.0);
  std::vector<float> ranges(60, std::numeric_limits<float>::infinity());
  auto stats = computeScanResidual(field, geometry, ranges, 60, 0.0, 25.0, 0.15, 5.0, 5.0, 0.0);
  EXPECT_EQ(stats.beams_used, 0) << "infinite returns are no-returns and must not be scored";
  EXPECT_EQ(stats.beams_selected, 60);
  EXPECT_DOUBLE_EQ(stats.inlier_fraction, 0.0);

  ranges.assign(60, std::numeric_limits<float>::quiet_NaN());
  stats = computeScanResidual(field, geometry, ranges, 60, 0.0, 25.0, 0.15, 5.0, 5.0, 0.0);
  EXPECT_EQ(stats.beams_used, 0);

  // Below the scan's own range_min, so beluga drops them too.
  ranges.assign(60, 0.01F);
  stats = computeScanResidual(field, geometry, ranges, 60, 0.0, 25.0, 0.15, 5.0, 5.0, 0.0);
  EXPECT_EQ(stats.beams_used, 0);
}

TEST(ScanResidual, KeepsAReturnSittingExactlyOnTheMaxRange)
{
  // beluga's filter is `range <= max_range`, inclusive. nav2 drops a return at exactly the max.
  // The difference is one beam, but it is the rule we claim to mirror.
  TestGrid grid(200, 200);
  grid.addRoom(10, 10, 189, 189);
  const auto field = buildDistanceField(grid.info, grid.data, 2.0);
  ASSERT_TRUE(field.valid());

  auto geometry = fullCircleGeometry(4, 5.0);
  const std::vector<float> ranges(4, 5.0F);
  const auto stats = computeScanResidual(field, geometry, ranges, 4, 0.0, 5.0, 0.15, 5.0, 5.0, 0.0);
  EXPECT_EQ(stats.beams_used, 4);
}

TEST(ScanResidual, AnInvalidFieldMeasuresNothing)
{
  const auto geometry = fullCircleGeometry(60, 25.0);
  const std::vector<float> ranges(60, 3.0F);
  const auto stats = computeScanResidual({}, geometry, ranges, 60, 0.0, 25.0, 0.15, 1.0, 1.0, 0.0);
  EXPECT_EQ(stats.beams_used, 0);
  EXPECT_DOUBLE_EQ(stats.inlier_fraction, 0.0);
}

// ---------------------------------------------------------------------------
// Seed geometry.
// ---------------------------------------------------------------------------

TEST(SeedGeometry, DriftLimitSitsInsideWhatTheSeedCanReach)
{
  namespace localization = hangar_sim_behaviors::localization;
  const double reach = localization::seedReachRadius(localization::kSeedXyStdDev);
  EXPECT_GT(reach, localization::kDriftLimit) << "A drift limit above the seed's reach can never trip, so it would "
                                                 "be a comment rather than a gate.";
  EXPECT_LT(localization::kDriftLimit, reach);
  EXPECT_GT(localization::kDriftLimit, localization::kSeedXyStdDev) << "and one below one sigma would reject "
                                                                       "refinements that merely started from a "
                                                                       "sloppy click";
}

// ---------------------------------------------------------------------------
// The honest limit. This case is expected to be indistinguishable, on purpose.
// ---------------------------------------------------------------------------

TEST(ScanResidual, APerfectlyPeriodicCorridorDefeatsTheResidual)
{
  // A corridor whose bays repeat exactly. A pose one full bay along sees an identical scan, so the
  // residual scores it just as well as the truth -- no fit-to-map test can separate them, because
  // there is nothing to separate. This is not a bug to fix: it is the reason the refinement is
  // seeded from an operator's click and bounded by a drift limit instead of searching the map.
  constexpr int kBayCells = 40;  // 2.0 m at 0.05 m/cell
  TestGrid grid(400, 60);
  for (int column = 0; column < 400; ++column)
  {
    grid.set(column, 10, kOccupied);
    grid.set(column, 49, kOccupied);
  }
  // Identical pillars, one per bay, on both walls.
  for (int column = 0; column < 400; column += kBayCells)
  {
    for (int row = 11; row < 15; ++row)
    {
      grid.set(column, row, kOccupied);
    }
    for (int row = 45; row < 49; ++row)
    {
      grid.set(column, row, kOccupied);
    }
  }
  const auto field = buildDistanceField(grid.info, grid.data, 2.0);
  ASSERT_TRUE(field.valid());

  constexpr std::size_t kBeamCount = 722;
  const auto geometry = fullCircleGeometry(kBeamCount, 25.0);
  const double x = 200.5 * kResolution;
  const double y = 30.5 * kResolution;
  const auto ranges = raycast(grid, x, y, 0.0, geometry, kBeamCount);

  const auto truth = computeScanResidual(field, geometry, ranges, 60, 0.0, 25.0, 0.15, x, y, 0.0);
  const double one_bay = kBayCells * kResolution;
  const auto alias = computeScanResidual(field, geometry, ranges, 60, 0.0, 25.0, 0.15, x + one_bay, y, 0.0);

  EXPECT_GT(truth.inlier_fraction, 0.9);
  EXPECT_NEAR(alias.inlier_fraction, truth.inlier_fraction, 0.05)
      << "FAILING HERE MEANS THE TEST GEOMETRY CHANGED, NOT THAT THE GATE IMPROVED. A periodic corridor is "
         "genuinely ambiguous; if a change makes the alias separable here, check it has not done so by "
         "widening the search, which is the failure mode the click exists to prevent.";
}
