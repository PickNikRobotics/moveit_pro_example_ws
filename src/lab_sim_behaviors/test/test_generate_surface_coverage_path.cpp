// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>

#include <lab_sim_behaviors/generate_surface_coverage_path.hpp>

namespace
{
using ::testing::IsEmpty;
using ::testing::SizeIs;

geometry_msgs::msg::PoseStamped identityRegion()
{
  geometry_msgs::msg::PoseStamped region;
  region.header.frame_id = "world";
  region.pose.orientation.w = 1.0;
  region.pose.position.x = 1.0;
  region.pose.position.y = 2.0;
  region.pose.position.z = 0.5;
  return region;
}
}  // namespace

TEST(GenerateRasterPath, RejectsDegenerateInput)
{
  // GIVEN a valid region but degenerate parameters/dimensions
  const auto region = identityRegion();
  const lab_sim_behaviors::CoveragePathParams params{
    .line_spacing = 0.05, .point_spacing = 0.02, .standoff = 0.10, .margin = 0.0
  };

  // THEN fewer than three dimensions, non-positive spacing, and a margin that consumes the area all
  // produce an empty path
  EXPECT_THAT(lab_sim_behaviors::generateRasterPath(region, { 0.4, 0.2 }, params), IsEmpty());
  EXPECT_THAT(lab_sim_behaviors::generateRasterPath(
                  region, { 0.4, 0.2, 0.05 },
                  { .line_spacing = 0.0, .point_spacing = 0.02, .standoff = 0.1, .margin = 0.0 }),
              IsEmpty());
  EXPECT_THAT(lab_sim_behaviors::generateRasterPath(
                  region, { 0.4, 0.2, 0.05 },
                  { .line_spacing = 0.05, .point_spacing = 0.02, .standoff = 0.1, .margin = 0.3 }),
              IsEmpty());
}

TEST(GenerateRasterPath, RejectsNonFiniteParameters)
{
  // GIVEN a valid region but a NaN line spacing (which would slip past the <= 0 guards)
  const auto region = identityRegion();
  const lab_sim_behaviors::CoveragePathParams params{
    .line_spacing = std::numeric_limits<double>::quiet_NaN(), .point_spacing = 0.02, .standoff = 0.10, .margin = 0.0
  };

  // THEN it is rejected as empty rather than reaching the narrowing cast as undefined behavior
  EXPECT_THAT(lab_sim_behaviors::generateRasterPath(region, { 0.4, 0.2, 0.05 }, params), IsEmpty());
}

TEST(GenerateRasterPath, RejectsNonFiniteDimensionsAndNegativeClearance)
{
  // GIVEN an otherwise valid region
  const auto region = identityRegion();
  const lab_sim_behaviors::CoveragePathParams good{
    .line_spacing = 0.05, .point_spacing = 0.02, .standoff = 0.10, .margin = 0.0
  };
  const double nan = std::numeric_limits<double>::quiet_NaN();

  // THEN a non-finite dimension (would reach the narrowing cast as UB), a negative margin (expands
  // past the region), and a negative standoff (drives poses into the surface) each produce empty
  EXPECT_THAT(lab_sim_behaviors::generateRasterPath(region, { nan, 0.2, 0.05 }, good), IsEmpty());
  EXPECT_THAT(lab_sim_behaviors::generateRasterPath(region, { 0.4, 0.2, nan }, good), IsEmpty());
  EXPECT_THAT(lab_sim_behaviors::generateRasterPath(
                  region, { 0.4, 0.2, 0.05 },
                  { .line_spacing = 0.05, .point_spacing = 0.02, .standoff = 0.1, .margin = -0.05 }),
              IsEmpty());
  EXPECT_THAT(lab_sim_behaviors::generateRasterPath(
                  region, { 0.4, 0.2, 0.05 },
                  { .line_spacing = 0.05, .point_spacing = 0.02, .standoff = -0.05, .margin = 0.0 }),
              IsEmpty());
}

TEST(GenerateRasterPath, CoversFarEdgeWhenSpacingDoesNotDivideEvenly)
{
  // GIVEN a 0.4 x 0.2 region whose extents are NOT evenly divisible by a 0.03 spacing
  const auto region = identityRegion();
  const lab_sim_behaviors::CoveragePathParams params{
    .line_spacing = 0.03, .point_spacing = 0.03, .standoff = 0.10, .margin = 0.0
  };

  // WHEN generating the raster
  const auto path = lab_sim_behaviors::generateRasterPath(region, { 0.4, 0.2, 0.05 }, params);

  // THEN both far edges are still reached (region center is world (1, 2)): X spans [0.8, 1.2] and
  // Y spans [1.9, 2.1], instead of stopping a fraction short of the +edge as floor() would.
  ASSERT_FALSE(path.empty());
  double min_x = path.front().pose.position.x;
  double max_x = min_x;
  double min_y = path.front().pose.position.y;
  double max_y = min_y;
  for (const auto& pose : path)
  {
    min_x = std::min(min_x, pose.pose.position.x);
    max_x = std::max(max_x, pose.pose.position.x);
    min_y = std::min(min_y, pose.pose.position.y);
    max_y = std::max(max_y, pose.pose.position.y);
  }
  EXPECT_NEAR(min_x, 0.8, 1e-6);
  EXPECT_NEAR(max_x, 1.2, 1e-6);
  EXPECT_NEAR(min_y, 1.9, 1e-6);
  EXPECT_NEAR(max_y, 2.1, 1e-6);

  // AND no gap between consecutive samples exceeds the requested spacing (each serpentine step is one
  // point_spacing in X or one line_spacing in Y; the clamped edge step is shorter, never longer).
  for (std::size_t i = 1; i < path.size(); ++i)
  {
    EXPECT_LE(std::abs(path[i].pose.position.x - path[i - 1].pose.position.x), params.point_spacing + 1e-9);
    EXPECT_LE(std::abs(path[i].pose.position.y - path[i - 1].pose.position.y), params.line_spacing + 1e-9);
  }
}

TEST(GenerateRasterPath, RejectsAnExcessivePoseCount)
{
  // GIVEN a huge region with tiny spacing that would generate an unbounded path
  const auto region = identityRegion();
  const lab_sim_behaviors::CoveragePathParams params{
    .line_spacing = 0.0001, .point_spacing = 0.0001, .standoff = 0.10, .margin = 0.0
  };

  // THEN it is rejected as empty rather than exploding past the pose cap
  EXPECT_THAT(lab_sim_behaviors::generateRasterPath(region, { 100.0, 100.0, 0.05 }, params), IsEmpty());
}

TEST(GenerateRasterPath, CoversTheRegionWithTheExpectedGrid)
{
  // GIVEN a 0.4 x 0.2 region and 0.05 line / 0.02 point spacing
  const auto region = identityRegion();
  const lab_sim_behaviors::CoveragePathParams params{
    .line_spacing = 0.05, .point_spacing = 0.02, .standoff = 0.10, .margin = 0.0
  };

  // WHEN generating the raster
  const auto path = lab_sim_behaviors::generateRasterPath(region, { 0.4, 0.2, 0.05 }, params);

  // THEN it is a full grid: floor(0.2/0.05)+1 = 5 lines x floor(0.4/0.02)+1 = 21 points
  ASSERT_THAT(path, SizeIs(105));
  // AND every pose inherits the region frame
  EXPECT_EQ(path.front().header.frame_id, "world");
  // AND poses sit a standoff above the top face: z = region_z + dz/2 + standoff = 0.5 + 0.025 + 0.1
  EXPECT_NEAR(path.front().pose.position.z, 0.625, 1e-6);
}

TEST(GenerateRasterPath, LaysOutInTheRegionFrameNotWorldAxes)
{
  // GIVEN the same 0.4 x 0.2 region, but rotated +90 deg about world Z so region local +X maps to
  // world +Y and region local +Y maps to world -X (standoff stays along world Z)
  auto region = identityRegion();
  region.pose.orientation.z = std::sqrt(2.0) / 2.0;
  region.pose.orientation.w = std::sqrt(2.0) / 2.0;
  const lab_sim_behaviors::CoveragePathParams params{
    .line_spacing = 0.05, .point_spacing = 0.02, .standoff = 0.10, .margin = 0.0
  };

  // WHEN generating the raster
  const auto path = lab_sim_behaviors::generateRasterPath(region, { 0.4, 0.2, 0.05 }, params);

  // THEN line 0 start maps through the region rotation: local (-0.2, -0.1) -> world delta (0.1, -0.2)
  // (a world-axis layout would instead give x = 0.8, y = 1.9 as in the identity case)
  ASSERT_THAT(path, SizeIs(105));
  EXPECT_NEAR(path[0].pose.position.x, 1.1, 1e-6);
  EXPECT_NEAR(path[0].pose.position.y, 1.8, 1e-6);
  // AND the standoff is still along world Z, unchanged by a rotation about Z
  EXPECT_NEAR(path[0].pose.position.z, 0.625, 1e-6);
}

TEST(GenerateRasterPath, IsSerpentineAndPointsToolIntoSurface)
{
  // GIVEN the same region
  const auto region = identityRegion();
  const lab_sim_behaviors::CoveragePathParams params{
    .line_spacing = 0.05, .point_spacing = 0.02, .standoff = 0.10, .margin = 0.0
  };

  // WHEN generating the raster
  const auto path = lab_sim_behaviors::generateRasterPath(region, { 0.4, 0.2, 0.05 }, params);

  ASSERT_THAT(path, SizeIs(105));
  // THEN line 0 starts at the -X edge (1.0 - 0.2)...
  EXPECT_NEAR(path[0].pose.position.x, 0.8, 1e-6);
  // ...and line 1 reverses, starting at the +X edge (1.0 + 0.2)
  EXPECT_NEAR(path[21].pose.position.x, 1.2, 1e-6);
  // AND the tool is flipped 180 deg about X (points +Z into the surface): quaternion (1, 0, 0, 0)
  EXPECT_NEAR(path[0].pose.orientation.x, 1.0, 1e-6);
  EXPECT_NEAR(path[0].pose.orientation.w, 0.0, 1e-6);
}
