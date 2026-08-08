// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <limits>

#include <cv_bridge/cv_bridge.hpp>

#include <lab_sim_behaviors/get_mask2d_from_region.hpp>

namespace
{
// Distinct width/height so a swapped x/y denormalization would be caught.
constexpr int kImageWidth = 1000;
constexpr int kImageHeight = 800;

geometry_msgs::msg::Point32 point(float x, float y)
{
  geometry_msgs::msg::Point32 p;
  p.x = x;
  p.y = y;
  return p;
}

// Axis-aligned box region in NORMALIZED [0..1] coordinates (as GetRegionFromUser returns),
// from (0.125, 0.25) to (0.5, 0.75), in the "camera" frame. Exact binary fractions so the
// denormalization to pixels is free of float rounding.
geometry_msgs::msg::PolygonStamped boxRegion()
{
  geometry_msgs::msg::PolygonStamped region;
  region.header.frame_id = "camera";
  region.polygon.points = { point(0.125f, 0.25f), point(0.5f, 0.25f), point(0.5f, 0.75f), point(0.125f, 0.75f) };
  return region;
}

// Concave region: a downward V-notch cut into the +y edge, apex at (0.5, 0.5). The apex sits inside
// the convex hull, so a point past it toward the +y edge is inside the hull but outside the true
// polygon — which separates cv::fillPoly (correct) from cv::fillConvexPoly (fills the whole hull).
geometry_msgs::msg::PolygonStamped concaveRegion()
{
  geometry_msgs::msg::PolygonStamped region;
  region.header.frame_id = "camera";
  region.polygon.points = { point(0.2f, 0.2f), point(0.8f, 0.2f), point(0.8f, 0.8f), point(0.5f, 0.5f),
                            point(0.2f, 0.8f) };
  return region;
}
}  // namespace

TEST(RegionToMask2D, RejectsFewerThanThreePoints)
{
  // GIVEN a region with only two points
  geometry_msgs::msg::PolygonStamped region;
  region.polygon.points = { point(0.0f, 0.0f), point(0.1f, 0.1f) };

  // THEN rasterization fails with a descriptive error
  const auto mask = lab_sim_behaviors::regionToMask2D(region, kImageWidth, kImageHeight);
  ASSERT_FALSE(mask.has_value());
  EXPECT_THAT(mask.error(), ::testing::HasSubstr("at least 3 points"));
}

TEST(RegionToMask2D, RejectsNonPositiveImageDimensions)
{
  // GIVEN a valid region
  const auto region = boxRegion();

  // THEN rasterization fails when either dimension is non-positive (both operands probed)
  const auto zero_width = lab_sim_behaviors::regionToMask2D(region, 0, kImageHeight);
  ASSERT_FALSE(zero_width.has_value());
  EXPECT_THAT(zero_width.error(), ::testing::HasSubstr("dimensions must be positive"));

  const auto zero_height = lab_sim_behaviors::regionToMask2D(region, kImageWidth, 0);
  ASSERT_FALSE(zero_height.has_value());
  EXPECT_THAT(zero_height.error(), ::testing::HasSubstr("dimensions must be positive"));
}

TEST(RegionToMask2D, RejectsNonFiniteOrOutOfRangeVertices)
{
  // GIVEN a region with a NaN vertex, and one with a vertex outside the normalized [0, 1] range
  geometry_msgs::msg::PolygonStamped nan_region;
  nan_region.polygon.points = { point(std::numeric_limits<float>::quiet_NaN(), 0.2f), point(0.4f, 0.2f),
                                point(0.4f, 0.6f) };
  geometry_msgs::msg::PolygonStamped out_of_range;
  out_of_range.polygon.points = { point(0.1f, 0.2f), point(1.5f, 0.2f), point(0.4f, 0.6f) };

  // THEN both are rejected before reaching the float->int casts (UB) or producing a huge mask
  const auto nan_mask = lab_sim_behaviors::regionToMask2D(nan_region, kImageWidth, kImageHeight);
  ASSERT_FALSE(nan_mask.has_value());
  EXPECT_THAT(nan_mask.error(), ::testing::HasSubstr("finite and within [0, 1]"));

  const auto oor_mask = lab_sim_behaviors::regionToMask2D(out_of_range, kImageWidth, kImageHeight);
  ASSERT_FALSE(oor_mask.has_value());
  EXPECT_THAT(oor_mask.error(), ::testing::HasSubstr("finite and within [0, 1]"));
}

TEST(RegionToMask2D, RejectsCollinearPointsWithZeroAreaBoundingBox)
{
  // GIVEN three collinear (constant-y) normalized points — a degenerate, zero-height box
  geometry_msgs::msg::PolygonStamped region;
  region.polygon.points = { point(0.1f, 0.2f), point(0.3f, 0.2f), point(0.5f, 0.2f) };

  // THEN rasterization fails because the bounding box has zero area
  const auto mask = lab_sim_behaviors::regionToMask2D(region, kImageWidth, kImageHeight);
  ASSERT_FALSE(mask.has_value());
  EXPECT_THAT(mask.error(), ::testing::HasSubstr("zero area"));
}

TEST(RegionToMask2D, DenormalizesToPixelsWithOffsetAndFill)
{
  // GIVEN a normalized box (0.125, 0.25)-(0.5, 0.75) and a 1000x800 image
  const auto region = boxRegion();

  // WHEN rasterizing it
  const auto mask = lab_sim_behaviors::regionToMask2D(region, kImageWidth, kImageHeight);

  // THEN the mask is denormalized to pixels: origin (0.125*1000, 0.25*800) = (125, 200),
  // size (0.375*1000, 0.5*800) = (375, 400). (Treating the input as raw pixels would give a ~1x1 mask.)
  ASSERT_TRUE(mask.has_value()) << mask.error();
  EXPECT_EQ(mask->x, 125);
  EXPECT_EQ(mask->y, 200);
  EXPECT_EQ(mask->pixels.width, 375u);
  EXPECT_EQ(mask->pixels.height, 400u);
  EXPECT_EQ(mask->pixels.encoding, "mono8");
  EXPECT_EQ(mask->pixels.header.frame_id, "camera");

  // AND the interior is filled (non-zero)
  const cv_bridge::CvImagePtr image = cv_bridge::toCvCopy(mask->pixels, "mono8");
  EXPECT_GT(image->image.at<uint8_t>(200, 180), 0);  // local (row 200, col 180) — interior
}

TEST(RegionToMask2D, ConcavePolygonFillsItselfNotTheConvexHull)
{
  // GIVEN a concave region with a V-notch cut into its +y edge (apex at 0.5, 0.5)
  const auto region = concaveRegion();

  // WHEN rasterizing it
  const auto mask = lab_sim_behaviors::regionToMask2D(region, kImageWidth, kImageHeight);
  ASSERT_TRUE(mask.has_value()) << mask.error();

  // origin = floor(0.2*1000, 0.2*800) = (200, 160); the mask is local to the bounding box.
  const cv_bridge::CvImagePtr image = cv_bridge::toCvCopy(mask->pixels, "mono8");

  // THEN a point past the apex toward the +y edge is NOT filled: inside the convex hull, outside the
  // true polygon. cv::fillConvexPoly would fill it; cv::fillPoly correctly leaves it empty.
  // normalized (0.5, 0.7) -> pixel (500, 560) -> local (row 400, col 300)
  EXPECT_EQ(image->image.at<uint8_t>(400, 300), 0);

  // AND a point in the solid body between the -y edge and the apex IS filled.
  // normalized (0.5, 0.3) -> pixel (500, 240) -> local (row 80, col 300)
  EXPECT_GT(image->image.at<uint8_t>(80, 300), 0);
}
