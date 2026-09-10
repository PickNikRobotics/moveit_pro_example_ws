// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include <gtest/gtest.h>

#include <hangar_sim_behaviors/planar_pose.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <tf2/LinearMath/Quaternion.hpp>

#include <cmath>

namespace
{
geometry_msgs::msg::Pose makePose(double x, double y, double z, double roll, double pitch, double yaw)
{
  tf2::Quaternion q;
  q.setRPY(roll, pitch, yaw);
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();
  pose.orientation.w = q.w();
  return pose;
}
}  // namespace

/// x, y and yaw survive; the axes the 2D filter never received do not.
namespace
{
/// Yaw read back independently of the code under test, so a bug in yawOf cannot hide itself here.
/// Deliberately not tf2::getYaw: that resolves through an inline in tf2_geometry_msgs.hpp rather
/// than tf2/utils.hpp, so whether it links depends on include order in a repository that sorts
/// includes, and the failure lands on whatever links the library rather than here.
double readBackYaw(const geometry_msgs::msg::Quaternion& q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}
}  // namespace

TEST(PlanarPose, KeepsPlanarComponentsAndDropsTheRest)
{
  const auto planar = hangar_sim_behaviors::localization::projectToPlane(makePose(1.25, -3.5, 0.9, 0.3, -0.2, 0.7));

  EXPECT_DOUBLE_EQ(planar.position.x, 1.25);
  EXPECT_DOUBLE_EQ(planar.position.y, -3.5);
  EXPECT_DOUBLE_EQ(planar.position.z, 0.0);
  EXPECT_DOUBLE_EQ(planar.orientation.x, 0.0);
  EXPECT_DOUBLE_EQ(planar.orientation.y, 0.0);
  EXPECT_NEAR(readBackYaw(planar.orientation), 0.7, 1e-9);
}

/// Flattening twice changes nothing, so a pose that is already planar is passed through untouched.
TEST(PlanarPose, IsIdempotent)
{
  const auto once = hangar_sim_behaviors::localization::projectToPlane(makePose(2.0, 4.0, 1.1, -0.4, 0.6, -1.3));
  const auto twice = hangar_sim_behaviors::localization::projectToPlane(once);

  EXPECT_DOUBLE_EQ(twice.position.x, once.position.x);
  EXPECT_DOUBLE_EQ(twice.position.y, once.position.y);
  EXPECT_DOUBLE_EQ(twice.position.z, once.position.z);
  EXPECT_NEAR(readBackYaw(twice.orientation), readBackYaw(once.orientation), 1e-12);
}

/**
 * @brief The invariant the drift gate rests on.
 *
 * A pose picked off a tilted, raised surface and the planar estimate the filter actually holds
 * differ by an arbitrary amount in z, roll and pitch. Once both are flattened, all that is left
 * between them is the planar difference the gate is meant to bound -- here, none at all.
 */
TEST(PlanarPose, TiltAndHeightDoNotSurviveIntoTheComparison)
{
  const auto clicked_on_a_tilted_face =
      hangar_sim_behaviors::localization::projectToPlane(makePose(5.0, 6.0, 1.8, 1.2, -0.9, 0.35));
  const auto filter_estimate =
      hangar_sim_behaviors::localization::projectToPlane(makePose(5.0, 6.0, 0.0, 0.0, 0.0, 0.35));

  EXPECT_DOUBLE_EQ(clicked_on_a_tilted_face.position.z, filter_estimate.position.z);
  EXPECT_NEAR(readBackYaw(clicked_on_a_tilted_face.orientation), readBackYaw(filter_estimate.orientation), 1e-9);
  EXPECT_NEAR(clicked_on_a_tilted_face.orientation.x, filter_estimate.orientation.x, 1e-12);
  EXPECT_NEAR(clicked_on_a_tilted_face.orientation.y, filter_estimate.orientation.y, 1e-12);
  EXPECT_NEAR(clicked_on_a_tilted_face.orientation.z, filter_estimate.orientation.z, 1e-12);
  EXPECT_NEAR(clicked_on_a_tilted_face.orientation.w, filter_estimate.orientation.w, 1e-12);
}
