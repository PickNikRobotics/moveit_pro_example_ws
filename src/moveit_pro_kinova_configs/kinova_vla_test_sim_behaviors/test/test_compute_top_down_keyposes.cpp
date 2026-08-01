// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include <gtest/gtest.h>

#include <cmath>

#include <kinova_vla_test_sim_behaviors/compute_top_down_keyposes.hpp>

namespace
{
using kinova_vla_test_sim_behaviors::chooseTopDownYaw;
using kinova_vla_test_sim_behaviors::computeTopDownKeyposes;
using kinova_vla_test_sim_behaviors::topDownGraspOrientation;
using kinova_vla_test_sim_behaviors::yawOf;

constexpr double kEpsilon = 1e-9;
constexpr double kQuarterTurn = M_PI / 2.0;

Eigen::Isometry3d makePose(const Eigen::Vector3d& position, double yaw)
{
  Eigen::Isometry3d pose(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
  pose.translation() = position;
  return pose;
}

// The oracle's Q_TOPDOWN, in the (x, y, z, w) order Eigen's constructor takes last.
Eigen::Quaterniond oracleTopDown()
{
  return Eigen::Quaterniond(0.0, 1.0, 0.0, 0.0);
}
}  // namespace

TEST(TopDownGraspOrientation, ZeroYawMatchesTheOracleBaseOrientation)
{
  // mujoco_ik.py pins Q_TOPDOWN = (w, x, y, z) = (0, 1, 0, 0). A different sign convention or
  // axis here would still point the jaws down but spin them 90 degrees off the cube.
  EXPECT_NEAR(std::abs(topDownGraspOrientation(0.0).dot(oracleTopDown())), 1.0, kEpsilon);
}

TEST(TopDownGraspOrientation, ApproachAxisPointsDown)
{
  for (const double yaw : { -2.0, 0.0, 0.7, 3.0 })
  {
    const Eigen::Vector3d approach = topDownGraspOrientation(yaw) * Eigen::Vector3d::UnitZ();
    EXPECT_NEAR(approach.z(), -1.0, kEpsilon) << "yaw " << yaw;
  }
}

TEST(TopDownGraspOrientation, SpinsTheJawAxisByYaw)
{
  // The jaw axis is the tip's +Y. Spinning by yaw must rotate it in the horizontal plane, or the
  // grasp does not line up with the cube's faces.
  // Rx(pi) flips +Y to -Y, then Rz(pi/2) carries that onto +X.
  const Eigen::Vector3d jaw = topDownGraspOrientation(kQuarterTurn) * Eigen::Vector3d::UnitY();
  EXPECT_NEAR(jaw.x(), 1.0, kEpsilon);
  EXPECT_NEAR(jaw.z(), 0.0, kEpsilon);
}

TEST(YawOf, RoundTripsTopDownGraspOrientation)
{
  for (const double yaw : { -1.5, -0.2, 0.0, 0.9 })
  {
    EXPECT_NEAR(yawOf(topDownGraspOrientation(yaw)), yaw, kEpsilon) << "yaw " << yaw;
  }
}

TEST(ChooseTopDownYaw, ReturnsACubeYawPlusAWholeNumberOfQuarterTurns)
{
  // Every candidate must be a symmetry of the cube. A yaw that is not one grasps a corner.
  for (const double reference : { -3.0, -1.1, 0.0, 0.4, 2.7 })
  {
    const double chosen = chooseTopDownYaw(0.3, reference);
    const double turns = (chosen - 0.3) / kQuarterTurn;
    EXPECT_NEAR(turns, std::round(turns), kEpsilon) << "reference " << reference;
  }
}

TEST(ChooseTopDownYaw, LandsWithinAnEighthTurnOfTheReference)
{
  // Candidates are a quarter turn apart, so the nearest is always within an eighth turn. A
  // farther pick means the wrist rotates further than it has to.
  for (const double cube_yaw : { -1.4, -0.05, 0.0, 0.8, 2.2 })
  {
    for (const double reference : { -2.5, 0.0, 1.3 })
    {
      EXPECT_LE(std::abs(chooseTopDownYaw(cube_yaw, reference) - reference), kQuarterTurn / 2.0 + kEpsilon)
          << "cube " << cube_yaw << " reference " << reference;
    }
  }
}

TEST(ChooseTopDownYaw, IsIdempotentUnderItsOwnResult)
{
  // The lift and place segments re-derive the yaw with the wrist already at the chosen one.
  // Drifting by a quarter turn there would twist a cube that is already gripped.
  const double first = chooseTopDownYaw(0.37, 1.1);
  EXPECT_NEAR(chooseTopDownYaw(0.37, first), first, kEpsilon);
}

TEST(ChooseTopDownYaw, IsUnchangedByCubeYawsAQuarterTurnApart)
{
  // The cube's own yaw is only known modulo a quarter turn, so equivalent readings of the same
  // physical cube must produce the same grasp.
  const double reference = 0.6;
  const double base = chooseTopDownYaw(0.2, reference);
  EXPECT_NEAR(chooseTopDownYaw(0.2 + kQuarterTurn, reference), base, kEpsilon);
  EXPECT_NEAR(chooseTopDownYaw(0.2 - kQuarterTurn, reference), base, kEpsilon);
}

TEST(ComputeTopDownKeyposes, StacksOneWaypointPerHeightAboveTheAimPose)
{
  const auto keyposes =
      computeTopDownKeyposes(makePose({ 0.5, -0.1, 0.115 }, 0.0), 0.0, Eigen::Vector3d::Zero(), { 0.12, 0.0 });

  ASSERT_EQ(keyposes.size(), 2u);
  EXPECT_NEAR(keyposes[0].translation().z(), 0.235, kEpsilon);
  EXPECT_NEAR(keyposes[1].translation().z(), 0.115, kEpsilon);
  for (const auto& keypose : keyposes)
  {
    EXPECT_NEAR(keypose.translation().x(), 0.5, kEpsilon);
    EXPECT_NEAR(keypose.translation().y(), -0.1, kEpsilon);
  }
}

TEST(ComputeTopDownKeyposes, GivesEveryWaypointTheSameOrientation)
{
  // The oracle descends straight down onto the cube. Re-deriving the orientation per waypoint
  // would let the wrist rotate mid-descent and shear the grasp.
  const auto keyposes =
      computeTopDownKeyposes(makePose({ 0.5, 0.0, 0.115 }, 0.4), 0.0, Eigen::Vector3d::Zero(), { 0.12, 0.06, 0.0 });

  ASSERT_EQ(keyposes.size(), 3u);
  for (const auto& keypose : keyposes)
  {
    EXPECT_NEAR(std::abs(Eigen::Quaterniond(keypose.rotation()).dot(Eigen::Quaterniond(keyposes[0].rotation()))), 1.0,
                kEpsilon);
  }
}

TEST(ComputeTopDownKeyposes, AlignsTheGraspToTheAimPoseYaw)
{
  const auto keyposes =
      computeTopDownKeyposes(makePose({ 0.5, 0.0, 0.115 }, 0.3), 0.3, Eigen::Vector3d::Zero(), { 0.0 });

  ASSERT_EQ(keyposes.size(), 1u);
  EXPECT_NEAR(yawOf(Eigen::Quaterniond(keyposes[0].rotation())), 0.3, kEpsilon);
}

TEST(ComputeTopDownKeyposes, PlacesTheHeldObjectRatherThanTheTipWhenOffsetIsSet)
{
  // The place segment aims the carried cube at the target, so the tip must land offset by
  // exactly the grip, rotated into the world.
  const Eigen::Vector3d grip_offset(0.0, 0.0, 0.02);
  const auto keyposes = computeTopDownKeyposes(makePose({ 0.4, 0.2, 0.115 }, 0.0), 0.0, grip_offset, { 0.031 });

  ASSERT_EQ(keyposes.size(), 1u);
  // The tip's +Z points down, so an object 20 mm along +Z sits 20 mm below the tip: the tip goes
  // that much higher for the object to land on the aim point.
  EXPECT_NEAR(keyposes[0].translation().z(), 0.115 + 0.031 + 0.02, kEpsilon);
  EXPECT_NEAR(keyposes[0].translation().x(), 0.4, kEpsilon);
  EXPECT_NEAR(keyposes[0].translation().y(), 0.2, kEpsilon);
}

TEST(ComputeTopDownKeyposes, RotatesALateralHeldObjectOffsetIntoTheWorld)
{
  // A grip that is off-center laterally must be corrected in the direction the wrist is actually
  // pointing. Ignoring the rotation would put the cube down on the wrong side of the target.
  const Eigen::Vector3d grip_offset(0.01, 0.0, 0.0);
  const auto aim = makePose({ 0.4, 0.2, 0.115 }, kQuarterTurn);
  const auto keyposes = computeTopDownKeyposes(aim, kQuarterTurn, grip_offset, { 0.0 });

  ASSERT_EQ(keyposes.size(), 1u);
  // Rz(pi/2) * Rx(pi) maps the tip's +X onto world +Y, so the tip shifts back along -Y.
  EXPECT_NEAR(keyposes[0].translation().x(), 0.4, kEpsilon);
  EXPECT_NEAR(keyposes[0].translation().y(), 0.19, kEpsilon);
}

TEST(ComputeTopDownKeyposes, ReturnsAnEmptyPathForNoHeights)
{
  EXPECT_TRUE(computeTopDownKeyposes(makePose({ 0.5, 0.0, 0.115 }, 0.0), 0.0, Eigen::Vector3d::Zero(), {}).empty());
}

TEST(ComputeTopDownKeyposes, IgnoresTiltInTheAimPose)
{
  // Cube poses come from live physics and are never exactly level. The grasp must stay top-down
  // regardless, since the arm approaches vertically.
  Eigen::Isometry3d tilted(Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitZ()) *
                           Eigen::AngleAxisd(0.05, Eigen::Vector3d::UnitX()));
  tilted.translation() = Eigen::Vector3d(0.5, 0.0, 0.115);

  const auto keyposes = computeTopDownKeyposes(tilted, 0.3, Eigen::Vector3d::Zero(), { 0.0 });

  ASSERT_EQ(keyposes.size(), 1u);
  const Eigen::Vector3d approach = keyposes[0].rotation() * Eigen::Vector3d::UnitZ();
  EXPECT_NEAR(approach.z(), -1.0, kEpsilon);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
