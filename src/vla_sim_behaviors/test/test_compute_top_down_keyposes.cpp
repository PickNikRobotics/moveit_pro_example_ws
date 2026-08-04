// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include <gtest/gtest.h>

#include <cmath>
#include <optional>
#include <vector>

#include <vla_sim_behaviors/compute_top_down_keyposes.hpp>

namespace
{
using vla_sim_behaviors::chooseTopDownYawByCost;
using vla_sim_behaviors::computeTopDownKeyposes;
using vla_sim_behaviors::jointDistanceCost;
using vla_sim_behaviors::reachableChainCost;
using vla_sim_behaviors::topDownGraspOrientation;
using vla_sim_behaviors::yawOf;

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

TEST(JointDistanceCost, IsZeroForTheSamePose)
{
  const std::vector<double> pose{ 0.1, -0.2, 0.3 };
  EXPECT_NEAR(jointDistanceCost(pose, pose), 0.0, kEpsilon);
}

TEST(JointDistanceCost, SumsTheSquaredPerJointDifferences)
{
  // The oracle's score, so a joint that moves twice as far counts four times as much and one
  // big wrist swing outweighs several small arm adjustments.
  EXPECT_NEAR(jointDistanceCost({ 0.0, 0.0 }, { 3.0, 4.0 }), 25.0, kEpsilon);
}

TEST(ChooseTopDownYawByCost, ReturnsACubeYawPlusAWholeNumberOfQuarterTurns)
{
  // Every candidate must be a symmetry of the cube. A yaw that is not one grasps a corner.
  const double chosen =
      chooseTopDownYawByCost(0.3, [](double yaw) { return std::optional<double>(std::abs(yaw)); }).value();
  const double turns = (chosen - 0.3) / kQuarterTurn;
  EXPECT_NEAR(turns, std::round(turns), kEpsilon);
}

TEST(ChooseTopDownYawByCost, PicksTheCheapestCandidateRatherThanTheNearest)
{
  // The whole point of scoring by IK: on eval_0 the oracle takes a candidate 180 degrees from
  // the wrist's current yaw because it holds joint_5 still. A nearest-yaw rule cannot do that.
  const double cube_yaw = 0.0;
  const auto cost = [](double yaw) -> std::optional<double> {
    // cheapest at two quarter turns, i.e. the candidate a half turn away
    return std::abs(std::remainder(yaw - M_PI, 2.0 * M_PI));
  };
  EXPECT_NEAR(chooseTopDownYawByCost(cube_yaw, cost).value(), M_PI, kEpsilon);
}

TEST(ChooseTopDownYawByCost, SkipsUnreachableCandidates)
{
  // IK fails on candidates that would put the wrist past a limit; those must not be chosen
  // even when a reachable one scores worse.
  const auto cost = [](double yaw) -> std::optional<double> {
    if (std::abs(std::remainder(yaw, 2.0 * M_PI)) < kEpsilon)
    {
      return 0.0;  // cheapest, but pretend it is the only reachable one below
    }
    return std::nullopt;
  };
  EXPECT_NEAR(chooseTopDownYawByCost(0.0, cost).value(), 0.0, kEpsilon);
}

TEST(ChooseTopDownYawByCost, ReturnsNulloptWhenNothingIsReachable)
{
  // The segment must fail loudly rather than plan a path the arm cannot follow.
  EXPECT_FALSE(chooseTopDownYawByCost(0.4, [](double) { return std::nullopt; }).has_value());
}

TEST(ChooseTopDownYawByCost, IsUnchangedByCubeYawsAQuarterTurnApart)
{
  // The cube's own yaw is only known modulo a quarter turn, so equivalent readings of the same
  // physical cube must produce the same grasp.
  const auto cost = [](double yaw) -> std::optional<double> { return std::abs(std::remainder(yaw - 0.9, 2.0 * M_PI)); };
  const double base = chooseTopDownYawByCost(0.2, cost).value();
  EXPECT_NEAR(std::remainder(chooseTopDownYawByCost(0.2 + kQuarterTurn, cost).value() - base, 2.0 * M_PI), 0.0,
              kEpsilon);
  EXPECT_NEAR(std::remainder(chooseTopDownYawByCost(0.2 - kQuarterTurn, cost).value() - base, 2.0 * M_PI), 0.0,
              kEpsilon);
}

TEST(ReachableChainCost, ScoresTheFirstKeyposeAndIgnoresTheRest)
{
  // The arm arrives at the first waypoint, so that is the motion this segment spends. Scoring
  // the whole chain would rank a candidate by travel it shares with every other candidate.
  const std::vector<Eigen::Isometry3d> keyposes = { makePose({ 0.5, 0.0, 0.2 }, 0.0), makePose({ 0.5, 0.0, 0.1 }, 0.0) };
  int call = 0;
  const auto solve = [&call](const Eigen::Isometry3d&) -> std::optional<std::vector<double>> {
    return std::vector<double>{ ++call == 1 ? 3.0 : 100.0 };
  };
  EXPECT_NEAR(reachableChainCost({ 0.0 }, keyposes, solve).value(), 9.0, kEpsilon);
}

TEST(ReachableChainCost, RejectsACandidateThatFailsOnALaterKeypose)
{
  // A yaw the arm can approach but not descend to is unusable: the planner solves every
  // waypoint and would fail the segment. Rejecting it here lets another cube symmetry win.
  const std::vector<Eigen::Isometry3d> keyposes = { makePose({ 0.5, 0.0, 0.2 }, 0.0), makePose({ 0.5, 0.0, 0.1 }, 0.0) };
  int call = 0;
  const auto solve = [&call](const Eigen::Isometry3d&) -> std::optional<std::vector<double>> {
    if (++call == 1)
    {
      return std::vector<double>{ 0.0 };  // the approach pose is reachable
    }
    return std::nullopt;
  };
  EXPECT_FALSE(reachableChainCost({ 0.0 }, keyposes, solve).has_value());
}

TEST(ReachableChainCost, WarmStartsEachSolveFromTheLast)
{
  // The chain must be solved in order, since the planner holds one IK branch across the whole
  // segment by seeding each waypoint from the previous solution.
  const std::vector<Eigen::Isometry3d> keyposes = { makePose({ 0.5, 0.0, 0.3 }, 0.0), makePose({ 0.5, 0.0, 0.2 }, 0.0),
                                                    makePose({ 0.5, 0.0, 0.1 }, 0.0) };
  std::vector<double> seen_heights;
  const auto solve = [&seen_heights](const Eigen::Isometry3d& keypose) -> std::optional<std::vector<double>> {
    seen_heights.push_back(keypose.translation().z());
    return std::vector<double>{ 0.0 };
  };
  ASSERT_TRUE(reachableChainCost({ 0.0 }, keyposes, solve).has_value());
  EXPECT_EQ(seen_heights, (std::vector<double>{ 0.3, 0.2, 0.1 }));
}

TEST(ComputeTopDownKeyposes, StacksOneWaypointPerHeightAboveTheAimPose)
{
  const auto keyposes = computeTopDownKeyposes(makePose({ 0.5, -0.1, 0.115 }, 0.0), topDownGraspOrientation(0.0),
                                               Eigen::Vector3d::Zero(), { 0.12, 0.0 });

  ASSERT_EQ(keyposes.size(), 2u);
  EXPECT_NEAR(keyposes[0].translation().z(), 0.235, kEpsilon);
  EXPECT_NEAR(keyposes[1].translation().z(), 0.115, kEpsilon);
  for (const auto& keypose : keyposes)
  {
    EXPECT_NEAR(keypose.translation().x(), 0.5, kEpsilon);
    EXPECT_NEAR(keypose.translation().y(), -0.1, kEpsilon);
  }
}

TEST(ComputeTopDownKeyposes, GivesEveryWaypointTheChosenOrientation)
{
  // The oracle descends straight down onto the cube. Re-deriving the orientation per waypoint
  // would let the wrist rotate mid-descent and shear the grasp.
  const Eigen::Quaterniond orientation = topDownGraspOrientation(0.4);
  const auto keyposes = computeTopDownKeyposes(makePose({ 0.5, 0.0, 0.115 }, 0.4), orientation, Eigen::Vector3d::Zero(),
                                               { 0.12, 0.06, 0.0 });

  ASSERT_EQ(keyposes.size(), 3u);
  for (const auto& keypose : keyposes)
  {
    EXPECT_NEAR(std::abs(Eigen::Quaterniond(keypose.rotation()).dot(orientation)), 1.0, kEpsilon);
  }
}

TEST(ComputeTopDownKeyposes, PlacesTheHeldObjectRatherThanTheTipWhenOffsetIsSet)
{
  // The place segment aims the carried cube at the target, so the tip must land offset by
  // exactly the grip, rotated into the world.
  const Eigen::Vector3d grip_offset(0.0, 0.0, 0.02);
  const auto keyposes =
      computeTopDownKeyposes(makePose({ 0.4, 0.2, 0.115 }, 0.0), topDownGraspOrientation(0.0), grip_offset, { 0.031 });

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
  const auto keyposes = computeTopDownKeyposes(makePose({ 0.4, 0.2, 0.115 }, kQuarterTurn),
                                               topDownGraspOrientation(kQuarterTurn), grip_offset, { 0.0 });

  ASSERT_EQ(keyposes.size(), 1u);
  // Rz(pi/2) * Rx(pi) maps the tip's +X onto world +Y, so the tip shifts back along -Y.
  EXPECT_NEAR(keyposes[0].translation().x(), 0.4, kEpsilon);
  EXPECT_NEAR(keyposes[0].translation().y(), 0.19, kEpsilon);
}

TEST(ComputeTopDownKeyposes, ReturnsAnEmptyPathForNoHeights)
{
  EXPECT_TRUE(computeTopDownKeyposes(makePose({ 0.5, 0.0, 0.115 }, 0.0), topDownGraspOrientation(0.0),
                                     Eigen::Vector3d::Zero(), {})
                  .empty());
}

TEST(ComputeTopDownKeyposes, TakesOnlyThePositionFromATiltedAimPose)
{
  // Cube poses come from live physics and are never exactly level. The waypoint must still sit
  // straight above the cube, since the arm approaches vertically.
  Eigen::Isometry3d tilted(Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitZ()) *
                           Eigen::AngleAxisd(0.05, Eigen::Vector3d::UnitX()));
  tilted.translation() = Eigen::Vector3d(0.5, 0.0, 0.115);

  const auto keyposes = computeTopDownKeyposes(tilted, topDownGraspOrientation(0.3), Eigen::Vector3d::Zero(), { 0.12 });

  ASSERT_EQ(keyposes.size(), 1u);
  EXPECT_NEAR(keyposes[0].translation().x(), 0.5, kEpsilon);
  EXPECT_NEAR(keyposes[0].translation().y(), 0.0, kEpsilon);
  EXPECT_NEAR(keyposes[0].translation().z(), 0.235, kEpsilon);
  const Eigen::Vector3d approach = keyposes[0].rotation() * Eigen::Vector3d::UnitZ();
  EXPECT_NEAR(approach.z(), -1.0, kEpsilon);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
