// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>

#include <kinova_vla_test_sim_behaviors/plan_joint_spline_through_poses.hpp>

namespace
{
using kinova_vla_test_sim_behaviors::JointSpline;
using kinova_vla_test_sim_behaviors::segmentDuration;
using kinova_vla_test_sim_behaviors::splineDuration;
using kinova_vla_test_sim_behaviors::splineKnotParameters;

Eigen::VectorXd vec(std::initializer_list<double> values)
{
  Eigen::VectorXd result(static_cast<Eigen::Index>(values.size()));
  Eigen::Index i = 0;
  for (const double value : values)
  {
    result[i++] = value;
  }
  return result;
}

/** Two knots one unit apart on a single joint, over the full parameter span. */
JointSpline twoKnotSpline()
{
  return JointSpline({ 0.0, 1.0 }, { vec({ 0.0 }), vec({ 1.0 }) });
}

/** Three knots, the interior one deliberately off the midpoint so corner-cutting shows. */
JointSpline threeKnotSpline()
{
  return JointSpline({ 0.0, 0.25, 1.0 }, { vec({ 0.0, 0.0 }), vec({ 0.5, -1.0 }), vec({ 2.0, 1.0 }) });
}
}  // namespace

TEST(JointSpline, RejectsFewerThanTwoKnots)
{
  EXPECT_THROW(JointSpline({ 0.0 }, { vec({ 0.0 }) }), std::invalid_argument);
  EXPECT_THROW(JointSpline({}, {}), std::invalid_argument);
}

TEST(JointSpline, RejectsParameterCountMismatch)
{
  EXPECT_THROW(JointSpline({ 0.0, 0.5, 1.0 }, { vec({ 0.0 }), vec({ 1.0 }) }), std::invalid_argument);
}

TEST(JointSpline, RejectsNonIncreasingParameters)
{
  EXPECT_THROW(JointSpline({ 0.0, 0.0 }, { vec({ 0.0 }), vec({ 1.0 }) }), std::invalid_argument);
  EXPECT_THROW(JointSpline({ 1.0, 0.0 }, { vec({ 0.0 }), vec({ 1.0 }) }), std::invalid_argument);
}

TEST(JointSpline, RejectsKnotsOfDifferentWidths)
{
  EXPECT_THROW(JointSpline({ 0.0, 1.0 }, { vec({ 0.0 }), vec({ 1.0, 2.0 }) }), std::invalid_argument);
}

TEST(JointSpline, PassesThroughEveryKnotExactly)
{
  const JointSpline spline = threeKnotSpline();
  EXPECT_NEAR(spline.position(0.0)[0], 0.0, 1e-12);
  EXPECT_NEAR(spline.position(0.25)[0], 0.5, 1e-12);
  EXPECT_NEAR(spline.position(0.25)[1], -1.0, 1e-12);
  EXPECT_NEAR(spline.position(1.0)[0], 2.0, 1e-12);
}

TEST(JointSpline, StartsAndEndsAtRest)
{
  const JointSpline spline = threeKnotSpline();
  EXPECT_NEAR(spline.velocity(0.0).cwiseAbs().maxCoeff(), 0.0, 1e-9);
  EXPECT_NEAR(spline.velocity(1.0).cwiseAbs().maxCoeff(), 0.0, 1e-9);
}

TEST(JointSpline, FlowsThroughInteriorKnotsWithoutStopping)
{
  // The reason for one spline over the whole chain rather than a clamped move per segment:
  // chaining would pin the interior knot to zero velocity and stop the arm at every waypoint.
  const JointSpline spline = threeKnotSpline();
  EXPECT_GT(spline.velocity(0.25).cwiseAbs().maxCoeff(), 0.5);
}

TEST(JointSpline, IsContinuousInVelocityAcrossAnInteriorKnot)
{
  const JointSpline spline = threeKnotSpline();
  const Eigen::VectorXd before = spline.velocity(0.25 - 1e-7);
  const Eigen::VectorXd after = spline.velocity(0.25 + 1e-7);
  EXPECT_LT((after - before).cwiseAbs().maxCoeff(), 1e-5);
}

TEST(JointSpline, PeaksAtOneAndAHalfTimesTheMeanOnASingleSegment)
{
  // Pins the profile: a clamped cubic peaks at 1.5x its mean speed. Smootherstep would be
  // 1.875 and a linear ramp 1.0, so this fails if the interpolation is swapped out.
  EXPECT_NEAR(twoKnotSpline().peakSpeed()[0], 1.5, 1e-9);
  EXPECT_NEAR(twoKnotSpline().velocity(0.5)[0], 1.5, 1e-9);
}

TEST(JointSpline, ReportsPeakSpeedPerJointIndependently)
{
  const JointSpline spline = JointSpline({ 0.0, 1.0 }, { vec({ 0.0, 0.0 }), vec({ 1.0, 4.0 }) });
  const Eigen::VectorXd peak = spline.peakSpeed();
  EXPECT_NEAR(peak[0], 1.5, 1e-9);
  EXPECT_NEAR(peak[1], 6.0, 1e-9);
}

TEST(JointSpline, SaturatesOutsideTheParameterRangeRatherThanExtrapolating)
{
  // A cubic run past its last knot diverges fast, so evaluating off the end holds the knot.
  const JointSpline spline = twoKnotSpline();
  EXPECT_NEAR(spline.position(-0.5)[0], 0.0, 1e-12);
  EXPECT_NEAR(spline.position(1.5)[0], 1.0, 1e-12);
  EXPECT_NEAR(spline.velocity(1.5)[0], 0.0, 1e-9);
}

TEST(SegmentDuration, TakesTheCartesianBudgetWhenTheJointCapIsSlack)
{
  // 0.2 m at 0.065 m/s is 3.08 s; the joint cap allows far quicker, so it does not bind.
  EXPECT_NEAR(segmentDuration(0.2, vec({ 0.1, 0.1 }), 0.065, vec({ 0.695, 0.695 })), 0.2 / 0.065, 1e-9);
}

TEST(SegmentDuration, TakesTheJointCapWhenThePathIsShortButTheArmTurnsFar)
{
  // A wrist flip in place: no Cartesian distance to pay for, but the joint still has to
  // stay under its limit.
  const double duration = segmentDuration(0.0, vec({ 3.0 }), 0.065, vec({ 0.695 }));
  EXPECT_NEAR(duration, 1.875 * 3.0 / 0.695, 1e-9);
}

TEST(SegmentDuration, IgnoresJointsWithoutAVelocityLimit)
{
  EXPECT_NEAR(segmentDuration(0.1, vec({ 3.0 }), 0.065, vec({ 0.0 })), 0.1 / 0.065, 1e-9);
}

TEST(SplineKnotParameters, SpansZeroToOneInProportionToDuration)
{
  const std::vector<double> parameters = splineKnotParameters({ 1.0, 3.0 });
  ASSERT_EQ(parameters.size(), 3u);
  EXPECT_NEAR(parameters[0], 0.0, 1e-12);
  EXPECT_NEAR(parameters[1], 0.25, 1e-12);
  EXPECT_NEAR(parameters[2], 1.0, 1e-12);
}

TEST(SplineKnotParameters, StaysStrictlyIncreasingThroughAZeroLengthSegment)
{
  // Two waypoints at the same place would otherwise collapse onto one parameter and make
  // the spline unsolvable.
  const std::vector<double> parameters = splineKnotParameters({ 1.0, 0.0, 1.0 });
  EXPECT_GT(parameters[2], parameters[1]);
  EXPECT_NO_THROW(JointSpline(parameters, { vec({ 0.0 }), vec({ 1.0 }), vec({ 1.0 }), vec({ 2.0 }) }));
}

TEST(SplineDuration, SpendsTheCartesianLengthAtTheRequestedSpeed)
{
  // The oracle's regime: the tip speed budget binds and the joint caps stay slack, so the
  // whole segment takes exactly length / speed.
  const JointSpline spline = twoKnotSpline();
  EXPECT_NEAR(splineDuration(spline, 0.381, 0.065, vec({ 0.695 })), 0.381 / 0.065, 1e-9);
}

TEST(SplineDuration, StretchesUntilThePeakJointSpeedFitsUnderTheCap)
{
  // Peak speed is 1.5 rad per unit parameter, so a 0.5 rad/s cap needs 3 s.
  const JointSpline spline = twoKnotSpline();
  EXPECT_NEAR(splineDuration(spline, 0.0, 0.065, vec({ 0.5 })), 3.0, 1e-9);
}

TEST(SplineDuration, ClampsToASaneRange)
{
  const JointSpline spline = twoKnotSpline();
  EXPECT_NEAR(splineDuration(spline, 1e-9, 0.065, vec({ 1e6 })), 0.2, 1e-9);
  EXPECT_NEAR(splineDuration(spline, 1e6, 0.065, vec({ 1e6 })), 60.0, 1e-9);
}
