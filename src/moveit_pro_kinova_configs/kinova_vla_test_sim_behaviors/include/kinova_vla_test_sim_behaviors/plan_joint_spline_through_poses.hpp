// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <vector>

#include <behaviortree_cpp/action_node.h>
#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_pro_behavior_interface/shared_resources_node.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace kinova_vla_test_sim_behaviors
{
/**
 * @brief Clamped cubic spline through joint-space knots.
 *
 * Passes through every knot exactly, is C1 continuous across them, and has zero first
 * derivative at both ends. So the arm eases from rest, flows through the interior knots
 * without stopping, and arrives at the last knot at rest.
 *
 * The parameter is dimensionless: divide the derivatives by the trajectory duration to
 * get joint velocities and accelerations.
 */
class JointSpline
{
public:
  /** @throws std::invalid_argument unless there are >= 2 knots, all of one width, at
   *          strictly increasing @p parameters of the same count. */
  JointSpline(const std::vector<double>& parameters, const std::vector<Eigen::VectorXd>& knots);

  [[nodiscard]] Eigen::VectorXd position(double s) const;
  [[nodiscard]] Eigen::VectorXd velocity(double s) const;
  [[nodiscard]] Eigen::VectorXd acceleration(double s) const;

  /** @brief Largest |d position / d parameter| over the spline, per joint. */
  [[nodiscard]] Eigen::VectorXd peakSpeed() const;

private:
  /** Where a parameter falls: the segment holding it, and the basis weights within it. */
  struct Segment
  {
    std::size_t index;
    double width;
    double from;
    double to;
  };

  /** @p s saturated to the knot range, resolved against the segment containing it. */
  [[nodiscard]] Segment locate(double s) const;

  std::vector<double> parameters_;
  std::vector<Eigen::VectorXd> knots_;
  std::vector<Eigen::VectorXd> moments_;
};

/**
 * @brief How long one straight knot-to-knot segment should take.
 *
 * The slower of two budgets: covering @p cartesian_length at @p cartesian_speed, and
 * holding every joint's peak speed under @p joint_velocity_cap.
 */
[[nodiscard]] double segmentDuration(double cartesian_length, const Eigen::VectorXd& joint_delta,
                                     double cartesian_speed, const Eigen::VectorXd& joint_velocity_cap);

/**
 * @brief Knot parameters spanning [0, 1], each segment's share proportional to @p segment_durations.
 *
 * Giving a long or slow segment more of the parameter is what makes one spline over the
 * whole chain respect a single speed budget instead of rushing its longest leg.
 */
[[nodiscard]] std::vector<double> splineKnotParameters(const std::vector<double>& segment_durations);

/**
 * @brief Duration to run @p spline over, as the slower of the same two budgets.
 *
 * Measured against the spline itself rather than the straight knot-to-knot deltas, since
 * flowing through an interior knot overshoots what those deltas predict.
 */
[[nodiscard]] double splineDuration(const JointSpline& spline, double cartesian_length, double cartesian_speed,
                                    const Eigen::VectorXd& joint_velocity_cap);

/**
 * @brief Plans one segment of the color-cube stacking oracle as a joint-space trajectory.
 */
class PlanJointSplineThroughPoses : public moveit_pro::behaviors::SharedResourcesNode<BT::SyncActionNode>
{
public:
  PlanJointSplineThroughPoses(const std::string& name, const BT::NodeConfiguration& config,
                              const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources);

  [[nodiscard]] static BT::PortsList providedPorts();
  [[nodiscard]] static BT::KeyValueVector metadata();

  BT::NodeStatus tick() override;
};
}  // namespace kinova_vla_test_sim_behaviors
