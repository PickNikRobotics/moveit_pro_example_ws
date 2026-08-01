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

namespace kinova_vla_test_sim_behaviors
{
/**
 * @brief Orientation of a top-down grasp whose jaw axis is spun to @p yaw about world +Z.
 *
 * The tip frame's +Z is the approach axis and +Y the jaw open/close axis, so pointing the
 * approach axis at world -Z is Rx(pi).
 */
[[nodiscard]] Eigen::Quaterniond topDownGraspOrientation(double yaw);

/** @brief Rotation about world +Z of @p orientation. */
[[nodiscard]] double yawOf(const Eigen::Quaterniond& orientation);

/**
 * @brief The one of a cube's four symmetry-equivalent top-down yaws nearest @p reference_yaw.
 *
 * A cube is 4-fold symmetric about its vertical axis, so all four grasps are equivalent; taking
 * the nearest keeps the wrist off its limits. The oracle instead scores the four by IK joint
 * distance (`oracle.py::_choose_topdown_yaw`), which selects the same candidate whenever wrist
 * rotation dominates the joint cost.
 */
[[nodiscard]] double chooseTopDownYaw(double cube_yaw, double reference_yaw);

/**
 * @brief Top-down waypoints at each of @p heights above @p aim_pose.
 *
 * All waypoints share the orientation from chooseTopDownYaw(yaw of @p aim_pose,
 * @p reference_yaw). @p held_object_offset is the held object's origin in the tip frame; it
 * shifts each waypoint so the *object* lands on the aim point rather than the tip. Pass zero
 * when the tip itself is the thing being positioned.
 */
[[nodiscard]] std::vector<Eigen::Isometry3d> computeTopDownKeyposes(const Eigen::Isometry3d& aim_pose,
                                                                    double reference_yaw,
                                                                    const Eigen::Vector3d& held_object_offset,
                                                                    const std::vector<double>& heights);

/**
 * @brief Builds the Cartesian path for one segment of the color-cube stacking oracle.
 */
class ComputeTopDownKeyposes : public moveit_pro::behaviors::SharedResourcesNode<BT::SyncActionNode>
{
public:
  ComputeTopDownKeyposes(const std::string& name, const BT::NodeConfiguration& config,
                         const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources);

  [[nodiscard]] static BT::PortsList providedPorts();
  [[nodiscard]] static BT::KeyValueVector metadata();

  BT::NodeStatus tick() override;
};
}  // namespace kinova_vla_test_sim_behaviors
