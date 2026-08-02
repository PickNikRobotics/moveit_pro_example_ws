// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <functional>
#include <optional>
#include <vector>

#include <behaviortree_cpp/action_node.h>
#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_pro_behavior_interface/shared_resources_node.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

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
 * @brief The one of a cube's four symmetry-equivalent top-down yaws that @p cost scores lowest.
 *
 * A cube is 4-fold symmetric about its vertical axis, so all four grasps are visually identical
 * and any of them stacks; which one the arm takes is purely a question of how far it has to
 * move. The oracle scores them by IK joint distance from the arm's current pose
 * (`oracle.py::_choose_topdown_yaw`), and this reproduces that with @p cost injected so the
 * ranking is testable without a robot model.
 *
 * @returns nullopt when @p cost scores no candidate, i.e. none is reachable.
 */
[[nodiscard]] std::optional<double> chooseTopDownYawByCost(double cube_yaw,
                                                           const std::function<std::optional<double>(double)>& cost);

/** @brief Sum of squared per-joint differences, the oracle's IK cost. */
[[nodiscard]] double jointDistanceCost(const std::vector<double>& from, const std::vector<double>& to);

/**
 * @brief One top-down waypoint @p height above @p aim_pose, at @p orientation.
 *
 * @p held_object_offset is the held object's origin in the tip frame; it shifts the waypoint so
 * the *object* lands on the aim point rather than the tip. Pass zero when the tip itself is the
 * thing being positioned.
 */
[[nodiscard]] Eigen::Isometry3d topDownKeypose(const Eigen::Isometry3d& aim_pose, const Eigen::Quaterniond& orientation,
                                               const Eigen::Vector3d& held_object_offset, double height);

/** @brief topDownKeypose() at each of @p heights, all sharing @p orientation. */
[[nodiscard]] std::vector<Eigen::Isometry3d> computeTopDownKeyposes(const Eigen::Isometry3d& aim_pose,
                                                                    const Eigen::Quaterniond& orientation,
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
