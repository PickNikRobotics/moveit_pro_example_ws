// Copyright 2025 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "moveit_pro_mpc/residual_functions/residual_function_interface.hpp"
#include "mujoco/mjdata.h"
#include "mujoco/mjmodel.h"
#include "tl_expected/expected.hpp"

namespace custom_mpc_behavior
{

/**
 * \brief Residual functions for tracking a moving pose.
 */
class SiteTrackingResidualFunction final : public moveit_pro_mpc::ResidualFunctionInterface
{
public:
  [[nodiscard]] int getNumDims() const override;

  tl::expected<bool, std::string> validateResidualParameters() override;

  void applyResidual(const mjModel& model, const mjData& data, double* residual) const override;

  [[nodiscard]] auto getResidualParameters()
  {
    return std::make_tuple(std::make_pair("gripper_site_name", &site_tip_),
                           std::make_pair("target_pose", &target_pose_),
                           std::make_pair("target_twist", &target_twist_));
  }

private:
  /** @brief The robot end effector site in the MuJoCo model to be used for tracking.*/
  std::string site_tip_;
  /** @brief The target pose to track. Must be in world frame.*/
  geometry_msgs::msg::PoseStamped target_pose_;
  /** @brief The target twist to track. Must be in world frame.*/
  geometry_msgs::msg::TwistStamped target_twist_;

  int site_tip_ind_ = 1;
};

}  // namespace custom_mpc_behavior
