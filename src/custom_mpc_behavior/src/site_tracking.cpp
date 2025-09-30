// Copyright 2025 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include "custom_mpc_behavior/site_tracking.hpp"

#include "fmt/format.h"
#include "mujoco/mujoco.h"

namespace custom_mpc_behavior
{

int SiteTrackingResidualFunction::getNumDims() const
{
  return 6;
}

tl::expected<bool, std::string> SiteTrackingResidualFunction::validateResidualParameters()
{
  site_tip_ind_ = mj_name2id(model, mjOBJ_SITE, site_tip_.c_str());
  if (site_tip_ind_ == -1)
  {
    return tl::make_unexpected(
        fmt::format("The specified {} '{}' does not exist in the MuJoCo model.", "site_tip_", site_tip_));
  }

  if ("world" != target_pose_.header.frame_id || "world" != target_twist_.header.frame_id)
  {
    return tl::make_unexpected(fmt::format("The specified `{}` does not match the frame_id of either pose or twist "
                                           "inputs. The frame of the pose input is {} and the twist input is {}.",
                                           "site_tip_", target_pose_.header.frame_id, target_twist_.header.frame_id));
  }

  return true;
}

void SiteTrackingResidualFunction::applyResidual(const mjModel& /*model*/, const mjData& data, double* residual) const
{
  mjtNum target_position_velocity[3] = {};
  mjtNum target_angular_velocity[3] = {};
  mjtNum target_position[3] = {};
  mjtNum target_quat[4] = {};
  target_position[0] = target_pose_.pose.position.x;
  target_position[1] = target_pose_.pose.position.y;
  target_position[2] = target_pose_.pose.position.z;
  target_quat[0] = target_pose_.pose.orientation.w;
  target_quat[1] = target_pose_.pose.orientation.x;
  target_quat[2] = target_pose_.pose.orientation.y;
  target_quat[3] = target_pose_.pose.orientation.z;
  target_position_velocity[0] = target_twist_.twist.linear.x;
  target_position_velocity[1] = target_twist_.twist.linear.y;
  target_position_velocity[2] = target_twist_.twist.linear.z;
  target_angular_velocity[0] = target_twist_.twist.angular.x;
  target_angular_velocity[1] = target_twist_.twist.angular.y;
  target_angular_velocity[2] = target_twist_.twist.angular.z;

  // ---------- Residual (0-2) ----------
  const mjtNum delta_time = data.time - initial_data->time;

  mjtNum current_target_pos[3];
  current_target_pos[0] = target_position[0] + target_position_velocity[0] * delta_time;
  current_target_pos[1] = target_position[1] + target_position_velocity[1] * delta_time;
  current_target_pos[2] = target_position[2] + target_position_velocity[2] * delta_time;

  const mjtNum* tip_xpos_current = &data.site_xpos[3 * site_tip_ind_];

  residual[0] = tip_xpos_current[0] - current_target_pos[0];
  residual[1] = tip_xpos_current[1] - current_target_pos[1];
  residual[2] = tip_xpos_current[2] - current_target_pos[2];

  // ---------- Residual (3-5) ----------

  // Set the current site quaternion
  const mjtNum* tip_xmat_current = &data.site_xmat[9 * site_tip_ind_];
  mjtNum tip_quat_current[4];
  mju_mat2Quat(tip_quat_current, tip_xmat_current);

  mjtNum current_target_quat[4];
  mjtNum current_target_quat_diff[4];
  mju_quatIntegrate(current_target_quat_diff, target_angular_velocity, delta_time);
  mju_mulQuat(current_target_quat, current_target_quat_diff, target_quat);

  mjtNum relative_quat_target[3];
  mju_subQuat(relative_quat_target, current_target_quat, tip_quat_current);
  residual[3] = relative_quat_target[0];
  residual[4] = relative_quat_target[1];
  residual[5] = relative_quat_target[2];
}

}  // namespace custom_mpc_behavior
