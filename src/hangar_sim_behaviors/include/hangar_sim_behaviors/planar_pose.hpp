// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <geometry_msgs/msg/pose.hpp>
#include <tf2/utils.hpp>

#include <cmath>

/**
 * @file
 * @brief The one projection from a 6-DOF pose to the plane the 2D localizer works in.
 *
 * Kept in a single place on purpose. The seed the filter is given and the poses that are later
 * measured against that seed have to be flattened the same way, or a gate comparing them measures
 * the flattening rather than the thing it claims to bound. This lives outside localization_gates.hpp
 * only so that header stays free of ROS message types.
 */
namespace hangar_sim_behaviors::localization
{
/**
 * @brief Drop everything the 2D localizer does not carry: keep x, y and yaw, zero z, roll and pitch.
 *
 * A pose picked off a surface carries whatever roll and pitch that surface had, and whatever height
 * the click landed at. Passing that through would leave a quaternion whose yaw is not the yaw that
 * was meant, and a z term that no 2D estimate can ever match.
 */
inline geometry_msgs::msg::Pose projectToPlane(const geometry_msgs::msg::Pose& pose)
{
  geometry_msgs::msg::Pose planar;
  planar.position.x = pose.position.x;
  planar.position.y = pose.position.y;
  planar.position.z = 0.0;

  const double yaw = tf2::getYaw(pose.orientation);
  planar.orientation.x = 0.0;
  planar.orientation.y = 0.0;
  planar.orientation.z = std::sin(yaw * 0.5);
  planar.orientation.w = std::cos(yaw * 0.5);
  return planar;
}

}  // namespace hangar_sim_behaviors::localization
