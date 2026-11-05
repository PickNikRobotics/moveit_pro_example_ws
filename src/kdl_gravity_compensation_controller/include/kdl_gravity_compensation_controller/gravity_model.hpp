// Copyright 2026 PickNik Inc.
// SPDX-License-Identifier: BSD-3-Clause

#pragma once

#include <memory>
#include <string>
#include <vector>

#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/jntarray.hpp>

namespace kdl_gravity_compensation_controller
{

// Gravity torques for the serial chain root -> tip of a URDF, from
// KDL::ChainDynParam::JntToGravity. The torques are what each joint must
// apply to hold the chain still, in the URDF joint frame.
class GravityModel
{
public:
  // gravity: acceleration due to gravity in the root frame, m/s^2. A base
  //   mounted on a wall or upside down changes it, not the URDF.
  // payload_mass / payload_com: a point mass (kg) at payload_com (m, tip
  //   frame), added to the tip link's own inertia.
  // Throws std::runtime_error when the URDF does not parse or has no such chain.
  GravityModel(const std::string& urdf, const std::string& root, const std::string& tip, const KDL::Vector& gravity,
               double payload_mass, const KDL::Vector& payload_com);

  // ChainDynParam keeps a reference to chain_, so this must stay put.
  GravityModel(const GravityModel&) = delete;
  GravityModel& operator=(const GravityModel&) = delete;

  // Movable joints along the chain, root first.
  const std::vector<std::string>& joint_names() const
  {
    return joint_names_;
  }

  // q and tau must both have joint_names().size() rows. Returns false on a
  // solver error. Allocation-free, so safe in a realtime loop.
  bool Compute(const KDL::JntArray& q, KDL::JntArray& tau);

private:
  KDL::Chain chain_;
  std::vector<std::string> joint_names_;
  std::unique_ptr<KDL::ChainDynParam> dyn_;
};

}  // namespace kdl_gravity_compensation_controller
