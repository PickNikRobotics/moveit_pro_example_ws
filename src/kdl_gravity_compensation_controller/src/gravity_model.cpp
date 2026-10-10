// Copyright 2026 PickNik Inc.
// SPDX-License-Identifier: BSD-3-Clause

#include "kdl_gravity_compensation_controller/gravity_model.hpp"

#include <stdexcept>

#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>

namespace kdl_gravity_compensation_controller
{

GravityModel::GravityModel(const std::string& urdf, const std::string& root, const std::string& tip,
                           const KDL::Vector& gravity, double payload_mass, const KDL::Vector& payload_com)
{
  KDL::Tree tree;
  if (!kdl_parser::treeFromString(urdf, tree))
  {
    throw std::runtime_error("Failed to parse the robot description into a KDL tree");
  }
  if (!tree.getChain(root, tip, chain_))
  {
    throw std::runtime_error("The robot description has no chain from '" + root + "' to '" + tip + "'");
  }
  KDL::Segment& last = chain_.segments.back();
  last.setInertia(last.getInertia() + KDL::RigidBodyInertia(payload_mass, payload_com));

  for (const KDL::Segment& segment : chain_.segments)
  {
    if (segment.getJoint().getType() != KDL::Joint::None)
    {
      joint_names_.push_back(segment.getJoint().getName());
    }
  }
  dyn_ = std::make_unique<KDL::ChainDynParam>(chain_, gravity);
}

bool GravityModel::Compute(const KDL::JntArray& q, KDL::JntArray& tau)
{
  return dyn_->JntToGravity(q, tau) >= 0;
}

}  // namespace kdl_gravity_compensation_controller
