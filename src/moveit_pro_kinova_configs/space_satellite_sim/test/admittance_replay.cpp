// Copyright 2026 PickNik Inc.
// SPDX-License-Identifier: BSD-3-Clause

#include <Eigen/Cholesky>
#include <joint_trajectory_admittance_controller/admittance.hpp>
#include <memory>

// Minimal C ABI for Python physics tests. The installed controller owns admittance
// integration; the test supplies MuJoCo transforms and damped least-squares kinematics.
using namespace moveit_pro_controllers;
using Row44 = Eigen::Matrix<double, 4, 4, Eigen::RowMajor>;
using Row67 = Eigen::Matrix<double, 6, 7, Eigen::RowMajor>;
struct Replay
{
  Eigen::Isometry3d ref = Eigen::Isometry3d::Identity(), actual = ref;
  Eigen::Matrix<double, 6, 7> jac;
  std::unique_ptr<Admittance> controller;
  Replay(const double* mass, const double* damping, const double* stiffness, const double* axes, double joint_damping)
  {
    AdmittanceParameters p;
    p.mass_inv = Eigen::Map<const Eigen::Vector6d>(mass).cwiseInverse();
    p.damping = Eigen::Map<const Eigen::Vector6d>(damping);
    p.stiffness = Eigen::Map<const Eigen::Vector6d>(stiffness);
    p.selected_axes = Eigen::Map<const Eigen::Vector6d>(axes);
    AdmittanceKinematics k;
    k.fk = [this](const auto&, const auto&, AdmittanceState& s) {
      s.base_poses_ft_reference[0] = ref;
      s.base_poses_ft[0] = actual;
      // grasp_link is 0.15 m beyond fts_link, with the gripper rotated pi about Z.
      s.base_poses_ee[0] = actual;
      s.base_poses_ee[0].linear() = actual.linear() * Eigen::Vector3d(-1.0, -1.0, 1.0).asDiagonal();
      s.base_poses_ee[0].translation() += actual.linear() * Eigen::Vector3d(0.0, 0.0, 0.15);
      return true;
    };
    k.diff_ik = [this](const auto&, const auto& x, Eigen::VectorNd& q) {
      q = jac.transpose() *
          (jac * jac.transpose() + 0.0001 * Eigen::Matrix<double, 6, 6>::Identity()).ldlt().solve(x[0]);
      return true;
    };
    k.diff_fk = [this](const auto&, const auto& q, auto& x) {
      x[0] = jac * q;
      return true;
    };
    controller = std::make_unique<Admittance>(7, 1, std::vector<AdmittanceParameters>{ p },
                                              Eigen::VectorNd::Constant(7, joint_damping), k);
  }
};
extern "C" void* replay_create(const double* m, const double* d, const double* k, const double* axes,
                               double joint_damping)
{ return new Replay(m, d, k, axes, joint_damping); }
extern "C" void replay_destroy(void* p)
{ delete static_cast<Replay*>(p); }
extern "C" int replay_step(void* p, const double* q, const double* r, const double* w, const double* tr,
                           const double* tq, const double* j, double dt, double* out)
{
  auto& a = *static_cast<Replay*>(p);
  a.ref.matrix() = Eigen::Map<const Row44>(tr);
  a.actual.matrix() = Eigen::Map<const Row44>(tq);
  a.jac = Eigen::Map<const Row67>(j);
  TimedJointState current, reference, desired;
  current.positions = Eigen::Map<const Eigen::VectorNd>(q, 7);
  reference.positions = Eigen::Map<const Eigen::VectorNd>(r, 7);
  reference.velocities = Eigen::VectorNd::Zero(7);
  reference.accelerations = Eigen::VectorNd::Zero(7);
  auto status =
      a.controller->update(current, { Eigen::Map<const Eigen::Vector6d>(w) }, reference, Duration(dt), desired);
  Eigen::Map<Eigen::VectorNd>(out, 7) = desired.positions;
  return status == AdmittanceResult::SUCCESS;
}
