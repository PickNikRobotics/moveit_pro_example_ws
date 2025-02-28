#include <example_behaviors/T_pushing.hpp>

#include <vector>
#include <Eigen/Dense>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <moveit_studio_behavior_interface/get_required_ports.hpp>
#include <moveit_studio_vision/utils/tf_utils.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include "OsqpEigen/OsqpEigen.h"


namespace
{
  // Port names for input and output ports.
  constexpr auto kPortIDTwistStamped = "twist_stamped";
  constexpr auto kPortIDWrenchStamped = "wrench_stamped";
} // namespace

namespace example_behaviors
{
  std::tuple<std::vector<Eigen::Vector3d>, Eigen::Vector3d> get_points(const Eigen::Matrix<double, 6, 6>& T)
  {
    std::vector<Eigen::Vector3d> points;
    Eigen::Vector3d com = Eigen::Vector3d::Zero(3, 1);
    for (long y_ind = 0; y_ind < T.rows(); ++y_ind)
    {
      for (long x_ind = 0; x_ind < T.cols(); ++x_ind)
      {
        if (T(y_ind, x_ind) > 0)
        {
          Eigen::Vector3d arr;
          arr << x_ind - T.cols() / 2 + .5, -(y_ind - T.rows() / 2) - .5, 0.0;
          arr *= 0.0284262;
          points.push_back(arr);
          com += arr;
        }
      }
    }
    com = com / points.size();
    for (auto& point : points)
    {
      point -= com;
    }
    com = Eigen::Vector3d::Zero(3, 1);

    return {points, com};
  }

  std::tuple<Eigen::Vector3d, Eigen::Vector3d> get_eqs(const Eigen::Vector3d& point, const Eigen::Vector3d& com)
  {
    Eigen::Vector3d diff = point - com;
    Eigen::Vector3d rot_axis;
    rot_axis << 0, 0, 1;
    Eigen::Vector3d rot_vel = diff.cross(rot_axis);
    Eigen::Vector3d row_x;
    row_x << 1, 0, -rot_vel[0]; // (p-com)w_x + xd
    Eigen::Vector3d row_y;
    row_y << 0, 1, -rot_vel[1]; // (p-com)w_x + xd
    return {row_x, row_y};
  }

  T_pushing::T_pushing(const std::string& name, const BT::NodeConfiguration& config,
                       const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources)
    : AsyncBehaviorBase(name, config, shared_resources)
  {
    marker_publisher_ = shared_resources_->node->create_publisher<visualization_msgs::msg::MarkerArray>(
      "visual_markers", rclcpp::SystemDefaultsQoS());


    // instantiate the solver_

    Eigen::Matrix<double, 6, 6> T;
    T << 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 1, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 1, 1, 0, 0;

    auto [points, com] = get_points(T);
    com_ = com;

    auto M = Eigen::Matrix<double, Eigen::Dynamic, 3>(2 * points.size(), 3);
    auto a = Eigen::VectorXd(2 * points.size());
    int row = 0;
    for (const auto& point : points)
    {
      auto [row_x, row_y] = get_eqs(point, com_);
      M(row, 0) = row_x[0];
      M(row, 1) = row_x[1];
      M(row, 2) = row_x[2];
      a[row] = 0;
      row++;
      M(row, 0) = row_y[0];
      M(row, 1) = row_y[1];
      M(row, 2) = row_y[2];
      a[row] = 0;
      row++;
    }

    // set the initial data of the QP solver_
    solver_.data()->setNumberOfVariables(3);
    solver_.data()->setNumberOfConstraints(1);

    gradient_ = -2 * (M.transpose() * a); // c = -2 * M^T a
    Eigen::Matrix<double, 3, 3> Q = 2 * (M.transpose() * M); // Q = 2 * M^T M
    hessian_ = Q.sparseView();

    if (!solver_.data()->setHessianMatrix(hessian_))
      throw std::runtime_error("Failed to set Hessian");
    if (!solver_.data()->setGradient(gradient_))
      throw std::runtime_error("Failed to set gradient");
  }

  Eigen::Vector3d T_pushing::get_velocities(const Eigen::Vector3d& rp, const Eigen::Vector3d& vp,
                                            const Eigen::Vector3d& N)
  {
    auto [row_x, row_y] = get_eqs(rp, com_);
    Eigen::Matrix<double, 1, 3> A;
    A(0, 0) = N[0] * row_x[0] + N[1] * row_y[0];
    A(0, 1) = N[0] * row_x[1] + N[1] * row_y[1];
    A(0, 2) = -N[0] * row_x[2] - N[1] * row_y[2];
    // N and vp should be opposite
    upperBound_[0] = N[0] * vp[0] + N[1] * vp[1];
    lowerBound_[0] = N[0] * vp[0] + N[1] * vp[1];
    linearMatrix_ = A.sparseView();

    if (upperBound_[0] > 0.0)
    {
      throw std::logic_error("bound should be negative");
    }

    solver_.data()->clearLinearConstraintsMatrix();
    if (!solver_.data()->setLinearConstraintsMatrix(linearMatrix_))
    {
      throw std::runtime_error("Failed to set linearMatrix");
    }
    if (!solver_.data()->setLowerBound(lowerBound_))
    {
      throw std::runtime_error("Failed to set lowerBound");
    }
    if (!solver_.data()->setUpperBound(upperBound_))
    {
      throw std::runtime_error("Failed to set upperBound");
    }
    // instantiate the solver
    if (!solver_.isInitialized() && !solver_.initSolver())
    {
      throw std::runtime_error("Failed to init solver");
    }

    // solve the QP problem
    if (solver_.solveProblem() != OsqpEigen::ErrorExitFlag::NoError)
    {
      throw std::runtime_error("Failed to solve the problem");
    }

    auto QPSolution = solver_.getSolution();
    Eigen::Vector3d vec = QPSolution.block(0, 0, 3, 0);


    return vec;
  }

  BT::PortsList T_pushing::providedPorts()
  {
    // This node has no input or output ports
    return BT::PortsList({
      BT::OutputPort<geometry_msgs::msg::TwistStamped>(kPortIDTwistStamped, "{desired_twist}",
                                                       "Desired twist in the velocity controlled axes."),
      BT::OutputPort<geometry_msgs::msg::WrenchStamped>(kPortIDWrenchStamped, "{desired_wrench}", "mcok."),
    });
  }

  BT::KeyValueVector T_pushing::metadata()
  {
    return {{"subcategory", "Example Behaviors"}, {"description", "Push the T"}};
  }

  tl::expected<bool, std::string> T_pushing::doWork()
  {
    const auto ports = moveit_studio::behaviors::getRequiredInputs();

    // If a port was set incorrectly, log an error message and return FAILURE
    if (!ports.has_value())
    {
      return tl::make_unexpected(fmt::format("Failed to get required value from input data port: {}", ports.error()));
    }
    // const auto& [] = ports.value();

    std::vector<Eigen::Isometry3d> site_transforms;
    Eigen::Isometry3d ee_world_transform;
    Eigen::Isometry3d com_world_transform;
    Eigen::Isometry3d target_com_transform;
    try
    {
      com_world_transform = tf2::transformToEigen(
        shared_resources_->transform_buffer_ptr->lookupTransform("world", "com", tf2::TimePointZero));

      ee_world_transform = tf2::transformToEigen(
        shared_resources_->transform_buffer_ptr->lookupTransform("world", "grasp_link", tf2::TimePointZero));
      target_com_transform = tf2::transformToEigen(
        shared_resources_->transform_buffer_ptr->lookupTransform("com", "target", tf2::TimePointZero));

      for (int i = 1; i <= 8; ++i)
      {
        auto tf_msg = shared_resources_->transform_buffer_ptr->lookupTransform("com", fmt::format("point_{}", i),
                                                                               tf2::TimePointZero);
        site_transforms.push_back(tf2::transformToEigen(tf_msg));
      }
    }
    catch (tf2::TransformException& ex)
    {
      return tl::make_unexpected(ex.what());
    }

    double min_cost = 1E99;
    Eigen::Vector3d velocities = Eigen::Vector3d::Zero();
    for (size_t i = 0; i < site_transforms.size() + 1; ++i)
    {
      const auto& TF1 = site_transforms[i];
      const auto& TF2 = site_transforms[(i + 1) % site_transforms.size()];
      Eigen::Vector3d going_direction = TF2.translation() - TF1.translation();
      going_direction.normalize();
      Eigen::Vector3d z_direction;
      z_direction << 0, 0, 1;
      Eigen::Vector3d N = going_direction.cross(z_direction);
      Eigen::Vector3d vp = -N;
      double t = Eigen::Matrix<double, 1, 1>::Random()(0, 0);
      t = 0.5 * t + 0.5;
      Eigen::Vector3d rp = t * TF1.translation() + (1 - t) * TF2.translation();
      velocities = get_velocities(rp, vp, N);

      auto xd = velocities[0];
      auto yd = velocities[1];
      auto thetad = velocities[2];

      // double current_theta = com_world_transform.linear().eulerAngles(0, 1, 2)[2];
      // double target_theta = target_com_transform.linear().eulerAngles(0, 1, 2)[2];
      // double theta_diff = acos(cos(target_theta)*cos(current_theta) + sin(target_theta)*sin(current_theta));
      double theta_diff = target_com_transform.rotation().eulerAngles(2, 1, 0)[0];

      Eigen::Vector3d pos_diff = target_com_transform.translation();
      double x_diff = pos_diff[0];
      double y_diff = pos_diff[1];

      // min ||t*xd - x_diff || + ||t*yd - y_diff || + ||t*thetad - theta_diff ||
      // Constructing A and b
      Eigen::Vector3d A(xd, yd, thetad);
      Eigen::Vector3d b(x_diff, y_diff, theta_diff);

      // Solving for t using SVD
      double x = A.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV).solve(b)[0];
      double cost = (A * x - b).norm();
      if (cost < min_cost && x > 0.0)
      {
        min_cost = cost;
      }
      int o = 0;
    }

    Eigen::Vector3d velocity;
    velocity << 0, 0, 0.02;
    velocity = ee_world_transform.linear().inverse() * velocity;

    geometry_msgs::msg::TwistStamped msg;
    msg.header.frame_id = "world";
    msg.header.stamp = shared_resources_->node->now();
    msg.twist.linear.x = velocity[0];
    msg.twist.linear.y = velocity[1];
    msg.twist.linear.z = velocity[2];

    setOutput(kPortIDTwistStamped, msg);
    geometry_msgs::msg::WrenchStamped wrench_msg;
    wrench_msg.header = msg.header;

    setOutput(kPortIDWrenchStamped, wrench_msg);

    return true;
  }
} // namespace example_behaviors
