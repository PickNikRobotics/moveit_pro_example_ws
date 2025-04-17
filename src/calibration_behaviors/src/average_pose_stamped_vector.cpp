#include "calibration_behaviors/average_pose_stamped_vector.hpp"

#include <vector>

#include "Eigen/Eigenvalues"
#include "Eigen/Geometry"
#include "fmt/format.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_eigen/tf2_eigen.hpp"
#include "tl_expected/expected.hpp"

#include "moveit_studio_behavior_interface/get_required_ports.hpp"

namespace
{
constexpr auto kPortIDPoseStampedVector = "pose_stamped_vector";
constexpr auto kPortIDAveragePoseStamped = "average_pose_stamped";
}  // namespace

tl::expected<Eigen::Quaterniond, std::string>
averageQuaternions(const std::vector<geometry_msgs::msg::PoseStamped>& pose_samples)
{
  // Matrix to accumulate q * q' into
  Eigen::Matrix4d mat_accumulate = Eigen::Matrix4d::Zero();

  // Lambda to perform q * q' calculation
  const auto multiply = [&mat_accumulate](const auto pose) {
    Eigen::Quaterniond q(pose.pose.orientation.w, pose.pose.orientation.x, pose.pose.orientation.y,
                         pose.pose.orientation.z);
    mat_accumulate += q.coeffs() * q.coeffs().transpose();
  };

  // Apply calculation and accumulation to each quaternion
  std::for_each(pose_samples.begin(), pose_samples.end(), multiply);

  // Normalize the accumulated matrix by the weights (In this case all quaternions are weighted equally)
  mat_accumulate *= 1.0 / static_cast<double>(pose_samples.size());

  // Solve for Eigenvalues and Eigenvectors
  Eigen::EigenSolver<Eigen::Matrix4d> eigensolver(mat_accumulate);

  if (eigensolver.info() == Eigen::Success)
  {
    // Get the eigenvector associated with the maximum eigenvalue
    size_t idx;
    eigensolver.eigenvalues().real().maxCoeff(&idx);
    Eigen::Vector4d eigvector = eigensolver.eigenvectors().col(idx).real();

    // Need to convert to tf2::Quaternion object
    Eigen::Quaterniond avg_q(eigvector[3], eigvector[0], eigvector[1], eigvector[2]);

    // Normalize the quaternion
    avg_q.normalize();

    return avg_q;
  }
  else
  {
    return tl::make_unexpected("Failed to average quaternions.");
  }
}

namespace calibration_behaviors
{
AveragePoseStampedVector::AveragePoseStampedVector(
    const std::string& name, const BT::NodeConfiguration& config,
    const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources)
  : moveit_studio::behaviors::SharedResourcesNode<BT::SyncActionNode>(name, config, shared_resources)
{
}

BT::PortsList AveragePoseStampedVector::providedPorts()
{
  return BT::PortsList(
      { BT::InputPort<std::vector<geometry_msgs::msg::PoseStamped>>(kPortIDPoseStampedVector, "{pose_stamped_vector}",
                                                                    "Vector of PoseStamped objects to be averaged."),
        BT::OutputPort<geometry_msgs::msg::PoseStamped>(kPortIDAveragePoseStamped, "{average_pose_stamped}",
                                                        "Computed average of all the PoseStamped objects.") });
}

BT::KeyValueVector AveragePoseStampedVector::metadata()
{
  return { { "description", "Calibrate scene camera extrinsics" } };
}

BT::NodeStatus AveragePoseStampedVector::tick()
{
  const auto ports = moveit_studio::behaviors::getRequiredInputs(
      getInput<std::vector<geometry_msgs::msg::PoseStamped>>(kPortIDPoseStampedVector));

  if (!ports.has_value())
  {
    shared_resources_->logger->publishFailureMessage(name(), fmt::format("Missing input port: {}", ports.error()));
    return BT::NodeStatus::FAILURE;
  }

  const auto& [pose_vector] = ports.value();

  if (pose_vector.empty())
  {
    shared_resources_->logger->publishFailureMessage(name(), "Received empty pose vector.");
    return BT::NodeStatus::FAILURE;
  }

  // Create object to hold average pose.
  Eigen::Isometry3d average_pose_eigen = Eigen::Isometry3d::Identity();

  // Compute average translation.
  for (const auto& pose : pose_vector)
  {
    Eigen::Vector3d translation_eigen;
    tf2::fromMsg(pose.pose.position, translation_eigen);
    average_pose_eigen.translation() += translation_eigen;
  }
  average_pose_eigen.translation() /= pose_vector.size();

  // Compute average orientation.
  auto expected_average_quaternion = averageQuaternions(pose_vector);
  if (!expected_average_quaternion)
  {
    shared_resources_->logger->publishFailureMessage(name(), expected_average_quaternion.error());
    return BT::NodeStatus::FAILURE;
  }
  average_pose_eigen.linear() = expected_average_quaternion.value().toRotationMatrix();

  // Create the PoseStamped object to return.
  geometry_msgs::msg::PoseStamped average_pose;
  average_pose.header.frame_id = pose_vector.at(0).header.frame_id;
  average_pose.pose = tf2::toMsg(average_pose_eigen);

  setOutput(kPortIDAveragePoseStamped, average_pose);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace calibration_behaviors
