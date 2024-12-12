#include <example_behaviors/example_ransac_registration.hpp>
#include <pcl/registration/ndt.h>
#include <string>
#include <tl_expected/expected.hpp>
#include <moveit_studio_behavior_interface/async_behavior_base.hpp>
#include <moveit_studio_behavior_interface/check_for_error.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tl_expected/expected.hpp>
#include <moveit_studio_vision_msgs/msg/mask3_d.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <moveit_studio_vision/common/get_all_indices.hpp>
#include <moveit_studio_vision/common/select_point_indices.hpp>
#include <moveit_studio_vision/pointcloud/point_cloud_tools.hpp>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/search/kdtree.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>

namespace example_behaviors
{
namespace
{

std::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>>
getFilteredPointCloudFromMessage(const sensor_msgs::msg::PointCloud2& cloud_msg)
{
  const auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
  pcl::fromROSMsg(cloud_msg, *cloud);

  const pcl::PointIndices valid_indices = moveit_studio::selectPointIndices(*cloud, moveit_studio::point_cloud_tools::getAllIndices(cloud),
                                                             moveit_studio::NeitherNanNorNearZeroPointValidator<pcl::PointXYZRGB>);
  auto filtered_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
  pcl::copyPointCloud(*cloud, valid_indices, *filtered_cloud);

  return filtered_cloud;
}

}  // namespace

ExampleRANSACRegistration::ExampleRANSACRegistration(
    const std::string& name, const BT::NodeConfiguration& config,
    const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources)
  : moveit_studio::behaviors::AsyncBehaviorBase(name, config, shared_resources)
{
}


BT::PortsList ExampleRANSACRegistration::providedPorts()
{
  return { BT::InputPort<sensor_msgs::msg::PointCloud2>("base_point_cloud", "{point_cloud}",
                                                        "Point cloud to align with the target point cloud."),
           BT::InputPort<sensor_msgs::msg::PointCloud2>("target_point_cloud", "{target_point_cloud}",
                                                        "Point cloud to which align the base point cloud."),
           BT::InputPort<int>("max_iterations", 100,
                              "Maximum number of attempts to find the transform. Setting a higher number of iterations "
                              "will allow the solver to converge even if the initial estimate of the transform was far "
                              "from the actual transform, but it may take longer to complete."),
           BT::InputPort<double>("transformation_epsilon", 0.001,
                                 "Minimum transformation difference for termination condition <double>"),
           BT::InputPort<double>("step_size", 0.1,
                                 "Maximum step size for More-Thuente line search <double>"),
           BT::InputPort<double>("resolution", 1.0,
                                 "Resolution of NDT grid structure (VoxelGridCovariance) <double>"),
           BT::InputPort<double>("max_correspondence_distance", 1.0,
                                 "Maximum correspondence distance for RANSAC <double>"),
           BT::InputPort<double>("uniform_sampling_radius", 0.02,
                                 "Radius for uniform sampling of keypoints <double>"),
           BT::InputPort<int>("k_search", 10,
                              "Number of nearest neighbors to use for normal estimation <int>"),
           BT::InputPort<double>("feature_radius", 0.05,
                                 "Radius for feature estimation <double>"),
           BT::InputPort<double>("max_allowable_fitness", 0.0001,
                                 "Maximum fitness score, above which this behavior will return failure. Set to 0 or negative to disable"),
           BT::OutputPort<geometry_msgs::msg::PoseStamped>("target_pose_in_base_frame", "{target_pose}",
                                                           "The pose of the target point cloud relative to the frame "
                                                           "of the base point cloud.") };
}

BT::KeyValueVector ExampleRANSACRegistration::metadata()
{
  return { {"subcategory", "Perception - 3D Point Cloud"}, {"description", "Finds the pose of a target point cloud relative to the base frame of a base point cloud using the Normal Distributions Transform (NDT) algorithm"} };
}


tl::expected<bool, std::string> ExampleRANSACRegistration::doWork()
{

  const auto base_point_cloud_msg = getInput<sensor_msgs::msg::PointCloud2>("base_point_cloud");
  const auto target_point_cloud_msg = getInput<sensor_msgs::msg::PointCloud2>("target_point_cloud");
  const auto max_iterations = getInput<int>("max_iterations");
  const auto transformation_epsilon = getInput<double>("transformation_epsilon");
  const auto max_correspondence_distance = getInput<double>("max_correspondence_distance");
  const auto uniform_sampling_radius = getInput<double>("uniform_sampling_radius");
  const auto k_search = getInput<int>("k_search");
  const auto feature_radius = getInput<double>("feature_radius");
  const auto max_allowable_fitness = getInput<double>("max_allowable_fitness");

  if (const auto error = moveit_studio::behaviors::maybe_error(base_point_cloud_msg, target_point_cloud_msg, max_iterations, transformation_epsilon, max_correspondence_distance, uniform_sampling_radius, k_search, feature_radius); error)
  {
    RCLCPP_ERROR(rclcpp::get_logger("Logger"), "Failed to get required value from input data port: %s", error.value().c_str());
    return tl::make_unexpected("Failed to get required value from input data port: " + error.value());
  }

  const auto base_cloud = getFilteredPointCloudFromMessage(base_point_cloud_msg.value());
  if (base_cloud->empty())
  {
    RCLCPP_ERROR(rclcpp::get_logger("Logger"), "Base point cloud has no valid points");
    return tl::make_unexpected("Base point cloud has no valid points");
  }
  const auto target_cloud = getFilteredPointCloudFromMessage(target_point_cloud_msg.value());
  if (target_cloud->empty())
  {
    RCLCPP_ERROR(rclcpp::get_logger("Logger"), "Target point cloud has no valid points");
    return tl::make_unexpected("Target point cloud has no valid points");
  }

  // PCL alignment
  pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> align;
  align.setMaximumIterations(max_iterations.value());
  align.setInputSource(base_cloud);
  align.setInputTarget(target_cloud);
  align.setMaxCorrespondenceDistance(max_correspondence_distance.value());

  pcl::PointCloud<pcl::PointXYZRGB> output_cloud;
  align.align(output_cloud);

  if (!align.hasConverged())
  {
    RCLCPP_ERROR(rclcpp::get_logger("Logger"), "RANSAC could not converge to a transform that aligns both point clouds");
    return tl::make_unexpected("RANSAC could not converge to a transform that aligns both point clouds");
  }

  if (max_allowable_fitness.value() > 0 && align.getFitnessScore() > max_allowable_fitness.value()) {
    RCLCPP_INFO(rclcpp::get_logger("Logger"), "RANSAC converged to a solution with fitness %f, above the maximum allowable %f", align.getFitnessScore(), max_allowable_fitness.value());
    return tl::make_unexpected("RANSAC converged to a solution with fitness " + std::to_string(align.getFitnessScore()) + ", above the maximum allowable " + std::to_string(max_allowable_fitness.value()));
  }

  geometry_msgs::msg::PoseStamped out;
  out.pose = tf2::toMsg(Eigen::Isometry3d{ align.getFinalTransformation().cast<double>() });
  out.header.frame_id = base_point_cloud_msg.value().header.frame_id;
  out.header.stamp = base_point_cloud_msg.value().header.stamp;
  setOutput("target_pose_in_base_frame", out);

  return true;
}


}  // namespace example_behaviors
