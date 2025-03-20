#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <example_behaviors/example_pointcloud_subtraction.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <moveit_studio_behavior_interface/async_behavior_base.hpp>
#include <moveit_studio_behavior_interface/check_for_error.hpp>
#include <moveit_studio_vision/common/get_all_indices.hpp>
#include <moveit_studio_vision/common/select_point_indices.hpp>
#include <moveit_studio_vision/pointcloud/point_cloud_tools.hpp>
#include <moveit_studio_vision_msgs/msg/mask3_d.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>
#include <string>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tl_expected/expected.hpp>
#include <vector>

namespace example_behaviors
{
namespace
{
// This helper converts an input sensor_msgs::PointCloud2 message into a
// filtered PCL point cloud.
std::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>>
getFilteredPointCloudFromMessage(const sensor_msgs::msg::PointCloud2& cloud_msg)
{
  auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
  pcl::fromROSMsg(cloud_msg, *cloud);

  const pcl::PointIndices valid_indices =
      moveit_studio::selectPointIndices(*cloud, moveit_studio::point_cloud_tools::getAllIndices(cloud),
                                        moveit_studio::NeitherNanNorNearZeroPointValidator<pcl::PointXYZRGB>);
  auto filtered_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
  pcl::copyPointCloud(*cloud, valid_indices, *filtered_cloud);

  return filtered_cloud;
}
}  // namespace

ExamplePointCloudSubtraction::ExamplePointCloudSubtraction(
    const std::string& name, const BT::NodeConfiguration& config,
    const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources)
  : moveit_studio::behaviors::AsyncBehaviorBase(name, config, shared_resources)
{
}

BT::PortsList ExamplePointCloudSubtraction::providedPorts()
{
  return { BT::InputPort<sensor_msgs::msg::PointCloud2>("base_point_cloud", "{point_cloud}",
                                                        "The base point cloud from which points will be subtracted."),
           BT::InputPort<sensor_msgs::msg::PointCloud2>("subtraction_point_cloud", "{point_cloud}",
                                                        "The point cloud whose points will be subtracted from the base "
                                                        "cloud."),
           BT::InputPort<double>("search_radius", 0.01,
                                 "Radius used to search for matching points in "
                                 "the subtraction cloud (in meters)."),
           BT::OutputPort<sensor_msgs::msg::PointCloud2>("subtracted_point_cloud", "{point_cloud}",
                                                         "The result point cloud after subtraction.") };
}

BT::KeyValueVector ExamplePointCloudSubtraction::metadata()
{
  return { { "subcategory", "Perception - 3D Point Cloud" },
           { "description", "Subtracts one point cloud from another using a "
                            "radius-based KD-Tree search." } };
}

tl::expected<bool, std::string> ExamplePointCloudSubtraction::doWork()
{
  // Retrieve input ports.
  const auto base_point_cloud_msg = getInput<sensor_msgs::msg::PointCloud2>("base_point_cloud");
  const auto subtraction_point_cloud_msg = getInput<sensor_msgs::msg::PointCloud2>("subtraction_point_cloud");
  const auto search_radius = getInput<double>("search_radius");

  // Check for errors in the required inputs.
  if (const auto error =
          moveit_studio::behaviors::maybe_error(base_point_cloud_msg, subtraction_point_cloud_msg, search_radius);
      error)
  {
    RCLCPP_ERROR(rclcpp::get_logger("Logger"), "Failed to get required value from input data port: %s",
                 error.value().c_str());
    return tl::make_unexpected("Failed to get required value from input data port: " + error.value());
  }

  // Filter the input point clouds.
  const auto base_cloud = getFilteredPointCloudFromMessage(base_point_cloud_msg.value());
  if (base_cloud->empty())
  {
    RCLCPP_ERROR(rclcpp::get_logger("Logger"), "Base point cloud has no valid points");
    return tl::make_unexpected("Base point cloud has no valid points");
  }
  const auto subtract_cloud = getFilteredPointCloudFromMessage(subtraction_point_cloud_msg.value());
  if (subtract_cloud->empty())
  {
    RCLCPP_ERROR(rclcpp::get_logger("Logger"), "Subtraction point cloud has no valid points");
    return tl::make_unexpected("Subtraction point cloud has no valid points");
  }

  // Build a KD-Tree on the subtraction cloud.
  pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
  kdtree.setInputCloud(subtract_cloud);

  // Identify the indices in the base cloud that have a neighbor in the
  // subtraction cloud within the provided search radius.
  std::vector<int> indices_to_remove;
  for (size_t i = 0; i < base_cloud->points.size(); ++i)
  {
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
    if (kdtree.radiusSearch(base_cloud->points[i], search_radius.value(), pointIdxRadiusSearch,
                            pointRadiusSquaredDistance) > 0)
    {
      // A neighbor was found in the subtraction cloud: mark this point for
      // removal.
      indices_to_remove.push_back(static_cast<int>(i));
    }
  }

  // Remove the points from the base cloud that have a corresponding match.
  pcl::PointIndices::Ptr indices(new pcl::PointIndices());
  indices->indices = indices_to_remove;

  auto subtracted_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
  pcl::ExtractIndices<pcl::PointXYZRGB> extract;
  extract.setInputCloud(base_cloud);
  extract.setIndices(indices);
  extract.setNegative(true);  // Keep only the points that were not found in the
                              // subtraction cloud.
  extract.filter(*subtracted_cloud);

  // Convert the subtracted cloud back to a ROS message.
  sensor_msgs::msg::PointCloud2 output_msg;
  pcl::toROSMsg(*subtracted_cloud, output_msg);
  // Use the header of the base cloud for the output.
  output_msg.header = base_point_cloud_msg.value().header;

  setOutput("subtracted_point_cloud", output_msg);

  RCLCPP_INFO(rclcpp::get_logger("Logger"),
              "Subtraction complete: removed %ld points from base cloud, "
              "leaving %ld points",
              indices_to_remove.size(), subtracted_cloud->size());

  return true;
}

}  // namespace example_behaviors
