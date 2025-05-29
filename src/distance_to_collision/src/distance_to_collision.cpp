// Copyright 2025 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include "distance_to_collision/distance_to_collision.hpp"

#include "geometry_msgs/msg/point.hpp"
#include "pcl/kdtree/kdtree_flann.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "spdlog/spdlog.h"

#include "moveit_studio_behavior_interface/get_required_ports.hpp"

namespace distance_to_collision
{
DistanceToCollision::DistanceToCollision(
    const std::string& name, const BT::NodeConfiguration& config,
    const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources)
  : moveit_studio::behaviors::SharedResourcesNode<BT::SyncActionNode>(name, config, shared_resources)
{
}

BT::PortsList DistanceToCollision::providedPorts()
{
  return BT::PortsList(
      { BT::InputPort<sensor_msgs::msg::PointCloud2>(kPortIDPointCloud, "{point_cloud}",
                                                     "Will contain the output sensor_msgs::msg::PointCloud2 message "
                                                     "after this Behavior has finished successfully."),
        BT::InputPort<geometry_msgs::msg::Point>(kPortIDPoint, "{point}", "Point to check collision distance."),
        BT::OutputPort<double>(kPortIDDistance, "{distance}", "Shortest distance between the point and point cloud.") });
}

BT::KeyValueVector DistanceToCollision::metadata()
{
  return { { "description",
             "Provided a point cloud and a point, get the closest distance between the point and the point cloud" },
           { "subcategory", "User Created Behaviors" } };
}

BT::NodeStatus DistanceToCollision::tick()
{
  // ----------------------------------------
  // Load data from the behavior input ports.
  // ----------------------------------------
  const auto ports = moveit_studio::behaviors::getRequiredInputs(
      getInput<sensor_msgs::msg::PointCloud2>(kPortIDPointCloud), getInput<geometry_msgs::msg::Point>(kPortIDPoint));

  if (!ports.has_value())
  {
    shared_resources_->logger->publishFailureMessage(name(), "Failed to get required value from input data port: " +
                                                                 ports.error());
    return BT::NodeStatus::FAILURE;
  }

  const auto& [point_cloud, point] = ports.value();

  // Convert PointCloud2 to pcl::PointCloud<pcl::PointXYZ>
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(point_cloud, *pcl_cloud);

  // Build a KD-Tree for the point cloud
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud(pcl_cloud);

  // Query point
  pcl::PointXYZ search_point;
  search_point.x = point.x;
  search_point.y = point.y;
  search_point.z = point.z;

  // Find the nearest neighbor
  std::vector<int> point_idx_nkn_search(1);
  std::vector<float> point_nkn_squared_distance(1);

  double min_distance = std::numeric_limits<double>::infinity();
  if (kdtree.nearestKSearch(search_point, 1, point_idx_nkn_search, point_nkn_squared_distance) > 0)
  {
    min_distance = std::sqrt(point_nkn_squared_distance[0]);
    setOutput<double>(kPortIDDistance, min_distance);
    return BT::NodeStatus::SUCCESS;
  }
  else
  {
    shared_resources_->logger->publishFailureMessage(name(), "No nearest neighbor found in point cloud.");
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::SUCCESS;
}

}  // namespace distance_to_collision
