#include <pcl_decompose_to_spheres/pcl_decompose_to_spheres.hpp>
#include "spdlog/spdlog.h"
#include <sensor_msgs/msg/detail/point_cloud2__struct.hpp>
#include <moveit_msgs/msg/detail/collision_object__struct.hpp>
#include <moveit_studio_behavior_interface/get_required_ports.hpp>
#include <moveit_studio_behavior_interface/metadata_fields.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/voxel_grid.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <Eigen/Geometry>

namespace
{
inline constexpr auto kDescriptionPclDecomposeToSpheres= R"(
                <p>
                    This behavior receives point cloud data representing environment obstacles 
                    and re-represents them as spheres within a distance threshold of the tool frame.
                    It ignores data greater than the specified distance threshold.
                    The spheres are returned as an moveit_msgs::msg::CollisionObject message.
                </p>
            )";
constexpr auto kPortIDPointCloud = "point_cloud";
constexpr auto kPortIdEndEffectorFrame= "grasp_link";
constexpr auto kPortXYDistThreshold = "xy_thresh";
constexpr auto kPortZDistThreshold = "z_thresh";
constexpr auto kPortSphereRadius = "sphere_radius";
constexpr auto kPortIDCollisionObject = "collision_object";
}  // namespace

namespace pcl_decompose_to_spheres
{

// Helper: Convert PointCloud2 to PCL PointCloud
pcl::PointCloud<pcl::PointXYZ>::Ptr PclDecomposeToSpheres::convertToPCL(const sensor_msgs::msg::PointCloud2& cloud_msg)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(cloud_msg, *pcl_cloud);
  return pcl_cloud;
}

// Helper: Filter points within XY and Z thresholds from gripper position
pcl::PointCloud<pcl::PointXYZ>::Ptr PclDecomposeToSpheres::filterPoints(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& pcl_cloud,
    const geometry_msgs::msg::Vector3& gripper_pos,
    double xy_thresh, double z_thresh)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
  for (const auto& pt : pcl_cloud->points)
  {
    float dx = pt.x - gripper_pos.x;
    float dy = pt.y - gripper_pos.y;
    float dz = pt.z - gripper_pos.z;
    float xy_dist = std::sqrt(dx * dx + dy * dy);
    if (xy_dist <= xy_thresh && std::abs(dz) <= z_thresh)
    {
      filtered->points.push_back(pt);
    }
  }
  filtered->width = filtered->points.size();
  filtered->height = 1;
  filtered->is_dense = true;
  return filtered;
}

// Helper: Cluster filtered points and create spheres
moveit_msgs::msg::CollisionObject PclDecomposeToSpheres::createSpheres(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& filtered_cloud,
    double sphere_radius,
    const std::string& frame_id)
{
  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = frame_id;
  collision_object.id = "spheres";
  collision_object.operation = moveit_msgs::msg::CollisionObject::ADD;

  if (filtered_cloud->points.empty())
    return collision_object;

  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(filtered_cloud);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(sphere_radius);
  ec.setMinClusterSize(1);
  ec.setMaxClusterSize(1000);
  ec.setSearchMethod(tree);
  ec.setInputCloud(filtered_cloud);
  ec.extract(cluster_indices);

  for (const auto& indices : cluster_indices)
  {
    double cx = 0;
    double cy = 0;
    double cz = 0;
    for (int idx : indices.indices)
    {
      cx += filtered_cloud->points[idx].x;
      cy += filtered_cloud->points[idx].y;
      cz += filtered_cloud->points[idx].z;
    }
    size_t n = indices.indices.size();
    cx /= n; 
    cy /= n; 
    cz /= n;

    shape_msgs::msg::SolidPrimitive sphere;
    sphere.type = shape_msgs::msg::SolidPrimitive::SPHERE;
    sphere.dimensions.resize(1);
    sphere.dimensions[shape_msgs::msg::SolidPrimitive::SPHERE_RADIUS] = sphere_radius;

    geometry_msgs::msg::Pose pose;
    pose.position.x = cx;
    pose.position.y = cy;
    pose.position.z = cz;
    pose.orientation.w = 1.0;

    collision_object.primitives.push_back(sphere);
    collision_object.primitive_poses.push_back(pose);
  }
  return collision_object;
}


// Helper: Transform PCL point cloud to target frame using TF2
pcl::PointCloud<pcl::PointXYZ>::Ptr PclDecomposeToSpheres::transformCloudToFrame(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    const std::string& source_frame,
    const std::string& target_frame,
    const rclcpp::Time& stamp,
    const std::shared_ptr<tf2_ros::Buffer>& tf_buffer,
    const std::string& node_name)
{
  if (source_frame == target_frame)
    return cloud;

  Eigen::Isometry3d tf_eigen;
  try
  {
    geometry_msgs::msg::TransformStamped tf_msg =
      tf_buffer->lookupTransform(target_frame, source_frame, stamp, rclcpp::Duration::from_seconds(0.5));
    tf_eigen = tf2::transformToEigen(tf_msg.transform);
  }
  catch (const tf2::TransformException& ex)
  {
     shared_resources_->logger->publishFailureMessage(
      node_name,
      std::string("Failed to lookup transform from '") + target_frame + "' to '" + source_frame + "': " + ex.what()
    );
    return nullptr;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed(new pcl::PointCloud<pcl::PointXYZ>);
  transformed->reserve(cloud->size());
  for (const auto& pt : cloud->points)
  {
    Eigen::Vector3d p(pt.x, pt.y, pt.z);
    Eigen::Vector3d p_tf = tf_eigen * p;
    transformed->points.emplace_back(static_cast<float>(p_tf.x()), static_cast<float>(p_tf.y()), static_cast<float>(p_tf.z()));
  }
  transformed->width = transformed->points.size();
  transformed->height = 1;
  transformed->is_dense = true;
  return transformed;
}


PclDecomposeToSpheres::PclDecomposeToSpheres(
    const std::string& name, const BT::NodeConfiguration& config,
    const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources)
  : moveit_studio::behaviors::SharedResourcesNode<BT::SyncActionNode>(name, config, shared_resources)
{
}

BT::PortsList PclDecomposeToSpheres::providedPorts()
{
  return BT::PortsList({
      BT::InputPort<sensor_msgs::msg::PointCloud2>(kPortIDPointCloud, "{point_cloud}",
                                 "Contains the sensor_msgs::msg::PointCloud2 input message representing current obstacles."),
      BT::InputPort<std::string>(kPortIdEndEffectorFrame, "{grasp_link}",
                                 "The TF frame of the tool, which is used as the reference frame for"
                                 " distance thresholding."),
      BT::InputPort<double>(kPortXYDistThreshold,"{xy_thresh}",
                            "Distance threshold in meters, specified for the range of positions "
                            "to convert to spheres within this ellipsoidal distances of the tool frame."),
      BT::InputPort<double>(kPortZDistThreshold, "{z_thresh}",
                            "Distance threshold in meters, specified for the range of positions"
                            " to convert to spheres within this ellipsoidal distances of the tool frame."),
      BT::InputPort<double>(kPortSphereRadius, "{sphere_radius}",
                            "Radius of the spheres to represent clusters of points."),
      BT::OutputPort<moveit_msgs::msg::CollisionObject>(kPortIDCollisionObject, "{collision_object}",
                                                        "The created collision object, populated with spheres.") 
  });
}

BT::KeyValueVector PclDecomposeToSpheres::metadata()
{
  return { { moveit_studio::behaviors::kSubcategoryMetadataKey, "Perception - Point Cloud Downsampling to Spheres" }, //????
           { moveit_studio::behaviors::kDescriptionMetadataKey, kDescriptionPclDecomposeToSpheres } };

}

BT::NodeStatus PclDecomposeToSpheres::tick()
{
  const auto ports = moveit_studio::behaviors::getRequiredInputs(
    getInput<sensor_msgs::msg::PointCloud2>(kPortIDPointCloud),
    getInput<std::string>(kPortIdEndEffectorFrame),
    getInput<double>(kPortXYDistThreshold), 
    getInput<double>(kPortZDistThreshold),
    getInput<double>(kPortSphereRadius)); 

  if (!ports.has_value())
  {
    shared_resources_->logger->publishFailureMessage(name(), "Failed to get required value from input data port: " +
                                                     ports.error());
    return BT::NodeStatus::FAILURE;
  }

  const auto& [input_cloud_msg, tool_frame, xy_thresh, z_thresh, sphere_radius] = ports.value();

  const std::string pcl_frame = input_cloud_msg.header.frame_id;
  if (pcl_frame.empty())
  {
    shared_resources_->logger->publishFailureMessage(
      name(),
      "Point cloud frame_id is empty. Cannot proceed without a valid frame."
    );
    return BT::NodeStatus::FAILURE;
  }

  const std::string world_frame = "world";

  auto pcl_cloud = convertToPCL(input_cloud_msg);

  // Transform cloud to world frame if needed
  if (pcl_frame != world_frame)
  {
    shared_resources_->logger->publishWarnMessage(
      name(),
      "Point cloud is not in the world frame. Attempting to transform to world frame."
    );
    pcl_cloud = transformCloudToFrame(
      pcl_cloud, pcl_frame, world_frame, input_cloud_msg.header.stamp,
      shared_resources_->transform_buffer_ptr, name());
    if (!pcl_cloud)
      return BT::NodeStatus::FAILURE;
  }

  // --- Figure out leaf size based on thresholds and add a small margin ---
  double min_thresh = std::min(xy_thresh, z_thresh);
  double leaf_size = min_thresh / 10.0;
  double xy_thresh_margin = xy_thresh + leaf_size;
  double z_thresh_margin = z_thresh + leaf_size;

  spdlog::info("Voxel grid leaf size set to: {}", leaf_size);

  // Downsample the cloud first using a voxel grid
  pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
  voxel_filter.setInputCloud(pcl_cloud);
  voxel_filter.setLeafSize(leaf_size, leaf_size, leaf_size);
  pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  voxel_filter.filter(*downsampled_cloud);

  // ---find the transform of the gripper frame in the world frame----
  geometry_msgs::msg::TransformStamped gripper_tf;
  try
  {
    gripper_tf = shared_resources_->transform_buffer_ptr->lookupTransform(
      world_frame, tool_frame, input_cloud_msg.header.stamp, rclcpp::Duration::from_seconds(0.5));
  }
  catch (const tf2::TransformException& ex)
  {
    shared_resources_->logger->publishFailureMessage(
      name(),
      std::string("Failed to lookup transform from '") + world_frame + "' to '" + tool_frame + "': " + ex.what()
    );
    return BT::NodeStatus::FAILURE;
  }

  const auto& gripper_pos = gripper_tf.transform.translation;
  spdlog::info("Gripper position in world frame: x={}, y={}, z={}", gripper_pos.x, gripper_pos.y, gripper_pos.z);

  //grab the downsampled points within the threshold of the gripper
  auto filtered_cloud = filterPoints(downsampled_cloud, gripper_pos, xy_thresh_margin, z_thresh_margin);

  // Log the number of points within the threshold
  spdlog::info("Number of points within threshold: {}", filtered_cloud->points.size());

  // Create spheres from the filtered points
  auto collision_object = createSpheres(filtered_cloud, sphere_radius, world_frame);

  setOutput(kPortIDCollisionObject, collision_object);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace pcl_decompose_to_spheres
