#pragma once

#include <behaviortree_cpp/action_node.h>
#include <moveit_studio_behavior_interface/get_required_ports.hpp>

// This header includes the SharedResourcesNode type
#include <moveit_studio_behavior_interface/shared_resources_node.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace pcl_decompose_to_spheres
{
/**
 * @brief TODO(...)
 */
class PclDecomposeToSpheres : public moveit_studio::behaviors::SharedResourcesNode<BT::SyncActionNode>
{
public:
  /**
   * @brief Constructor for the pcl_decompose_to_spheres behavior.
   * @param name The name of a particular instance of this Behavior. This will be set by the behavior tree factory when this Behavior is created within a new behavior tree.
   * @param shared_resources A shared_ptr to a BehaviorContext that is shared among all SharedResourcesNode Behaviors in the behavior tree. This BehaviorContext is owned by the Studio Agent's ObjectiveServerNode.
   * @param config This contains runtime configuration info for this Behavior, such as the mapping between the Behavior's data ports on the behavior tree's blackboard. This will be set by the behavior tree factory when this Behavior is created within a new behavior tree.
   * @details An important limitation is that the members of the base Behavior class are not instantiated until after the initialize() function is called, so these classes should not be used within the constructor.
   */
  PclDecomposeToSpheres(const std::string& name, const BT::NodeConfiguration& config, const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources);

  
  /**
   * @brief Implementation of the required providedPorts() function for the pcl_decompose_to_spheres Behavior.
   * @details The BehaviorTree.CPP library requires that Behaviors must implement a static function named providedPorts() which defines their input and output ports. If the Behavior does not use any ports, this function must return an empty BT::PortsList.
   * This function returns a list of ports with their names and port info, which is used internally by the behavior tree.
   * @return pcl_decompose_to_spheres does not expose any ports, so this function returns an empty list.
   */
  static BT::PortsList providedPorts();

  /**
   * @brief Implementation of the metadata() function for displaying metadata, such as Behavior description and
   * subcategory, in the MoveIt Studio Developer Tool.
   * @return A BT::KeyValueVector containing the Behavior metadata.
   */
  static BT::KeyValueVector metadata();

  /**
   * @brief Implementation of BT::SyncActionNode::tick() for PclDecomposeToSpheres.
   * @details This function is where the Behavior performs its work when the behavior tree is being run. Since PclDecomposeToSpheres is derived from BT::SyncActionNode, it is very important that its tick() function always finishes very quickly. If tick() blocks before returning, it will block execution of the entire behavior tree, which may have undesirable consequences for other Behaviors that require a fast update rate to work correctly.
   */
  BT::NodeStatus tick() override;

private:
  // Helper: Convert PointCloud2 to PCL PointCloud
  /**
   * @brief Convert a ROS PointCloud2 message to a PCL PointCloud.
   * @param cloud_msg The input PointCloud2 message.
   * @return A pointer to the converted PCL PointCloud.
   */
  pcl::PointCloud<pcl::PointXYZ>::Ptr convertToPCL(const sensor_msgs::msg::PointCloud2& cloud_msg);

  // Helper: Filter points within XY and Z thresholds from gripper position
  /**
   * @brief Filter points in a PCL PointCloud based on XY and Z distance thresholds from a given gripper position.
   * @param pcl_cloud The input PCL PointCloud.
   * @param gripper_pos The position of the gripper as a geometry_msgs::msg::Vector3.
   * @param xy_thresh The XY distance threshold.
   * @param z_thresh The Z distance threshold.
   * @return A pointer to the filtered PCL PointCloud containing only points within the specified thresholds.
   */
  pcl::PointCloud<pcl::PointXYZ>::Ptr filterPoints(
      const pcl::PointCloud<pcl::PointXYZ>::Ptr& pcl_cloud,
      const geometry_msgs::msg::Vector3& gripper_pos,
      double xy_thresh, double z_thresh);

  // Helper: Cluster filtered points and create spheres
  /**
   * @brief Create spheres representing clusters of points in a filtered PCL PointCloud.
   * @param filtered_cloud The input filtered PCL PointCloud.
   * @param sphere_radius The radius of the spheres to create.
   * @param frame_id The frame ID to assign to the created CollisionObject.
   * @return A moveit_msgs::msg::CollisionObject populated with spheres representing the clusters
   */
  moveit_msgs::msg::CollisionObject createSpheres(
      const pcl::PointCloud<pcl::PointXYZ>::Ptr& filtered_cloud,
      double sphere_radius,
      const std::string& frame_id);

 // Helper: Transform PCL point cloud to target frame using TF2
 /**
  * @brief Transform a PCL PointCloud to a target frame using TF2.
  * @param cloud The input PCL PointCloud.
  * @param source_frame The source frame of the input PointCloud.
  * @param target_frame The target frame to transform the PointCloud to.
  * @param stamp The timestamp for the transform lookup.
  * @param tf_buffer A shared pointer to a TF2 buffer for looking up transforms.
  * @param node_name The name of the node, used for logging purposes.
  * @return A pointer to the transformed PCL PointCloud, or nullptr if the transform
  */
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformCloudToFrame(
      const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
      const std::string& source_frame,
      const std::string& target_frame,
      const rclcpp::Time& stamp,
      const std::shared_ptr<tf2_ros::Buffer>& tf_buffer,
      const std::string& node_name);

};

}  // namespace pcl_decompose_to_spheres
