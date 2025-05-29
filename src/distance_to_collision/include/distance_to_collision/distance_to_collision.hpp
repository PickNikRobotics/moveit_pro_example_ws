#pragma once

#include <behaviortree_cpp/action_node.h>
#include <moveit_studio_behavior_interface/get_required_ports.hpp>

// This header includes the SharedResourcesNode type
#include <moveit_studio_behavior_interface/shared_resources_node.hpp>

namespace distance_to_collision
{
/**
 * @brief Outputs the shortest distance between a given point and a point cloud.
 * *
 * @details
 * | Data Port Name | Port Type | Object Type                   | Description                                          |
 * |----------------|-----------|-------------------------------|------------------------------------------------------|
 * | point_cloud    | Input     | sensor_msgs::msg::PointCloud2 | Point cloud to check collision distances.            |
 * | point          | Input     | geometry_msgs::msg::Point     | Point to check collision distance.                   |
 * | distance       | Output    | double                        | Shortest distance between the point and point cloud. |
 */
class DistanceToCollision : public moveit_studio::behaviors::SharedResourcesNode<BT::SyncActionNode>
{
public:
  // Port names for input and output ports.
  static constexpr auto kPortIDPointCloud = "point_cloud";
  static constexpr auto kPortIDPoint = "point";
  static constexpr auto kPortIDDistance = "distance";

  DistanceToCollision(const std::string& name, const BT::NodeConfiguration& config,
                      const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources);

  static BT::PortsList providedPorts();

  static BT::KeyValueVector metadata();

  BT::NodeStatus tick() override;
};
}  // namespace distance_to_collision
