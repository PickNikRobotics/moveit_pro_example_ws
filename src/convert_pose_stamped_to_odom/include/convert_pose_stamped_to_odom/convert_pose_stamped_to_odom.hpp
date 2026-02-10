#pragma once

#include <behaviortree_cpp/action_node.h>
#include <moveit_pro_behavior_interface/get_required_ports.hpp>

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/basic_types.h>   // BT::PortsList, BT::KeyValueVector

namespace convert_pose_stamped_to_odom
{
/**
 * @brief TODO(...)
 */
class ConvertPoseStampedToOdom : public BT::SyncActionNode
{
public:
  ConvertPoseStampedToOdom(const std::string& name,
                           const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts();
  static BT::KeyValueVector metadata();

  BT::NodeStatus tick() override;
};

}  // namespace convert_pose_stamped_to_odom