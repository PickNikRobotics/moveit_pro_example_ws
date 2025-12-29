// Copyright 2025 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include <convert_pose_stamped_to_transform_stamped/convert_pose_stamped_to_transform_stamped.hpp>

#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/tree_node.h>

#include "spdlog/spdlog.h"
#include "moveit_studio_behavior_interface/get_required_ports.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

namespace
{
constexpr auto kPortIDPoseStamped = "pose_stamped";
constexpr auto kPortIDTransformStamped = "transform_stamped";
constexpr auto kPortIDChildFrameId = "child_frame_id";
}  // namespace

namespace convert_pose_stamped_to_transform_stamped
{

ConvertPoseStampedToTransformStamped::ConvertPoseStampedToTransformStamped(
    const std::string& name, const BT::NodeConfiguration& config,
    const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources)
  : moveit_studio::behaviors::SharedResourcesNode<BT::SyncActionNode>(name, config, shared_resources)
{
}

BT::PortsList ConvertPoseStampedToTransformStamped::providedPorts()
{
  return BT::PortsList(
      { BT::InputPort<geometry_msgs::msg::PoseStamped>(kPortIDPoseStamped, "{pose_stamped}",
                                                       "The geometry_msgs::msg::PoseStamped message to convert."),
        BT::InputPort<std::string>(kPortIDChildFrameId, "child_frame",
                                   "The child frame ID to set in the TransformStamped message."),
        BT::OutputPort<geometry_msgs::msg::TransformStamped>(kPortIDTransformStamped, "{transform_stamped}",
                                                             "The converted geometry_msgs::msg::TransformStamped message.") });
}

BT::KeyValueVector ConvertPoseStampedToTransformStamped::metadata()
{
  return { { "description", "Converts a geometry_msgs::msg::PoseStamped message into a "
                            "geometry_msgs::msg::TransformStamped message." },
           { "subcategory", "Conversions" } };
}

BT::NodeStatus ConvertPoseStampedToTransformStamped::tick()
{
  const auto ports = moveit_studio::behaviors::getRequiredInputs(
      getInput<geometry_msgs::msg::PoseStamped>(kPortIDPoseStamped),
      getInput<std::string>(kPortIDChildFrameId));

  if (!ports.has_value())
  {
    spdlog::warn("Failed to get required value from input data port: {}", ports.error());
    return BT::NodeStatus::FAILURE;
  }

  const auto& [pose_stamped, child_frame_id] = ports.value();

  geometry_msgs::msg::TransformStamped transform_out;
  transform_out.header = pose_stamped.header;
  transform_out.child_frame_id = child_frame_id;
  
  // Convert the pose to transform
  transform_out.transform.translation.x = pose_stamped.pose.position.x;
  transform_out.transform.translation.y = pose_stamped.pose.position.y;
  transform_out.transform.translation.z = pose_stamped.pose.position.z;
  transform_out.transform.rotation = pose_stamped.pose.orientation;

  setOutput(kPortIDTransformStamped, transform_out);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace convert_pose_stamped_to_transform_stamped