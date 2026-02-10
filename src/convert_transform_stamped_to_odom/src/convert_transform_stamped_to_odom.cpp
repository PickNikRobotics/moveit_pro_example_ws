// Copyright 2025 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include <convert_transform_stamped_to_odom/convert_transform_stamped_to_odom.hpp>

#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/tree_node.h>

#include "spdlog/spdlog.h"
#include "moveit_pro_behavior_interface/get_required_ports.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

namespace
{
constexpr auto kPortIDTransformStamped = "transform_stamped";
constexpr auto kPortIDOdometry = "odometry";
}  // namespace

namespace convert_transform_stamped_to_odom
{

ConvertTransformStampedToOdom::ConvertTransformStampedToOdom(
    const std::string& name, const BT::NodeConfiguration& config,
    const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
  : moveit_pro::behaviors::SharedResourcesNode<BT::SyncActionNode>(name, config, shared_resources)
{
}

BT::PortsList ConvertTransformStampedToOdom::providedPorts()
{
  return BT::PortsList(
      { BT::InputPort<geometry_msgs::msg::TransformStamped>(kPortIDTransformStamped, "{transform_stamped}",
                                                            "The geometry_msgs::msg::TransformStamped message to "
                                                            "convert."),
        BT::OutputPort<nav_msgs::msg::Odometry>(kPortIDOdometry, "{odometry}",
                                                "The converted nav_msgs::msg::Odometry message.") });
}

BT::KeyValueVector ConvertTransformStampedToOdom::metadata()
{
  return { { "description", "Converts a geometry_msgs::msg::TransformStamped message into a "
                            "nav_msgs::msg::Odometry message." },
           { "subcategory", "Conversions" } };
}

BT::NodeStatus ConvertTransformStampedToOdom::tick()
{
  const auto ports = moveit_pro::behaviors::getRequiredInputs(
      getInput<geometry_msgs::msg::TransformStamped>(kPortIDTransformStamped));

  if (!ports.has_value())
  {
    spdlog::warn("Failed to get required value from input data port: {}", ports.error());
    return BT::NodeStatus::FAILURE;
  }

  const auto& transform_stamped = std::get<0>(ports.value());

  nav_msgs::msg::Odometry odom_out;
  odom_out.header = transform_stamped.header;
  odom_out.child_frame_id = transform_stamped.child_frame_id;
  
  shared_resources_->logger->publishInfoMessage(
        name(), "Converting transform to pose message ...");

  // Convert the transform to pose
  odom_out.pose.pose.position.x = transform_stamped.transform.translation.x;
  odom_out.pose.pose.position.y = transform_stamped.transform.translation.y;
  odom_out.pose.pose.position.z = transform_stamped.transform.translation.z;
  odom_out.pose.pose.orientation = transform_stamped.transform.rotation;
  
  // shared_resources_->logger->publishInfoMessage(
  //       name(), fmt::format("This is pose.position.x: {}, pose.position.y: {}, pose.position.z: {}", 
  //         odom_out.pose.pose.position.x, odom_out.pose.pose.position.y, odom_out.pose.pose.position.z));

  // Note: Twist (velocity) data is not available from TransformStamped, so it remains zero-initialized

  setOutput(kPortIDOdometry, odom_out);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace convert_transform_stamped_to_odom