#include <convert_odom_to_pose_stamped/convert_odom_to_pose_stamped.hpp>

#include "spdlog/spdlog.h"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

namespace
{
constexpr auto kPortIDOdometry = "odometry";
constexpr auto kPortIDPoseStamped = "pose_stamped";
}  // namespace

namespace convert_odom_to_pose_stamped
{
ConvertOdomToPoseStamped::ConvertOdomToPoseStamped(const std::string& name, const BT::NodeConfiguration& config)
  : BT::SyncActionNode(name, config)
{
}

BT::PortsList ConvertOdomToPoseStamped::providedPorts()
{
  return BT::PortsList(
      { BT::InputPort<nav_msgs::msg::Odometry>(kPortIDOdometry, "{odometry}",
                                               "The gnav_msgs::msg::Odometry message to "
                                               "convert."),
        BT::OutputPort<geometry_msgs::msg::PoseStamped>(kPortIDPoseStamped, "{pose_stamped}",
                                                        "The converted geometry_msgs::msg::PoseStamped message.") });
}

BT::KeyValueVector ConvertOdomToPoseStamped::metadata()
{
  return { { "description", "Converts a nav_msgs::msg::Odometry message into a "
                            "geometry_msgs::msg::PoseStamped message." },
           { "subcategory", "Conversions" } };
}

BT::NodeStatus ConvertOdomToPoseStamped::tick()
{
  const auto ports = moveit_pro::behaviors::getRequiredInputs(getInput<nav_msgs::msg::Odometry>(kPortIDOdometry));

  if (!ports.has_value())
  {
    spdlog::warn("Failed to get required value from input data port: {}", ports.error());
    return BT::NodeStatus::FAILURE;
  }

  const auto& odom_msg = std::get<0>(ports.value());

  geometry_msgs::msg::PoseStamped pose_out;
  pose_out.header = odom_msg.header;
  pose_out.pose = odom_msg.pose.pose;

  setOutput(kPortIDPoseStamped, pose_out);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace convert_odom_to_pose_stamped
