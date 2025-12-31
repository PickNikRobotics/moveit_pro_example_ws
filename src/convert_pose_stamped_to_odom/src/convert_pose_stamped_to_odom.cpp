#include <convert_pose_stamped_to_odom/convert_pose_stamped_to_odom.hpp>

#include "spdlog/spdlog.h"
#include <string> 

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

namespace
{
  constexpr auto kPortIDPoseStamped = "pose_stamped";
  constexpr auto kPortIDOdometry = "odom";
  constexpr auto kPortIDChildFrame = "child_frame";
}  // namespace

namespace convert_pose_stamped_to_odom
{
ConvertPoseStampedToOdom::ConvertPoseStampedToOdom(const std::string& name, const BT::NodeConfiguration& config)
  : BT::SyncActionNode(name, config)
{
}

BT::PortsList ConvertPoseStampedToOdom::providedPorts()
{
  return {
    BT::InputPort<geometry_msgs::msg::PoseStamped>(
        kPortIDPoseStamped, "{pose_stamped}",
        "The PoseStamped message to convert."),

    BT::InputPort<std::string>(
        kPortIDChildFrame, "child_frame",
        "Optional child_frame_id for the Odometry message."),

    BT::OutputPort<nav_msgs::msg::Odometry>(
        kPortIDOdometry, "{odom}",
        "The converted Odometry message")
  };
}

BT::KeyValueVector ConvertPoseStampedToOdom::metadata()
{
  return { { "description", "Converts a geometry_msgs::msg::PoseStamped message into a "
                            "nav_msgs::msg::Odometry message, leaving the Twist field 0." },
           { "subcategory", "Conversions" } };
}

BT::NodeStatus ConvertPoseStampedToOdom::tick()
{
  // Required input
  const auto ports = moveit_studio::behaviors::getRequiredInputs(
      getInput<geometry_msgs::msg::PoseStamped>(kPortIDPoseStamped));

  if (!ports.has_value())
  {
    spdlog::warn("Failed to get PoseStamped input: {}", ports.error());
    return BT::NodeStatus::FAILURE;
  }

  const auto& pose_stamped = std::get<0>(ports.value());

  nav_msgs::msg::Odometry odom_out;
  odom_out.header = pose_stamped.header;
  odom_out.pose.pose = pose_stamped.pose;

  // Optional child_frame_id
  if (auto child_frame = getInput<std::string>(kPortIDChildFrame))
  {
    odom_out.child_frame_id = child_frame.value();
  }
  else
  {
    // Optional fallback — perfectly valid
    odom_out.child_frame_id = "";
  }

  // Twist left zero-initialized intentionally
  // odom_out.twist.twist.{linear,angular} == 0

  setOutput(kPortIDOdometry, odom_out);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace convert_pose_stamped_to_odom
