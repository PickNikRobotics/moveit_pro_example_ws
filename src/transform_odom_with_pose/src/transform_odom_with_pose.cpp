#include <transform_odom_with_pose/transform_odom_with_pose.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "spdlog/spdlog.h"
#include <fmt/format.h> 

namespace transform_odom_with_pose
{

TransformOdomWithPose::TransformOdomWithPose(
    const std::string& name, const BT::NodeConfiguration& config,
    const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources)
  : moveit_studio::behaviors::SharedResourcesNode<BT::SyncActionNode>(name, config, shared_resources)
{
}

BT::PortsList TransformOdomWithPose::providedPorts()
{
  return {
    BT::InputPort<nav_msgs::msg::Odometry>(
        "odom_in", "Input odometry message"),
    BT::InputPort<geometry_msgs::msg::PoseStamped>(
        "transform_pose", "PoseStamped defining the transform between frames"),
    BT::OutputPort<nav_msgs::msg::Odometry>(
        "odom_out", "Transformed odometry message")
  };
}

BT::KeyValueVector TransformOdomWithPose::metadata()
{
  return {
    {"description",
     "Transforms an Odometry message into a different frame using a PoseStamped-defined transform."},
    {"subcategory", "User Created Behaviors"}
  };
}

BT::NodeStatus TransformOdomWithPose::tick()
{
  // --- Get inputs ----------------------------------------------------
  auto odom_in_res = getInput<nav_msgs::msg::Odometry>("odom_in");
  auto pose_res    = getInput<geometry_msgs::msg::PoseStamped>("transform_pose");

  if (!odom_in_res && !pose_res)
  {
    shared_resources_->logger->publishFailureMessage(
        name(), "Missing both required inputs: odom_in or transform_pose! ");
    return BT::NodeStatus::FAILURE;
  }


  if (!odom_in_res || !pose_res)
  {
    if (!odom_in_res){
      shared_resources_->logger->publishFailureMessage(
        name(), "Missing required input: odom_in!");
    return BT::NodeStatus::FAILURE;
    }
    if (!pose_res){
      shared_resources_->logger->publishFailureMessage(
        name(), "Missing required input: transform_pose!");
    return BT::NodeStatus::FAILURE;
    }
  }

  const auto& odom_in = odom_in_res.value();
  const auto& transform_pose = pose_res.value();

  // --- Convert PoseStamped → TransformStamped ------------------------
  geometry_msgs::msg::TransformStamped transform;
  transform.header = transform_pose.header;
  shared_resources_->logger->publishInfoMessage(
    name(),
    fmt::format(
        "Transform header: frame_id='{}', stamp={}.{}",
        transform_pose.header.frame_id,
        transform_pose.header.stamp.sec,
        transform_pose.header.stamp.nanosec));

  transform.child_frame_id = odom_in.header.frame_id;
  
  shared_resources_->logger->publishInfoMessage(
    name(),
    fmt::format(
        "Input odometry frame_id: '{}'",
        odom_in.header.frame_id));

  transform.transform.translation.x = transform_pose.pose.position.x;
  transform.transform.translation.y = transform_pose.pose.position.y;
  transform.transform.translation.z = transform_pose.pose.position.z;
  transform.transform.rotation      = transform_pose.pose.orientation;

  // --- Transform pose ------------------------------------------------
  geometry_msgs::msg::PoseStamped pose_in;
  pose_in.header = odom_in.header;
  pose_in.pose   = odom_in.pose.pose;

  geometry_msgs::msg::PoseStamped pose_out;
  tf2::doTransform(pose_in, pose_out, transform);

  // --- Transform twist (rotation only) -------------------------------
  tf2::Quaternion q;
  tf2::fromMsg(transform.transform.rotation, q);
  tf2::Matrix3x3 R(q);

  tf2::Vector3 lin_in(
      odom_in.twist.twist.linear.x,
      odom_in.twist.twist.linear.y,
      odom_in.twist.twist.linear.z);

  tf2::Vector3 ang_in(
      odom_in.twist.twist.angular.x,
      odom_in.twist.twist.angular.y,
      odom_in.twist.twist.angular.z);

  tf2::Vector3 lin_out = R * lin_in;
  tf2::Vector3 ang_out = R * ang_in;

  // --- Assemble output odometry --------------------------------------
  nav_msgs::msg::Odometry odom_out;
  odom_out.header.stamp    = odom_in.header.stamp;
  odom_out.header.frame_id = transform.header.frame_id;
  odom_out.child_frame_id  = odom_in.child_frame_id;
  
  odom_out.pose.pose = pose_out.pose;

  odom_out.pose.covariance = odom_in.pose.covariance;

  odom_out.twist.twist.linear.x  = lin_out.x();
  odom_out.twist.twist.linear.y  = lin_out.y();
  odom_out.twist.twist.linear.z  = lin_out.z();

  odom_out.twist.twist.angular.x = ang_out.x();
  odom_out.twist.twist.angular.y = ang_out.y();
  odom_out.twist.twist.angular.z = ang_out.z();

  odom_out.twist.covariance = odom_in.twist.covariance;

  shared_resources_->logger->publishInfoMessage(
    name(),
    fmt::format("Odom out frame_id: '{}'", odom_out.header.frame_id));
  
  shared_resources_->logger->publishInfoMessage(
  name(),
  fmt::format("Odom out child_frame_id: '{}'", odom_out.child_frame_id));

  shared_resources_->logger->publishInfoMessage(
      name(),
      fmt::format(
          "Odom out stamp: {}.{}",
          odom_out.header.stamp.sec,
          odom_out.header.stamp.nanosec));

  shared_resources_->logger->publishInfoMessage(
      name(),
      fmt::format(
          "Odom out pose: position=({:.3f}, {:.3f}, {:.3f}), "
          "orientation=({:.3f}, {:.3f}, {:.3f}, {:.3f})",
          odom_out.pose.pose.position.x,
          odom_out.pose.pose.position.y,
          odom_out.pose.pose.position.z,
          odom_out.pose.pose.orientation.x,
          odom_out.pose.pose.orientation.y,
          odom_out.pose.pose.orientation.z,
          odom_out.pose.pose.orientation.w));

  // --- Output --------------------------------------------------------
  setOutput("odom_out", odom_out);

  shared_resources_->logger->publishInfoMessage(
      name(), "Successfully transformed odometry.");

  return BT::NodeStatus::SUCCESS;
}

}  // namespace transform_odom_with_pose
