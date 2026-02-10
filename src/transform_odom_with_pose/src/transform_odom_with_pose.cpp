#include <transform_odom_with_pose/transform_odom_with_pose.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "spdlog/spdlog.h"
#include <fmt/format.h> 

#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace transform_odom_with_pose
{

TransformOdomWithPose::TransformOdomWithPose(
    const std::string& name, const BT::NodeConfiguration& config,
    const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
  : moveit_pro::behaviors::SharedResourcesNode<BT::SyncActionNode>(name, config, shared_resources)
{
}

BT::PortsList TransformOdomWithPose::providedPorts()
{
  return {
    BT::InputPort<nav_msgs::msg::Odometry>(
        "odom_in", "Input odometry message"),
    BT::InputPort<geometry_msgs::msg::PoseStamped>(
        "ee_pose_at_grip", "EE pose in world frame at grip time"),
    BT::InputPort<geometry_msgs::msg::PoseStamped>(
        "controller_pose_at_grip", "Controller pose in world frame at grip time"),  // NEW
    BT::InputPort<std::string>(
        "destination_child_frame", "The child frame you want the transformed odom message to have"),
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
  auto odom_in_res = getInput<nav_msgs::msg::Odometry>("odom_in");
  auto ee_pose_res = getInput<geometry_msgs::msg::PoseStamped>("ee_pose_at_grip");
  auto controller_pose_res = getInput<geometry_msgs::msg::PoseStamped>("controller_pose_at_grip");
  auto child_frame_res = getInput<std::string>("destination_child_frame");

  if (!odom_in_res || !ee_pose_res || !controller_pose_res || !child_frame_res)
  {
    shared_resources_->logger->publishFailureMessage(
        name(), "Missing required input(s)!");
    return BT::NodeStatus::FAILURE;
  }

  const auto& odom_in = odom_in_res.value();
  const auto& ee_pose_at_grip = ee_pose_res.value();
  const auto& controller_pose_at_grip = controller_pose_res.value();
  const auto& child_frame_id = child_frame_res.value();

  // --- Get poses at grip time (both in world frame) -------------------
  tf2::Transform T_world_controller_grip;
  tf2::fromMsg(controller_pose_at_grip.pose, T_world_controller_grip);

  tf2::Transform T_world_ee_grip;
  tf2::fromMsg(ee_pose_at_grip.pose, T_world_ee_grip);

  // --- Compute the fixed offset from controller to EE -----------------
  // T_controller_ee = T_world_controller^(-1) * T_world_ee
  tf2::Transform T_controller_ee = T_world_controller_grip.inverse() * T_world_ee_grip;

  // --- Get current controller pose from odometry ----------------------
  tf2::Transform T_world_controller_now;
  tf2::fromMsg(odom_in.pose.pose, T_world_controller_now);

  // --- Apply offset to get current EE pose ----------------------------
  tf2::Transform T_world_ee_now = T_world_controller_now * T_controller_ee;

  // --- Assemble output odometry ---------------------------------------
  nav_msgs::msg::Odometry odom_out;
  odom_out.header.stamp = odom_in.header.stamp;
  odom_out.header.frame_id = ee_pose_at_grip.header.frame_id;  // world
  odom_out.child_frame_id = child_frame_id;

  geometry_msgs::msg::Pose ee_pose_msg;
  ee_pose_msg.position.x = T_world_ee_now.getOrigin().x();
  ee_pose_msg.position.y = T_world_ee_now.getOrigin().y();
  ee_pose_msg.position.z = T_world_ee_now.getOrigin().z();
  ee_pose_msg.orientation = tf2::toMsg(T_world_ee_now.getRotation());

  odom_out.pose.pose = ee_pose_msg;
  odom_out.pose.covariance = odom_in.pose.covariance;

  // --- Rotate twist into EE frame -------------------------------------
  tf2::Matrix3x3 R(T_controller_ee.getRotation());

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

  odom_out.twist.twist.linear.x = lin_out.x();
  odom_out.twist.twist.linear.y = lin_out.y();
  odom_out.twist.twist.linear.z = lin_out.z();
  odom_out.twist.twist.angular.x = ang_out.x();
  odom_out.twist.twist.angular.y = ang_out.y();
  odom_out.twist.twist.angular.z = ang_out.z();
  odom_out.twist.covariance = odom_in.twist.covariance;

  setOutput("odom_out", odom_out);

  shared_resources_->logger->publishInfoMessage(
      name(), "Successfully transformed odometry using pose transform.");

  return BT::NodeStatus::SUCCESS;
}

}  // namespace transform_odom_with_pose
