#include <example_behaviors/example_publish_arrow_marker.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_studio_behavior_interface/get_required_ports.hpp>
#include <moveit_studio_vision/utils/tf_utils.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace
{
// Port names for input and output ports.
constexpr auto kPortIDStartPose = "start_pose";
constexpr auto kPortIDEndPose = "end_pose";
}  // namespace

namespace example_behaviors
{
ExamplePublishArrowMarker::ExamplePublishArrowMarker(
    const std::string& name, const BT::NodeConfiguration& config,
    const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources)
  : moveit_studio::behaviors::SharedResourcesNode<BT::SyncActionNode>(name, config, shared_resources)
{
  marker_publisher_ = shared_resources_->node->create_publisher<visualization_msgs::msg::MarkerArray>(
      "visual_markers", rclcpp::SystemDefaultsQoS());
}

BT::PortsList ExamplePublishArrowMarker::providedPorts()
{
  // This node has no input or output ports
  return BT::PortsList({
      BT::InputPort<geometry_msgs::msg::PoseStamped>(kPortIDStartPose, "", "The start of the arrow"),
      BT::InputPort<geometry_msgs::msg::PoseStamped>(kPortIDEndPose, "", "The end of the arrow."),
  });
}

BT::KeyValueVector ExamplePublishArrowMarker::metadata()
{
  return { { "subcategory", "Example Behaviors" }, { "description", "Publish marker" } };
}

BT::NodeStatus ExamplePublishArrowMarker::tick()
{
  const auto ports =
      moveit_studio::behaviors::getRequiredInputs(getInput<geometry_msgs::msg::PoseStamped>(kPortIDStartPose),
                                                  getInput<geometry_msgs::msg::PoseStamped>(kPortIDEndPose));

  // If a port was set incorrectly, log an error message and return FAILURE
  if (!ports.has_value())
  {
    shared_resources_->logger->publishFailureMessage(
        name(), fmt::format("Failed to get required value from input data port: {}", ports.error()));

    return BT::NodeStatus::FAILURE;
  }
  const auto& [start_pose_stamped, end_pose_stamped] = ports.value();

  geometry_msgs::msg::TransformStamped transform_end;
  geometry_msgs::msg::TransformStamped transform_start;
  try
  {
    transform_start = shared_resources_->transform_buffer_ptr->lookupTransform(
        "world", start_pose_stamped.header.frame_id, tf2::TimePointZero);
    transform_end = shared_resources_->transform_buffer_ptr->lookupTransform(
        "world", end_pose_stamped.header.frame_id, tf2::TimePointZero);
  }
  catch (tf2::TransformException& ex)
  {
    shared_resources_->logger->publishFailureMessage(name(), ex.what());
    return BT::NodeStatus::FAILURE;
  }

  geometry_msgs::msg::PoseStamped pose_start;
  geometry_msgs::msg::PoseStamped pose_end;
  tf2::doTransform(start_pose_stamped, pose_start, transform_start);
  tf2::doTransform(end_pose_stamped, pose_end, transform_end);

  visualization_msgs::msg::Marker msg;
  msg.pose = pose_start.pose;
  msg.header.frame_id = "world";
  msg.header.stamp = shared_resources_->node->get_clock()->now();
  msg.ns = "arrow_marker";
  msg.id = 0;
  msg.type = visualization_msgs::msg::Marker::ARROW;
  msg.action = visualization_msgs::msg::Marker::ADD;

  // Define the start and end points of the arrow
  geometry_msgs::msg::Point start;
  start.x = 0;
  start.y = 0;
  start.z = 0;

  Eigen::Isometry3d start_tform = tf2::poseToEigen(pose_start);
  Eigen::Isometry3d end_tform = tf2::poseToEigen(pose_end);
  end_tform = start_tform.inverse() * end_tform;
  geometry_msgs::msg::Point end;
  end.x = end_tform.translation().x();
  end.y = end_tform.translation().y();
  end.z = end_tform.translation().z();

  msg.points.push_back(start);
  msg.points.push_back(end);

  // Set the scale of the arrow
  msg.scale.x = 0.01;  // Shaft diameter
  msg.scale.y = 0.04;  // Head diameter
  msg.scale.z = 0.02;  // Head length (only if non-zero)

  // Set the color (RGBA)
  msg.color.r = 1.0f;
  msg.color.g = 0.0f;
  msg.color.b = 0.0f;
  msg.color.a = 1.0f;  // Ensure alpha is non-zero

  visualization_msgs::msg::MarkerArray msg_array;
  msg_array.markers = { msg };

  marker_publisher_->publish(msg_array);

  return BT::NodeStatus::SUCCESS;
}
}  // namespace example_behaviors
