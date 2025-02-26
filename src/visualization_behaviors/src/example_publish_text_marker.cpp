#include <example_behaviors/example_publish_text_marker.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_studio_behavior_interface/get_required_ports.hpp>

namespace
{
// Port names for input and output ports.
constexpr auto kPortIDPose = "pose";
constexpr auto kPortIDMessage = "message";
constexpr auto kPortIDScale = "scale";
constexpr auto kPortIDPositionXyz = "position_xyz";
constexpr auto kPortIDOrientationXyzw = "orientation_xyzw";
}  // namespace

namespace example_behaviors
{
ExamplePublishTextMarker::ExamplePublishTextMarker(
    const std::string& name, const BT::NodeConfiguration& config,
    const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources)
  : moveit_studio::behaviors::SharedResourcesNode<BT::SyncActionNode>(name, config, shared_resources)
{
  marker_publisher_ = shared_resources_->node->create_publisher<visualization_msgs::msg::MarkerArray>(
      "visual_markers", rclcpp::SystemDefaultsQoS());
}

BT::PortsList ExamplePublishTextMarker::providedPorts()
{
  // This node has no input or output ports
  return BT::PortsList(
      { BT::InputPort<geometry_msgs::msg::PoseStamped>(kPortIDPose, "", "The text pose."),
        BT::InputPort<std::string>(kPortIDMessage, "", "The text message to display."),
        BT::InputPort<double>(kPortIDScale, 1.0, "The text scale."),
        BT::InputPort<std::vector<double>>(kPortIDPositionXyz, { 0.0, 0.0, 0.0 },
                                           "The x, y, z values of the position, separated by semicolons.") });
}

BT::KeyValueVector ExamplePublishTextMarker::metadata()
{
  return { { "subcategory", "Example Behaviors" }, { "description", "Publish marker" } };
}

BT::NodeStatus ExamplePublishTextMarker::tick()
{
  const auto ports =
      moveit_studio::behaviors::getRequiredInputs(getInput<geometry_msgs::msg::PoseStamped>(kPortIDPose),
                                                  getInput<std::string>(kPortIDMessage), getInput<double>(kPortIDScale),
                                                  getInput<std::vector<double>>(kPortIDPositionXyz));

  // If a port was set incorrectly, log an error message and return FAILURE
  if (!ports.has_value())
  {
    shared_resources_->logger->publishFailureMessage(
        name(), fmt::format("Failed to get required value from input data port: {}", ports.error()));

    return BT::NodeStatus::FAILURE;
  }
  const auto& [pose_stamped, message, scale, position] = ports.value();

  if (position.size() != 3)
  {
    shared_resources_->logger->publishFailureMessage(name(), "position_xyz must contain 3 elements 'x; y; z'");
    return BT::NodeStatus::FAILURE;
  }

  visualization_msgs::msg::Marker msg;
  msg.scale.x = scale, msg.scale.y = scale;
  msg.scale.z = scale;
  msg.pose = pose_stamped.pose;
  msg.pose.position.x += position[0];
  msg.pose.position.y += position[1];
  msg.pose.position.z += position[2];
  msg.color.r = 0.;
  msg.color.g = 1.0;
  msg.color.b = 0.0;
  msg.color.a = 1.0;

  msg.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  msg.text = message;
  msg.header.frame_id = pose_stamped.header.frame_id;
  msg.header.stamp = shared_resources_->node->now();

  visualization_msgs::msg::MarkerArray msg_array;
  msg_array.markers = { msg };

  marker_publisher_->publish(msg_array);

  return BT::NodeStatus::SUCCESS;
}
}  // namespace example_behaviors
