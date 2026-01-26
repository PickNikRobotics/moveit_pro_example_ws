#include <example_behaviors/example_publish_color_rgba.hpp>

namespace example_behaviors
{
ExamplePublishColorRGBA::ExamplePublishColorRGBA(
    const std::string& name, const BT::NodeConfiguration& config,
    const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
  : moveit_pro::behaviors::SharedResourcesNode<BT::SyncActionNode>(name, config, shared_resources)
  , publisher_{ shared_resources_->node->create_publisher<std_msgs::msg::ColorRGBA>("/my_topic", rclcpp::QoS(1)) }
{
}

BT::PortsList ExamplePublishColorRGBA::providedPorts()
{
  return {};
}

BT::KeyValueVector ExamplePublishColorRGBA::metadata()
{
  return { { "subcategory", "Example Behaviors" },
           { "description", "Publishes a fixed std_msgs::msg::ColorRGBA message to a topic named \"/my_topic\"" } };
}

BT::NodeStatus ExamplePublishColorRGBA::tick()
{
  std_msgs::msg::ColorRGBA color_msg;
  color_msg.r = 0.5;
  color_msg.g = 0.5;
  color_msg.b = 0.5;
  color_msg.a = 0.5;
  publisher_->publish(color_msg);

  return BT::NodeStatus::SUCCESS;
}
}  // namespace example_behaviors
