#include <publish_pose/publish_pose.hpp>

#include <moveit_pro_behavior_interface/metadata_fields.hpp>

#include <spdlog/spdlog.h>

namespace publish_pose
{

PublishPose::PublishPose(
    const std::string& name,
    const BT::NodeConfiguration& config,
    const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
  : moveit_pro::behaviors::SharedResourcesNode<BT::SyncActionNode>(
        name, config, shared_resources)
{
}

BT::PortsList PublishPose::providedPorts()
{
  return {
      BT::InputPort<geometry_msgs::msg::PoseStamped>(
          "message",
          geometry_msgs::msg::PoseStamped(),
          "The pose stamped message to publish"),
      BT::InputPort<std::string>(
          "topic_name",
          "",
          "The topic the pose stamped message should be published to."),
      BT::InputPort<size_t>(
          "queue_size",
          1,
          "The queue size for the publisher.")
  };
}

BT::KeyValueVector PublishPose::metadata()
{
  return {
      { moveit_pro::behaviors::kSubcategoryMetadataKey, "ROS Messaging" },
      { moveit_pro::behaviors::kDescriptionMetadataKey,
        "Publish a geometry_msgs::msg::PoseStamped message to a ROS 2 topic." }
  };
}

BT::NodeStatus PublishPose::tick()
{
  // --- Read inputs --------------------------------------------------
  auto msg_res   = getInput<geometry_msgs::msg::PoseStamped>("message");
  auto topic_res = getInput<std::string>("topic_name");
  auto queue_res = getInput<size_t>("queue_size");

  if (!msg_res || !topic_res || !queue_res)
  {
    shared_resources_->logger->publishFailureMessage(
        name(), "Missing required input port(s) for PublishPose.");
    return BT::NodeStatus::FAILURE;
  }

  const auto& msg        = msg_res.value();
  const auto& topic_name = topic_res.value();
  const auto  queue_size = queue_res.value();

  if (topic_name.empty())
  {
    shared_resources_->logger->publishFailureMessage(
        name(), "Topic name is empty.");
    return BT::NodeStatus::FAILURE;
  }

  // --- Create or update publisher ----------------------------------
  if (!publisher_ || topic_name != topic_name_)
  {
    publisher_ = shared_resources_->node->create_publisher<geometry_msgs::msg::PoseStamped>(
        topic_name, queue_size);

    topic_name_ = topic_name;

    // shared_resources_->logger->publishInfoMessage(
    //     name(),
    //     "Created pose stamped publisher on topic '" + topic_name_ + "'");
  }

  // --- Publish ------------------------------------------------------
  publisher_->publish(msg);

  // shared_resources_->logger->publishInfoMessage(
  // name(),
  // fmt::format("Published pose stamped message has child frame id: '{}'", msg.child_frame_id));

  // shared_resources_->logger->publishInfoMessage(
  // name(),
  // fmt::format("Published pose stamped message has frame id: '{}'", msg.header.frame_id));

  // Optional: lightweight debug (console only)
  // spdlog::debug(
  //     "[PublishPose] Published pose stamped on '{}', frame_id='{}'",
  //     topic_name_,
  //     msg.header.frame_id);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace publish_pose
