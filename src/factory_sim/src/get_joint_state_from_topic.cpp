// Copyright 2026 PickNik Inc.
// SPDX-License-Identifier: BSD-3-Clause

#include <factory_sim/get_joint_state_from_topic.hpp>

#include <cmath>
#include <moveit_pro_behavior_interface/impl/get_message_from_topic_impl.hpp>

namespace factory_sim
{
BT::PortsList GetJointStateFromTopic::providedPorts()
{
  return { BT::InputPort<std::string>("topic_name", "/joint_states", "Joint-state feedback topic."),
           BT::InputPort<double>("message_timeout_sec", 0.5, "Positive finite message timeout, in seconds."),
           BT::InputPort<double>("publisher_timeout_sec", 0.5, "Positive finite publisher timeout, in seconds."),
           BT::OutputPort<sensor_msgs::msg::JointState>("message_out", "New joint-state feedback."),
           BT::OutputPort<double>("current_time", "ROS-clock time at receipt, in seconds; same clock as the header.") };
}

BT::KeyValueVector GetJointStateFromTopic::metadata()
{
  return { { "subcategory", "Motion - Planning" },
           { "description", "Receives new joint-state feedback for a stopped planning handoff." } };
}

namespace
{
tl::expected<std::chrono::duration<double>, std::string> boundedTimeout(BT::Expected<double> input)
{
  if (!input || !std::isfinite(input.value()) || input.value() <= 0.0)
    return tl::make_unexpected("Joint-state timeout must be positive and finite, in seconds.");
  return std::chrono::duration<double>{ input.value() };
}
}  // namespace

tl::expected<std::chrono::duration<double>, std::string> GetJointStateFromTopic::getWaitForMessageTimeout()
{
  return boundedTimeout(getInput<double>("message_timeout_sec"));
}

tl::expected<std::chrono::duration<double>, std::string> GetJointStateFromTopic::getWaitForPublisherTimeout()
{
  return boundedTimeout(getInput<double>("publisher_timeout_sec"));
}

BT::NodeStatus GetJointStateFromTopic::onRunning()
{
  const auto status = GetMessageFromTopicBehaviorBase::onRunning();
  if (status == BT::NodeStatus::SUCCESS && !setOutput("current_time", getBehaviorContext()->node->now().seconds()))
    throw BT::RuntimeError("Joint-state handoff cannot write current_time.");
  return status;
}
}  // namespace factory_sim

template class moveit_pro::behaviors::GetMessageFromTopicBehaviorBase<sensor_msgs::msg::JointState>;
