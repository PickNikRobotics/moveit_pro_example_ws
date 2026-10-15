// Copyright 2026 PickNik Inc.
// SPDX-License-Identifier: BSD-3-Clause

#pragma once

#include <moveit_pro_behavior_interface/get_message_from_topic.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace factory_sim
{
/**
 * @brief Receive new joint-state feedback with bounded, cancellable waits.
 * @details Receipt time uses the ROS clock shared with the feedback header.
 *
 * | Data Port Name | Port Type | Object Type |
 * | --- | --- | --- |
 * | topic_name | input | std::string |
 * | message_timeout_sec | input | double |
 * | publisher_timeout_sec | input | double |
 * | message_out | output | sensor_msgs::msg::JointState |
 * | current_time | output | double |
 */
class GetJointStateFromTopic final
  : public moveit_pro::behaviors::GetMessageFromTopicBehaviorBase<sensor_msgs::msg::JointState>
{
public:
  using GetMessageFromTopicBehaviorBase::GetMessageFromTopicBehaviorBase;
  static BT::PortsList providedPorts();
  static BT::KeyValueVector metadata();
  BT::NodeStatus onRunning() override;

private:
  tl::expected<std::chrono::duration<double>, std::string> getWaitForMessageTimeout() override;
  tl::expected<std::chrono::duration<double>, std::string> getWaitForPublisherTimeout() override;
  std::shared_future<tl::expected<bool, std::string>>& getFuture() override
  {
    return future_;
  }
  std::shared_future<tl::expected<bool, std::string>> future_;
};
}  // namespace factory_sim
