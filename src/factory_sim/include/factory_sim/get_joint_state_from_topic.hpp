// Copyright 2026 PickNik Inc.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the PickNik Inc. nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

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
