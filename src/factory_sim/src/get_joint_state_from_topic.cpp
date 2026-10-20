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
