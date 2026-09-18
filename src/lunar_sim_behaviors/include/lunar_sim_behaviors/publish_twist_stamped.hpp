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

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <memory>
#include <moveit_pro_behavior_interface/send_message_to_topic.hpp>

namespace lunar_sim_behaviors
{

/**
 * @brief Publish a geometry_msgs::msg::TwistStamped message to a topic.
 *
 * @details
 * | Data Port Name  | Port Type | Object Type                       |
 * |-----------------|-----------|------------------------------------|
 * | message         | Input     | geometry_msgs::msg::TwistStamped  |
 * | topic           | Input     | std::string                       |
 * | queue_size      | Input     | size_t                            |
 * | use_best_effort | Input     | bool                              |
 */
class PublishTwistStamped final
  : public moveit_pro::behaviors::SendMessageToTopicBehaviorBase<geometry_msgs::msg::TwistStamped>
{
public:
  PublishTwistStamped(const std::string& name, const BT::NodeConfiguration& config,
                      const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources);

  static BT::PortsList providedPorts();

  static BT::KeyValueVector metadata();
};

}  // namespace lunar_sim_behaviors
