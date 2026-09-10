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

#include <lunar_sim_behaviors/publish_twist_stamped.hpp>
#include <moveit_pro_behavior_interface/metadata_fields.hpp>

namespace lunar_sim_behaviors
{
inline constexpr auto kDescriptionPublishTwistStamped = R"(
                <p>
                    Publish a <code>geometry_msgs::msg::TwistStamped</code> message to a topic.
                </p>
            )";

using Base = moveit_pro::behaviors::SendMessageToTopicBehaviorBase<geometry_msgs::msg::TwistStamped>;

PublishTwistStamped::PublishTwistStamped(const std::string& name, const BT::NodeConfiguration& config,
                                         const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
  : Base(name, config, shared_resources)
{
}

BT::PortsList PublishTwistStamped::providedPorts()
{
  return BT::PortsList({
      BT::InputPort<geometry_msgs::msg::TwistStamped>(Base::kPortIDMessage, "The TwistStamped message to publish."),
      BT::InputPort<std::string>(Base::kPortIDTopicName, "", "The topic the message should be published to."),
      BT::InputPort<size_t>(Base::kPortIDQueueSize, 1, "The queue size for the publisher."),
      BT::InputPort<bool>(Base::kPortIDUseBestEffort, false,
                          "Whether the publisher's reliability should be best effort (true) or reliable (false)."),
  });
}

BT::KeyValueVector PublishTwistStamped::metadata()
{
  return { { moveit_pro::behaviors::kSubcategoryMetadataKey, "ROS Messaging" },
           { moveit_pro::behaviors::kDescriptionMetadataKey, kDescriptionPublishTwistStamped } };
}

}  // namespace lunar_sim_behaviors

template class moveit_pro::behaviors::SendMessageToTopicBehaviorBase<geometry_msgs::msg::TwistStamped>;
