// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <rclcpp/node.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/subscription_options.hpp>
#include <rclcpp/wait_for_message.hpp>

#include <chrono>
#include <memory>
#include <string>

namespace hangar_sim_behaviors
{
/**
 * @brief Block until one message arrives on @p topic, or @p timeout_s elapses.
 *
 * @details Two details make this different from calling rclcpp::wait_for_message directly on the
 * Behavior's node:
 *
 * 1. The subscription goes into a callback group that is deliberately NOT added to the node's
 *    executor. The agent spins that node continuously, and a subscription it can see would let the
 *    executor take the sample before this function's wait set does -- so the wait would time out
 *    while the message it wanted sailed past.
 * 2. The QoS is passed in, so a latched publisher can be matched with `transient_local`. The QoS
 *    negotiation in `RclcppSubscriberInterface` only copies the publisher's *reliability*, which is
 *    enough for /scan_merged but silently wrong for /map: a volatile subscriber joining after
 *    map_server has published gets nothing at all, forever.
 *
 * @return True if a message was received and written to @p out.
 */
template <typename MessageT>
bool waitForOneMessage(const std::shared_ptr<rclcpp::Node>& node, const std::string& topic, const rclcpp::QoS& qos,
                       double timeout_s, MessageT& out)
{
  if (!node || topic.empty() || timeout_s <= 0.0)
  {
    return false;
  }

  rclcpp::SubscriptionOptions options;
  options.callback_group = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive,
                                                       /*automatically_add_to_executor_with_node=*/false);

  auto subscription = node->create_subscription<MessageT>(
      topic, qos, [](const std::shared_ptr<const MessageT>) {}, options);

  const auto timeout = std::chrono::duration<double>{ timeout_s };
  return rclcpp::wait_for_message<MessageT>(out, subscription, node->get_node_options().context(), timeout);
}
}  // namespace hangar_sim_behaviors
