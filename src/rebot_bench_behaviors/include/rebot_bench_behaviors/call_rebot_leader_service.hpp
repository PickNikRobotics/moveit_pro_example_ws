// Copyright 2026 PickNik Inc.
// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <moveit_pro_behavior_interface/service_client_behavior_base.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace rebot_bench_behaviors
{
/** Call a leader Trigger service, reporting its refusal through the Objective's UI failure logger.
 * service_name is required and restricted to prepare/mirror/clutch. The supported service-client base owns
 * asynchronous execution, a 3 s server wait, a 1 s response timeout, and cancellation on halt.
 */
class CallRebotLeaderService final : public moveit_pro::behaviors::ServiceClientBehaviorBase<std_srvs::srv::Trigger>
{
public:
  using ServiceClientBehaviorBase::ServiceClientBehaviorBase;
  static BT::PortsList providedPorts();
  static BT::KeyValueVector metadata();
  tl::expected<std::string, std::string> getServiceName() override;
  tl::expected<std::chrono::duration<double>, std::string> getResponseTimeout() override;
  tl::expected<std_srvs::srv::Trigger::Request, std::string> createRequest() override;
  tl::expected<bool, std::string> processResponse(const std_srvs::srv::Trigger::Response& response) override;

private:
  std::shared_future<tl::expected<bool, std::string>>& getFuture() override
  {
    return future_;
  }
  std::shared_future<tl::expected<bool, std::string>> future_;
};
}  // namespace rebot_bench_behaviors
