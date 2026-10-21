// Copyright 2026 PickNik Inc.
// SPDX-License-Identifier: BSD-3-Clause
#include <rebot_bench_behaviors/call_rebot_leader_service.hpp>

#include <moveit_pro_behavior_interface/metadata_fields.hpp>

namespace rebot_bench_behaviors
{
BT::PortsList CallRebotLeaderService::providedPorts()
{
  return { BT::InputPort<std::string>("service_name", "Leader prepare, mirror or clutch service.") };
}

BT::KeyValueVector CallRebotLeaderService::metadata()
{
  return { { moveit_pro::behaviors::kSubcategoryMetadataKey, "reBot Bench" },
           { moveit_pro::behaviors::kDescriptionMetadataKey,
             "Calls the leader bridge and reports its operator instructions as the Objective's failure message." } };
}

tl::expected<std::string, std::string> CallRebotLeaderService::getServiceName()
{
  const auto service = getInput<std::string>("service_name");
  if (!service || (service.value() != "/rebot_leader_bridge/prepare" &&
                   service.value() != "/rebot_leader_bridge/mirror" && service.value() != "/rebot_leader_bridge/clutch"))
  {
    return tl::make_unexpected("Configure service_name as a reBot leader prepare, mirror or clutch service.");
  }
  return service.value();
}

tl::expected<std::chrono::duration<double>, std::string> CallRebotLeaderService::getResponseTimeout()
{
  return std::chrono::duration<double>{ 1.0 };
}

tl::expected<std_srvs::srv::Trigger::Request, std::string> CallRebotLeaderService::createRequest()
{
  return std_srvs::srv::Trigger::Request{};
}

tl::expected<bool, std::string> CallRebotLeaderService::processResponse(const std_srvs::srv::Trigger::Response& response)
{
  if (response.success)
  {
    return true;
  }
  // AsyncBehaviorBase publishes unexpected errors to the UI's failure buffer; a plain false loses this text.
  return tl::make_unexpected(response.message.empty() ?
                                 "Following was refused without a reason. Check the leader connection and backend "
                                 "diagnostics before retrying Mirror reBot Leader." :
                                 response.message);
}
}  // namespace rebot_bench_behaviors
