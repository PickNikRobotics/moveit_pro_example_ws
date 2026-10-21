// Copyright 2026 PickNik Inc.
// SPDX-License-Identifier: BSD-3-Clause
#include <rebot_bench_behaviors/call_rebot_leader_service.hpp>

#include <moveit_pro_behavior_interface/metadata_fields.hpp>

namespace rebot_bench_behaviors
{
namespace
{
using Trigger = std_srvs::srv::Trigger;

/** Replaces the base's bare "Server not available" with why the bridge is missing and how to start it. */
class LeaderClient final : public moveit_pro::behaviors::ClientInterfaceBase<Trigger>
{
public:
  explicit LeaderClient(std::unique_ptr<ClientInterfaceBase<Trigger>> client) : client_(std::move(client))
  {
  }
  void initialize(const std::string& service_name, std::chrono::duration<double> wait_for_server_timeout,
                  std::chrono::duration<double> response_timeout) override
  {
    client_->initialize(service_name, wait_for_server_timeout, response_timeout);
  }
  bool waitForServiceServer() const override
  {
    available_ = client_->waitForServiceServer();
    return true;
  }
  tl::expected<Trigger::Response, std::string> syncSendRequest(const Trigger::Request& request) override
  {
    if (!available_)
    {
      return tl::make_unexpected(
          "The reBot leader bridge is not running. The backend starts it only when "
          "ros_global_params.rebot_leader_enabled is true in the selected configuration's config.yaml; the "
          "shipped default is false. Set it to true, then restart the backend (moveit_pro run) and run Mirror reBot "
          "Leader again. If it is already true, the bridge exited at startup: check the backend log for "
          "rebot_leader_bridge errors.");
    }
    return client_->syncSendRequest(request);
  }
  void cancelRequest() override
  {
    client_->cancelRequest();
  }

private:
  std::unique_ptr<ClientInterfaceBase<Trigger>> client_;
  mutable bool available_ = false;
};
}  // namespace

CallRebotLeaderService::CallRebotLeaderService(
    const std::string& name, const BT::NodeConfiguration& config,
    const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
  : CallRebotLeaderService(name, config, shared_resources,
                           std::make_unique<moveit_pro::behaviors::RclcppClientInterface<Trigger>>(shared_resources))
{
}

CallRebotLeaderService::CallRebotLeaderService(
    const std::string& name, const BT::NodeConfiguration& config,
    const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources,
    std::unique_ptr<moveit_pro::behaviors::ClientInterfaceBase<Trigger>> client)
  : ServiceClientBehaviorBase(name, config, shared_resources, std::make_unique<LeaderClient>(std::move(client)))
{
}

BT::PortsList CallRebotLeaderService::providedPorts()
{
  return { BT::InputPort<std::string>("service_name", "Leader bridge Trigger service to call.") };
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
  if (!service)
  {
    return tl::make_unexpected("Configure service_name: " + service.error());
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
