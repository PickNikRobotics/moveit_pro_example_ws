// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include <hangar_sim_behaviors/call_empty_service.hpp>

#include <fmt/format.h>

#include <moveit_pro_behavior_interface/behavior_subcategories.hpp>
#include <moveit_pro_behavior_interface/impl/service_client_behavior_base_impl.hpp>
#include <moveit_pro_behavior_interface/metadata_fields.hpp>

#include <string>

namespace
{
constexpr auto kPortServiceName = "service_name";
constexpr auto kPortResponseTimeout = "response_timeout";
constexpr auto kPortWaitForServerTimeout = "wait_for_server_available_timeout";

/// Negative means wait forever, matching CallTriggerService.
constexpr double kDefaultResponseTimeout = -1.0;
constexpr double kDefaultWaitForServerTimeout = 3.0;

inline constexpr auto kDescription = R"(
                <p>Send a request to a <code>std_srvs::srv::Empty</code> service server and wait until the server sends a response.</p>
                <p>This is the Empty counterpart of <code>CallTriggerService</code>, which only handles <code>std_srvs::srv::Trigger</code>. The two service types are unrelated, so a tree that needs an Empty service cannot get there by configuring CallTriggerService differently.</p>
                <p><code>Empty</code> carries no response fields, so acknowledgement is the only signal there is: this Behavior exits SUCCESS as soon as the server answers, and FAILURE if no server appears within <code>wait_for_server_available_timeout</code> or none answers within <code>response_timeout</code>. If you need to read whether the work actually succeeded, the service should be a Trigger and you want <code>CallTriggerService</code>.</p>
                <p>On this robot beluga_amcl advertises two Empty services: <code>/request_nomotion_update</code>, which forces one stationary filter update, and <code>/reinitialize_global_localization</code>, which scatters particles uniformly over the whole map. Do not reach for the second inside a refinement seeded from an operator's click -- it discards exactly the bound that click provides.</p>
            )";
}  // namespace

namespace hangar_sim_behaviors
{
CallEmptyService::CallEmptyService(const std::string& name, const BT::NodeConfiguration& config,
                                   const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
  : moveit_pro::behaviors::ServiceClientBehaviorBase<Empty>(name, config, shared_resources)
{
}

BT::PortsList CallEmptyService::providedPorts()
{
  return {
    BT::InputPort<std::string>(kPortServiceName, "Name of the service to send a request to."),
    BT::InputPort<double>(kPortResponseTimeout, kDefaultResponseTimeout,
                          "Timeout in seconds when waiting for service response (negative value for infinite "
                          "timeout)."),
    BT::InputPort<double>(kPortWaitForServerTimeout, kDefaultWaitForServerTimeout,
                          "Timeout in seconds when waiting for service server to be available."),
  };
}

BT::KeyValueVector CallEmptyService::metadata()
{
  return { { moveit_pro::behaviors::kSubcategoryMetadataKey,
             std::string(moveit_pro::behaviors::toString(moveit_pro::behaviors::Subcategory::ControlFlow)) },
           { moveit_pro::behaviors::kDescriptionMetadataKey, kDescription } };
}

tl::expected<std::string, std::string> CallEmptyService::getServiceName()
{
  const auto service_name = getInput<std::string>(kPortServiceName);
  if (!service_name.has_value())
  {
    return tl::make_unexpected(fmt::format("Failed to get service name: {}", service_name.error()));
  }
  if (service_name.value().empty())
  {
    return tl::make_unexpected(fmt::format("The '{}' port must contain a valid service name.", kPortServiceName));
  }
  return service_name.value();
}

tl::expected<std::chrono::duration<double>, std::string> CallEmptyService::getResponseTimeout()
{
  const auto timeout = getInput<double>(kPortResponseTimeout);
  if (!timeout.has_value())
  {
    return tl::make_unexpected(fmt::format("Failed to get value for response timeout duration: {}", timeout.error()));
  }
  return std::chrono::duration<double>{ timeout.value() };
}

tl::expected<std::chrono::duration<double>, std::string> CallEmptyService::getWaitForServerAvailableTimeout()
{
  const auto timeout = getInput<double>(kPortWaitForServerTimeout);
  if (!timeout.has_value())
  {
    return tl::make_unexpected(
        fmt::format("Failed to get value for wait for service server timeout duration: {}", timeout.error()));
  }
  return std::chrono::duration<double>{ timeout.value() };
}

tl::expected<Empty::Request, std::string> CallEmptyService::createRequest()
{
  return Empty::Request{};
}

tl::expected<bool, std::string> CallEmptyService::processResponse(const Empty::Response& /*response*/)
{
  return true;
}

}  // namespace hangar_sim_behaviors

// std_srvs::srv::Empty is not among the specializations moveit_pro_behavior_interface instantiates,
// so this translation unit provides it.
template class moveit_pro::behaviors::ServiceClientBehaviorBase<std_srvs::srv::Empty>;
