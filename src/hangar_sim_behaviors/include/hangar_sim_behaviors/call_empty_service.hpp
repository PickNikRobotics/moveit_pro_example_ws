// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <moveit_pro_behavior_interface/service_client_behavior_base.hpp>
#include <moveit_pro_behavior_interface/shared_resources_node.hpp>
#include <std_srvs/srv/empty.hpp>

#include <memory>
#include <string>

namespace hangar_sim_behaviors
{
using Empty = std_srvs::srv::Empty;

/**
 * @brief Call a service that takes a `std_srvs/srv/Empty` message.
 *
 * @details The counterpart of core's `CallTriggerService`, which handles `std_srvs/srv/Trigger` and
 * nothing else. Empty is a distinct service type, so no amount of configuration makes
 * CallTriggerService reach one; a tree that needs an Empty service has, until now, needed a
 * purpose-built Behavior per service.
 *
 * beluga_amcl alone advertises two of them -- `/request_nomotion_update` and
 * `/reinitialize_global_localization` -- so this is generic on the service name rather than fixed to
 * either. Note the second is emphatically not a substitute for the first inside a click-bounded
 * refinement: it redraws particles uniformly over the whole map and destroys the bound the click
 * buys you.
 *
 * Empty carries no response fields, so an acknowledgement is the only success signal there is. This
 * Behavior exits SUCCESS on acknowledgement, and FAILURE if no server appears or none answers in
 * time. A service whose outcome you need to *read* wants Trigger and CallTriggerService instead.
 *
 * | Data Port Name                    | Port Type | Object Type |
 * | --------------------------------- | --------- | ----------- |
 * | service_name                      | Input     | std::string |
 * | response_timeout                  | Input     | double      |
 * | wait_for_server_available_timeout  | Input     | double      |
 */
class CallEmptyService final : public moveit_pro::behaviors::ServiceClientBehaviorBase<Empty>
{
public:
  CallEmptyService(const std::string& name, const BT::NodeConfiguration& config,
                   const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources);

  static BT::PortsList providedPorts();

  static BT::KeyValueVector metadata();

private:
  tl::expected<std::string, std::string> getServiceName() override;

  tl::expected<std::chrono::duration<double>, std::string> getResponseTimeout() override;

  tl::expected<std::chrono::duration<double>, std::string> getWaitForServerAvailableTimeout() override;

  /** @brief Empty::Request has no fields, so building one always succeeds. */
  tl::expected<Empty::Request, std::string> createRequest() override;

  /** @brief Empty::Response has no fields, so receiving one at all is the success signal. */
  tl::expected<bool, std::string> processResponse(const Empty::Response& response) override;

  /** @brief Classes derived from AsyncBehaviorBase must return a shared_future class member here. */
  std::shared_future<tl::expected<bool, std::string>>& getFuture() override
  {
    return future_;
  }

  std::shared_future<tl::expected<bool, std::string>> future_;
};
}  // namespace hangar_sim_behaviors
