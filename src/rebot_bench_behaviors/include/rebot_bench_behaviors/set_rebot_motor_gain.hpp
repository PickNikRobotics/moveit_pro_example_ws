// Copyright 2026 PickNik Inc.
// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <moveit_pro_behavior_interface/service_client_behavior_base.hpp>
#include <robstride_interfaces/srv/set_data_to_robstride.hpp>

namespace rebot_bench_behaviors
{
/** Write one arm motor's motion-mode kp or kd through the owning driver's service.
 * Only reBot arm IDs 1-6 are accepted; the jaw and arbitrary register writes are excluded.
 * The standard service-client base owns the 3 s discovery wait, 0.5 s response timeout,
 * and cancellation of the wait. Success acknowledges queuing in the driver, not motor
 * readback. Halt/timeout cannot undo a request that has already reached the driver.
 */
class SetRebotMotorGain final
  : public moveit_pro::behaviors::ServiceClientBehaviorBase<robstride_interfaces::srv::SetDataToRobstride>
{
public:
  using Service = robstride_interfaces::srv::SetDataToRobstride;
  SetRebotMotorGain(const std::string& name, const BT::NodeConfiguration& config,
                    const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& context);
  SetRebotMotorGain(const std::string& name, const BT::NodeConfiguration& config,
                    const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& context,
                    std::unique_ptr<moveit_pro::behaviors::ClientInterfaceBase<Service>> client);

  static BT::PortsList providedPorts();
  static BT::KeyValueVector metadata();
  tl::expected<std::string, std::string> getServiceName() override;
  tl::expected<std::chrono::duration<double>, std::string> getResponseTimeout() override;
  tl::expected<Service::Request, std::string> createRequest() override;
  tl::expected<bool, std::string> processResponse(const Service::Response& response) override;

private:
  std::shared_future<tl::expected<bool, std::string>>& getFuture() override
  {
    return future_;
  }
  std::shared_future<tl::expected<bool, std::string>> future_;
};
}  // namespace rebot_bench_behaviors
