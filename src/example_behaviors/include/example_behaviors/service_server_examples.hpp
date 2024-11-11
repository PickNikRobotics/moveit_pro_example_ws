#pragma once
#ifndef CUSTOM_INTERFACE__CUSTOM_INTERFACE_HPP_
#define CUSTOM_INTERFACE__CUSTOM_INTERFACE_HPP_

#include "service_server_base.hpp"
#include <std_srvs/srv/trigger.hpp>
#include <std_srvs/srv/set_bool.hpp>

namespace example_behaviors
{
/**
 * @brief Specific implementation of ServiceServerBase for std_srvs::srv::Trigger service.
 */
class TriggerServer : public ServiceServerBase<std_srvs::srv::Trigger>
{
public:
  TriggerServer(const std::string& name, const BT::NodeConfiguration& config,
                    const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources);

  static BT::KeyValueVector metadata();

protected:
  void processRequest([[maybe_unused]] const std_srvs::srv::Trigger::Request::SharedPtr& request,
                      const std_srvs::srv::Trigger::Response::SharedPtr& response) override;

};

/**
 * @brief Specific implementation of ServiceServerBase for std_srvs::srv::SetBool service.
 */
class SetBoolServer : public ServiceServerBase<std_srvs::srv::SetBool>
{
public:
  SetBoolServer(const std::string& name, const BT::NodeConfiguration& config,
                    const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources);

  BT::PortsList provideAdditionalPorts() override;

  static BT::KeyValueVector metadata();

protected:
  void processRequest([[maybe_unused]] const std_srvs::srv::SetBool::Request::SharedPtr& request,
                      const std_srvs::srv::SetBool::Response::SharedPtr& response) override;

};

}  // namespace example_behaviors

#endif  // CUSTOM_INTERFACE__CUSTOM_INTERFACE_HPP_
