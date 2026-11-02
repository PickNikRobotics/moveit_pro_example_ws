// Copyright 2026 PickNik Inc.
// SPDX-License-Identifier: BSD-3-Clause
#include <rebot_bench_behaviors/set_rebot_motor_gain.hpp>

#include <cmath>
#include <moveit_pro_behavior_interface/get_required_ports.hpp>
#include <moveit_pro_behavior_interface/metadata_fields.hpp>

namespace rebot_bench_behaviors
{
SetRebotMotorGain::SetRebotMotorGain(const std::string& name, const BT::NodeConfiguration& config,
                                     const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& context)
  : ServiceClientBehaviorBase(name, config, context)
{
}

SetRebotMotorGain::SetRebotMotorGain(const std::string& name, const BT::NodeConfiguration& config,
                                     const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& context,
                                     std::unique_ptr<moveit_pro::behaviors::ClientInterfaceBase<Service>> client)
  : ServiceClientBehaviorBase(name, config, context, std::move(client))
{
}

BT::PortsList SetRebotMotorGain::providedPorts()
{
  return { BT::InputPort<int>("motor_id", "Arm motor ID, 1-6. Never the jaw."),
           BT::InputPort<std::string>("gain", "Motion-mode gain: kp or kd only."),
           BT::InputPort<double>("value", "Nonnegative gain: kp in Nm/rad, kd in Nm s/rad. Must fit the motor's "
                                          "protocol range.") };
}

BT::KeyValueVector SetRebotMotorGain::metadata()
{
  return { { moveit_pro::behaviors::kSubcategoryMetadataKey, "reBot Bench" },
           { moveit_pro::behaviors::kDescriptionMetadataKey,
             "Queues one arm motor kp/kd update through the existing RobStride driver. Requires motion mode. "
             "Re-anchor the trajectory controller to the measured pose before restoring kp. "
             "Success is a driver acknowledgement, not motor readback; cancellation does not roll back writes." } };
}

tl::expected<std::string, std::string> SetRebotMotorGain::getServiceName()
{
  return "/robstride_hardware_interface/set_data_to_robstride";
}

tl::expected<std::chrono::duration<double>, std::string> SetRebotMotorGain::getResponseTimeout()
{
  return std::chrono::duration<double>{ 0.5 };
}

tl::expected<SetRebotMotorGain::Service::Request, std::string> SetRebotMotorGain::createRequest()
{
  const auto inputs = moveit_pro::behaviors::getRequiredInputs(getInput<int>("motor_id"), getInput<std::string>("gain"),
                                                               getInput<double>("value"));
  if (!inputs)
    return tl::make_unexpected("Provide motor_id, gain and value: " + inputs.error());
  const auto& [id, gain, value] = inputs.value();
  if (id < 1 || id > 6)
    return tl::make_unexpected("motor_id must be an arm motor ID from 1 to 6; the jaw is excluded.");
  if (gain != "kp" && gain != "kd")
    return tl::make_unexpected("gain must be kp or kd; arbitrary register writes are not supported.");
  // Type-1 encoding ranges from robstride_sdk/robstride_protocol.hpp, not torque clamps:
  // arm IDs 1-3 are RS06, 4-6 are RS00. Reject rather than silently saturate the gain.
  const double maximum = gain == "kp" ? (id <= 3 ? 5000.0 : 500.0) : (id <= 3 ? 100.0 : 5.0);
  if (!std::isfinite(value) || value < 0.0 || value > maximum)
    return tl::make_unexpected("value must be finite and in [0, " + std::to_string(maximum) + "] for this motor/gain.");
  Service::Request request;
  request.header.stamp = getBehaviorContext()->node->now();
  request.id = static_cast<std::uint8_t>(id);
  request.item_name = gain;
  request.item_data = static_cast<float>(value);
  return request;
}

tl::expected<bool, std::string> SetRebotMotorGain::processResponse(const Service::Response& response)
{
  if (!response.result)
    return tl::make_unexpected(
        "RobStride driver rejected the motor gain write. Earlier writes, if any, remain applied.");
  return true;
}
}  // namespace rebot_bench_behaviors
