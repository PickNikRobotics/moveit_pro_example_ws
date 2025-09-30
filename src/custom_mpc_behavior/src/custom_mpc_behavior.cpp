#include "../include/custom_mpc_behavior/custom_mpc_behavior.hpp"

#include <custom_mpc_behavior/custom_mpc_behavior.hpp>

#include "spdlog/spdlog.h"

namespace custom_mpc_behavior
{
CustomMpcBehavior::CustomMpcBehavior(const std::string& name, const BT::NodeConfiguration& config,
                                     const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources)
  : moveit_studio::behaviors::MPCBehaviorBase<CustomMpcBehavior>(name, config, shared_resources)
{
}

BT::PortsList CustomMpcBehavior::providedPorts()
{
  return moveit_studio::behaviors::MPCBehaviorBase<CustomMpcBehavior>::providedPorts();
}

BT::KeyValueVector CustomMpcBehavior::metadata()
{
  return { { "description", "Custom MPC behavior" }, { "subcategory", "Motion - Execute" } };
}

}  // namespace custom_mpc_behavior
