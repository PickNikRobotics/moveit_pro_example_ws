// Copyright 2026 PickNik Inc.
// SPDX-License-Identifier: BSD-3-Clause

#include <factory_sim/is_lookahead_enabled.hpp>

namespace factory_sim
{
BT::PortsList IsLookaheadEnabled::providedPorts()
{
  return { BT::InputPort<bool>("enabled", "Whether to use lookahead planning.") };
}

BT::KeyValueVector IsLookaheadEnabled::metadata()
{
  return { { "subcategory", "Motion - Planning" }, { "description", "Reads the boolean lookahead planning flag." } };
}

BT::NodeStatus IsLookaheadEnabled::tick()
{
  const auto enabled = getInput<bool>("enabled");
  if (!enabled)
    throw BT::RuntimeError("Lookahead enabled port must contain a boolean: " + enabled.error());
  return enabled.value() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}
}  // namespace factory_sim
