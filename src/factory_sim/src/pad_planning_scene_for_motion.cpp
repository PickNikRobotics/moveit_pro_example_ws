// Copyright 2026 PickNik Inc.
// SPDX-License-Identifier: BSD-3-Clause
#include <algorithm>
#include <cmath>
#include <factory_sim/pad_planning_scene_for_motion.hpp>
#include <map>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_pro_base/robot_model/robot_model.hpp>
#include <moveit_pro_behavior_interface/get_required_ports.hpp>
namespace factory_sim
{
BT::PortsList PadPlanningSceneForMotion::providedPorts()
{
  return { BT::InputPort<moveit_msgs::msg::PlanningScene>("planning_scene", "Current full planning scene."),
           BT::InputPort<double>("minimum_padding_m", 0.01, "Finite nonnegative padding floor, in meters."),
           BT::OutputPort<moveit_msgs::msg::PlanningScene>("padded_scene",
                                                           "Private scene copy for trajectory validation.") };
}
BT::KeyValueVector PadPlanningSceneForMotion::metadata()
{
  return { { "subcategory", "Motion - Planning" },
           { "description", "Preserves collision margins when checking cached motion against the current scene." } };
}
BT::NodeStatus PadPlanningSceneForMotion::tick()
{
  const auto inputs = moveit_pro::behaviors::getRequiredInputs(
      getInput<moveit_msgs::msg::PlanningScene>("planning_scene"), getInput<double>("minimum_padding_m"));
  if (!inputs)
    return BT::NodeStatus::FAILURE;
  const auto& [input, padding] = inputs.value();
  if (input.is_diff || input.robot_state.is_diff || !std::isfinite(padding) || padding < 0.0 || !getBehaviorContext() ||
      !getBehaviorContext()->robot_model)
    return BT::NodeStatus::FAILURE;
  std::map<std::string, double> values;
  for (const auto& existing : input.link_padding)
    if (existing.link_name.empty() || !std::isfinite(existing.padding) || existing.padding < 0.0 ||
        !values.emplace(existing.link_name, existing.padding).second)
      return BT::NodeStatus::FAILURE;
  for (const auto& name : getBehaviorContext()->robot_model->getLinkModelNamesWithCollisionGeometry())
    values[name] = std::max(values[name], padding);
  auto output = input;
  output.link_padding.clear();
  for (const auto& [name, value] : values)
  {
    moveit_msgs::msg::LinkPadding link;
    link.link_name = name;
    link.padding = value;
    output.link_padding.push_back(link);
  }
  if (!setOutput("padded_scene", output))
    throw BT::RuntimeError("Cannot write the padded planning scene.");
  return BT::NodeStatus::SUCCESS;
}
}  // namespace factory_sim
