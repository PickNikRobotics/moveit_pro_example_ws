// Copyright 2026 PickNik Inc.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the PickNik Inc. nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
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
