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
#include <rebot_bench_behaviors/report_unreachable_cube.hpp>

#include <format>
#include <moveit_pro_behavior_interface/get_required_ports.hpp>
#include <moveit_pro_behavior_interface/metadata_fields.hpp>

namespace rebot_bench_behaviors
{
std::string unreachableCubeMessage(const std::string& cube, const geometry_msgs::msg::PoseStamped& pose)
{
  return std::format("{} at x={:.3f} y={:.3f} is outside the arm's reach", cube, pose.pose.position.x,
                     pose.pose.position.y);
}

BT::PortsList ReportUnreachableCube::providedPorts()
{
  return { BT::InputPort<std::string>("cube", "Cube to name, such as blue cube."),
           BT::InputPort<geometry_msgs::msg::PoseStamped>("pose", "Where the cube sits.") };
}

BT::KeyValueVector ReportUnreachableCube::metadata()
{
  return { { moveit_pro::behaviors::kSubcategoryMetadataKey, "reBot Bench" },
           { moveit_pro::behaviors::kDescriptionMetadataKey,
             "Fails with a message naming the cube and where it sits. Use it after a reach check fails, before any "
             "motion toward the cube." } };
}

BT::NodeStatus ReportUnreachableCube::tick()
{
  const auto inputs = moveit_pro::behaviors::getRequiredInputs(getInput<std::string>("cube"),
                                                               getInput<geometry_msgs::msg::PoseStamped>("pose"));
  const auto message = inputs ? unreachableCubeMessage(std::get<0>(inputs.value()), std::get<1>(inputs.value())) :
                                "A cube is outside the arm's reach: " + inputs.error();
  getBehaviorContext()->logger->publishFailureMessage(name(), message);
  return BT::NodeStatus::FAILURE;
}
}  // namespace rebot_bench_behaviors
