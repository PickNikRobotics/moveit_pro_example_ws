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
#include <rebot_bench_behaviors/draw_place_spot.hpp>

#include <cmath>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_pro_behavior_interface/get_required_ports.hpp>
#include <moveit_pro_behavior_interface/metadata_fields.hpp>
#include <string>
#include <vector>

namespace rebot_bench_behaviors
{
BT::PortsList DrawPlaceSpot::providedPorts()
{
  return { BT::InputPort<std::vector<double>>("x_range", "Lowest and highest x of the spot, as min;max, meters."),
           BT::InputPort<std::vector<double>>("y_range", "Lowest and highest y of the spot, as min;max, meters."),
           BT::InputPort<geometry_msgs::msg::PoseStamped>("keep_clear_of", "Pose the spot keeps its distance from; the "
                                                                           "ranges are in its frame."),
           BT::InputPort<double>("clearance", "Smallest horizontal distance from keep_clear_of, meters."),
           BT::OutputPort<geometry_msgs::msg::PoseStamped>("spot", "The drawn point at z=0 in keep_clear_of's frame, "
                                                                   "with its orientation.") };
}

BT::KeyValueVector DrawPlaceSpot::metadata()
{
  return { { moveit_pro::behaviors::kSubcategoryMetadataKey, "reBot Bench" },
           { moveit_pro::behaviors::kDescriptionMetadataKey,
             "Draws a uniformly random spot inside an x/y rectangle, at least a clearance away from another pose. "
             "Every tick draws a new spot; pair it with reachability checks inside a retry to reject unreachable "
             "ones." } };
}

BT::NodeStatus DrawPlaceSpot::tick()
{
  const auto fail = [this](const std::string& message) {
    getBehaviorContext()->logger->publishFailureMessage(name(), message);
    return BT::NodeStatus::FAILURE;
  };
  const auto inputs = moveit_pro::behaviors::getRequiredInputs(
      getInput<std::vector<double>>("x_range"), getInput<std::vector<double>>("y_range"),
      getInput<geometry_msgs::msg::PoseStamped>("keep_clear_of"), getInput<double>("clearance"));
  if (!inputs)
    return fail("Provide x_range, y_range, keep_clear_of and clearance: " + inputs.error());
  const auto& [x_range, y_range, avoid, clearance] = inputs.value();
  if (x_range.size() != 2 || y_range.size() != 2 || !(x_range[0] <= x_range[1]) || !(y_range[0] <= y_range[1]) ||
      !(clearance >= 0))
    return fail("x_range and y_range must each be min;max, and clearance must not be negative.");
  std::uniform_real_distribution<double> x(x_range[0], x_range[1]);
  std::uniform_real_distribution<double> y(y_range[0], y_range[1]);
  // ponytail: rejection sampling; a region mostly inside the clearance needs a smarter draw.
  for (int attempt = 0; attempt < 1000; ++attempt)
  {
    geometry_msgs::msg::PoseStamped spot = avoid;
    spot.pose.position.x = x(generator_);
    spot.pose.position.y = y(generator_);
    spot.pose.position.z = 0.0;
    if (std::hypot(spot.pose.position.x - avoid.pose.position.x, spot.pose.position.y - avoid.pose.position.y) >=
        clearance)
    {
      setOutput("spot", spot);
      return BT::NodeStatus::SUCCESS;
    }
  }
  return fail("No spot in the region is " + std::to_string(clearance) + " m clear of keep_clear_of.");
}
}  // namespace rebot_bench_behaviors
