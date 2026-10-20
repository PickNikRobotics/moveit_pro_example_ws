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
#include <rebot_bench_behaviors/is_cube_stacked.hpp>

#include <algorithm>
#include <cmath>
#include <format>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_pro_behavior_interface/get_required_ports.hpp>
#include <moveit_pro_behavior_interface/metadata_fields.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <string>
#include <vector>

namespace rebot_bench_behaviors
{
BT::PortsList IsCubeStacked::providedPorts()
{
  return { BT::InputPort<geometry_msgs::msg::PoseStamped>("cube_top", "Located top face of the upper cube."),
           BT::InputPort<geometry_msgs::msg::PoseStamped>("support_top", "Located top face of the cube it should rest "
                                                                         "on, in cube_top's frame."),
           BT::InputPort<double>("max_offset", "Largest horizontal distance between the two centres, meters."),
           BT::InputPort<sensor_msgs::msg::PointCloud2>("cube_points", "Depth points inside the upper cube's mask, in "
                                                                       "cube_top's frame."),
           BT::InputPort<double>("min_height", "Lowest height of cube_points above z=0 at height_quantile, meters."),
           BT::InputPort<double>("height_quantile", 0.75,
                                 "Fraction of cube_points below the height compared with min_height, 0 to 1. Above "
                                 "the median, so depth pixels at the mask's edge that land on the mat behind the cube "
                                 "do not pull a stacked cube down.") };
}

BT::KeyValueVector IsCubeStacked::metadata()
{
  return { { moveit_pro::behaviors::kSubcategoryMetadataKey, "reBot Bench" },
           { moveit_pro::behaviors::kDescriptionMetadataKey,
             "Succeeds only when the located upper cube is centred over the located lower one, within max_offset, "
             "and its depth points at height_quantile stand at least min_height high. One camera's silhouette cannot "
             "tell "
             "a cube resting on another from a cube on the mat just behind it; depth can." } };
}

BT::NodeStatus IsCubeStacked::tick()
{
  const auto fail = [this](const std::string& message) {
    getBehaviorContext()->logger->publishFailureMessage(name(), message);
    return BT::NodeStatus::FAILURE;
  };
  const auto inputs = moveit_pro::behaviors::getRequiredInputs(
      getInput<geometry_msgs::msg::PoseStamped>("cube_top"), getInput<geometry_msgs::msg::PoseStamped>("support_top"),
      getInput<double>("max_offset"), getInput<sensor_msgs::msg::PointCloud2>("cube_points"),
      getInput<double>("min_height"), getInput<double>("height_quantile"));
  if (!inputs)
    return fail("Provide cube_top, support_top, max_offset, cube_points and min_height: " + inputs.error());
  const auto& [cube, support, max_offset, points, min_height, quantile] = inputs.value();
  if (!(quantile >= 0.0 && quantile <= 1.0))
    return fail("height_quantile must be between 0 and 1.");
  if (cube.header.frame_id != support.header.frame_id || points.header.frame_id != cube.header.frame_id)
    return fail("cube_top, support_top and cube_points must be in the same frame.");
  const auto& a = cube.pose.position;
  const auto& b = support.pose.position;
  const double offset = std::hypot(a.x - b.x, a.y - b.y);
  if (!(offset <= max_offset))
    return fail(std::format("The cube at x={:.3f} y={:.3f} is {:.0f} mm from the centre of the cube under it at "
                            "x={:.3f} y={:.3f}, more than {:.0f} mm: it is not stacked.",
                            a.x, a.y, offset * 1000, b.x, b.y, max_offset * 1000));
  std::vector<float> heights;
  for (sensor_msgs::PointCloud2ConstIterator<float> z(points, "z"); z != z.end(); ++z)
    if (std::isfinite(*z))
      heights.push_back(*z);
  if (heights.empty())
    return fail("No depth points fall inside the cube's mask; check the aligned depth stream.");
  const auto at = heights.begin() +
                  static_cast<std::ptrdiff_t>(std::min(heights.size() - 1, std::size_t(quantile * heights.size())));
  std::nth_element(heights.begin(), at, heights.end());
  if (!(*at >= min_height))
    return fail(
        std::format("The cube's depth points stand {:.0f} mm high at quantile {:.2f}, below {:.0f} mm: it rests "
                    "on the mat, not on the cube under it.",
                    *at * 1000, quantile, min_height * 1000));
  return BT::NodeStatus::SUCCESS;
}
}  // namespace rebot_bench_behaviors
