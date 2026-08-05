// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include <lab_sim_behaviors/get_mask2d_from_region.hpp>

#include <algorithm>
#include <cmath>
#include <limits>

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/imgproc.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include <moveit_pro_behavior_interface/get_required_ports.hpp>

namespace lab_sim_behaviors
{
namespace
{
constexpr auto kPortIDRegion = "region";
constexpr auto kPortIDCameraInfo = "camera_info";
constexpr auto kPortIDMasks2D = "masks2d";

inline constexpr auto kDescriptionGetMask2DFromRegion = R"(
                <p>
                    Rasterizes a user-selected 2D region (see GetRegionFromUser) into a Mask2D so
                    the 3D-lifting Behaviors (GetMasks3DFromMasks2D) can project it onto the camera
                    point cloud and recover the region in 3D.
                </p>
            )";
}  // namespace

tl::expected<moveit_studio_vision_msgs::msg::Mask2D, std::string>
regionToMask2D(const geometry_msgs::msg::PolygonStamped& region, int image_width, int image_height)
{
  const auto& points = region.polygon.points;
  if (points.size() < 3)
  {
    return tl::make_unexpected("GetMask2DFromRegion: region needs at least 3 points, got " +
                               std::to_string(points.size()) + ".");
  }
  if (image_width <= 0 || image_height <= 0)
  {
    return tl::make_unexpected("GetMask2DFromRegion: camera image dimensions must be positive, got " +
                               std::to_string(image_width) + "x" + std::to_string(image_height) + ".");
  }

  // Reject non-finite or out-of-range vertices before they reach the float->int casts (UB) or
  // produce a huge/negative mask. GetRegionFromUser's coordinates are normalized to [0, 1].
  for (const geometry_msgs::msg::Point32& point : points)
  {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || point.x < 0.0F || point.x > 1.0F || point.y < 0.0F ||
        point.y > 1.0F)
    {
      return tl::make_unexpected("GetMask2DFromRegion: region vertices must be finite and within [0, 1].");
    }
  }

  // GetRegionFromUser returns normalized [0..1] image coordinates; scale them to pixels so the mask
  // aligns with the camera projection used by GetMasks3DFromMasks2D.
  const auto to_px_x = [image_width](float nx) { return nx * static_cast<float>(image_width); };
  const auto to_px_y = [image_height](float ny) { return ny * static_cast<float>(image_height); };

  // Axis-aligned pixel bounding box of the polygon.
  float min_x = std::numeric_limits<float>::max();
  float min_y = std::numeric_limits<float>::max();
  float max_x = std::numeric_limits<float>::lowest();
  float max_y = std::numeric_limits<float>::lowest();
  for (const geometry_msgs::msg::Point32& point : points)
  {
    min_x = std::min(min_x, to_px_x(point.x));
    min_y = std::min(min_y, to_px_y(point.y));
    max_x = std::max(max_x, to_px_x(point.x));
    max_y = std::max(max_y, to_px_y(point.y));
  }

  const int origin_x = static_cast<int>(std::floor(min_x));
  const int origin_y = static_cast<int>(std::floor(min_y));
  const int width = static_cast<int>(std::ceil(max_x)) - origin_x;
  const int height = static_cast<int>(std::ceil(max_y)) - origin_y;
  if (width <= 0 || height <= 0)
  {
    return tl::make_unexpected("GetMask2DFromRegion: region bounding box has zero area.");
  }

  // Fill the polygon into a mono8 image local to the bounding box (offset by the box origin).
  cv::Mat mask(height, width, CV_8UC1, cv::Scalar(0));
  std::vector<cv::Point> polygon;
  polygon.reserve(points.size());
  for (const geometry_msgs::msg::Point32& point : points)
  {
    polygon.emplace_back(static_cast<int>(std::lround(to_px_x(point.x))) - origin_x,
                         static_cast<int>(std::lround(to_px_y(point.y))) - origin_y);
  }
  cv::fillConvexPoly(mask, polygon, cv::Scalar(255));

  cv_bridge::CvImage cv_image;
  cv_image.header = region.header;
  cv_image.encoding = "mono8";
  cv_image.image = mask;

  moveit_studio_vision_msgs::msg::Mask2D result;
  result.x = origin_x;
  result.y = origin_y;
  result.pixels = *cv_image.toImageMsg();
  return result;
}

GetMask2DFromRegion::GetMask2DFromRegion(const std::string& name, const BT::NodeConfiguration& config,
                                         const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
  : moveit_pro::behaviors::SharedResourcesNode<BT::SyncActionNode>(name, config, shared_resources)
{
}

BT::PortsList GetMask2DFromRegion::providedPorts()
{
  return { BT::InputPort<geometry_msgs::msg::PolygonStamped>(kPortIDRegion, "{region}",
                                                             "The normalized [0..1] region to rasterize (e.g. the "
                                                             "region output of GetRegionFromUser)."),
           BT::InputPort<sensor_msgs::msg::CameraInfo>(kPortIDCameraInfo, "{camera_info}",
                                                       "Camera info whose width/height denormalize the region into "
                                                       "pixels, matching the point cloud."),
           BT::OutputPort<std::vector<moveit_studio_vision_msgs::msg::Mask2D>>(
               kPortIDMasks2D, "{masks2d}", "Single-element mask vector for GetMasks3DFromMasks2D.") };
}

BT::KeyValueVector GetMask2DFromRegion::metadata()
{
  return { { "subcategory", "Perception - 2D Image" }, { "description", kDescriptionGetMask2DFromRegion } };
}

BT::NodeStatus GetMask2DFromRegion::tick()
{
  const auto ports =
      moveit_pro::behaviors::getRequiredInputs(getInput<geometry_msgs::msg::PolygonStamped>(kPortIDRegion),
                                               getInput<sensor_msgs::msg::CameraInfo>(kPortIDCameraInfo));
  if (!ports.has_value())
  {
    getBehaviorContext()->logger->publishFailureMessage(
        name(), "Failed to get required values from input data ports: " + ports.error());
    return BT::NodeStatus::FAILURE;
  }
  const auto& [region, camera_info] = ports.value();

  const auto mask = regionToMask2D(region, static_cast<int>(camera_info.width), static_cast<int>(camera_info.height));
  if (!mask.has_value())
  {
    getBehaviorContext()->logger->publishFailureMessage(name(), mask.error());
    return BT::NodeStatus::FAILURE;
  }

  setOutput(kPortIDMasks2D, std::vector<moveit_studio_vision_msgs::msg::Mask2D>{ mask.value() });
  return BT::NodeStatus::SUCCESS;
}
}  // namespace lab_sim_behaviors
