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
#include <rebot_bench_behaviors/fit_cube_to_mask.hpp>

#include <algorithm>
#include <cmath>
#include <cv_bridge/cv_bridge.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <limits>
#include <moveit_pro_behavior_interface/get_required_ports.hpp>
#include <moveit_pro_behavior_interface/metadata_fields.hpp>
#include <moveit_pro_behavior_interface/tf_tools.hpp>
#include <moveit_studio_vision_msgs/msg/mask2_d.hpp>
#include <numbers>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

namespace rebot_bench_behaviors
{
BT::PortsList FitCubeToMask::providedPorts()
{
  return { BT::InputPort<std::vector<moveit_studio_vision_msgs::msg::Mask2D>>("masks2d"),
           BT::InputPort<sensor_msgs::msg::CameraInfo>("camera_info"),
           BT::OutputPort<geometry_msgs::msg::PoseStamped>("cube_top"), BT::OutputPort<double>("silhouette_rms_px") };
}

BT::KeyValueVector FitCubeToMask::metadata()
{
  return { { moveit_pro::behaviors::kSubcategoryMetadataKey, "reBot Bench" },
           { moveit_pro::behaviors::kDescriptionMetadataKey,
             "Fits the complete silhouette of one upright 50 mm cube resting on world z=0. Requires calibrated "
             "RGB intrinsics and optical-frame TF at the image timestamp. Rejects occluded or incomplete masks." } };
}

BT::NodeStatus FitCubeToMask::tick()
{
  const auto fail = [this](const std::string& message) {
    getBehaviorContext()->logger->publishFailureMessage(name(), message);
    return BT::NodeStatus::FAILURE;
  };
  const auto inputs =
      moveit_pro::behaviors::getRequiredInputs(getInput<std::vector<moveit_studio_vision_msgs::msg::Mask2D>>("masks2d"),
                                               getInput<sensor_msgs::msg::CameraInfo>("camera_info"));
  if (!inputs)
    return fail(inputs.error());
  const auto& [masks, info] = inputs.value();
  if (masks.empty() || info.k[0] <= 0 || info.k[4] <= 0)
    return fail("Provide SAM3 cube masks and calibrated RGB CameraInfo.");
  struct Candidate
  {
    geometry_msgs::msg::PoseStamped pose;
    double rms;
  };
  std::vector<Candidate> candidates;
  std::string last_error;
  const auto fit = [&](const auto& mask) -> tl::expected<Candidate, std::string> {
    const auto fail = [](const std::string& message) -> tl::expected<Candidate, std::string> {
      return tl::make_unexpected(message);
    };
    if (mask.pixels.header.frame_id != info.header.frame_id || mask.x < 0 || mask.y < 0 ||
        mask.x + mask.pixels.width > info.width || mask.y + mask.pixels.height > info.height)
      return fail("SAM3 mask and CameraInfo must describe the same RGB optical frame and image dimensions.");
    const auto world_camera = moveit_pro::helper_functions::getTransformAsIsometry(
        *getBehaviorContext()->transform_buffer_ptr, "world", mask.pixels.header, std::chrono::duration<double>(0.2));
    if (!world_camera)
      return fail(world_camera.error());
    const Eigen::Isometry3d camera_world = world_camera->inverse();
    std_msgs::msg::Header base_header = mask.pixels.header;
    base_header.frame_id = "base_link";
    const auto world_base = moveit_pro::helper_functions::getTransformAsIsometry(
        *getBehaviorContext()->transform_buffer_ptr, "world", base_header, std::chrono::duration<double>(0.2));
    if (!world_base)
      return fail(world_base.error());
    const double base_yaw = std::atan2(world_base->linear()(1, 0), world_base->linear()(0, 0));
    cv::Mat binary;
    try
    {
      const cv::Mat pixels = cv_bridge::toCvCopy(mask.pixels)->image;
      if (pixels.channels() != 1)
        return fail("The SAM3 mask must have one channel.");
      cv::compare(pixels, pixels.depth() == CV_32F || pixels.depth() == CV_64F ? 0.5 : 0, binary, cv::CMP_GT);
    }
    catch (const cv_bridge::Exception& error)
    {
      return fail(error.what());
    }
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
    if (contours.size() != 1 || cv::contourArea(contours.front()) < 100)
      return fail("The cube mask must contain one connected silhouette of at least 100 pixels.");
    const cv::Rect box = cv::boundingRect(contours.front());
    if (box.x + mask.x <= 1 || box.y + mask.y <= 1 || box.br().x + mask.x >= int(info.width) - 1 ||
        box.br().y + mask.y >= int(info.height) - 1)
      return fail("The cube touches the image edge; move it fully into view before locating it.");

    const cv::Mat camera = (cv::Mat_<double>(3, 3) << info.k[0], 0, info.k[2], 0, info.k[4], info.k[5], 0, 0, 1);
    if (!info.distortion_model.empty() && info.distortion_model != "plumb_bob" &&
        info.distortion_model != "rational_polynomial")
      return fail("Use a plumb_bob/rational_polynomial RGB camera model or a rectified image.");
    const cv::Mat distortion(info.d, true);
    std::vector<cv::Point2f> observed;
    for (const auto& p : contours.front())
      observed.emplace_back(p.x + mask.x, p.y + mask.y);
    // Work in undistorted pixel coordinates, retaining pixel units for the fit quality check.
    cv::undistortPoints(observed, observed, camera, distortion, cv::noArray(), camera);
    std::vector<cv::Point2f> sparse;
    for (std::size_t i = 0; i < observed.size(); i += std::max<std::size_t>(1, observed.size() / 100))
      sparse.push_back(observed[i]);
    const cv::Moments moments = cv::moments(observed);
    const double u = moments.m10 / moments.m00, v = moments.m01 / moments.m00;
    const Eigen::Vector3d ray =
        world_camera->linear() * Eigen::Vector3d((u - info.k[2]) / info.k[0], (v - info.k[5]) / info.k[4], 1);
    const Eigen::Vector3d origin = world_camera->translation();
    if (ray.z() >= -0.1 || origin.z() <= 0.10)
      return fail("The calibrated RGB camera must look down at the mat in the world frame.");
    const Eigen::Vector3d initial = origin + ray * ((0.025 - origin.z()) / ray.z());

    const auto hull = [&](const Eigen::Vector3d& pose) {
      std::vector<cv::Point2f> pixels;
      const double c = std::cos(pose.z()), s = std::sin(pose.z());
      for (const double x : { -0.025, 0.025 })
        for (const double y : { -0.025, 0.025 })
          for (const double z : { 0.0, 0.05 })
          {
            const Eigen::Vector3d p =
                camera_world * Eigen::Vector3d(pose.x() + c * x - s * y, pose.y() + s * x + c * y, z);
            if (p.z() <= 0.05)
              return std::vector<cv::Point2f>{};
            pixels.emplace_back(info.k[0] * p.x() / p.z() + info.k[2], info.k[4] * p.y() / p.z() + info.k[5]);
          }
      std::vector<cv::Point2f> result;
      cv::convexHull(pixels, result);
      return result;
    };
    const double expected_area = cv::contourArea(hull(Eigen::Vector3d(initial.x(), initial.y(), 0)));
    const double observed_area = cv::contourArea(observed);
    if (observed_area < expected_area * 0.5 || observed_area > expected_area * 2)
      return fail("Mask area is inconsistent with a 50 mm cube at its calibrated mat-plane position.");
    const auto loss = [&](const Eigen::Vector3d& pose) {
      const auto projected = hull(pose);
      if (projected.empty())
        return std::numeric_limits<double>::infinity();
      double total = 0;
      std::size_t count = 0;
      for (const auto& p : sparse)
      {
        const double d = cv::pointPolygonTest(projected, p, true);
        total += d * d;
        ++count;
      }
      // Symmetric contour distance penalizes missing/occluded parts of a cube's silhouette.
      for (std::size_t i = 0; i < projected.size(); ++i)
        for (int j = 0; j < 12; ++j)
        {
          const cv::Point2f p = projected[i] * (1.0F - j / 12.0F) + projected[(i + 1) % projected.size()] * (j / 12.0F);
          const double d = cv::pointPolygonTest(observed, p, true);
          total += d * d;
          ++count;
        }
      return total / count;
    };
    Eigen::Vector3d best;
    double best_loss = std::numeric_limits<double>::infinity();
    // Multi-start covers the square's 90 degree rotational symmetry; refine translation and yaw jointly.
    for (int seed = 0; seed < 12; ++seed)
    {
      Eigen::Vector3d pose(initial.x(), initial.y(), seed * std::numbers::pi / 24.0);
      double cost = loss(pose);
      for (double scale = 1; scale >= 1.0 / 128; scale *= 0.5)
      {
        const Eigen::Vector3d step(0.01 * scale, 0.01 * scale, 0.08 * scale);
        for (int iteration = 0; iteration < 60; ++iteration)
        {
          bool improved = false;
          for (int axis = 0; axis < 3; ++axis)
            for (int sign : { -1, 1 })
            {
              Eigen::Vector3d candidate = pose;
              candidate[axis] += sign * step[axis];
              const double candidate_cost = loss(candidate);
              if (candidate_cost < cost)
              {
                pose = candidate;
                cost = candidate_cost;
                improved = true;
              }
            }
          if (!improved)
            break;
        }
      }
      if (cost < best_loss)
      {
        best = pose;
        best_loss = cost;
      }
    }
    const double rms = std::sqrt(best_loss);
    if (!std::isfinite(rms) || rms > 2.0)
      return fail("The SAM3 silhouette does not fit a complete 50 mm cube on the calibrated mat: RMS " +
                  std::to_string(rms) + " px. Check occlusion, cube size, and calibration.");
    geometry_msgs::msg::PoseStamped top;
    top.header = mask.pixels.header;
    top.header.frame_id = "world";
    top.pose.position.x = best.x();
    top.pose.position.y = best.y();
    top.pose.position.z = 0.05;
    // A square has four equivalent grasp axes. Choose the one nearest the robot's heading,
    // avoiding a gratuitous half-turn that consumes this arm's wrist-roll range.
    const double yaw = base_yaw + std::remainder(best.z() - base_yaw, std::numbers::pi / 2);
    top.pose.orientation.z = std::sin(yaw / 2);
    top.pose.orientation.w = std::cos(yaw / 2);
    return Candidate{ top, rms };
  };
  for (const auto& mask : masks)
  {
    const auto candidate = fit(mask);
    if (candidate)
      candidates.push_back(*candidate);
    else
      last_error = candidate.error();
  }
  getBehaviorContext()->logger->publishInfoMessage(name(), std::to_string(candidates.size()) +
                                                               " cube-sized fitted candidates from " +
                                                               std::to_string(masks.size()) + " SAM3 masks.");
  if (candidates.empty())
    return fail("No SAM3 candidate fits a 50 mm cube on the mat. " + last_error);
  std::ranges::sort(candidates, {}, &Candidate::rms);
  const auto& best = candidates.front();
  for (const auto& candidate : candidates)
  {
    const auto& a = best.pose.pose.position;
    const auto& b = candidate.pose.pose.position;
    if (std::hypot(a.x - b.x, a.y - b.y) > 0.01)
      return fail("More than one distinct 50 mm cube matches this prompt; remove the ambiguity before moving.");
  }
  setOutput("cube_top", best.pose);
  setOutput("silhouette_rms_px", best.rms);
  return BT::NodeStatus::SUCCESS;
}
}  // namespace rebot_bench_behaviors
