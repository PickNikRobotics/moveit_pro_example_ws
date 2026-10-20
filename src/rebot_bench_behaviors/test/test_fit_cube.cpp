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
#include <gtest/gtest.h>
#include <cv_bridge/cv_bridge.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <limits>
#include <moveit_pro_test_utils/ros_test.hpp>
#include <moveit_studio_vision_msgs/msg/mask2_d.hpp>
#include <numbers>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <rebot_bench_behaviors/fit_cube_to_mask.hpp>
#include <rebot_bench_behaviors/is_cube_stacked.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

class CubeFitTest : public moveit_pro::test_utils::RosTest
{
public:
  void SetUp() override
  {
    config.blackboard = BT::Blackboard::create();
    config.input_ports = {
      { "masks2d", "=" }, { "camera_info", "=" }, { "support_height", "=" }, { "top_face_fallback", "=" }
    };
    config.output_ports = { { "cube_top", "=" }, { "silhouette_rms_px", "=" } };
    context = std::make_shared<moveit_pro::behaviors::BehaviorContext>(std::make_shared<rclcpp::Node>("cube_fit_test"),
                                                                       false);
    world_camera = Eigen::Isometry3d::Identity();
    world_camera.translation() = Eigen::Vector3d(0.25, 0.10, 0.6);
    world_camera.linear() = (Eigen::AngleAxisd(0.35, Eigen::Vector3d::UnitY()) *
                             Eigen::AngleAxisd(std::numbers::pi, Eigen::Vector3d::UnitX()))
                                .toRotationMatrix();
    auto tf = tf2::eigenToTransform(world_camera);
    tf.header.frame_id = "world";
    tf.child_frame_id = "camera";
    context->transform_buffer_ptr->setUsingDedicatedThread(true);
    context->transform_buffer_ptr->setTransform(tf, "test", true);
    auto base = tf2::eigenToTransform(Eigen::Isometry3d::Identity());
    base.header.frame_id = "world";
    base.child_frame_id = "base_link";
    context->transform_buffer_ptr->setTransform(base, "test", true);
    info.width = 640;
    info.height = 480;
    info.header.frame_id = "camera";
    info.k = { 600, 0, 320, 0, 600, 240, 0, 0, 1 };
    info.distortion_model = "plumb_bob";
    info.d = { 0, 0, 0, 0, 0 };
  }
  void observe(double x, double y, double yaw, double support = 0.0, bool top_only = false, bool side_only = false)
  {
    std::vector<cv::Point3f> corners;
    // side_only: just the +x face, which faces the camera from a cube at negative x.
    for (double dx : side_only ? std::vector{ 0.025 } : std::vector{ -0.025, 0.025 })
      for (double dy : { -0.025, 0.025 })
        for (double z : { top_only ? support + 0.05 : support, support + 0.05 })
        {
          const Eigen::Vector3d p = world_camera.inverse() * Eigen::Vector3d(x + cos(yaw) * dx - sin(yaw) * dy,
                                                                             y + sin(yaw) * dx + cos(yaw) * dy, z);
          corners.emplace_back(p.x(), p.y(), p.z());
        }
    const cv::Mat k = (cv::Mat_<double>(3, 3) << info.k[0], 0, info.k[2], 0, info.k[4], info.k[5], 0, 0, 1);
    std::vector<cv::Point2f> pixels;
    cv::projectPoints(corners, cv::Vec3d(0, 0, 0), cv::Vec3d(0, 0, 0), k, info.d, pixels);
    std::vector<cv::Point2f> hull;
    cv::convexHull(pixels, hull);
    std::vector<cv::Point> rounded;
    for (auto p : hull)
      rounded.emplace_back(cvRound(p.x), cvRound(p.y));
    cv::Mat image = cv::Mat::zeros(int(info.height), int(info.width), CV_8UC1);
    cv::fillConvexPoly(image, rounded, cv::Scalar(255));
    mask = moveit_studio_vision_msgs::msg::Mask2D{};
    mask.pixels = *cv_bridge::CvImage(info.header, "mono8", image).toImageMsg();
  }
  BT::NodeStatus run(std::vector<moveit_studio_vision_msgs::msg::Mask2D> candidates = {}, double support = 0.0,
                     bool top_face_fallback = false)
  {
    config.blackboard->set("support_height", support);
    config.blackboard->set("top_face_fallback", top_face_fallback);
    if (candidates.empty())
      candidates.push_back(mask);
    config.blackboard->set("masks2d", candidates);
    config.blackboard->set("camera_info", info);
    rebot_bench_behaviors::FitCubeToMask behavior("FitCubeToMask", config, context);
    return behavior.executeTick();
  }
  BT::NodeConfiguration config;
  std::shared_ptr<moveit_pro::behaviors::BehaviorContext> context;
  sensor_msgs::msg::CameraInfo info;
  moveit_studio_vision_msgs::msg::Mask2D mask;
  Eigen::Isometry3d world_camera;
};

TEST_F(CubeFitTest, RecoversScatteredCubeCentersAndYawFromRgbSilhouettes)
{
  for (const auto& pose :
       { Eigen::Vector3d(0.1, 0.10, 0), Eigen::Vector3d(0.2, 0.12, 0.2), Eigen::Vector3d(0.05, 0.02, -0.3),
         Eigen::Vector3d(0.12, 0.19, 0.65), Eigen::Vector3d(0.2, 0.00, -0.5) })
  {
    observe(pose.x(), pose.y(), pose.z());
    ASSERT_EQ(run(), BT::NodeStatus::SUCCESS);
    const auto top = config.blackboard->get<geometry_msgs::msg::PoseStamped>("cube_top");
    EXPECT_EQ(top.header.frame_id, "world");
    EXPECT_NEAR(top.pose.position.x, pose.x(), 0.002);
    EXPECT_NEAR(top.pose.position.y, pose.y(), 0.002);
    EXPECT_DOUBLE_EQ(top.pose.position.z, 0.05);
    const double yaw = 2 * atan2(top.pose.orientation.z, top.pose.orientation.w);
    EXPECT_NEAR(std::remainder(yaw - pose.z(), std::numbers::pi / 2), 0, 0.035);
  }
}
TEST_F(CubeFitTest, RecoversACubeStackedOnAnother)
{
  for (const auto& pose : { Eigen::Vector3d(0.1, 0.10, 0), Eigen::Vector3d(0.2, 0.12, 0.2) })
  {
    observe(pose.x(), pose.y(), pose.z(), 0.05);
    ASSERT_EQ(run({}, 0.05), BT::NodeStatus::SUCCESS);
    const auto top = config.blackboard->get<geometry_msgs::msg::PoseStamped>("cube_top");
    EXPECT_NEAR(top.pose.position.x, pose.x(), 0.002);
    EXPECT_NEAR(top.pose.position.y, pose.y(), 0.002);
    EXPECT_DOUBLE_EQ(top.pose.position.z, 0.10);
  }
}
TEST_F(CubeFitTest, RejectsACubeOnADifferentSupport)
{
  // A stacked cube does not pass for one on the mat, nor a cube on the mat for a stacked one.
  observe(0.1, 0.1, 0.2, 0.05);
  EXPECT_EQ(run({}, 0.0), BT::NodeStatus::FAILURE);
  observe(0.1, 0.1, 0.2);
  EXPECT_EQ(run({}, 0.05), BT::NodeStatus::FAILURE);
}
TEST_F(CubeFitTest, TopFaceFallbackLocatesAStackedCubeFromItsTopFaceAlone)
{
  for (const auto& pose : { Eigen::Vector3d(0.1, 0.10, 0), Eigen::Vector3d(0.2, 0.12, 0.2) })
  {
    observe(pose.x(), pose.y(), pose.z(), 0.05, true);
    ASSERT_EQ(run({}, 0.05, true), BT::NodeStatus::SUCCESS);
    const auto top = config.blackboard->get<geometry_msgs::msg::PoseStamped>("cube_top");
    EXPECT_NEAR(top.pose.position.x, pose.x(), 0.002);
    EXPECT_NEAR(top.pose.position.y, pose.y(), 0.002);
    EXPECT_DOUBLE_EQ(top.pose.position.z, 0.10);
  }
}
TEST_F(CubeFitTest, TopFaceFallbackIsNotUsedWhenAWholeCubeFits)
{
  // A face-sized blue patch elsewhere fits a top face; with the whole cube in view it must not count.
  observe(0.1, 0.1, 0.2, 0.05);
  const auto whole = mask;
  observe(0.2, 0.0, 0.0, 0.05, true);
  ASSERT_EQ(run({ whole, mask }, 0.05, true), BT::NodeStatus::SUCCESS);
  const auto top = config.blackboard->get<geometry_msgs::msg::PoseStamped>("cube_top");
  EXPECT_NEAR(top.pose.position.x, 0.1, 0.002);
  EXPECT_NEAR(top.pose.position.y, 0.1, 0.002);
}
TEST_F(CubeFitTest, TopFaceFallbackRejectsALoneSideFace)
{
  // Far from the camera, as on the bench, the camera-facing side of a stacked cube looks square and fits
  // a top face a cube width nearer the camera, though less well than a real top face.
  observe(-0.25, -0.1, 0.3, 0.05, false, true);
  EXPECT_EQ(run({ mask }, 0.05, true), BT::NodeStatus::FAILURE);
  observe(-0.25, -0.1, 0.3, 0.05, true);
  const auto top_face = mask;
  observe(-0.25, -0.1, 0.3, 0.05, false, true);
  ASSERT_EQ(run({ mask, top_face }, 0.05, true), BT::NodeStatus::SUCCESS);
  const auto top = config.blackboard->get<geometry_msgs::msg::PoseStamped>("cube_top");
  EXPECT_NEAR(top.pose.position.x, -0.25, 0.002);
  EXPECT_NEAR(top.pose.position.y, -0.1, 0.002);
}
TEST_F(CubeFitTest, MasksSplittingOneCubeAreFittedTogether)
{
  observe(0.1, 0.1, 0.2, 0.05);
  const cv::Mat image = cv_bridge::toCvCopy(mask.pixels)->image;
  const cv::Rect bounds = cv::boundingRect(image);
  const int cut = bounds.x + bounds.width / 2;
  cv::Mat left = image.clone(), right = image.clone();
  left.colRange(cut, left.cols).setTo(0);
  right.colRange(0, cut).setTo(0);
  auto first = mask, second = mask;
  first.pixels = *cv_bridge::CvImage(mask.pixels.header, "mono8", left).toImageMsg();
  second.pixels = *cv_bridge::CvImage(mask.pixels.header, "mono8", right).toImageMsg();
  ASSERT_EQ(run({ first, second }, 0.05), BT::NodeStatus::SUCCESS);
  const auto top = config.blackboard->get<geometry_msgs::msg::PoseStamped>("cube_top");
  EXPECT_NEAR(top.pose.position.x, 0.1, 0.002);
  EXPECT_NEAR(top.pose.position.y, 0.1, 0.002);
}
TEST_F(CubeFitTest, ClosingCanRepairOneSplitMaskWithAnUnrelatedDistractor)
{
  observe(0.1, 0.1, 0.2, 0.05);
  cv::Mat split = cv_bridge::toCvCopy(mask.pixels)->image;
  const auto bounds = cv::boundingRect(split);
  const int cut = bounds.x + bounds.width / 2;
  split.colRange(cut, cut + 2).setTo(0);
  mask.pixels = *cv_bridge::CvImage(info.header, "mono8", split).toImageMsg();
  auto distractor = mask;
  cv::Mat tiny = cv::Mat::zeros(split.size(), CV_8UC1);
  cv::rectangle(tiny, cv::Rect(20, 20, 3, 3), cv::Scalar(255), cv::FILLED);
  distractor.pixels = *cv_bridge::CvImage(info.header, "mono8", tiny).toImageMsg();
  EXPECT_EQ(run({ mask }, 0.05), BT::NodeStatus::FAILURE);
  ASSERT_EQ(run({ mask, distractor }, 0.05), BT::NodeStatus::SUCCESS);
  const auto top = config.blackboard->get<geometry_msgs::msg::PoseStamped>("cube_top");
  EXPECT_NEAR(top.pose.position.x, 0.1, 0.002);
  EXPECT_NEAR(top.pose.position.y, 0.1, 0.002);
}

TEST_F(CubeFitTest, AcceptsTheSameSilhouetteAsFullFrameAndCroppedMasks)
{
  observe(0.1, 0.1, 0.2);
  const cv::Mat image = cv_bridge::toCvCopy(mask.pixels)->image;
  const auto bounds = cv::boundingRect(image);
  auto cropped = mask;
  cropped.x = bounds.x;
  cropped.y = bounds.y;
  cropped.pixels = *cv_bridge::CvImage(info.header, "mono8", image(bounds)).toImageMsg();
  ASSERT_EQ(run({ mask, cropped }), BT::NodeStatus::SUCCESS);
  const auto top = config.blackboard->get<geometry_msgs::msg::PoseStamped>("cube_top");
  EXPECT_NEAR(top.pose.position.x, 0.1, 0.002);
  EXPECT_NEAR(top.pose.position.y, 0.1, 0.002);
}

TEST_F(CubeFitTest, TopFaceFallbackStillRejectsTheWrongSupport)
{
  observe(0.1, 0.1, 0.2, 0.05, true);
  EXPECT_EQ(run({}, 0.0, true), BT::NodeStatus::FAILURE);
  observe(0.1, 0.1, 0.2, 0.0, true);
  EXPECT_EQ(run({}, 0.05, true), BT::NodeStatus::FAILURE);
  // A whole cube on the mat does not pass for a stacked one either.
  observe(0.1, 0.1, 0.2);
  EXPECT_EQ(run({}, 0.05, true), BT::NodeStatus::FAILURE);
}
TEST_F(CubeFitTest, RejectsAnOccludedCube)
{
  observe(0.1, 0.1, 0.2);
  cv::Mat image = cv_bridge::toCvCopy(mask.pixels)->image;
  const cv::Rect bounds = cv::boundingRect(image);
  image(cv::Rect(bounds.x, bounds.y, bounds.width / 2, bounds.height)).setTo(0);
  mask.pixels = *cv_bridge::CvImage(info.header, "mono8", image).toImageMsg();
  EXPECT_EQ(run(), BT::NodeStatus::FAILURE);
}
TEST_F(CubeFitTest, RejectsMismatchedOpticalFrame)
{
  observe(0.1, 0.1, 0);
  mask.pixels.header.frame_id = "wrong_camera";
  EXPECT_EQ(run(), BT::NodeStatus::FAILURE);
}
TEST_F(CubeFitTest, RejectsUncalibratedCamera)
{
  observe(0.1, 0.1, 0);
  info.k[0] = 0;
  EXPECT_EQ(run(), BT::NodeStatus::FAILURE);
}

TEST_F(CubeFitTest, FiltersBackgroundMaskBeforeSelectingOneCube)
{
  observe(0.1, 0.1, 0);
  auto background = mask;
  cv::Mat whole = cv::Mat::ones(480, 640, CV_8UC1) * 255;
  background.pixels = *cv_bridge::CvImage(info.header, "mono8", whole).toImageMsg();
  EXPECT_EQ(run({ background, mask }), BT::NodeStatus::SUCCESS);
}
TEST_F(CubeFitTest, RejectsTwoDistinctMatchingCubes)
{
  observe(0.1, 0.05, 0);
  const auto first = mask;
  observe(0.15, 0.15, 0);
  EXPECT_EQ(run({ first, mask }), BT::NodeStatus::FAILURE);
}
TEST_F(CubeFitTest, AcceptsDuplicateMasksOfTheSameCube)
{
  observe(0.1, 0.1, 0);
  EXPECT_EQ(run({ mask, mask }), BT::NodeStatus::SUCCESS);
}

TEST_F(CubeFitTest, ChoosesEquivalentGraspYawNearRobotBaseHeading)
{
  Eigen::Isometry3d world_base = Eigen::Isometry3d::Identity();
  world_base.linear() = Eigen::AngleAxisd(std::numbers::pi, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  auto base = tf2::eigenToTransform(world_base);
  base.header.frame_id = "world";
  base.child_frame_id = "base_link";
  context->transform_buffer_ptr->setTransform(base, "test", true);
  observe(0.1, 0.1, 0);
  ASSERT_EQ(run(), BT::NodeStatus::SUCCESS);
  const auto top = config.blackboard->get<geometry_msgs::msg::PoseStamped>("cube_top");
  const double yaw = 2 * atan2(top.pose.orientation.z, top.pose.orientation.w);
  EXPECT_NEAR(yaw, std::numbers::pi, 0.035);
}

// The bench scene camera as shipped: config/config.yaml's scene_camera pose and the OAK's 640x400 stream.
class BenchCubeFitTest : public CubeFitTest
{
public:
  void SetUp() override
  {
    CubeFitTest::SetUp();
    const auto rpy = [](double roll, double pitch, double yaw) {
      return Eigen::Matrix3d(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                             Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                             Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()));
    };
    world_camera.translation() = Eigen::Vector3d(-0.075, 0.069, 0.565);
    world_camera.linear() = rpy(0.169, 0.975, 0.246) * rpy(-std::numbers::pi / 2, 0, -std::numbers::pi / 2);
    auto tf = tf2::eigenToTransform(world_camera);
    tf.header.frame_id = "world";
    tf.child_frame_id = "camera";
    context->transform_buffer_ptr->setTransform(tf, "test", true);
    info.width = 640;
    info.height = 400;
    info.k = { 282.5, 0, 320, 0, 282.5, 200, 0, 0, 1 };
  }
  // Blue on the mat with red resting beside it; red hides whatever part of blue it stands in front of.
  void observeBeside(const Eigen::Vector2d& blue, const Eigen::Vector2d& red)
  {
    observe(red.x(), red.y(), 0.0);
    const cv::Mat red_pixels = cv_bridge::toCvCopy(mask.pixels)->image;
    observe(blue.x(), blue.y(), 0.0);
    const Eigen::Vector3d camera = world_camera.translation();
    if ((Eigen::Vector3d(red.x(), red.y(), 0.025) - camera).norm() <
        (Eigen::Vector3d(blue.x(), blue.y(), 0.025) - camera).norm())
    {
      cv::Mat pixels = cv_bridge::toCvCopy(mask.pixels)->image;
      pixels.setTo(0, red_pixels);
      mask.pixels = *cv_bridge::CvImage(info.header, "mono8", pixels).toImageMsg();
    }
  }
  // Aligned depth inside the mask: where each masked pixel's ray meets blue resting on `support`, in world.
  sensor_msgs::msg::PointCloud2 maskedPoints(const Eigen::Vector2d& blue, double support) const
  {
    const cv::Mat pixels = cv_bridge::toCvCopy(mask.pixels)->image;
    const Eigen::Vector3d low(blue.x() - 0.025, blue.y() - 0.025, support);
    const Eigen::Vector3d high(blue.x() + 0.025, blue.y() + 0.025, support + 0.05);
    const Eigen::Vector3d from = world_camera.translation();
    std::vector<Eigen::Vector3d> hits;
    for (int v = 0; v < pixels.rows; ++v)
      for (int u = 0; u < pixels.cols; ++u)
      {
        if (!pixels.at<uint8_t>(v, u))
          continue;
        const Eigen::Vector3d ray =
            world_camera.linear() * Eigen::Vector3d((u - info.k[2]) / info.k[0], (v - info.k[5]) / info.k[4], 1);
        double near = 0, far = std::numeric_limits<double>::infinity();
        for (int axis = 0; axis < 3; ++axis)
        {
          const double a = (low[axis] - from[axis]) / ray[axis], b = (high[axis] - from[axis]) / ray[axis];
          near = std::max(near, std::min(a, b));
          far = std::min(far, std::max(a, b));
        }
        if (near <= far)
          hits.push_back(from + near * ray);
      }
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.frame_id = "world";
    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(hits.size());
    sensor_msgs::PointCloud2Iterator<float> x(cloud, "x"), y(cloud, "y"), z(cloud, "z");
    for (const auto& hit : hits)
    {
      *x = float(hit.x());
      *y = float(hit.y());
      *z = float(hit.z());
      ++x, ++y, ++z;
    }
    return cloud;
  }
  // The cycle's stack confirmation: blue fits a cube on a 50 mm support, sits over red's located top, and its
  // depth points stand as high as a stacked cube's.
  bool stackConfirmed(const Eigen::Vector2d& red, const sensor_msgs::msg::PointCloud2& points)
  {
    if (run({}, 0.05, true) != BT::NodeStatus::SUCCESS)
      return false;
    geometry_msgs::msg::PoseStamped red_top;
    red_top.header.frame_id = "world";
    red_top.pose.position.x = red.x();
    red_top.pose.position.y = red.y();
    red_top.pose.position.z = 0.05;
    config.blackboard->set("red_top", red_top);
    config.blackboard->set("cube_points", points);
    BT::NodeConfiguration check;
    check.blackboard = config.blackboard;
    check.input_ports = { { "cube_top", "{cube_top}" }, { "support_top", "{red_top}" },
                          { "max_offset", "0.012" },    { "cube_points", "{cube_points}" },
                          { "min_height", "0.075" },    { "height_quantile", "0.75" } };
    rebot_bench_behaviors::IsCubeStacked behavior("IsCubeStacked", check, context);
    return behavior.executeTick() == BT::NodeStatus::SUCCESS;
  }
  // Red resting `distance` from blue on the mat, `degrees` off the horizontal ray from blue to the camera.
  Eigen::Vector2d inFrontOf(const Eigen::Vector2d& blue, double distance, double degrees) const
  {
    const Eigen::Vector2d to_camera = world_camera.translation().head<2>() - blue;
    const double angle = std::atan2(to_camera.y(), to_camera.x()) + degrees * std::numbers::pi / 180;
    return blue + distance * Eigen::Vector2d(std::cos(angle), std::sin(angle));
  }
  const std::vector<Eigen::Vector2d> spots{ { 0.40, 0.15 }, { 0.30, 0.05 }, { 0.45, 0.25 }, { 0.35, 0.20 } };
};

TEST_F(BenchCubeFitTest, RgbAlonePassesALoneCubeJustBehindRedForStacked)
{
  // Red hides blue's lower front; blue's top face alone fits a stacked top face over red.
  const Eigen::Vector2d blue(0.40, 0.15), red(0.343, 0.145);
  observeBeside(blue, red);
  ASSERT_EQ(run({}, 0.05, true), BT::NodeStatus::SUCCESS);
  const auto top = config.blackboard->get<geometry_msgs::msg::PoseStamped>("cube_top");
  EXPECT_LT(std::hypot(top.pose.position.x - red.x(), top.pose.position.y - red.y()), 0.012);
  // Its depth points stand at the mat's cube height, so the stack check refuses it.
  EXPECT_FALSE(stackConfirmed(red, maskedPoints(blue, 0.0)));
  // Unstack infers red directly underneath this RGB fit: depth must still refuse it.
  EXPECT_FALSE(stackConfirmed({ top.pose.position.x, top.pose.position.y }, maskedPoints(blue, 0.0)));
}
TEST_F(BenchCubeFitTest, StackConfirmationRejectsALoneCubeOnTheMatJustBehindRed)
{
  for (const auto& blue : spots)
    for (const double distance : { 0.055, 0.059, 0.063 })
      for (const double degrees : { -6.0, -5.0, -4.0, 4.0, 5.0, 6.0 })
      {
        const Eigen::Vector2d red = inFrontOf(blue, distance, degrees);
        observeBeside(blue, red);
        EXPECT_FALSE(stackConfirmed(red, maskedPoints(blue, 0.0)))
            << "blue " << blue.transpose() << ", red " << red.transpose();
      }
}
TEST_F(BenchCubeFitTest, StackConfirmationAcceptsAStackWhoseMaskBleedsOntoTheMat)
{
  // A real stereo cloud puts some of the mask's edge pixels on the mat behind the cube: 15% on the bench.
  const Eigen::Vector2d red(0.35, 0.19);
  observe(red.x(), red.y(), 0.0, 0.05);
  auto points = maskedPoints(red, 0.05);
  const auto stacked = points.width;
  sensor_msgs::PointCloud2Modifier modifier(points);
  modifier.resize(stacked + stacked / 4);
  sensor_msgs::PointCloud2Iterator<float> z(points, "z");
  for (std::size_t i = 0; i < points.width; ++i, ++z)
    if (i >= stacked)
      *z = 0.0f;
  EXPECT_TRUE(stackConfirmed(red, points));
}
TEST_F(BenchCubeFitTest, StackConfirmationAcceptsBlueStackedOnRed)
{
  for (const auto& red : spots)
  {
    observe(red.x(), red.y(), 0.0, 0.05);
    EXPECT_TRUE(stackConfirmed(red, maskedPoints(red, 0.05))) << "red " << red.transpose();
  }
}
