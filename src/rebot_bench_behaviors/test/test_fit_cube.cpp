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
#include <moveit_pro_test_utils/ros_test.hpp>
#include <moveit_studio_vision_msgs/msg/mask2_d.hpp>
#include <numbers>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <rebot_bench_behaviors/fit_cube_to_mask.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

class CubeFitTest : public moveit_pro::test_utils::RosTest
{
public:
  void SetUp() override
  {
    config.blackboard = BT::Blackboard::create();
    config.input_ports = { { "masks2d", "=" }, { "camera_info", "=" } };
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
  void observe(double x, double y, double yaw)
  {
    std::vector<cv::Point3f> corners;
    for (double dx : { -0.025, 0.025 })
      for (double dy : { -0.025, 0.025 })
        for (double z : { 0.0, 0.05 })
        {
          const Eigen::Vector3d p = world_camera.inverse() * Eigen::Vector3d(x + cos(yaw) * dx - sin(yaw) * dy,
                                                                             y + sin(yaw) * dx + cos(yaw) * dy, z);
          corners.emplace_back(p.x(), p.y(), p.z());
        }
    const cv::Mat k = (cv::Mat_<double>(3, 3) << 600, 0, 320, 0, 600, 240, 0, 0, 1);
    std::vector<cv::Point2f> pixels;
    cv::projectPoints(corners, cv::Vec3d(0, 0, 0), cv::Vec3d(0, 0, 0), k, info.d, pixels);
    std::vector<cv::Point2f> hull;
    cv::convexHull(pixels, hull);
    std::vector<cv::Point> rounded;
    for (auto p : hull)
      rounded.emplace_back(cvRound(p.x), cvRound(p.y));
    cv::Mat image = cv::Mat::zeros(480, 640, CV_8UC1);
    cv::fillConvexPoly(image, rounded, cv::Scalar(255));
    mask = moveit_studio_vision_msgs::msg::Mask2D{};
    mask.pixels = *cv_bridge::CvImage(info.header, "mono8", image).toImageMsg();
  }
  BT::NodeStatus run(std::vector<moveit_studio_vision_msgs::msg::Mask2D> candidates = {})
  {
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
