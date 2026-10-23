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

#include <chrono>
#include <cmath>
#include <mutex>
#include <optional>
#include <thread>
#include <vector>

#include <behaviortree_cpp/bt_factory.h>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <lunar_sim_behaviors/dead_reckon_motion.hpp>
#include <moveit_pro_behavior_interface/shared_resources_node_loader.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;
using lunar_sim_behaviors::PlanarPose;
using lunar_sim_behaviors::detail::driveDistanceCommand;
using lunar_sim_behaviors::detail::turnCommand;

TEST(DriveDistanceCommand, ProgressIsMeasuredAlongTheStartHeading)
{
  const PlanarPose start{ 1.0, 2.0, M_PI_2 };
  // 0.5 m sideways counts for nothing; 0.5 m along +y (the start heading) leaves 1.0 m.
  const auto command = driveDistanceCommand(start, { 1.5, 2.5, M_PI_2 }, 1.5, 0.3, 0.3);
  EXPECT_FALSE(command.done);
  EXPECT_DOUBLE_EQ(command.twist.linear.x, 0.3);
  EXPECT_DOUBLE_EQ(command.twist.angular.z, 0.0);
}

TEST(DriveDistanceCommand, SteersBackToTheStartHeading)
{
  const auto command = driveDistanceCommand({ 0, 0, 0 }, { 0.2, 0, 0.1 }, 1.0, 0.3, 0.3);
  EXPECT_LT(command.twist.angular.z, 0.0);
  EXPECT_GE(command.twist.angular.z, -0.3);
}

TEST(DriveDistanceCommand, TapersThenStopsAtTheDistance)
{
  const auto near = driveDistanceCommand({ 0, 0, 0 }, { 0.9, 0, 0 }, 1.0, 0.3, 0.3);
  EXPECT_FALSE(near.done);
  EXPECT_LT(near.twist.linear.x, 0.3);
  EXPECT_GT(near.twist.linear.x, 0.0);

  const auto arrived = driveDistanceCommand({ 0, 0, 0 }, { 1.0, 0, 0 }, 1.0, 0.3, 0.3);
  EXPECT_TRUE(arrived.done);
  EXPECT_DOUBLE_EQ(arrived.twist.linear.x, 0.0);
}

TEST(DriveDistanceCommand, StopsWithinToleranceAndOnOvershoot)
{
  EXPECT_FALSE(driveDistanceCommand({ 0, 0, 0 }, { 0.98, 0, 0 }, 1.0, 0.3, 0.3).done);
  EXPECT_TRUE(driveDistanceCommand({ 0, 0, 0 }, { 0.995, 0, 0 }, 1.0, 0.3, 0.3).done);
  EXPECT_TRUE(driveDistanceCommand({ 0, 0, 0 }, { 1.2, 0, 0 }, 1.0, 0.3, 0.3).done);
}

TEST(DriveDistanceCommand, ClampsHeadingHoldAndFloorsTheTaper)
{
  const auto command = driveDistanceCommand({ 0, 0, 0 }, { 0.98, 0, 0.3 }, 1.0, 0.3, 0.2);
  EXPECT_DOUBLE_EQ(command.twist.angular.z, -0.2);
  EXPECT_DOUBLE_EQ(command.twist.linear.x, 0.05);
}

TEST(DriveDistanceCommand, PausesForwardMotionWhileFarOffHeading)
{
  const auto command = driveDistanceCommand({ 0, 0, 0 }, { 0.2, 0, -0.8 }, 1.0, 0.3, 0.3);
  EXPECT_DOUBLE_EQ(command.twist.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(command.twist.angular.z, 0.3);
}

TEST(TurnCommand, StopsWithinToleranceAndFloorsTheTaper)
{
  EXPECT_FALSE(turnCommand(M_PI_2 - 0.02, M_PI_2, 0.5).done);
  EXPECT_TRUE(turnCommand(M_PI_2 - 0.005, M_PI_2, 0.5).done);
  EXPECT_DOUBLE_EQ(turnCommand(M_PI_2 - 0.02, M_PI_2, 0.5).twist.angular.z, 0.15);
}

TEST(TurnCommand, TurnsTowardTheTargetAndStops)
{
  EXPECT_GT(turnCommand(0.0, M_PI_2, 0.5).twist.angular.z, 0.0);
  EXPECT_LT(turnCommand(0.0, -M_PI_2, 0.5).twist.angular.z, 0.0);
  // Overshoot reverses.
  EXPECT_LT(turnCommand(M_PI_2 + 0.2, M_PI_2, 0.5).twist.angular.z, 0.0);
  EXPECT_TRUE(turnCommand(M_PI_2, M_PI_2, 0.5).done);
  EXPECT_DOUBLE_EQ(turnCommand(0.0, M_PI_2, 0.5).twist.angular.z, 0.5);
}

/**
 * @brief Runs the real Behaviors over ROS against a scripted odometry stream.
 */
class DeadReckonMotionRosTest : public testing::Test
{
protected:
  void SetUp() override
  {
    behavior_node_ = std::make_shared<rclcpp::Node>("behavior_node");
    test_node_ = std::make_shared<rclcpp::Node>("test_node");
    executor_.add_node(behavior_node_);
    executor_.add_node(test_node_);
    spin_thread_ = std::jthread([this] { executor_.spin(); });

    odometry_publisher_ =
        test_node_->create_publisher<nav_msgs::msg::Odometry>("/odom_filtered", rclcpp::SystemDefaultsQoS());
    cmd_vel_subscription_ = test_node_->create_subscription<geometry_msgs::msg::TwistStamped>(
        "/cmd_vel", rclcpp::SystemDefaultsQoS(), [this](const geometry_msgs::msg::TwistStamped& msg) {
          const std::scoped_lock lock(mutex_);
          received_.push_back(msg);
        });

    shared_resources_ = std::make_shared<moveit_pro::behaviors::BehaviorContext>(behavior_node_, false);
    class_loader_ = std::make_unique<pluginlib::ClassLoader<moveit_pro::behaviors::SharedResourcesNodeLoaderBase>>(
        "moveit_pro_behavior_interface", "moveit_pro::behaviors::SharedResourcesNodeLoaderBase");
    plugin_ = class_loader_->createUniqueInstance("lunar_sim_behaviors::LunarSimBehaviorsLoader");
    plugin_->registerBehaviors(factory_, shared_resources_);
  }

  void TearDown() override
  {
    executor_.cancel();
    spin_thread_.join();
  }

  BT::Tree makeTree(const std::string& action)
  {
    return factory_.createTreeFromText(R"(<root BTCPP_format="4"><BehaviorTree ID="Test">)" + action +
                                       "</BehaviorTree></root>");
  }

  void publishOdometry(const double x, const double y, const double yaw,
                       const std::optional<rclcpp::Time>& stamp = std::nullopt) const
  {
    nav_msgs::msg::Odometry msg;
    msg.header.frame_id = "odom";
    msg.header.stamp = stamp.value_or(test_node_->now());
    msg.pose.pose.position.x = x;
    msg.pose.pose.position.y = y;
    msg.pose.pose.orientation.z = std::sin(yaw / 2);
    msg.pose.pose.orientation.w = std::cos(yaw / 2);
    odometry_publisher_->publish(msg);
  }

  // Publishes each pose in turn, ticking for up to 300 ms after each (or until the Behavior finishes), and returns
  // the final status.
  BT::NodeStatus run(BT::Tree& tree, const std::vector<PlanarPose>& poses)
  {
    auto status = tree.tickOnce();
    for (const auto& pose : poses)
    {
      publishOdometry(pose.x, pose.y, pose.yaw);
      for (int i = 0; i < 30 && status == BT::NodeStatus::RUNNING; ++i)
      {
        std::this_thread::sleep_for(10ms);
        status = tree.tickOnce();
      }
    }
    return status;
  }

  std::size_t receivedCount()
  {
    const std::scoped_lock lock(mutex_);
    return received_.size();
  }

  // Ticks until the Behavior leaves RUNNING or the deadline passes.
  static BT::NodeStatus tickUntilDone(BT::Tree& tree, const std::chrono::milliseconds timeout)
  {
    auto status = tree.tickOnce();
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (status == BT::NodeStatus::RUNNING && std::chrono::steady_clock::now() < deadline)
    {
      std::this_thread::sleep_for(20ms);
      status = tree.tickOnce();
    }
    return status;
  }

  geometry_msgs::msg::TwistStamped lastReceived()
  {
    // Let the final zero twist arrive.
    std::this_thread::sleep_for(100ms);
    const std::scoped_lock lock(mutex_);
    if (received_.empty())
    {
      ADD_FAILURE() << "no TwistStamped received";
      return geometry_msgs::msg::TwistStamped();
    }
    return received_.back();
  }

  rclcpp::Node::SharedPtr behavior_node_;
  rclcpp::Node::SharedPtr test_node_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::jthread spin_thread_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_subscription_;
  std::mutex mutex_;
  std::vector<geometry_msgs::msg::TwistStamped> received_;

  std::shared_ptr<moveit_pro::behaviors::BehaviorContext> shared_resources_;
  std::unique_ptr<pluginlib::ClassLoader<moveit_pro::behaviors::SharedResourcesNodeLoaderBase>> class_loader_;
  pluginlib::UniquePtr<moveit_pro::behaviors::SharedResourcesNodeLoaderBase> plugin_;
  BT::BehaviorTreeFactory factory_;
};

TEST_F(DeadReckonMotionRosTest, DriveDistanceSucceedsWhenOdometryReachesTheDistance)
{
  auto tree = makeTree(R"(<Action ID="DriveDistance" distance="1.0" />)");
  EXPECT_EQ(run(tree, { { 0, 0, 0 }, { 0.5, 0, 0 } }), BT::NodeStatus::RUNNING);
  EXPECT_GT(lastReceived().twist.linear.x, 0.0);
  EXPECT_EQ(run(tree, { { 1.0, 0, 0 } }), BT::NodeStatus::SUCCESS);
  EXPECT_DOUBLE_EQ(lastReceived().twist.linear.x, 0.0);
}

TEST_F(DeadReckonMotionRosTest, FailsAndStopsWhenOdometryGoesStale)
{
  auto tree = makeTree(R"(<Action ID="DriveDistance" distance="1.0" />)");
  ASSERT_EQ(run(tree, { { 0, 0, 0 } }), BT::NodeStatus::RUNNING);
  EXPECT_EQ(tickUntilDone(tree, 3s), BT::NodeStatus::FAILURE);
  EXPECT_DOUBLE_EQ(lastReceived().twist.linear.x, 0.0);
}

TEST_F(DeadReckonMotionRosTest, FailsAndStopsWhenTheOdometryStampFreezes)
{
  // An estimator that stalls but keeps republishing its last pose.
  auto tree = makeTree(R"(<Action ID="DriveDistance" distance="1.0" />)");
  const auto frozen = test_node_->now();
  auto status = tree.tickOnce();
  const auto deadline = std::chrono::steady_clock::now() + 3s;
  while (status == BT::NodeStatus::RUNNING && std::chrono::steady_clock::now() < deadline)
  {
    publishOdometry(0, 0, 0, frozen);
    std::this_thread::sleep_for(20ms);
    status = tree.tickOnce();
  }
  EXPECT_EQ(status, BT::NodeStatus::FAILURE);
  EXPECT_DOUBLE_EQ(lastReceived().twist.linear.x, 0.0);
}

TEST_F(DeadReckonMotionRosTest, RestartsAfterFinishing)
{
  auto tree = makeTree(R"(<Action ID="DriveDistance" distance="1.0" />)");
  ASSERT_EQ(run(tree, { { 0, 0, 0 }, { 1.0, 0, 0 } }), BT::NodeStatus::SUCCESS);
  // A fresh start pose: the same odometry is now the start of a new 1 m drive.
  EXPECT_EQ(run(tree, { { 1.0, 0, 0 }, { 1.5, 0, 0 } }), BT::NodeStatus::RUNNING);
  EXPECT_GT(lastReceived().twist.linear.x, 0.0);
  EXPECT_EQ(run(tree, { { 2.0, 0, 0 } }), BT::NodeStatus::SUCCESS);
}

TEST_F(DeadReckonMotionRosTest, FailsAndStopsOnNonFiniteOdometry)
{
  auto tree = makeTree(R"(<Action ID="DriveDistance" distance="1.0" />)");
  EXPECT_EQ(run(tree, { { 0, 0, 0 }, { std::nan(""), 0, 0 } }), BT::NodeStatus::FAILURE);
  EXPECT_DOUBLE_EQ(lastReceived().twist.linear.x, 0.0);
}

TEST_F(DeadReckonMotionRosTest, FailsAndStopsOnTimeout)
{
  auto tree = makeTree(R"(<Action ID="TurnInPlace" angle="1.0" timeout_sec="0.3" />)");
  auto status = tree.tickOnce();
  const auto deadline = std::chrono::steady_clock::now() + 3s;
  while (status == BT::NodeStatus::RUNNING && std::chrono::steady_clock::now() < deadline)
  {
    publishOdometry(0, 0, 0);
    std::this_thread::sleep_for(20ms);
    status = tree.tickOnce();
  }
  EXPECT_EQ(status, BT::NodeStatus::FAILURE);
  EXPECT_DOUBLE_EQ(lastReceived().twist.angular.z, 0.0);
}

TEST_F(DeadReckonMotionRosTest, HaltStopsTheBase)
{
  auto tree = makeTree(R"(<Action ID="TurnInPlace" angle="1.0" />)");
  ASSERT_EQ(run(tree, { { 0, 0, 0 }, { 0, 0, 0.1 } }), BT::NodeStatus::RUNNING);
  ASSERT_GT(lastReceived().twist.angular.z, 0.0);
  tree.haltTree();
  EXPECT_DOUBLE_EQ(lastReceived().twist.angular.z, 0.0);
}

TEST_F(DeadReckonMotionRosTest, TurnInPlaceCountsYawAcrossTheWrap)
{
  // Starts at 170 deg and turns +90 deg, crossing +/-180 deg.
  auto tree = makeTree(R"(<Action ID="TurnInPlace" angle="1.5707963" />)");
  const double start = 170.0 * M_PI / 180.0;
  EXPECT_EQ(run(tree, { { 0, 0, start }, { 0, 0, start + 0.5 }, { 0, 0, start + 1.0 } }), BT::NodeStatus::RUNNING);
  EXPECT_GT(lastReceived().twist.angular.z, 0.0);
  EXPECT_EQ(run(tree, { { 0, 0, start + M_PI_2 - 2 * M_PI } }), BT::NodeStatus::SUCCESS);
  EXPECT_DOUBLE_EQ(lastReceived().twist.angular.z, 0.0);
}

TEST_F(DeadReckonMotionRosTest, FailsWithoutOdometry)
{
  auto tree = makeTree(R"(<Action ID="TurnInPlace" angle="1.0" />)");
  EXPECT_EQ(tickUntilDone(tree, 3s), BT::NodeStatus::FAILURE);
  // Never moved, so the only command is the zero twist published on stop.
  std::this_thread::sleep_for(100ms);
  EXPECT_EQ(receivedCount(), 1u);
  EXPECT_DOUBLE_EQ(lastReceived().twist.angular.z, 0.0);
}

TEST_F(DeadReckonMotionRosTest, RejectsInvalidPorts)
{
  for (const auto* action :
       { R"(<Action ID="DriveDistance" distance="-1.0" />)", R"(<Action ID="DriveDistance" max_linear_velocity="0" />)",
         R"(<Action ID="TurnInPlace" max_angular_velocity="-0.5" />)",
         R"(<Action ID="TurnInPlace" timeout_sec="0" />)" })
  {
    auto tree = makeTree(action);
    EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE) << action;
  }
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
