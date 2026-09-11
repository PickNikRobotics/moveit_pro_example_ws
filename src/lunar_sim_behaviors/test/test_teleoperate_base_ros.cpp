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
#include <mutex>
#include <thread>
#include <vector>

#include <behaviortree_cpp/bt_factory.h>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <moveit_pro_behavior_interface/shared_resources_node_loader.hpp>
#include <moveit_pro_controllers_msgs/msg/velocity_force_command.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

/**
 * @brief Runs the real TeleoperateBase Behavior over ROS: a VelocityForceCommand on /pose_jog/base must come out as a
 * clamped, freshly-stamped TwistStamped on /cmd_vel, nothing is re-published until a new command arrives, and halting
 * publishes a single zero twist.
 */
class TeleoperateBaseRosTest : public testing::Test
{
protected:
  void SetUp() override
  {
    behavior_node_ = std::make_shared<rclcpp::Node>("behavior_node");
    test_node_ = std::make_shared<rclcpp::Node>("test_node");
    executor_.add_node(behavior_node_);
    executor_.add_node(test_node_);
    spin_thread_ = std::thread([this] { executor_.spin(); });

    command_publisher_ = test_node_->create_publisher<moveit_pro_controllers_msgs::msg::VelocityForceCommand>(
        "/pose_jog/base", rclcpp::SystemDefaultsQoS());
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
    tree_ = factory_.createTreeFromText(R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="Test">
          <Action ID="TeleoperateBase" planning_group="base" cmd_vel_topic="/cmd_vel" frame_id="footprint"
                  max_linear_velocity="0.8" max_angular_velocity="2.0" />
        </BehaviorTree>
      </root>)");
  }

  void TearDown() override
  {
    executor_.cancel();
    spin_thread_.join();
  }

  // Ticks the tree until /cmd_vel has delivered `count` messages, or the deadline passes.
  bool tickUntilReceived(const std::size_t count, const std::chrono::milliseconds timeout = 5s)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline)
    {
      tree_.tickOnce();
      {
        const std::scoped_lock lock(mutex_);
        if (received_.size() >= count)
        {
          return true;
        }
      }
      std::this_thread::sleep_for(10ms);
    }
    return false;
  }

  std::size_t receivedCount()
  {
    const std::scoped_lock lock(mutex_);
    return received_.size();
  }

  geometry_msgs::msg::TwistStamped received(const std::size_t index)
  {
    const std::scoped_lock lock(mutex_);
    return received_.at(index);
  }

  rclcpp::Node::SharedPtr behavior_node_;
  rclcpp::Node::SharedPtr test_node_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::thread spin_thread_;
  rclcpp::Publisher<moveit_pro_controllers_msgs::msg::VelocityForceCommand>::SharedPtr command_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_subscription_;
  std::mutex mutex_;
  std::vector<geometry_msgs::msg::TwistStamped> received_;

  std::shared_ptr<moveit_pro::behaviors::BehaviorContext> shared_resources_;
  std::unique_ptr<pluginlib::ClassLoader<moveit_pro::behaviors::SharedResourcesNodeLoaderBase>> class_loader_;
  pluginlib::UniquePtr<moveit_pro::behaviors::SharedResourcesNodeLoaderBase> plugin_;
  BT::BehaviorTreeFactory factory_;
  BT::Tree tree_;
};

TEST_F(TeleoperateBaseRosTest, ForwardsRestampedClampedTwistAndZeroesOnHalt)
{
  // onStart: subscription + publisher come up, Behavior stays RUNNING and publishes nothing on its own.
  ASSERT_EQ(tree_.tickOnce(), BT::NodeStatus::RUNNING);
  const auto discovery_deadline = std::chrono::steady_clock::now() + 5s;
  while (command_publisher_->get_subscription_count() == 0 && std::chrono::steady_clock::now() < discovery_deadline)
  {
    std::this_thread::sleep_for(10ms);
  }
  ASSERT_GT(command_publisher_->get_subscription_count(), 0u);
  EXPECT_FALSE(tickUntilReceived(1, 300ms));

  // A command with a stale stamp (the browser's clock) and an over-limit linear.x.
  moveit_pro_controllers_msgs::msg::VelocityForceCommand command;
  command.header.stamp = behavior_node_->now() - rclcpp::Duration(100s);
  command.velocity_controlled_axes.x = true;
  command.velocity_controlled_axes.rz = true;
  command.twist.linear.x = 5.0;
  command.twist.angular.z = -0.5;
  command.twist.linear.y = 1.0;
  command_publisher_->publish(command);

  ASSERT_TRUE(tickUntilReceived(1));
  const auto forwarded = received(0);
  EXPECT_EQ(forwarded.header.frame_id, "footprint");
  EXPECT_DOUBLE_EQ(forwarded.twist.linear.x, 0.8);
  EXPECT_DOUBLE_EQ(forwarded.twist.angular.z, -0.5);
  EXPECT_DOUBLE_EQ(forwarded.twist.linear.y, 0.0);
  EXPECT_LT((behavior_node_->now() - rclcpp::Time(forwarded.header.stamp)).seconds(), 1.0);

  // No new command -> nothing more is published, however many ticks happen.
  EXPECT_FALSE(tickUntilReceived(2, 300ms));
  EXPECT_EQ(receivedCount(), 1u);

  // Halting stops the base with exactly one zero twist.
  tree_.haltTree();
  const auto halt_deadline = std::chrono::steady_clock::now() + 5s;
  while (receivedCount() < 2 && std::chrono::steady_clock::now() < halt_deadline)
  {
    std::this_thread::sleep_for(10ms);
  }
  ASSERT_EQ(receivedCount(), 2u);
  const auto zero = received(1);
  EXPECT_EQ(zero.header.frame_id, "footprint");
  EXPECT_DOUBLE_EQ(zero.twist.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(zero.twist.angular.z, 0.0);
  EXPECT_LT((behavior_node_->now() - rclcpp::Time(zero.header.stamp)).seconds(), 1.0);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
