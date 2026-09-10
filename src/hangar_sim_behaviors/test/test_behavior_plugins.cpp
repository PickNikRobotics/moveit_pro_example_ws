// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include <gtest/gtest.h>

#include <behaviortree_cpp/bt_factory.h>
#include <moveit_pro_behavior_interface/shared_resources_node_loader.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/node.hpp>

/**
 * @brief Makes sure the Behaviors in this package register and instantiate.
 *
 * @details Instantiating is the part that matters. A Behavior that registers but cannot be built --
 * a port default that does not parse, a missing template specialization -- fails here rather than
 * when an operator presses run.
 */
TEST(BehaviorTests, test_load_behavior_plugins)
{
  pluginlib::ClassLoader<moveit_pro::behaviors::SharedResourcesNodeLoaderBase> class_loader(
      "moveit_pro_behavior_interface", "moveit_pro::behaviors::SharedResourcesNodeLoaderBase");

  auto node = std::make_shared<rclcpp::Node>("test_node");
  auto shared_resources = std::make_shared<moveit_pro::behaviors::BehaviorContext>(node);

  BT::BehaviorTreeFactory factory;
  {
    auto plugin_instance = class_loader.createUniqueInstance("hangar_sim_behaviors::HangarSimBehaviorsLoader");
    ASSERT_NO_THROW(plugin_instance->registerBehaviors(factory, shared_resources));
  }

  EXPECT_NO_THROW((void)factory.instantiateTreeNode("test_behavior_name", "CallEmptyService", BT::NodeConfiguration()));
  EXPECT_NO_THROW(
      (void)factory.instantiateTreeNode("test_behavior_name", "ProjectPoseToPlane", BT::NodeConfiguration()));
  EXPECT_NO_THROW((void)factory.instantiateTreeNode("test_behavior_name", "ScanMatchResidual", BT::NodeConfiguration()));
  EXPECT_NO_THROW(
      (void)factory.instantiateTreeNode("test_behavior_name", "SeedLocalizationAtPose", BT::NodeConfiguration()));
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
