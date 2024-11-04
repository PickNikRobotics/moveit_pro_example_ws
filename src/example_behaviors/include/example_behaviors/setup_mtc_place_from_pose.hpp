#pragma once

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/bt_factory.h>
#include <moveit_studio_behavior_interface/behavior_context.hpp>
#include <moveit_studio_behavior_interface/shared_resources_node.hpp>

namespace example_behaviors
{
/**
 * @brief Given an existing MTC Task object and a target grasp pose, appends MTC stages to describe a motion plan to
 * approach, grasp, and lift an object at that pose.
 *
 * @details
 * | Data Port Name | Port Type     | Object Type                                     |
 * | -------------- |---------------|-------------------------------------------------|
 * | task           | Bidirectional | std::shared_ptr<moveit::task_constructor::Task> |
 * | grasp_pose     | Input         | geometry_msgs::msg::PoseStamped                 |
 */
class SetupMtcPlaceFromPose final : public moveit_studio::behaviors::SharedResourcesNode<BT::SyncActionNode>
{
public:
  SetupMtcPlaceFromPose(const std::string& name, const BT::NodeConfiguration& config,
                        const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources);

  static BT::PortsList providedPorts();

  static BT::KeyValueVector metadata();

  BT::NodeStatus tick() override;
};
}  // namespace example_behaviors
