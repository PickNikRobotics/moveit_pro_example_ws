// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <behaviortree_cpp/action_node.h>
#include <moveit_pro_behavior_interface/behavior_context.hpp>
#include <moveit_pro_behavior_interface/shared_resources_node.hpp>

#include <memory>
#include <string>

namespace hangar_sim_behaviors
{
/**
 * @brief Relabel a pose's frame without transforming it. The numbers are kept; only the name changes.
 *
 * @details This is NOT `TransformPoseFrame`. It performs no TF lookup and moves nothing: it asserts
 * that the coordinates already mean the same thing in the target frame. That assertion is only ever
 * true when the two frames are related by an identity transform, and it is the caller's job to know
 * that and to say why at the call site.
 *
 * It exists because a TF lookup is not always available or not always meaningful. A pose that must
 * be expressed in a localizer's global frame *before* the localizer is seeded cannot be transformed
 * into it -- the edge that would carry the transform is the very thing the seed creates, and
 * whatever the localizer publishes on it beforehand is not a measurement. Relabeling in the open,
 * with the assumption written next to it, is honest where a transform through an unconverged
 * estimate silently is not.
 *
 * Prefer `TransformPoseFrame` in every case where the transform exists and means something.
 *
 * Set `expected_input_frame_id` wherever the caller knows which frame the pose must arrive in. The
 * identity assumption is made about a specific PAIR of frames, so a pose arriving in some other
 * frame does not inherit it; relabelling it anyway yields a pose that is silently somewhere else,
 * and no downstream frame check can catch that, because the pose already carries the target frame's
 * name by then. With the port set the Behavior refuses instead. Left empty, it relabels whatever
 * arrives, which keeps it usable where the caller genuinely does not care.
 *
 * | Data Port Name          | Port Type | Object Type                     |
 * | ----------------------- | --------- | ------------------------------- |
 * | input_pose              | Input     | geometry_msgs::msg::PoseStamped |
 * | frame_id                | Input     | std::string                     |
 * | expected_input_frame_id | Input     | std::string                     |
 * | output_pose             | Output    | geometry_msgs::msg::PoseStamped |
 */
class ReinterpretPoseFrame final : public moveit_pro::behaviors::SharedResourcesNode<BT::SyncActionNode>
{
public:
  ReinterpretPoseFrame(const std::string& name, const BT::NodeConfiguration& config,
                       const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources);

  static BT::PortsList providedPorts();

  static BT::KeyValueVector metadata();

  BT::NodeStatus tick() override;
};

}  // namespace hangar_sim_behaviors
