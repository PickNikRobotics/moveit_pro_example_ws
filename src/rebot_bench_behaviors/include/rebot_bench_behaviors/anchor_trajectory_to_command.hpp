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

#pragma once

#include <memory>
#include <string>

#include <behaviortree_cpp/action_node.h>
#include <moveit_pro_behavior_interface/behavior_context.hpp>
#include <moveit_pro_behavior_interface/shared_resources_node.hpp>

namespace rebot_bench_behaviors
{
/**
 * @brief Moves a trajectory planned from the measured joint state so that it starts at the controller's command.
 *
 * @details A joint held in impedance mode settles short of its command under load. A trajectory planned from the
 * measured state therefore starts away from the command, and a controller that adopts the first point of each
 * trajectory as its new command steps the arm by that difference before it moves. This Behavior adds to every point
 * of each planned joint the offset that puts its first point on commanded_state, so the command stays continuous and
 * the loaded arm ends where the plan does. Joints that commanded_state names but the trajectory leaves out are added
 * and held at their commanded positions, so a goal for part of the arm does not let the controller fill them from
 * the measured state. Missing endpoints and starts more than 0.10 from measured_state use measurement instead,
 * including held joints. The caller commits end_state only after successful execution.
 *
 * fade_out ramps the offset to zero over the trajectory, smoothly in velocity and acceleration, so that it ends
 * exactly at its planned last point, for a move to a pose where the arm is not loaded. measured_joints start from
 * measured_state even when commanded_state names them, such as a jaw planned from its measured position.
 *
 * | Data Port Name      | Port Type | Object Type                           |
 * | ------------------- | --------- | ------------------------------------- |
 * | joint_trajectory    | input     | trajectory_msgs::msg::JointTrajectory |
 * | measured_state      | input     | moveit_msgs::msg::RobotState          |
 * | commanded_state     | input     | moveit_msgs::msg::RobotState          |
 * | fade_out            | input     | bool                                  |
 * | measured_joints     | input     | std::vector<std::string>              |
 * | anchored_trajectory | output    | trajectory_msgs::msg::JointTrajectory |
 * | end_state           | output    | moveit_msgs::msg::RobotState          |
 */
class AnchorTrajectoryToCommand final : public moveit_pro::behaviors::SharedResourcesNode<BT::SyncActionNode>
{
public:
  AnchorTrajectoryToCommand(const std::string& name, const BT::NodeConfiguration& config,
                            const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources);

  static BT::PortsList providedPorts();

  static BT::KeyValueVector metadata();

  BT::NodeStatus tick() override;
};
}  // namespace rebot_bench_behaviors
