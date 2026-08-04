// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include <vla_sim_behaviors/send_gripper_command.hpp>

#include <chrono>

#include <fmt/format.h>
#include <moveit_pro_behavior_interface/get_required_ports.hpp>
#include <moveit_pro_behavior_interface/metadata_fields.hpp>

namespace
{
inline constexpr auto kDescriptionSendGripperCommand = R"(
                <p>
                    Sends the gripper a position goal and succeeds as soon as it is
                    dispatched, without waiting for the server to accept it or the jaws to
                    arrive.
                </p>
                <p>
                    This matches how <code>ExecutePolicy</code> drives the gripper at deploy time:
                    one goal per change of target, left to run while the arm keeps moving. Use it
                    to collect demonstrations of a grip the policy can reproduce. The goal stays
                    live afterwards, holding its commanded target against whatever the jaws have
                    closed onto.
                </p>
                <p>
                    Pair with a <code>WaitForDuration</code> to hold the arm still while the jaws
                    travel.
                </p>
            )";

constexpr auto kPortIDActionName = "gripper_command_action_name";
constexpr auto kPortIDPosition = "position";
constexpr auto kPortIDMaxEffort = "max_effort";
constexpr auto kPortIDServerTimeout = "wait_for_server_timeout";
}  // namespace

namespace vla_sim_behaviors
{
SendGripperCommand::SendGripperCommand(const std::string& name, const BT::NodeConfiguration& config,
                                       const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
  : SharedResourcesNode<BT::SyncActionNode>(name, config, shared_resources)
{
}

BT::PortsList SendGripperCommand::providedPorts()
{
  return {
    BT::InputPort<std::string>(kPortIDActionName, "/robotiq_gripper_controller/gripper_cmd",
                               "GripperCommand action that actuates the gripper."),
    BT::InputPort<double>(kPortIDPosition, "Gripper joint target position."),
    BT::InputPort<double>(kPortIDMaxEffort, "0.0", "Effort ceiling; 0 leaves it to the controller."),
    BT::InputPort<double>(kPortIDServerTimeout, "3.0", "Seconds to wait for the action server to appear."),
  };
}

BT::KeyValueVector SendGripperCommand::metadata()
{
  return { { moveit_pro::behaviors::kSubcategoryMetadataKey, "Cube Stacking" },
           { moveit_pro::behaviors::kDescriptionMetadataKey, kDescriptionSendGripperCommand } };
}

BT::NodeStatus SendGripperCommand::tick()
{
  const auto ports =
      moveit_pro::behaviors::getRequiredInputs(getInput<std::string>(kPortIDActionName),
                                               getInput<double>(kPortIDPosition), getInput<double>(kPortIDMaxEffort),
                                               getInput<double>(kPortIDServerTimeout));
  if (!ports.has_value())
  {
    getBehaviorContext()->logger->publishFailureMessage(
        name(), "Failed to get required values from input data ports: " + ports.error());
    return BT::NodeStatus::FAILURE;
  }
  const auto& [action_name, position, max_effort, server_timeout] = ports.value();

  // The client is kept between ticks: it outlives the goal request either way, and the
  // shared node's executor is what delivers it.
  if (client_ == nullptr || action_name_ != action_name)
  {
    client_ =
        rclcpp_action::create_client<control_msgs::action::GripperCommand>(getBehaviorContext()->node, action_name);
    action_name_ = action_name;
  }

  const auto timeout = std::chrono::duration<double>(server_timeout);
  if (!client_->wait_for_action_server(std::chrono::duration_cast<std::chrono::nanoseconds>(timeout)))
  {
    getBehaviorContext()->logger->publishFailureMessage(name(), fmt::format("No GripperCommand action server on '{}'.",
                                                                            action_name));
    return BT::NodeStatus::FAILURE;
  }

  control_msgs::action::GripperCommand::Goal goal;
  goal.command.position = position;
  goal.command.max_effort = max_effort;
  // The goal handle is dropped, since waiting on it is the blocking this Behavior exists to
  // avoid. A rejected goal surfaces as a gripper that did not move.
  client_->async_send_goal(goal);

  return BT::NodeStatus::SUCCESS;
}
}  // namespace vla_sim_behaviors
