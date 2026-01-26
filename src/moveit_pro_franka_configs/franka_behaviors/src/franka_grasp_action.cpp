#include "franka_behaviors/franka_grasp_action.hpp"

#include <moveit_pro_behavior_interface/get_required_ports.hpp>
#include <moveit_pro_behavior_interface/impl/action_client_behavior_base_impl.hpp>
#include <moveit_pro_behavior_interface/metadata_fields.hpp>

namespace
{
inline constexpr auto kDescriptionGraspAction = R"(TODO)";
constexpr auto PortIDActionName = "action_name";
constexpr auto kPortIDTimeout = "timeout";
constexpr auto kPortIDWidth = "width";
constexpr auto kPortIDSpeed = "speed";
constexpr auto kPortIDForce = "force";
constexpr auto kPortIDInnerEpsilon = "inner_epsilon";
constexpr auto kPortIDOuterEpsilon = "outer_epsilon";
}  // namespace

namespace franka_behaviors
{
GraspAction::GraspAction(const std::string& name, const BT::NodeConfiguration& config,
                         const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
  : moveit_pro::behaviors::ActionClientBehaviorBase<Grasp>(name, config, shared_resources)
{
}

BT::PortsList GraspAction::providedPorts()
{
  return { BT::InputPort<std::string>(PortIDActionName, "Action name."),
           BT::InputPort<double>(kPortIDWidth, "Gripper width target position."),
           BT::InputPort<double>(kPortIDSpeed, 0.1, "Gripper speed."),
           BT::InputPort<double>(kPortIDForce, "Gripper force."),
           BT::InputPort<double>(kPortIDTimeout, 10.0, "Seconds before behavior fails if no response is received."),
           BT::InputPort<double>(kPortIDInnerEpsilon, 0.005, "Lower grasp width tolerance [m]."),
           BT::InputPort<double>(kPortIDOuterEpsilon, 0.005, "Upper grasp width tolerance [m].") };
}

BT::KeyValueVector GraspAction::metadata()
{
  return { { moveit_pro::behaviors::kSubcategoryMetadataKey, "Grasping" },
           { moveit_pro::behaviors::kDescriptionMetadataKey, kDescriptionGraspAction } };
}

tl::expected<std::string, std::string> GraspAction::getActionName()
{
  const auto ports = moveit_pro::behaviors::getRequiredInputs(getInput<std::string>(PortIDActionName));
  if (!ports.has_value())
  {
    return tl::make_unexpected("Failed to retrieve required Grasp action namespace from input data port: " +
                               ports.error());
  }
  else
  {
    const auto& [action_name] = ports.value();
    return action_name;
  }
}

tl::expected<std::chrono::duration<double>, std::string> GraspAction::getResultTimeout()
{
  const auto ports = moveit_pro::behaviors::getRequiredInputs(getInput<double>(kPortIDTimeout));

  if (!ports.has_value())
  {
    return tl::make_unexpected("Failed to retrieve required value from input data port: " + ports.error());
  }
  const auto& [timeout] = ports.value();
  return std::chrono::duration<double>{ timeout };
}

tl::expected<Grasp::Goal, std::string> GraspAction::createGoal()
{
  const auto ports =
      moveit_pro::behaviors::getRequiredInputs(getInput<double>(kPortIDWidth), getInput<double>(kPortIDSpeed),
                                               getInput<double>(kPortIDForce), getInput<double>(kPortIDInnerEpsilon),
                                               getInput<double>(kPortIDOuterEpsilon));

  if (!ports.has_value())
  {
    return tl::make_unexpected("Failed to retrieve required value from input data port: " + ports.error());
  }

  const auto& [width, speed, force, inner_epsilon, outer_epsilon] = ports.value();

  Grasp::Goal goal;
  goal.width = width;
  goal.speed = speed;
  goal.force = force;
  goal.epsilon.inner = inner_epsilon;
  goal.epsilon.outer = outer_epsilon;
  return goal;
}
}  // namespace franka_behaviors

// This specializes ActionClientBehaviorBase for Grasp actions
template class moveit_pro::behaviors::ActionClientBehaviorBase<franka_behaviors::Grasp>;
