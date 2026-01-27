#pragma once

#include <behaviortree_cpp/action_node.h>

// This header includes the SharedResourcesNode type
#include <moveit_pro_behavior_interface/async_behavior_base.hpp>
#include <moveit_pro_behavior_interface/shared_resources_node.hpp>

namespace example_behaviors
{
/**
 * @brief TODO(...)
 */
class ExampleNDTRegistration : public moveit_pro::behaviors::AsyncBehaviorBase
{
public:
  /**
   * @brief Constructor for the behavior.
   * @param name The name of a particular instance of this Behavior. This will be set by the behavior tree factory when
   * this Behavior is created within a new behavior tree.
   * @param shared_resources A shared_ptr to a BehaviorContext that is shared among all SharedResourcesNode Behaviors in
   * the behavior tree. This BehaviorContext is owned by the Studio Agent's ObjectiveServerNode.
   * @param config This contains runtime configuration info for this Behavior, such as the mapping between the
   * Behavior's data ports on the behavior tree's blackboard. This will be set by the behavior tree factory when this
   * Behavior is created within a new behavior tree.
   * @details An important limitation is that the members of the base Behavior class are not instantiated until after
   * the initialize() function is called, so these classes should not be used within the constructor.
   */
  ExampleNDTRegistration(const std::string& name, const BT::NodeConfiguration& config,
                         const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources);

  /**
   * @brief Implementation of the required providedPorts() function for the Behavior.
   * @details The BehaviorTree.CPP library requires that Behaviors must implement a static function named
   * providedPorts() which defines their input and output ports. If the Behavior does not use any ports, this function
   * must return an empty BT::PortsList. This function returns a list of ports with their names and port info, which is
   * used internally by the behavior tree.
   * @return See ndt_registration.cpp for port list and description.
   */
  static BT::PortsList providedPorts();

  /**
   * @brief Implementation of the metadata() function for displaying metadata, such as Behavior description and
   * subcategory, in the MoveIt Studio Developer Tool.
   * @return A BT::KeyValueVector containing the Behavior metadata.
   */
  static BT::KeyValueVector metadata();

protected:
  tl::expected<bool, std::string> doWork() override;

private:
  /** @brief Classes derived from AsyncBehaviorBase must implement getFuture() so that it returns a shared_future class member */
  std::shared_future<tl::expected<bool, std::string>>& getFuture() override
  {
    return future_;
  }

  /** @brief Classes derived from AsyncBehaviorBase must have this shared_future as a class member */
  std::shared_future<tl::expected<bool, std::string>> future_;
};
}  // namespace example_behaviors
